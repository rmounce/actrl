#!/usr/bin/env python3
"""Calibration data loader: align data/raw/ archives (see docs/data.md,
tools/export_history.py) into a single uniform-grid DataFrame for
simulator-calibration analysis (docs/ideas.md #3).

Dev-only tooling. Requires pandas + pyarrow (`uv add --dev pandas pyarrow`).
The deployed apps (actrl.py, control.py, statctrl.py) must never import this
module.

Pure functions (parsing, binning, each fill rule, seed scan) are unit tested
offline in tests/test_calib.py; thin I/O wrappers read data_dir/raw/.

Resampling rules by kind -- see docs/data.md "Loading for analysis" for the
narrative version:
  - temperature: per-bin mean; interpolate gaps <= 15 min (time-linear).
  - state:       unlimited ffill (LOCF); seeded by scanning back seed_days
                 for the most recent point before `start`.
  - power:       clamp >= 0 before binning, then per-bin mean; no fill.
  - controller:  per-bin mean, then ffill up to 2 min only.
  - slow:        time-linear interpolation onto the grid, no extrapolation.
"""
from __future__ import annotations

import argparse
import csv
import gzip
import io
import sys
from datetime import date, timedelta
from pathlib import Path

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_DATA_DIR = REPO_ROOT / "data"
DEFAULT_FREQ = "1min"
DEFAULT_SEED_DAYS = 14

# --------------------------------------------------------------------------
# Column mapping: (measurement, entity_id, field) -> (column_name, kind)
# --------------------------------------------------------------------------

_ROOM_TEMPS = ["bed_1", "bed_2", "bed_3", "kitchen", "study"]
_M5ATOM_TEMPS = [
    "m5atom_inside_temp",
    "m5atom_outside_temp",
    "m5atom_inside_coil_inlet_temp",
    "m5atom_outside_coil_temp",
]
_DAMPERS = ["bed_1", "bed_2", "bed_3", "kitchen", "study"]
_AIRCONS = ["bed_1_aircon", "bed_2_aircon", "bed_3_aircon", "kitchen_aircon", "study_aircon"]
_CONTROLLER_ENTITIES = [
    "aircon_avg_deriv",
    "aircon_comp_speed",
    "aircon_meta_integral",
    "aircon_weighted_error",
    "bed_1_damper_target",
    "bed_1_pid",
    "bed_2_damper_target",
    "bed_2_pid",
    "bed_3_damper_target",
    "bed_3_pid",
    "kitchen_damper_target",
    "kitchen_pid",
    "study_damper_target",
    "study_pid",
]


def _build_column_specs() -> dict[tuple[str, str, str], tuple[str, str]]:
    specs: dict[tuple[str, str, str], tuple[str, str]] = {}

    for room in _ROOM_TEMPS:
        entity = f"{room}_average_temperature"
        specs[("sensor__temperature", entity, "value")] = (entity, "temperature")
    for entity in _M5ATOM_TEMPS:
        specs[("sensor__temperature", entity, "value")] = (entity, "temperature")

    for room in _DAMPERS:
        specs[("cover__damper", room, "current_position")] = (f"damper.{room}", "state")

    for aircon in _AIRCONS:
        specs[("climate", aircon, "target_temp_high")] = (f"climate.{aircon}.target_temp_high", "state")
        specs[("climate", aircon, "target_temp_low")] = (f"climate.{aircon}.target_temp_low", "state")
    specs[("climate", "m5atom_climate", "temperature")] = ("climate.m5atom_climate.temperature", "state")

    specs[("sensor__power", "shellyem_ec64c9c6932b_channel_1_power", "value")] = ("power.outdoor_unit", "power")
    specs[("sensor__power", "shellyem_ec64c9c6932b_channel_2_power", "value")] = ("power.indoor_unit", "power")

    for entity in _CONTROLLER_ENTITIES:
        specs[("input_number", entity, "value")] = (entity, "controller")

    specs[("temperature_adelaide", "adelaide_west_terrace_ngayirdapira_temp", "mean_value")] = (
        "temperature_adelaide",
        "slow",
    )
    specs[("humidity_adelaide", "adelaide_west_terrace_ngayirdapira_humidity", "mean_value")] = (
        "humidity_adelaide",
        "slow",
    )
    specs[("power_load_5m", "sigen_plant_consumed_power", "mean_value")] = ("power_load_5m", "slow")
    specs[("power_pv_5m", "solcast_pv_forecast_power_now", "mean_value")] = ("power_pv_5m", "slow")

    return specs


COLUMN_SPECS: dict[tuple[str, str, str], tuple[str, str]] = _build_column_specs()

KINDS = ("temperature", "state", "power", "controller", "slow")

TEMPERATURE_INTERP_CAP = pd.Timedelta("15min")
CONTROLLER_FFILL_CAP = pd.Timedelta("2min")


# --------------------------------------------------------------------------
# Pure parsing helpers
# --------------------------------------------------------------------------


def parse_csv_text(text: str) -> list[tuple[str, str, str]]:
    """Parse decompressed `time,field,value` CSV text into a list of
    (time_str, field, value_str) rows (header row dropped)."""
    reader = csv.reader(io.StringIO(text))
    rows = list(reader)
    if not rows:
        return []
    return [tuple(row) for row in rows[1:] if row]


def rows_for_field(rows: list[tuple[str, str, str]], field: str) -> list[tuple[pd.Timestamp, float]]:
    """Filter parsed rows to a single field name and parse timestamp/value.
    Timestamps are UTC ISO8601 with variable sub-second precision."""
    out = []
    for time_str, row_field, value_str in rows:
        if row_field != field:
            continue
        out.append((pd.Timestamp(time_str), float(value_str)))
    return out


def points_to_series(points: list[tuple[pd.Timestamp, float]]) -> pd.Series:
    """Build a sorted, deduplicated (keep last) UTC-indexed Series from raw
    (timestamp, value) points."""
    if not points:
        return pd.Series(dtype=float, index=pd.DatetimeIndex([], tz="UTC"))
    idx = pd.DatetimeIndex([p[0] for p in points])
    s = pd.Series([p[1] for p in points], index=idx)
    s = s[~s.index.duplicated(keep="last")].sort_index()
    return s


# --------------------------------------------------------------------------
# Grid + per-kind resampling (pure, operate on pandas Series/Index)
# --------------------------------------------------------------------------


def build_grid(start: date, end: date, freq: str) -> pd.DatetimeIndex:
    """Uniform UTC grid covering [start 00:00, end+1day 00:00) at freq."""
    start_ts = pd.Timestamp(start, tz="UTC")
    end_ts = pd.Timestamp(end, tz="UTC") + pd.Timedelta(days=1)
    return pd.date_range(start=start_ts, end=end_ts, freq=freq, inclusive="left")


def cap_periods(cap: pd.Timedelta, freq: str) -> int:
    """Number of grid periods a time cap spans, minimum 1."""
    freq_td = pd.Timedelta(freq)
    return max(1, int(cap / freq_td))


def bin_mean(raw: pd.Series, grid: pd.DatetimeIndex, freq: str) -> pd.Series:
    """Per-bin mean of raw points onto grid bins [t, t+freq); empty bins are
    NaN. Bins align to grid[0]."""
    if len(grid) == 0:
        return pd.Series(dtype=float, index=grid)
    if raw.empty:
        return pd.Series(float("nan"), index=grid)
    binned = raw.resample(freq, origin=grid[0]).mean()
    return binned.reindex(grid)


def long_gap_mask(is_na: pd.Series, limit: int) -> pd.Series:
    """Boolean mask, True for positions belonging to a run of consecutive
    True values longer than `limit` (i.e. gaps too long to bridge)."""
    values = is_na.to_numpy()
    mask = np.zeros(len(values), dtype=bool)
    n = len(values)
    i = 0
    while i < n:
        if values[i]:
            j = i
            while j < n and values[j]:
                j += 1
            if (j - i) > limit:
                mask[i:j] = True
            i = j
        else:
            i += 1
    return pd.Series(mask, index=is_na.index)


def temperature_series(raw: pd.Series, grid: pd.DatetimeIndex, freq: str) -> pd.Series:
    """Per-bin mean, then bridge gaps <= 15 min by time-linear interpolation.
    Gaps longer than 15 min are left entirely NaN (not partially filled)."""
    binned = bin_mean(raw, grid, freq)
    limit = cap_periods(TEMPERATURE_INTERP_CAP, freq)
    interpolated = binned.interpolate(method="time", limit_area="inside")
    too_long = long_gap_mask(binned.isna(), limit)
    return interpolated.where(~too_long, other=float("nan"))


def power_series(raw: pd.Series, grid: pd.DatetimeIndex, freq: str) -> pd.Series:
    """Clamp values at >= 0 before binning (CT offset), then per-bin mean.
    No fill of empty bins."""
    clamped = raw.clip(lower=0) if not raw.empty else raw
    return bin_mean(clamped, grid, freq)


def controller_series(raw: pd.Series, grid: pd.DatetimeIndex, freq: str) -> pd.Series:
    """Per-bin mean, then ffill up to 2 min only (longer gaps mean the app
    wasn't running)."""
    binned = bin_mean(raw, grid, freq)
    limit = cap_periods(CONTROLLER_FFILL_CAP, freq)
    return binned.ffill(limit=limit)


def state_series(
    raw: pd.Series, grid: pd.DatetimeIndex, seed: tuple[pd.Timestamp, float] | None
) -> pd.Series:
    """Last-observation-carried-forward with unlimited ffill. `seed` (if
    given) is a (timestamp, value) point known to precede everything in
    `raw` and the grid, used so dampers/setpoints idle since before `start`
    still show their held value rather than NaN."""
    points = raw
    if seed is not None:
        seed_ts, seed_val = seed
        seed_series = pd.Series([seed_val], index=pd.DatetimeIndex([seed_ts]))
        points = pd.concat([seed_series, points])
        points = points[~points.index.duplicated(keep="last")].sort_index()
    if points.empty:
        return pd.Series(float("nan"), index=grid)
    combined_index = points.index.union(grid)
    full = points.reindex(combined_index).ffill()
    return full.reindex(grid)


def slow_series(raw: pd.Series, grid: pd.DatetimeIndex) -> pd.Series:
    """Time-linear interpolation onto the grid; no extrapolation beyond the
    first/last raw point."""
    if raw.empty:
        return pd.Series(float("nan"), index=grid)
    combined_index = raw.index.union(grid)
    full = raw.reindex(combined_index).interpolate(method="time", limit_area="inside")
    return full.reindex(grid)


def resample_kind(
    kind: str, raw: pd.Series, grid: pd.DatetimeIndex, freq: str, seed: tuple[pd.Timestamp, float] | None
) -> pd.Series:
    if kind == "temperature":
        return temperature_series(raw, grid, freq)
    if kind == "power":
        return power_series(raw, grid, freq)
    if kind == "controller":
        return controller_series(raw, grid, freq)
    if kind == "state":
        return state_series(raw, grid, seed)
    if kind == "slow":
        return slow_series(raw, grid)
    raise ValueError(f"unknown kind: {kind}")


# --------------------------------------------------------------------------
# Seed scan (cross-day lookback for state series)
# --------------------------------------------------------------------------


def day_list(start: date, end: date) -> list[date]:
    """Inclusive [start, end] list of dates."""
    if end < start:
        raise ValueError(f"end {end} before start {start}")
    out = []
    d = start
    while d <= end:
        out.append(d)
        d = d + timedelta(days=1)
    return out


def pick_seed(points_by_day: list[list[tuple[pd.Timestamp, float]]]) -> tuple[pd.Timestamp, float] | None:
    """Given per-day point lists ordered most-recent-day-first, return the
    single most recent point from the first day (scanning backwards) that
    has any points. None if no day in the lookback window has data."""
    for day_points in points_by_day:
        if day_points:
            return max(day_points, key=lambda p: p[0])
    return None


# --------------------------------------------------------------------------
# I/O wrappers
# --------------------------------------------------------------------------


def raw_file_path(data_dir: Path, day: date, measurement: str, entity_id: str) -> Path:
    return data_dir / "raw" / day.isoformat() / f"{measurement}__{entity_id}.csv.gz"


def read_raw_rows(path: Path) -> list[tuple[str, str, str]]:
    """Read+decompress a data/raw/<day>/<measurement>__<entity>.csv.gz file.
    Missing files (no points that day) return []."""
    if not path.exists():
        return []
    with gzip.open(path, "rt", newline="") as f:
        return parse_csv_text(f.read())


def load_field_points(
    data_dir: Path, days: list[date], measurement: str, entity_id: str, field: str
) -> list[tuple[pd.Timestamp, float]]:
    """Read + concatenate one (measurement, entity, field) series' raw
    points across a list of days."""
    points: list[tuple[pd.Timestamp, float]] = []
    for day in days:
        path = raw_file_path(data_dir, day, measurement, entity_id)
        rows = read_raw_rows(path)
        points.extend(rows_for_field(rows, field))
    return points


def scan_seed(
    data_dir: Path, start: date, measurement: str, entity_id: str, field: str, seed_days: int
) -> tuple[pd.Timestamp, float] | None:
    """Scan up to seed_days days before `start` (most recent first) for the
    most recent point of a state series, to seed unlimited ffill."""
    lookback_days = day_list(start - timedelta(days=seed_days), start - timedelta(days=1))
    lookback_days.reverse()  # most recent day first
    points_by_day = []
    for day in lookback_days:
        path = raw_file_path(data_dir, day, measurement, entity_id)
        rows = read_raw_rows(path)
        points_by_day.append(rows_for_field(rows, field))
    return pick_seed(points_by_day)


# --------------------------------------------------------------------------
# Top-level API
# --------------------------------------------------------------------------


def load_range(
    start: str | date,
    end: str | date,
    data_dir: str | Path = DEFAULT_DATA_DIR,
    freq: str = DEFAULT_FREQ,
    seed_days: int = DEFAULT_SEED_DAYS,
) -> pd.DataFrame:
    """Load data/raw/ archives for [start, end] (inclusive UTC dates) into a
    single uniform-grid DataFrame indexed by UTC DatetimeIndex at `freq`,
    one column per (series, field) per COLUMN_SPECS. See module docstring
    for the resampling rules by kind."""
    start_d = date.fromisoformat(start) if isinstance(start, str) else start
    end_d = date.fromisoformat(end) if isinstance(end, str) else end
    data_dir = Path(data_dir)

    grid = build_grid(start_d, end_d, freq)
    days = day_list(start_d, end_d)

    columns: dict[str, pd.Series] = {}
    for (measurement, entity_id, field), (colname, kind) in COLUMN_SPECS.items():
        points = load_field_points(data_dir, days, measurement, entity_id, field)
        raw = points_to_series(points)

        seed = None
        if kind == "state":
            seed = scan_seed(data_dir, start_d, measurement, entity_id, field, seed_days)

        columns[colname] = resample_kind(kind, raw, grid, freq, seed)

    df = pd.DataFrame(columns, index=grid)
    df.index.name = "time"
    return df


# --------------------------------------------------------------------------
# Coverage report
# --------------------------------------------------------------------------


def gap_runs_over(series: pd.Series, freq: str, threshold: pd.Timedelta) -> int:
    """Count runs of consecutive NaN values whose duration exceeds
    `threshold`."""
    if series.empty:
        return 0
    freq_td = pd.Timedelta(freq)
    min_periods = int(threshold / freq_td) + 1  # a run must span > threshold
    is_na = series.isna().to_numpy()
    count = 0
    run_len = 0
    for v in is_na:
        if v:
            run_len += 1
        else:
            if run_len >= min_periods:
                count += 1
            run_len = 0
    if run_len >= min_periods:
        count += 1
    return count


def coverage_report(df: pd.DataFrame, freq: str) -> pd.DataFrame:
    """Per-column coverage table: non-NaN %, first/last non-NaN timestamp,
    count of gap runs > 30 min."""
    rows = []
    for col in df.columns:
        s = df[col]
        notna = s.notna()
        first_ts = s[notna].index.min() if notna.any() else pd.NaT
        last_ts = s[notna].index.max() if notna.any() else pd.NaT
        rows.append(
            {
                "column": col,
                "coverage_pct": round(100.0 * notna.mean(), 2) if len(s) else 0.0,
                "first": first_ts,
                "last": last_ts,
                "gap_runs_gt_30min": gap_runs_over(s, freq, pd.Timedelta("30min")),
            }
        )
    return pd.DataFrame(rows).set_index("column")


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--start", required=True, help="ISO date (YYYY-MM-DD), inclusive UTC.")
    p.add_argument("--end", required=True, help="ISO date (YYYY-MM-DD), inclusive UTC.")
    p.add_argument(
        "--data-dir",
        type=Path,
        default=DEFAULT_DATA_DIR,
        help="Input data directory containing raw/ (default: data/).",
    )
    p.add_argument("--freq", default=DEFAULT_FREQ, help="Output grid frequency (default: 1min).")
    p.add_argument(
        "--seed-days", type=int, default=DEFAULT_SEED_DAYS, help="State-series seed lookback (default: 14)."
    )
    p.add_argument("--report", action="store_true", help="Print a per-column coverage table.")
    p.add_argument("--out", type=Path, help="Write the aligned DataFrame to parquet here.")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    df = load_range(
        start=args.start,
        end=args.end,
        data_dir=args.data_dir,
        freq=args.freq,
        seed_days=args.seed_days,
    )

    print(f"Loaded {df.shape[0]} rows x {df.shape[1]} columns [{args.start} .. {args.end}] freq={args.freq}", file=sys.stderr)

    if args.report:
        report = coverage_report(df, args.freq)
        with pd.option_context("display.max_rows", None, "display.width", 160):
            print(report.to_string())

    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        df.to_parquet(args.out)
        print(f"Wrote {args.out}", file=sys.stderr)

    return 0


if __name__ == "__main__":
    sys.exit(main())
