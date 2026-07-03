"""June-wide simulator fidelity scorecard (docs/tasks/010).

Replays every complete local day in the June archive through the same
closed-loop simulator that `analysis/replay_day.py` validates day-by-day
(`load_day`/`replay` are imported, not duplicated), and prints one summary
row per day plus a median scorecard across the month. This is the standing
regression gate / baseline the controller-tuning phase will be scored
against -- see docs/calibration.md "Whole-day closed-loop replay".

Usage:
    uv run python analysis/scorecard.py [--parquet PATH] [--out csv_path]
        [--days 2026-06-22,2026-06-27]   # optional subset for quick runs
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

_ROOT = Path(__file__).resolve().parent.parent
for p in (str(_ROOT), str(_ROOT / "tests")):
    if p not in sys.path:
        sys.path.insert(0, p)

from replay_day import LOCAL_TZ, load_day, replay  # noqa: E402
from sim.closed_loop import ROOMS  # noqa: E402

# Column order for the per-day table / CSV / summary.
COLUMNS = [
    "kit_rmse", "kit_bias", "rms_all", "energy_pct",
    "on_frac_sim", "on_frac_rec",
    "abovemin_sim", "abovemin_rec",
    "starts_sim", "starts_rec",
    "on_min_sim", "on_min_rec",
    "off_min_sim", "off_min_rec",
]
# (sim_col, rec_col, label) triples for the summary's side-by-side block.
PAIRED = [
    ("on_frac_sim", "on_frac_rec", "on_frac"),
    ("abovemin_sim", "abovemin_rec", "abovemin"),
    ("starts_sim", "starts_rec", "starts"),
    ("on_min_sim", "on_min_rec", "on_min"),
    ("off_min_sim", "off_min_rec", "off_min"),
]


def _cycle_stats(on: pd.Series) -> tuple[int, float, float]:
    """(off->on start count, mean on-run minutes, mean off-run minutes).

    `on` is a boolean series at 1-minute resolution. A leading run that is
    already "on" at the start of the day is not counted as a start (its
    off->on transition, if any, isn't observed within the window)."""
    on = on.astype(bool)
    grp = (on != on.shift()).cumsum()
    sizes = on.groupby(grp).size()
    values = on.groupby(grp).first()
    on_sizes = sizes[values]
    off_sizes = sizes[~values]
    starts = len(on_sizes) - (1 if len(on) and bool(on.iloc[0]) else 0)
    on_min = float(on_sizes.mean()) if len(on_sizes) else float("nan")
    off_min = float(off_sizes.mean()) if len(off_sizes) else float("nan")
    return starts, on_min, off_min


def score_day(day: pd.DataFrame, sim: pd.DataFrame) -> dict:
    """Per-day metrics dict, keys matching COLUMNS (docs/tasks/010)."""
    metrics: dict = {}
    room_rmses = {}
    for r in ROOMS:
        rec = day[f"{r}_average_temperature"].astype(float)
        err = sim[f"Tm_{r}"] - rec
        rmse = float(np.sqrt((err**2).mean()))
        room_rmses[r] = rmse
        if r == "kitchen":
            metrics["kit_rmse"] = rmse
            metrics["kit_bias"] = float(err.mean())
    metrics["rms_all"] = float(np.mean(list(room_rmses.values())))

    # Same recorded-energy definition as replay_day.report: outdoor+indoor
    # unit power, each clipped at 0.
    sim_kwh = sim["p_kw"].sum() / 60.0
    rec_w = day["power.outdoor_unit"].clip(lower=0) + day["power.indoor_unit"].clip(lower=0)
    rec_kwh = rec_w.sum() / 60.0 / 1000.0
    metrics["energy_pct"] = float((sim_kwh - rec_kwh) / rec_kwh)

    on_sim = sim["p_kw"] > 0
    on_rec = day["power.outdoor_unit"] > 300
    hi_sim = sim["increment"] > 0
    hi_rec = day["power.outdoor_unit"] > 800

    metrics["on_frac_sim"] = float(on_sim.mean())
    metrics["on_frac_rec"] = float(on_rec.mean())
    metrics["abovemin_sim"] = float(hi_sim.mean())
    metrics["abovemin_rec"] = float(hi_rec.mean())

    starts_sim, on_min_sim, off_min_sim = _cycle_stats(on_sim)
    starts_rec, on_min_rec, off_min_rec = _cycle_stats(on_rec)
    metrics["starts_sim"] = starts_sim
    metrics["starts_rec"] = starts_rec
    metrics["on_min_sim"] = on_min_sim
    metrics["on_min_rec"] = on_min_rec
    metrics["off_min_sim"] = off_min_sim
    metrics["off_min_rec"] = off_min_rec
    return metrics


def _local_dates(parquet: Path) -> list[str]:
    """Every local calendar date with at least one row in the archive."""
    df = pd.read_parquet(parquet, columns=[])
    if df.index.tz is None:
        df.index = df.index.tz_localize("UTC")
    dates = sorted(set(df.index.tz_convert(LOCAL_TZ).date))
    return [d.isoformat() for d in dates]


def _row_line(date: str, m: dict) -> str:
    fields = " ".join(
        f"{m[c]:+.2f}" if c in ("kit_bias", "energy_pct")
        else f"{m[c]:.2f}" if isinstance(m[c], float)
        else f"{m[c]:d}"
        for c in COLUMNS
    )
    return f"{date}  {fields}"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--parquet", default="data/processed/june.parquet", type=Path)
    ap.add_argument("--out", type=Path, help="write per-day scorecard CSV")
    ap.add_argument(
        "--days",
        help="comma-separated local dates (e.g. 2026-06-22,2026-06-27); "
        "default is every date present in the archive",
    )
    args = ap.parse_args()

    dates = (
        [d.strip() for d in args.days.split(",") if d.strip()]
        if args.days
        else _local_dates(args.parquet)
    )

    print(f"{'date':<12} " + " ".join(f"{c:>11}" for c in COLUMNS))
    sys.stdout.flush()

    rows: dict[str, dict] = {}
    for date in dates:
        try:
            day = load_day(args.parquet, date)
            sim = replay(day)
            metrics = score_day(day, sim)
            if not all(np.isfinite(v) for v in metrics.values() if isinstance(v, float)):
                raise ValueError("non-finite metric (archive gap not caught by load_day)")
        except (SystemExit, Exception) as exc:  # noqa: BLE001
            # load_day only checks row *count*; some archived days have full
            # row counts but mid-day column outages (e.g. setpoint/humidity
            # all-NaN for hours) that ffill can't repair and that crash or
            # corrupt the replay downstream -- treat those as incomplete
            # archive coverage too, same graceful skip.
            print(f"{date}  skipped: {exc}")
            sys.stdout.flush()
            continue
        rows[date] = metrics
        print(_row_line(date, metrics))
        sys.stdout.flush()

    if not rows:
        print("\nno complete days replayed")
        return

    table = pd.DataFrame.from_dict(rows, orient="index")
    table.index.name = "date"

    if args.out:
        table.to_csv(args.out)
        print(f"\nwrote {args.out}")

    print(f"\n{len(table)} day(s) scored")
    print("\nsummary (median over scored days):")
    for c in COLUMNS:
        print(f"  {c:<12} {table[c].median():+.3f}" if c in ("kit_bias", "energy_pct")
              else f"  {c:<12} {table[c].median():.3f}")
    print(f"  energy_pct (mean) {table['energy_pct'].mean():+.3f}")

    print("\nsim vs recorded (median):")
    for sim_col, rec_col, label in PAIRED:
        print(
            f"  {label:<10} sim {table[sim_col].median():.3f}"
            f"  rec {table[rec_col].median():.3f}"
        )


if __name__ == "__main__":
    main()
