"""Find recorded periods where the compressor runs continuously above
minimum power and modulates (e.g. 2-3-4-3-2-3-2-3-2), as opposed to
cycling fully on/off at minimum power — the two look similar as "many
extrema" in a naive oscillation scan but are different phenomena (Ryan,
2026-07-03): on/off cycling at minimum power mostly reflects heat-loss/
heat-rate modelling mismatch, while above-minimum modulation is where PID
hunting/overshoot is most relevant to a tuning study.

A "modulation run" is a maximal period where the outdoor unit stays above
the running threshold continuously (no drop to off), lasting at least
MIN_RUN_MIN, with mean comp_speed >= MIN_MEAN_SPEED (i.e. genuinely above
minimum, not just idling at/near 0) and multiple direction reversals in
comp_speed (hunting, not a single ramp-and-hold).

Usage:
    uv run python analysis/modulation_scan.py [--parquet data/processed/june.parquet]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lag_fit import _load_series  # noqa: E402

RUNNING_W = 300.0
MIN_RUN_MIN = 30
MIN_MEAN_SPEED = 2.0
GRID = "30s"  # 1-min parquet averaging smooths out fine oscillation


def reversals(speed: np.ndarray) -> int:
    """Count direction reversals (local extrema) in a speed sequence."""
    diffs = np.diff(speed)
    signs = np.sign(diffs)
    signs = signs[signs != 0]
    if len(signs) < 2:
        return 0
    return int((np.diff(signs) != 0).sum())


def load_raw_grid(data_dir: Path) -> pd.DataFrame:
    """comp_speed + outdoor power from raw per-day CSVs, resampled to a
    common fine grid (ffill) so brief modulation isn't averaged away."""
    speeds, powers = [], []
    for day_dir in sorted((data_dir / "raw").iterdir()):
        if not day_dir.is_dir() or not day_dir.name.startswith("2026-06"):
            continue
        speeds.append(_load_series(day_dir, "input_number__aircon_comp_speed.csv.gz"))
        powers.append(
            _load_series(day_dir, "sensor__power__shellyem_ec64c9c6932b_channel_1_power.csv.gz")
        )
    speed = pd.concat(speeds).sort_index()
    power = pd.concat(powers).sort_index()
    speed = speed[~speed.index.duplicated()].resample(GRID).ffill()
    power = power[~power.index.duplicated()].resample(GRID).ffill()
    return pd.DataFrame({"speed": speed, "power": power}).dropna()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default="data", type=Path)
    ap.add_argument("--top", default=15, type=int)
    args = ap.parse_args()

    df = load_raw_grid(args.data_dir)
    running = df["power"] > RUNNING_W
    grp = (running != running.shift()).cumsum()
    steps_per_min = 60 / pd.Timedelta(GRID).total_seconds()

    rows = []
    for _, seg in df.index.to_series().groupby(grp):
        if not running.loc[seg.iloc[0]]:
            continue
        dur_min = len(seg) / steps_per_min
        if dur_min < MIN_RUN_MIN:
            continue
        speed = df.loc[seg.index, "speed"].to_numpy()
        mean_speed = float(np.nanmean(speed))
        if mean_speed < MIN_MEAN_SPEED:
            continue
        rev = reversals(speed)
        rows.append(
            {
                "start": seg.index[0],
                "end": seg.index[-1],
                "dur_min": round(dur_min),
                "mean_speed": round(mean_speed, 1),
                "min_speed": int(np.nanmin(speed)),
                "max_speed": int(np.nanmax(speed)),
                "reversals": rev,
                "reversals_per_hour": round(rev / (dur_min / 60), 1),
            }
        )

    out = pd.DataFrame(rows).sort_values("reversals_per_hour", ascending=False)
    pd.set_option("display.width", 140)
    print(out.head(args.top).to_string(index=False))


if __name__ == "__main__":
    main()
