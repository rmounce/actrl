"""Find the most oscillatory room/day in the June archive, to spot-check
whether the closed-loop simulator (docs/ideas.md #3) reproduces real
overshoot/cycling behaviour before using it for comfort/PID tuning
(Ryan, 2026-07-03) — whole-day RMSE/bias (docs/calibration.md "Whole-day
closed-loop replay") can hide oscillation even when it matches on average.

Metric: local extrema (peaks/troughs) in each room's raw
average_temperature trace per day, after light smoothing to remove sensor
noise, counted only if they clear a minimum prominence — a proxy for
"hunting" around the setpoint rather than a single monotonic
approach/settle.

Usage:
    uv run python analysis/oscillation_scan.py [--data-dir data] [--top 10]
"""
from __future__ import annotations

import argparse
import gzip
import io
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lag_fit import _load_series  # noqa: E402

ROOMS = ("bed_1", "bed_2", "bed_3", "kitchen", "study")
SMOOTH_WIN = "90s"
MIN_PROMINENCE_C = 0.15
LOCAL_TZ = "Australia/Adelaide"


def count_extrema(temp: pd.Series) -> int:
    """Local peaks/troughs in a smoothed series clearing MIN_PROMINENCE_C."""
    if len(temp) < 10:
        return 0
    smooth = temp.rolling(SMOOTH_WIN, min_periods=1).mean()
    v = smooth.values
    extrema_idx = []
    for i in range(1, len(v) - 1):
        if (v[i] > v[i - 1] and v[i] >= v[i + 1]) or (v[i] < v[i - 1] and v[i] <= v[i + 1]):
            extrema_idx.append(i)
    if len(extrema_idx) < 2:
        return 0
    count = 0
    last_kept = extrema_idx[0]
    for idx in extrema_idx[1:]:
        if abs(v[idx] - v[last_kept]) >= MIN_PROMINENCE_C:
            count += 1
            last_kept = idx
    return count


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default="data", type=Path)
    ap.add_argument("--top", default=10, type=int)
    args = ap.parse_args()

    rows = []
    for day_dir in sorted((args.data_dir / "raw").iterdir()):
        if not day_dir.is_dir() or not day_dir.name.startswith("2026-06"):
            continue
        for room in ROOMS:
            s = _load_series(day_dir, f"sensor__temperature__{room}_average_temperature.csv.gz")
            if s.empty:
                continue
            local = s.index.tz_convert(LOCAL_TZ)
            n = count_extrema(s)
            span = float(s.max() - s.min())
            rows.append(
                {"date": day_dir.name, "room": room, "extrema": n, "span_c": span, "n_samples": len(s)}
            )

    df = pd.DataFrame(rows).sort_values("extrema", ascending=False)
    print(df.head(args.top).to_string(index=False))


if __name__ == "__main__":
    main()
