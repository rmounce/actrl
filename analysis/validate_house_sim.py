#!/usr/bin/env python3
"""Validate sim/house.py by replaying recorded free-running nights.

Reuses the night-window detection from analysis/sysid_june.py (imported,
not modified) to find free-running (outdoor unit off, no-solar) windows in
the real June archive. For each window: initialise the model from the
recorded room temperatures at the window start, drive it minute-by-minute
with the recorded outdoor temperature (temperature_adelaide) and q=0 (no
heat input -- these windows are free-running by construction), and compare
the simulated trajectory against the recorded one.

Run from the repo root (regenerate the parquet first if needed):

    uv run python calib.py --data-dir /home/saltspork/actrl/data \
        --start 2026-06-01 --end 2026-06-30 --out data/processed/june.parquet
    uv run python analysis/validate_house_sim.py [data/processed/june.parquet]

Dev-only analysis tooling (pandas); never imported by sim/ or the deployed
apps.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd

# Allow running as `python analysis/validate_house_sim.py` (sys.path[0] is
# then analysis/, not the repo root) as well as `python -m
# analysis.validate_house_sim` / pytest-style invocation.
_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from analysis.sysid_june import night_windows  # noqa: E402
from sim.house import ROOMS, House, HouseParams  # noqa: E402

STEP_MIN = 1  # data is on a 1-min grid (see docs/data.md)
FLAG_RMSE_C = 0.5


def replay_window(df: pd.DataFrame, w0: pd.Timestamp, w1: pd.Timestamp) -> dict | None:
    """Simulate one free-running window from recorded data; return per-room
    and house-average RMSE (°C), or None if the window has unusable gaps."""
    cols = {r: f"{r}_average_temperature" for r in ROOMS}
    seg = df.loc[w0:w1, list(cols.values()) + ["temperature_adelaide"]].copy()
    seg = seg.resample(f"{STEP_MIN}min").mean()
    seg = seg.dropna()
    if len(seg) < 4:
        return None

    initial = {room: float(seg.iloc[0][cols[room]]) for room in ROOMS}
    house = House(HouseParams(), initial, dt_s=STEP_MIN * 60)

    actual = {room: [initial[room]] for room in ROOMS}
    predicted = {room: [initial[room]] for room in ROOMS}

    for _, row in seg.iloc[1:].iterrows():
        t_out = float(row["temperature_adelaide"])
        house.step(t_out=t_out, q=None, dt_s=STEP_MIN * 60)
        for room in ROOMS:
            predicted[room].append(house.temps[room])
            actual[room].append(float(row[cols[room]]))

    per_room_rmse = {}
    for room in ROOMS:
        a = np.array(actual[room])
        p = np.array(predicted[room])
        per_room_rmse[room] = float(np.sqrt(np.mean((a - p) ** 2)))

    actual_avg = np.mean([actual[r] for r in ROOMS], axis=0)
    predicted_avg = np.mean([predicted[r] for r in ROOMS], axis=0)
    house_rmse = float(np.sqrt(np.mean((actual_avg - predicted_avg) ** 2)))

    return {"n": len(seg), "per_room": per_room_rmse, "house_avg": house_rmse}


def main() -> int:
    path = sys.argv[1] if len(sys.argv) > 1 else "data/processed/june.parquet"
    df = pd.read_parquet(path)
    windows = night_windows(df)
    print(f"{len(windows)} free-running night windows\n")

    header = f"{'window start':20s} {'window end':20s} {'n':>5s} " + " ".join(
        f"{r:>8s}" for r in ROOMS
    ) + f" {'house':>8s}"
    print(header)
    print("-" * len(header))

    results = []
    for w0, w1 in windows:
        r = replay_window(df, w0, w1)
        if r is None:
            print(f"{str(w0):20s} {str(w1):20s}   -- (skipped, insufficient data)")
            continue
        results.append(r)
        row = f"{str(w0):20s} {str(w1):20s} {r['n']:5d} " + " ".join(
            f"{r['per_room'][room]:8.3f}" for room in ROOMS
        ) + f" {r['house_avg']:8.3f}"
        print(row)

    print("-" * len(header))
    if results:
        medians = {
            room: float(np.median([r["per_room"][room] for r in results])) for room in ROOMS
        }
        house_median = float(np.median([r["house_avg"] for r in results]))
        median_row = f"{'median':20s} {'':20s} {'':5s} " + " ".join(
            f"{medians[room]:8.3f}" for room in ROOMS
        ) + f" {house_median:8.3f}"
        print(median_row)
        print(f"\n{len(results)} windows replayed.")
        if house_median > FLAG_RMSE_C:
            print(
                f"FLAG: house-average median RMSE {house_median:.3f} C exceeds "
                f"{FLAG_RMSE_C} C -- reviewer to judge acceptability."
            )
        else:
            print(f"house-average median RMSE {house_median:.3f} C (<= {FLAG_RMSE_C} C).")
    else:
        print("No usable windows.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
