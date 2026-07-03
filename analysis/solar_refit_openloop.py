"""Refit per-room solar gains in simulation-trajectory space.

The original analysis/solar_fit.py regresses each room's model residual
against PV using *recorded* neighbour temperatures in the coupling term.
That is inconsistent with how the terms act in replay: there the
neighbours are *simulated*, and their own (weak) solar terms leave them
cold, so rooms that lean on coupling (kitchen: tau_cpl 2.9 h) never
receive the inflow the fit assumed. The per-room fit even goes negative
for the kitchen — "sun makes it colder" — because recorded sunlit
bedrooms inflate its predicted coupling inflow. Result: replayed mild
days read cold everywhere and the sim heats a house that was coasting
(docs/tasks/010 scorecard: mild-day energy up to +86%).

This script fits the five solar coefficients jointly and consistently:
simulate the whole House open-loop (q=0, unit off) over every unit-off
window, compare simulated vs recorded 30-min temperature deltas, regress
the per-room error on (filtered) PV through the origin, update the
coefficients, and iterate to a fixed point. The window heads are trimmed
so the fast measured-air node has settled (Tm == T with q=0 after ~3
tau) and the night calibration is untouched (PV=0 -> no change).

Usage:
    uv run python analysis/solar_refit_openloop.py \
        [--parquet data/processed/june.parquet] [--iters 4]
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pandas as pd

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from sim.house import House, HouseParams, ROOMS  # noqa: E402

OFF_W = 50
MIN_WINDOW_MIN = 120
TRIM_HEAD_MIN = 45
STEP_MIN = 30  # horizon for the delta comparison
DT_S = 60.0


def off_windows(df: pd.DataFrame) -> list[tuple[pd.Timestamp, pd.Timestamp]]:
    off = df["power.outdoor_unit"].fillna(np.inf) < OFF_W
    grp = (off != off.shift()).cumsum()
    head = pd.Timedelta(minutes=TRIM_HEAD_MIN)
    out = []
    for _, seg in df.index.to_series().groupby(grp):
        if not off.loc[seg.iloc[0]]:
            continue
        w0, w1 = seg.iloc[0] + head, seg.iloc[-1]
        if (w1 - w0) >= pd.Timedelta(minutes=MIN_WINDOW_MIN):
            out.append((w0, w1))
    return out


def simulate_window(params: HouseParams, day: pd.DataFrame,
                    w0: pd.Timestamp, w1: pd.Timestamp) -> pd.DataFrame:
    """Open-loop (q=0) simulation of the window, re-anchored to recorded
    temps every STEP_MIN so errors don't compound across hours — each
    30-min segment contributes an independent delta sample."""
    win = day.loc[w0:w1]
    tout = win["temperature_adelaide"].ffill()
    pv = (win["power_pv_5m"].fillna(0) / 1000.0).clip(lower=0)
    rows = []
    seg_starts = win.index[::STEP_MIN]
    for s0 in seg_starts:
        s1 = s0 + pd.Timedelta(minutes=STEP_MIN)
        if s1 > w1:
            break
        try:
            temps0 = {r: float(win.loc[s0, f"{r}_average_temperature"]) for r in ROOMS}
            temps1 = {r: float(win.loc[s1, f"{r}_average_temperature"]) for r in ROOMS}
        except KeyError:
            continue
        if any(np.isnan(v) for v in list(temps0.values()) + list(temps1.values())):
            continue
        house = House(params, temps0)
        seg = win.loc[s0:s1]
        for ts in seg.index[1:]:
            house.step(float(tout.loc[ts]), {r: 0.0 for r in ROOMS},
                       dt_s=DT_S, pv_kw=float(pv.loc[ts]))
        pv_mean = float(pv.loc[s0:s1].mean())
        rows.append({
            "pv": pv_mean,
            **{f"err_{r}": temps1[r] - house.temps[r] for r in ROOMS},
        })
    return pd.DataFrame(rows)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--parquet", default="data/processed/june.parquet", type=Path)
    ap.add_argument("--iters", default=4, type=int)
    args = ap.parse_args()
    df = pd.read_parquet(args.parquet)
    if df.index.tz is None:
        df.index = df.index.tz_localize("UTC")
    windows = off_windows(df)
    print(f"{len(windows)} unit-off windows (head-trimmed {TRIM_HEAD_MIN} min)")

    params = HouseParams()
    solar = {r: params.rooms[r].solar for r in ROOMS}
    for it in range(args.iters):
        rooms = {r: replace(params.rooms[r], solar=solar[r]) for r in ROOMS}
        p = HouseParams(rooms=rooms)
        samples = pd.concat(
            [simulate_window(p, df, w0, w1) for w0, w1 in windows],
            ignore_index=True,
        )
        line = [f"iter {it}: n={len(samples)}"]
        for r in ROOMS:
            x = samples["pv"].to_numpy()
            # err is the 30-min recorded-minus-simulated delta [K]; convert
            # the through-origin slope to K/h per kW.
            y = samples[f"err_{r}"].to_numpy() / (STEP_MIN / 60.0)
            delta = float((x * y).sum() / (x * x).sum())
            solar[r] = max(0.0, solar[r] + delta)
            line.append(f"{r}={solar[r]:.3f}(d{delta:+.3f})")
        print("  ".join(line))

    print("\nfitted solar terms (K/h per kW PV):")
    for r in ROOMS:
        print(f"  {r:8s} {solar[r]:.3f}")


if __name__ == "__main__":
    main()
