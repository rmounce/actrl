"""Fit the per-room measured-air lead node (docs/tasks/009, sim/house.py
tau_meas_h/lead_h) from recorded unit-stop transients.

The room sensors read a small fast air mass, not the bulk RC state: after
the compressor cuts, measured temperature sags well above the RC decay
rate for ~10-20 min before relaxing onto it (docs/calibration.md). Model:

    Tm = T_bulk + lead_h * q_room   (steady, while heating)
    after stop: (Tm - T_bulk) decays exp(-t / tau_meas_h)

Method, per room:
  1. Find clean stop events: unit ran >= MIN_RUN_MIN continuously, then
     off >= MIN_OFF_MIN, with this room's damper open >= MIN_DAMPER
     (mean over the run's last 20 min) so the room was actually fed.
  2. Superpose the measured temperature for STOP_WIN after each stop and
     subtract the slow trend (linear fit to the 30-45 min tail, where the
     fast node has equilibrated) -> the excess sag curve.
  3. Fit A * (exp(-t/tau) - tail) by grid search on tau, A by lstsq.
  4. lead_h = A / q_room_pre: pre-stop q_room [K/h] estimated from
     recorded electrical power -> Hvac COP at recorded outdoor temp ->
     the closed-loop damper/mass split (sim.closed_loop._room_q formula
     with recorded damper positions).

Usage:
    uv run python analysis/fast_node_fit.py [--parquet data/processed/june.parquet]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from sim.closed_loop import AIRFLOW_WEIGHTS, MASS_WEIGHTS  # noqa: E402
from sim.hvac import Hvac  # noqa: E402

ROOMS = ["bed_1", "bed_2", "bed_3", "kitchen", "study"]
ON_W = 300.0
MIN_RUN_MIN = 30
MIN_OFF_MIN = 45
MIN_DAMPER = 40.0
STOP_WIN_MIN = 45
TAIL_MIN = (30, 45)  # slow-trend estimation window


def stop_events(df: pd.DataFrame) -> list[tuple[pd.Timestamp, pd.Timestamp]]:
    """(run_start, stop_time) for qualifying runs."""
    on = df["power.outdoor_unit"].fillna(0) > ON_W
    grp = (on != on.shift()).cumsum()
    segs = [
        (s.index[0], s.index[-1], bool(on.loc[s.index[0]]))
        for _, s in df.index.to_series().groupby(grp)
    ]
    out = []
    for i in range(len(segs) - 1):
        t0, t1, is_on = segs[i]
        nt0, nt1, _ = segs[i + 1]
        if not is_on:
            continue
        if (t1 - t0) < pd.Timedelta(minutes=MIN_RUN_MIN):
            continue
        if (nt1 - nt0) < pd.Timedelta(minutes=MIN_OFF_MIN):
            continue
        out.append((t0, t1))
    return out


def room_q_pre(df: pd.DataFrame, room: str, t0: pd.Timestamp, stop: pd.Timestamp,
               hvac: Hvac) -> float:
    """Pre-stop per-room forcing [K/h]: recorded power -> COP -> damper split."""
    w = df.loc[stop - pd.Timedelta("20min"): stop]
    p_kw = w["power.outdoor_unit"].clip(lower=0).mean() / 1000.0
    t_out = w["temperature_adelaide"].mean()
    q_kw = hvac.cop(p_kw, t_out) * p_kw
    q_house = q_kw / hvac.params.c_eff_kwh_per_k  # K/h house-average
    dampers = {r: w[f"damper.{r}"].mean() / 100.0 for r in ROOMS}
    flows = {r: dampers[r] * AIRFLOW_WEIGHTS[r] for r in ROOMS}
    total_flow = sum(flows.values())
    if total_flow <= 0:
        return 0.0
    sum_mass = sum(MASS_WEIGHTS.values())
    return q_house * (flows[room] / total_flow) / (MASS_WEIGHTS[room] / sum_mass)


def fit_room(df: pd.DataFrame, room: str, hvac: Hvac) -> dict | None:
    temp = df[f"{room}_average_temperature"].astype(float)
    damper = df[f"damper.{room}"].astype(float)
    curves, qs = [], []
    pv = df["power_pv_5m"].fillna(0)
    for t0, stop in stop_events(df):
        if damper.loc[stop - pd.Timedelta("20min"): stop].mean() < MIN_DAMPER:
            continue
        # exclude sunlit windows: solar gain contaminates the sag curve
        # (bed_3's raw fit comes out with the wrong sign otherwise)
        if pv.loc[stop: stop + pd.Timedelta(minutes=STOP_WIN_MIN)].max() > 100.0:
            continue
        w = temp.loc[stop: stop + pd.Timedelta(minutes=STOP_WIN_MIN)]
        if len(w) < STOP_WIN_MIN - 2 or w.isna().any():
            continue
        q_pre = room_q_pre(df, room, t0, stop, hvac)
        if not np.isfinite(q_pre) or q_pre <= 0.2:
            continue  # too little forcing to measure a sag against
        rel = ((w.index - stop).total_seconds() / 60).round().astype(int)
        curves.append(pd.Series(w.values - w.values[0], index=rel))
        qs.append(q_pre)
    if len(curves) < 4:
        return None
    M = pd.concat(curves, axis=1)
    mean_c = M.mean(axis=1)
    t = mean_c.index.values.astype(float)  # minutes

    # slow trend from the tail (fast node equilibrated)
    tail = (t >= TAIL_MIN[0]) & (t <= TAIL_MIN[1])
    slope, icpt = np.polyfit(t[tail], mean_c.values[tail], 1)

    # excess = A*exp(-t/tau); at t in tail it's ~0 by construction of the
    # trend, so fit A,tau on the full window against (curve - trend).
    excess = mean_c.values - (slope * t + icpt)
    best = None
    for tau_min in np.arange(4, 40, 0.5):
        basis = np.exp(-t / tau_min)
        a = float(np.linalg.lstsq(basis[:, None], excess, rcond=None)[0][0])
        r = excess - a * basis
        sse = float((r**2).sum())
        if best is None or sse < best[2]:
            best = (a, tau_min, sse)
    a, tau_min, sse = best
    ss_tot = float((excess - excess.mean())**2.0).__abs__() if False else float(((excess - excess.mean())**2).sum())
    r2 = 1 - sse / ss_tot if ss_tot > 0 else float("nan")
    q_med = float(np.median(qs))
    # A = excess above the slow trend at the stop = the steady lead offset
    # that then decays away: lead_h * q_pre.
    lead_h = a / q_med
    return {
        "room": room,
        "n_events": len(curves),
        "tau_meas_min": tau_min,
        "sag_amp_c": round(a, 3),
        "q_pre_med": round(q_med, 2),
        "lead_h": round(lead_h, 3),
        "r2": round(r2, 3),
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--parquet", default="data/processed/june.parquet", type=Path)
    args = ap.parse_args()
    df = pd.read_parquet(args.parquet)
    if df.index.tz is None:
        df.index = df.index.tz_localize("UTC")
    df["temperature_adelaide"] = df["temperature_adelaide"].ffill()
    hvac = Hvac()
    rows = []
    for room in ROOMS:
        r = fit_room(df, room, hvac)
        if r is None:
            print(f"{room}: too few clean events")
            continue
        rows.append(r)
    print(pd.DataFrame(rows).to_string(index=False))


if __name__ == "__main__":
    main()
