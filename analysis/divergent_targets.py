"""Synthetic divergent-target scenario -- the summer-override proxy.

The winter schedule never exercises sub-K multi-zone contention (one
dominant zone at a time; docs/calibration.md rebalance probes), but that
is exactly the regime Ryan's summer per-zone overrides live in and where
the detuned room gains are suspected of sluggish rebalancing. This
driver manufactures the regime on a winter replay day:

- From --hold-hh (default 09:00): every zone's heat target is pinned to
  (its own recorded temp at that minute + --lift, default 0.4 K), highs
  at low+4 -- all five zones mildly calling, the unit trickling, errors
  sub-K. Physics, weather, feels-like offsets stay the recorded day's.
- At --event-hh (default 13:00): bed_2's target steps +1.0 K (an
  occupant wants it warmer) and bed_3's steps -1.0 K (satisfied /
  overridden cooler). Held to --end-hh (default 18:00).

Per gain arm (room_kp x room_ki multipliers, measured noise 0.012 K/15 s
always on -- docs/calibration.md sweep standard config):

- serve_min: running-minutes until bed_2's controller-space error stays
  below 0.2 K for 15 consecutive running minutes (time for the "wants
  more" zone to actually get served)
- d2_shift / d3_shift: bed_2 / bed_3 damper mean in the settled tail
  (last hour before end) minus their pre-event (11:00-13:00) means
- pre_mv_h / post_mv_h: damper moves >5 pts per running hour, mean over
  the five zones (hunting proxy; recorded winter envelope is ~0.4-0.5
  on active zones)
- err_all: mean over zones of mean |controller error| in the settled
  tail (does faster rebalancing cost tracking elsewhere)
- kwh: scenario-window energy

Usage:
    uv run python analysis/divergent_targets.py [--date 2026-06-22]
        [--gains 1x1,1x2,1x4,2x4,1x8]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from unittest import mock

import numpy as np

_ROOT = Path(__file__).resolve().parent.parent
for p in (str(_ROOT), str(_ROOT / "tests")):
    if p not in sys.path:
        sys.path.insert(0, p)

import sim.closed_loop as cl  # noqa: E402  (installs the hassapi stub)
import actrl  # noqa: E402  (must follow closed_loop, needs the stub)
from analysis.ctrl_overrides import ctrl_overrides  # noqa: E402
from analysis.replay_day import feels_like_offsets, load_day, replay  # noqa: E402

NOISE = {"sigma": 0.012, "tau_s": 15.0, "seed": 622}
UP_ROOM, DOWN_ROOM = "bed_2", "bed_3"
BAND = 4.0  # heat_cool high = low + BAND (stays out of cooling)


def synth_day(day, hold_hh: float, event_hh: float, end_hh: float, lift: float,
              up_k: float = 1.0, down_k: float = 2.0):
    """Recorded day with the target columns rewritten to the scenario."""
    day = day.copy()
    local = day.index.tz_convert("Australia/Adelaide")
    hh = local.hour + local.minute / 60.0
    hold_i = int(np.argmax(hh >= hold_hh))
    lows = {}
    for r in cl.ROOMS:
        rec_low = day[f"climate.{r}_aircon.target_temp_low"].ffill().to_numpy(float)
        base = float(day[f"{r}_average_temperature"].astype(float).ffill().iloc[hold_i]) + lift
        low = rec_low.copy()
        in_hold = (hh >= hold_hh) & (hh < end_hh)
        low[in_hold] = base
        if r == UP_ROOM:
            low[(hh >= event_hh) & (hh < end_hh)] = base + up_k
        if r == DOWN_ROOM:
            # big enough that natural evening cooling cannot re-create
            # demand inside the window -- isolates the SHED response
            low[(hh >= event_hh) & (hh < end_hh)] = base - down_k
        day[f"climate.{r}_aircon.target_temp_low"] = low
        day[f"climate.{r}_aircon.target_temp_high"] = low + BAND
        lows[r] = low
    return day, lows


def run_arm(day, kp_mult: float, ki_mult: float):
    overrides = {"actrl.room_kp": actrl.room_kp * kp_mult,
                 "actrl.room_ki": actrl.room_ki * ki_mult}
    with ctrl_overrides(overrides):
        return replay(day, ctrl_noise=dict(NOISE))


def _mv_h(dmp: np.ndarray, running: np.ndarray) -> float:
    idx = np.flatnonzero(running)
    if len(idx) < 30:
        return float("nan")
    return float((np.abs(np.diff(dmp[idx])) > 5).sum() / (len(idx) / 60.0))


def analyse(sim, day, lows, hold_hh, event_hh, end_hh):
    local = day.index.tz_convert("Australia/Adelaide")
    hh = (local.hour + local.minute / 60.0).to_numpy()
    offsets = feels_like_offsets(day)
    running = np.asarray(sim["p_kw"], float) > 0
    pre = (hh >= event_hh - 2) & (hh < event_hh)
    tail = (hh >= end_hh - 1) & (hh < end_hh)
    post = (hh >= event_hh) & (hh < end_hh)

    dampers = {r: np.asarray(sim[f"damper_{r}"], float) * 100 for r in cl.ROOMS}
    # controller-space error: target_low - (Tm + feels offset); >0 == calling
    errs = {r: lows[r] - (np.asarray(sim[f"Tm_{r}"], float) + offsets[r])
            for r in cl.ROOMS}

    up_err = errs[UP_ROOM][post]
    up_run = running[post]
    run_idx = np.flatnonzero(up_run)
    serve = float("nan")
    streak = 0
    for j, i in enumerate(run_idx):
        streak = streak + 1 if up_err[i] < 0.2 else 0
        if streak >= 15:
            serve = float(j - 14)
            break

    return {
        "serve_min": serve,
        "d2_shift": float(np.mean(dampers[UP_ROOM][tail]) - np.mean(dampers[UP_ROOM][pre])),
        "d3_shift": float(np.mean(dampers[DOWN_ROOM][tail]) - np.mean(dampers[DOWN_ROOM][pre])),
        "pre_mv_h": float(np.nanmean([_mv_h(dampers[r][pre], running[pre]) for r in cl.ROOMS])),
        "post_mv_h": float(np.nanmean([_mv_h(dampers[r][post], running[post]) for r in cl.ROOMS])),
        "err_all": float(np.mean([np.mean(np.abs(errs[r][tail])) for r in cl.ROOMS])),
        "kwh": float(np.asarray(sim["p_kw"], float)[(hh >= hold_hh) & (hh < end_hh)].sum() / 60.0),
        "run_frac": float(running[post].mean()),
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--date", default="2026-06-22")
    ap.add_argument("--gains", default="1x1,1x2,1x4,2x4,1x8")
    ap.add_argument("--hold-hh", type=float, default=9.0)
    ap.add_argument("--event-hh", type=float, default=13.0)
    ap.add_argument("--end-hh", type=float, default=18.0)
    ap.add_argument("--lift", type=float, default=0.4)
    ap.add_argument("--up-k", type=float, default=1.0)
    ap.add_argument("--down-k", type=float, default=2.0)
    ap.add_argument("--parquet", default=_ROOT / "data/processed/june.parquet", type=Path)
    args = ap.parse_args()

    raw = load_day(args.parquet, args.date)
    day, lows = synth_day(raw, args.hold_hh, args.event_hh, args.end_hh, args.lift,
                          args.up_k, args.down_k)
    print(f"{args.date}: all zones held at T({args.hold_hh:.0f}h)+{args.lift}K from "
          f"{args.hold_hh:.0f}h; {UP_ROOM} +{args.up_k}K / {DOWN_ROOM} -{args.down_k}K at "
          f"{args.event_hh:.0f}h; noise {NOISE['sigma']}K")
    print(f"{'gains':>7}{'serve_min':>10}{'d2_shift':>9}{'d3_shift':>9}"
          f"{'pre_mv':>7}{'post_mv':>8}{'err_all':>8}{'kwh':>7}{'run%':>6}")
    for g in args.gains.split(","):
        kp_m, ki_m = (float(x) for x in g.strip().split("x"))
        sim = run_arm(day, kp_m, ki_m)
        m = analyse(sim, day, lows, args.hold_hh, args.event_hh, args.end_hh)
        print(f"{g.strip():>7}{m['serve_min']:>10.0f}{m['d2_shift']:>+9.1f}"
              f"{m['d3_shift']:>+9.1f}{m['pre_mv_h']:>7.2f}{m['post_mv_h']:>8.2f}"
              f"{m['err_all']:>8.2f}{m['kwh']:>7.2f}{m['run_frac']:>6.2f}", flush=True)


if __name__ == "__main__":
    main()
