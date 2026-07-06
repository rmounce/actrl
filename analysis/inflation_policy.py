"""Candidate min-airflow inflation policies (docs/tuning.md follow-up to the
divergent-target sweep: the room-gain question closed with "the lever is the
inflation loop's design", this sweeps that lever).

Production behaviour (actrl.min_airflow_inflation): while the
airflow-weighted sum of positive PID outputs is below min_sum, add +0.0001
to EVERY room's integral (rooms at full range skipped). Indiscriminate and
persistent -- satisfied zones get dragged back into airflow, and the writes
ratchet integrals (docs/calibration.md kitchen-onset root cause).

Candidates vary two orthogonal design axes:

- WHO inflates first (eligibility tiers, expanding on shortfall so the
  min-airflow guarantee is never weakened):
    equal   -- everyone below range (production's rule)
    calling -- zones with output > 0 first, satisfied zones only if the
               callers can't cover min_sum
    nodeep  -- everyone except deeply-satisfied zones (output < -0.5,
               i.e. a real P-term excursion, not integral wind-down --
               the negative clamp keeps i-driven sheds >= -0.1)
- WHETHER the top-up persists:
    stateful  -- adjust_integral like production (the ratchet)
    stateless -- bump this cycle's outputs only; integrals untouched.
               Production's negative-integral clamp reads post-inflation
               outputs, so stateless policies replicate that clamp on the
               raw outputs first (else satisfied zones' integrals wind
               down unboundedly and the zone can never rejoin).

Within a tier the fill is an equal output-space increment delta for every
room below range (bisection on the piecewise-linear airflow gain) -- the
continuous limit of production's 0.0001-step loop, so `equal`+stateful
reproduces production exactly (modulo step granularity) and rank-flapping
between near-tied zones can't happen inside a tier.

Arms:
    baseline     production (no override)
    calling      stateful,  tiers calling -> rest
    nodeep       stateful,  tiers (out > -0.5) -> rest
    free_equal   stateless, production eligibility
    free_calling stateless, tiers calling -> rest

Scored on the divergent-target scenario (analysis/divergent_targets.py:
sub-K hold from 09:00, bed_2 +1K / bed_3 -2K at 13:00, end 18:00; noise
0.012 K/15 s) over 3 days x 2 seeds. Extra metrics vs divergent_targets:
shed_min (running-min until bed_3 damper <=15% for 15 consecutive running
min), d3_tail/kit_tail (tail damper means), err_hold (mean |err| during
the sub-K hold phase 11:00-13:00).

Usage:
    uv run python analysis/inflation_policy.py [--dates 2026-06-21,2026-06-22,2026-06-27]
        [--seeds 622,1622] [--arms baseline,calling,nodeep,free_equal,free_calling]
        [--out analysis/out/inflation_policy.csv]
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parent.parent
for p in (str(_ROOT), str(_ROOT / "tests")):
    if p not in sys.path:
        sys.path.insert(0, p)

import sim.closed_loop as cl  # noqa: E402  (installs the hassapi stub)
import actrl  # noqa: E402  (must follow closed_loop, needs the stub)
from analysis.ctrl_overrides import ctrl_overrides  # noqa: E402
from analysis.divergent_targets import (  # noqa: E402
    DOWN_ROOM,
    NOISE,
    analyse,
    synth_day,
)
from analysis.replay_day import feels_like_offsets, load_day, replay  # noqa: E402

DEEP_THRESHOLD = -0.5


# ---------------------------------------------------------------- fill core

def _gain(outputs, w, rooms, delta, range_):
    """Airflow gained if every room in `rooms` bumps by `delta` (range-capped)."""
    total = 0.0
    for r in rooms:
        o = outputs[r]
        total += (min(max(o + delta, 0.0), range_) - max(o, 0.0)) * w[r]
    return total


def _tier_fill(outputs, w, rooms, short, range_):
    """Equal output-space increment for `rooms` covering `short` airflow.

    Returns ({room: bump}, airflow_still_short). Bisection on the monotone
    piecewise-linear gain function.
    """
    rooms = [r for r in rooms if outputs[r] < range_]
    if not rooms or short <= 0:
        return {}, short
    hi = max(range_ - outputs[r] for r in rooms)
    max_gain = _gain(outputs, w, rooms, hi, range_)
    if max_gain <= short:
        delta = hi
    else:
        lo = 0.0
        for _ in range(80):
            mid = 0.5 * (lo + hi)
            if _gain(outputs, w, rooms, mid, range_) < short:
                lo = mid
            else:
                hi = mid
        delta = hi
    bumps = {r: min(delta, range_ - outputs[r]) for r in rooms}
    return bumps, short - _gain(outputs, w, rooms, delta, range_)


def _positive_sum(outputs, w):
    return sum(max(0.0, o) * w[r] for r, o in outputs.items())


def _replicate_negative_clamp(pids, pid_outputs):
    """Production's pass-4 clamp, applied to raw outputs (stateless policies
    only -- see module docstring)."""
    for room, out in pid_outputs.items():
        diff = out - actrl.room_pid_minimum
        if diff < 0 and pids[room].i_term < 0:
            if pids[room].i_term < diff:
                pids[room].adjust_integral(-diff)
            else:
                pids[room].set_integral(0)
            pid_outputs[room] = pids[room].get_output()


def make_policy(tiers_fn, stateful: bool):
    """Build a drop-in replacement for actrl.min_airflow_inflation."""

    def policy(pids, pid_outputs, adjusted_room_airflow, min_sum):
        if not stateful:
            _replicate_negative_clamp(pids, pid_outputs)
        range_ = actrl.normalised_damper_range
        short = min_sum - _positive_sum(pid_outputs, adjusted_room_airflow)
        for tier in tiers_fn(pid_outputs):
            if short <= 1e-12:
                break
            bumps, short = _tier_fill(
                pid_outputs, adjusted_room_airflow, tier, short, range_
            )
            for room, bump in bumps.items():
                if stateful:
                    pids[room].adjust_integral(bump)
                    pid_outputs[room] = pids[room].get_output()
                else:
                    pid_outputs[room] = pid_outputs[room] + bump

    return policy


def _tiers_equal(outputs):
    return [list(outputs)]


def _tiers_calling(outputs):
    calling = [r for r, o in outputs.items() if o > 0]
    rest = [r for r in outputs if r not in calling]
    return [calling, rest]


def _tiers_nodeep(outputs):
    shallow = [r for r, o in outputs.items() if o > DEEP_THRESHOLD]
    rest = [r for r in outputs if r not in shallow]
    return [shallow, rest]


ARMS: dict[str, object] = {
    "baseline": None,
    "calling": make_policy(_tiers_calling, stateful=True),
    "nodeep": make_policy(_tiers_nodeep, stateful=True),
    "free_equal": make_policy(_tiers_equal, stateful=False),
    "free_calling": make_policy(_tiers_calling, stateful=False),
}


# ---------------------------------------------------------------- scoring

def analyse_extra(sim, day, lows, hold_hh, event_hh, end_hh):
    local = day.index.tz_convert("Australia/Adelaide")
    hh = (local.hour + local.minute / 60.0).to_numpy()
    offsets = feels_like_offsets(day)
    running = np.asarray(sim["p_kw"], float) > 0
    post = (hh >= event_hh) & (hh < end_hh)
    tail = (hh >= end_hh - 1) & (hh < end_hh)
    hold = (hh >= event_hh - 2) & (hh < event_hh)

    d3 = np.asarray(sim[f"damper_{DOWN_ROOM}"], float) * 100
    kit = np.asarray(sim["damper_kitchen"], float) * 100

    run_idx = np.flatnonzero(running[post])
    d3_post = d3[post]
    shed = float("nan")
    streak = 0
    for j, i in enumerate(run_idx):
        streak = streak + 1 if d3_post[i] <= 15 else 0
        if streak >= 15:
            shed = float(j - 14)
            break

    errs = {r: lows[r] - (np.asarray(sim[f"Tm_{r}"], float) + offsets[r])
            for r in cl.ROOMS}

    # Early post-event window: sub-K down-steps get erased by natural
    # evening cooling well before the tail, so the shed must be measured
    # right after the event.
    early = (hh >= event_hh) & (hh < event_hh + 1.5)
    pre = (hh >= event_hh - 2) & (hh < event_hh)
    run_e = np.flatnonzero(running[early])
    d3_e = d3[early]
    shed30 = float("nan")
    streak = 0
    for j, i in enumerate(run_e):
        streak = streak + 1 if d3_e[i] <= 30 else 0
        if streak >= 10:
            shed30 = float(j - 9)
            break

    return {
        "shed_min": shed,
        "shed30_min": shed30,
        "d3_early": float(np.mean(d3[early][running[early]]))
        if running[early].any() else float("nan"),
        "d3_pre": float(np.mean(d3[pre][running[pre]]))
        if running[pre].any() else float("nan"),
        "d3_tail": float(np.mean(d3[tail])),
        "kit_tail": float(np.mean(kit[tail])),
        "err_hold": float(np.mean([np.mean(np.abs(errs[r][hold])) for r in cl.ROOMS])),
    }


COLS = ["serve_min", "shed_min", "shed30_min", "d3_pre", "d3_early", "d2_shift",
        "d3_shift", "d3_tail", "kit_tail", "pre_mv_h", "post_mv_h", "err_hold",
        "err_all", "kwh", "run_frac"]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dates", default="2026-06-21,2026-06-22,2026-06-27")
    ap.add_argument("--seeds", default="622,1622")
    ap.add_argument("--arms", default=",".join(ARMS))
    ap.add_argument("--hold-hh", type=float, default=9.0)
    ap.add_argument("--event-hh", type=float, default=13.0)
    ap.add_argument("--end-hh", type=float, default=18.0)
    ap.add_argument("--lift", type=float, default=0.4)
    ap.add_argument("--up-k", type=float, default=1.0)
    ap.add_argument("--down-k", type=float, default=2.0)
    ap.add_argument("--out", default=_ROOT / "analysis/out/inflation_policy.csv",
                    type=Path)
    ap.add_argument("--parquet", default=_ROOT / "data/processed/june.parquet",
                    type=Path)
    args = ap.parse_args()

    dates = args.dates.split(",")
    seeds = [int(s) for s in args.seeds.split(",")]
    arms = {a: ARMS[a] for a in args.arms.split(",")}

    args.out.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    hdr = f"{'arm':>13}{'date':>12}{'seed':>6}" + "".join(f"{c:>10}" for c in COLS)
    print(hdr, flush=True)
    for date in dates:
        raw = load_day(args.parquet, date)
        day, lows = synth_day(raw, args.hold_hh, args.event_hh, args.end_hh,
                              args.lift, args.up_k, args.down_k)
        for seed in seeds:
            noise = dict(NOISE, seed=seed)
            for name, policy in arms.items():
                overrides = {} if policy is None else {
                    "actrl.min_airflow_inflation": policy}
                with ctrl_overrides(overrides):
                    sim = replay(day, ctrl_noise=noise)
                m = analyse(sim, day, lows, args.hold_hh, args.event_hh,
                            args.end_hh)
                m.update(analyse_extra(sim, day, lows, args.hold_hh,
                                       args.event_hh, args.end_hh))
                rows.append({"arm": name, "date": date, "seed": seed, **m})
                print(f"{name:>13}{date:>12}{seed:>6}"
                      + "".join(f"{m[c]:>10.2f}" for c in COLS), flush=True)

    with open(args.out, "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=["arm", "date", "seed"] + COLS)
        wr.writeheader()
        wr.writerows(rows)
    print(f"\nwrote {args.out}")

    print("\nmedians per arm:")
    print(f"{'arm':>13}" + "".join(f"{c:>10}" for c in COLS))
    for name in arms:
        sub = [r for r in rows if r["arm"] == name]
        meds = {c: float(np.nanmedian([r[c] for r in sub])) for c in COLS}
        print(f"{name:>13}" + "".join(f"{meds[c]:>10.2f}" for c in COLS))


if __name__ == "__main__":
    main()
