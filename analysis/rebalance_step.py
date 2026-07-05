"""Small-signal zone-rebalance response (room-PID layer, ideas follow-up).

The recorded morning setbacks answer the LARGE-step case: while the unit
runs, bed_1's damper closes ~10-12 min after a 3.3 K demand step (kp-
driven; the 27-56 min outliers are the unit cycling off mid-setback and
freezing the dampers — scratch analysis 2026-07-05). The regime Ryan
actually feels as sluggish is SMALL demand imbalances (sub-K), where
room_ki (0.001, detuned after real-world hunting) does the work — the
summer per-zone-override regime. No clean natural experiment exists in
the winter archive, so this is a sim probe:

From an injection minute onward, the kitchen's controller-read is biased
+bias K (it believes it is warmer -> should shed damper authority to the
other zones). Everything else (plant, scoring, noise) is untouched.
Reported per gain config: the kitchen's damper shift latency counted in
RUNNING minutes (dampers freeze when the unit is off — wall-clock latency
conflates run decisions with PID speed), the achieved shift, and the
damper move rate (hunting proxy) with measured sensor noise injected
(sigma 0.012 K / tau 15 s -- always inject for gain studies, the
noise-free sim never jitters a pinned PID, docs/calibration.md).

Usage:
    uv run python analysis/rebalance_step.py \
        [--date 2026-06-22] [--inject-hh 19.0] [--bias 0.5] \
        [--gains 1x1,1x2,1x4,2x4]   # kp_mult x ki_mult
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
from analysis.replay_day import load_day, replay  # noqa: E402

NOISE = {"sigma": 0.012, "tau_s": 15.0}
ROOM = "kitchen"


def make_biased_write(inject_cycle: int, bias: float):
    orig = cl.ClosedLoop._write_room_temps

    def _write(self):
        orig(self)
        if self.cycle >= inject_cycle:
            ent = f"sensor.{ROOM}_average_temperature"
            t = float(self.world.entities[ent]["state"]) + bias
            self.world.update(ent, {"state": f"{t:.2f}"})

    return _write


def run_arm(day, inject_cycle: int, bias: float, kp_mult: float, ki_mult: float):
    overrides = {"actrl.room_kp": actrl.room_kp * kp_mult,
                 "actrl.room_ki": actrl.room_ki * ki_mult}
    write = make_biased_write(inject_cycle, bias)
    with ctrl_overrides(overrides), \
            mock.patch.object(cl.ClosedLoop, "_write_room_temps", write):
        sim = replay(day, ctrl_noise={**NOISE, "seed": 1})
    return sim


def analyse(sim, inject_min: int, room: str) -> dict:
    dmp = np.asarray(sim[f"damper_{room}"], float) * 100.0
    running = np.asarray(sim["p_kw"], float) > 0
    pre = dmp[inject_min - 30:inject_min][running[inject_min - 30:inject_min]]
    pre_mean = float(np.mean(pre)) if len(pre) else float("nan")
    post = dmp[inject_min:]
    post_run = running[inject_min:]
    # settle level: mean over the last 60 running minutes of the day
    run_idx = np.flatnonzero(post_run)
    if len(run_idx) < 90:
        return {"pre": pre_mean, "settle": float("nan"),
                "latency_run_min": float("nan"), "mv_h": float("nan")}
    settle = float(np.mean(post[run_idx[-60:]]))
    half = pre_mean - 0.5 * (pre_mean - settle)
    # latency in RUNNING minutes to first sustained (10 running-min) crossing
    below = post[run_idx] < half
    lat = float("nan")
    streak = 0
    for j, b in enumerate(below):
        streak = streak + 1 if b else 0
        if streak >= 10:
            lat = float(j - 9)
            break
    moves = np.abs(np.diff(post[run_idx])) > 5
    mv_h = float(moves.sum() / (len(run_idx) / 60.0))
    return {"pre": pre_mean, "settle": settle, "latency_run_min": lat, "mv_h": mv_h}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--date", default="2026-06-22")
    ap.add_argument("--inject-hh", type=float, default=19.0)
    ap.add_argument("--bias", type=float, default=0.5)
    ap.add_argument("--gains", default="1x1,1x2,1x4,2x4",
                    help="comma list of KPxKI multipliers, e.g. 1x1,1x4")
    ap.add_argument("--parquet", default=_ROOT / "data/processed/june.parquet", type=Path)
    args = ap.parse_args()

    day = load_day(args.parquet, args.date)
    inject_min = int(args.inject_hh * 60)
    inject_cycle = inject_min * 6  # 10 s cycles

    print(f"{args.date}, +{args.bias} K on {ROOM} reads from "
          f"{int(args.inject_hh):02d}:{int(args.inject_hh % 1 * 60):02d} "
          f"(noise {NOISE['sigma']} K)")
    print(f"{'gains kpxki':>12}{'pre':>7}{'settle':>8}{'lat_run_min':>12}{'mv/h':>7}")
    for g in args.gains.split(","):
        kp_m, ki_m = (float(x) for x in g.strip().split("x"))
        sim = run_arm(day, inject_cycle, args.bias, kp_m, ki_m)
        r = analyse(sim, inject_min, ROOM)
        print(f"{g.strip():>12}{r['pre']:>7.1f}{r['settle']:>8.1f}"
              f"{r['latency_run_min']:>12.1f}{r['mv_h']:>7.2f}", flush=True)


if __name__ == "__main__":
    main()
