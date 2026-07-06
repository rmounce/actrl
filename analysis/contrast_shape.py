"""Damper contrast shaping -- convex output->damper mapping sweep.

The inflation-policy study's structural finding (docs/tuning.md): the
renorm bounds damper CONTRAST by error contrast, so a sub-K-satisfied
zone keeps a ~70-85% damper no matter what the min-airflow pass does.
This sweeps the remaining lever: damper_share(output) = (out/range)**gamma,
gamma > 1 widening damper contrast at the same output contrast. Pure
output-stage change -- PID state, renorm and clamp untouched; the
min-airflow constraint is evaluated on the shaped shares (physical), so
satisfied zones dropping shares makes the top-up work harder when it
binds.

Risk profile vs raising room gains: shaping does not feed back into PID
state, but d(share)/d(output) = gamma/range * (o/range)**(gamma-1) is
STEEPER near the top of the range, so noise on high-output zones is
amplified ~gamma-fold in damper space (dampers quantize to 5 pts with a
7.5-pt deadband downstream, which absorbs some). Sweep with measured
noise on; watch pre/post_mv_h vs the recorded winter envelope (~0.4-0.5
moves/h on active zones).

Scored on the sub-K divergent-target scenario (the regime that matters;
K-scale punches through regardless) + the controller-CI canonical days
(--ci; gamma is NOT winter-neutral by construction, unlike the inflation
study -- normal all-calling days have K-scale output contrast that
shaping will widen).

Usage:
    uv run python analysis/contrast_shape.py [--gammas 1.0,1.5,2.0,3.0]
        [--dates 2026-06-21,2026-06-27] [--seeds 622,1622]
    uv run python analysis/contrast_shape.py --ci --gammas 1.5,2.0
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
import actrl  # noqa: E402
from analysis.ctrl_overrides import ctrl_overrides  # noqa: E402
from analysis.divergent_targets import NOISE, analyse, synth_day  # noqa: E402
from analysis.inflation_policy import COLS, analyse_extra  # noqa: E402
from analysis.replay_day import load_day, replay  # noqa: E402


def make_share(gamma: float):
    """Convex damper_share; gamma=1.0 reproduces production exactly."""

    def share(output):
        base = max(0.0, output) / actrl.normalised_damper_range
        return base if gamma == 1.0 else base ** gamma

    return share


def overrides_for(gamma: float) -> dict:
    # Always override explicitly: production's damper_share_gamma is no
    # longer 1.0 (gamma 1.5 adopted 2026-07-06), so "no override" is NOT
    # the linear arm.
    return {"actrl.damper_share": make_share(gamma)}


def run_scenario(args) -> None:
    gammas = [float(g) for g in args.gammas.split(",")]
    dates = args.dates.split(",")
    seeds = [int(s) for s in args.seeds.split(",")]

    args.out.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    print(f"{'gamma':>6}{'date':>12}{'seed':>6}" + "".join(f"{c:>10}" for c in COLS),
          flush=True)
    for date in dates:
        raw = load_day(args.parquet, date)
        day, lows = synth_day(raw, args.hold_hh, args.event_hh, args.end_hh,
                              args.lift, args.up_k, args.down_k)
        for seed in seeds:
            noise = dict(NOISE, seed=seed)
            for gamma in gammas:
                with ctrl_overrides(overrides_for(gamma)):
                    sim = replay(day, ctrl_noise=noise)
                m = analyse(sim, day, lows, args.hold_hh, args.event_hh,
                            args.end_hh)
                m.update(analyse_extra(sim, day, lows, args.hold_hh,
                                       args.event_hh, args.end_hh))
                rows.append({"gamma": gamma, "date": date, "seed": seed, **m})
                print(f"{gamma:>6.1f}{date:>12}{seed:>6}"
                      + "".join(f"{m[c]:>10.2f}" for c in COLS), flush=True)

    with open(args.out, "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=["gamma", "date", "seed"] + COLS)
        wr.writeheader()
        wr.writerows(rows)
    print(f"\nwrote {args.out}\n\nmedians per gamma:")
    print(f"{'gamma':>6}" + "".join(f"{c:>10}" for c in COLS))
    for gamma in gammas:
        sub = [r for r in rows if r["gamma"] == gamma]
        meds = {c: float(np.nanmedian([r[c] for r in sub])) for c in COLS}
        print(f"{gamma:>6.1f}" + "".join(f"{meds[c]:>10.2f}" for c in COLS))


def run_ci(args) -> None:
    from analysis.controller_ci import (
        BASELINE_PATH, CANONICAL_DAYS, GATED, _worst, classify, measure)
    import json

    baseline = json.loads(BASELINE_PATH.read_text())["metrics"]
    for gamma in (float(g) for g in args.gammas.split(",")):
        current = measure(overrides_for(gamma))
        print(f"\n=== gamma {gamma} vs standing CI baseline "
              f"(worst of {CANONICAL_DAYS}) ===")
        print(f"{'metric':<20}{'worst-day':>12}{'base':>9}{'current':>9}"
              f"{'delta':>9}  status")
        failed = False
        for k, (worse_dir, thresh) in GATED.items():
            day, b, c, reg = _worst(baseline, current, k, worse_dir)
            status = classify(c - b, reg, thresh)
            failed = failed or status == "FAIL"
            print(f"{k:<20}{day:>12}{b:>9.3f}{c:>9.3f}{c - b:>+9.3f}  {status}")
        print(f"gamma {gamma}: {'REGRESSION' if failed else 'PASS'}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--gammas", default="1.0,1.5,2.0,3.0")
    ap.add_argument("--dates", default="2026-06-21,2026-06-27")
    ap.add_argument("--seeds", default="622,1622")
    ap.add_argument("--hold-hh", type=float, default=16.0)
    ap.add_argument("--event-hh", type=float, default=19.0)
    ap.add_argument("--end-hh", type=float, default=23.0)
    ap.add_argument("--lift", type=float, default=0.5)
    ap.add_argument("--up-k", type=float, default=0.5)
    ap.add_argument("--down-k", type=float, default=0.5)
    ap.add_argument("--ci", action="store_true",
                    help="run the controller-CI comparison instead of the scenario")
    ap.add_argument("--out", default=_ROOT / "analysis/out/contrast_shape.csv",
                    type=Path)
    ap.add_argument("--parquet", default=_ROOT / "data/processed/june.parquet",
                    type=Path)
    args = ap.parse_args()
    run_ci(args) if args.ci else run_scenario(args)


if __name__ == "__main__":
    main()
