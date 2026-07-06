"""Where does the min-airflow inflation loop actually bind?

Replays recorded days with a counting shim around production
min_airflow_inflation: per cycle, records the pre-inflation shortfall
(min_sum - weighted positive output) and which rooms were re-inflated
from a negative output (satisfied zones dragged back into airflow).
Context for the inflation-policy study (analysis/inflation_policy.py):
a redesign can only matter where the loop binds.

Usage:
    uv run python analysis/inflation_bind.py [--dates 2026-06-09,2026-06-21,2026-06-27]
"""
from __future__ import annotations

import argparse
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
from analysis.inflation_policy import NOISE, _positive_sum  # noqa: E402
from analysis.replay_day import load_day, replay  # noqa: E402

_production = actrl.min_airflow_inflation


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dates", default="2026-06-09,2026-06-21,2026-06-27")
    ap.add_argument("--parquet", default=_ROOT / "data/processed/june.parquet",
                    type=Path)
    args = ap.parse_args()

    for date in args.dates.split(","):
        stats = {"cycles": 0, "bound": 0, "short": [], "dragged": {r: 0 for r in cl.ROOMS}}

        def shim(pids, pid_outputs, adjusted_room_airflow, min_sum):
            stats["cycles"] += 1
            short = min_sum - _positive_sum(pid_outputs, adjusted_room_airflow)
            before = dict(pid_outputs)
            _production(pids, pid_outputs, adjusted_room_airflow, min_sum)
            if short > 1e-9:
                stats["bound"] += 1
                stats["short"].append(short)
                for r, o in before.items():
                    if o < 0 and pid_outputs[r] > o + 1e-9:
                        stats["dragged"][r] += 1

        day = load_day(args.parquet, date)
        with ctrl_overrides({"actrl.min_airflow_inflation": shim}):
            replay(day, ctrl_noise=dict(NOISE))

        sh = np.array(stats["short"]) if stats["short"] else np.array([0.0])
        dragged = {r: round(v / 6) for r, v in stats["dragged"].items()}  # cycles -> min
        print(f"{date}: loop bound {stats['bound']}/{stats['cycles']} cycles "
              f"({100 * stats['bound'] / max(1, stats['cycles']):.0f}%), shortfall "
              f"median {np.median(sh):.3f} / p90 {np.percentile(sh, 90):.3f} "
              f"(min_sum units, range=2.0); satisfied-zone drag minutes: {dragged}",
              flush=True)


if __name__ == "__main__":
    main()
