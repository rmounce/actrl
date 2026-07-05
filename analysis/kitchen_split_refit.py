"""Refit the kitchen heat-split weight against recorded damper authority.

The damper fidelity pass (docs/calibration.md "Multi-zone damper
fidelity") found the sim holds the kitchen on target with ~13 fewer
damper points than reality while its temperature tracks (~0 bias): the
heat split credits the kitchen too much heat per opening point. Of the
two split factors, AIRFLOW_WEIGHTS is physically grounded (2 ducts,
Ryan-confirmed); MASS_WEIGHTS["kitchen"]=2.0 is the carried-over
equal-mass-era value that was never re-derived when the other rooms got
NatHERS floor areas -- and the kitchen is the open zone
(kitchen/living/dining), so its true mass share is plausibly larger.
Raising it lowers K/h per damper point, so the room PID must hold the
damper wider to track the same target -- which is what reality does.

Grid kitchen mass weight over the double-ramp mornings, watching:
- gated kitchen damper bias (target: 0; currently −13)
- kitchen temp bias while running (must stay ~0)
- whole-day starts + energy (the kitchen lead constant 0.45 was tuned
  with mass 2.0 to reproduce recorded min-power cycling -- sim/house.py
  says these are loop thresholds, so cycling texture must be re-checked,
  especially 06-22)

Usage:
    uv run python analysis/kitchen_split_refit.py [--weights 2.0,2.5,3.0,3.5]
"""
from __future__ import annotations

import argparse
import statistics
import sys
from pathlib import Path
from unittest import mock

_ROOT = Path(__file__).resolve().parent.parent
for p in (str(_ROOT), str(_ROOT / "tests")):
    if p not in sys.path:
        sys.path.insert(0, p)

import sim.closed_loop as cl  # noqa: E402
from analysis.damper_fidelity import one_day  # noqa: E402

# the five clean-handoff mornings + a mild morning (run-decision texture)
GRID_DAYS = ["2026-06-15", "2026-06-22", "2026-06-24", "2026-06-26",
             "2026-06-29", "2026-06-08"]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default="2.0,2.5,3.0,3.5")
    ap.add_argument("--days", default=",".join(GRID_DAYS))
    ap.add_argument("--parquet", default=_ROOT / "data/processed/june.parquet", type=Path)
    args = ap.parse_args()
    weights = [float(w) for w in args.weights.split(",")]
    days = [d.strip() for d in args.days.split(",") if d.strip()]

    print(f"{'w_kit':>6}{'kit_dmp_bias':>13}{'kit_tmp_bias':>13}{'b1_dmp_bias':>12}"
          f"{'starts s|r':>11}{'kwh_sum':>9}")
    for w in weights:
        rows = []
        with mock.patch.dict(cl.MASS_WEIGHTS, {"kitchen": w}):
            for d in days:
                try:
                    r = one_day(args.parquet, d)
                except (SystemExit, Exception) as e:  # noqa: BLE001
                    print(f"  {d} skipped: {e}", flush=True)
                    continue
                if r:
                    rows.extend(r)
        kit = [r for r in rows if r["room"] == "kitchen"]
        b1 = [r for r in rows if r["room"] == "bed_1"]
        med = lambda rs, k: statistics.median(r[k] for r in rs)  # noqa: E731
        print(f"{w:>6.2f}{med(kit, 'damper_bias'):>+13.1f}{med(kit, 'temp_bias'):>+13.2f}"
              f"{med(b1, 'damper_bias'):>+12.1f}"
              f"{sum(r['sim_starts'] for r in kit):>5}|{sum(r['rec_starts'] for r in kit):<5}"
              f"{sum(r['sim_kwh'] for r in kit):>9.2f}", flush=True)


if __name__ == "__main__":
    main()
