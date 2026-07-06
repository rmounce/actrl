"""Run the controller-CI regression gate for a named inflation-policy arm.

controller_ci.py --set only expresses numeric constants; inflation policies
are function-valued overrides, so this wrapper feeds them to the same
measure/gate machinery. Winter-schedule regression check for candidates
that win the divergent-target scenario (analysis/inflation_policy.py).

Usage:
    uv run python analysis/inflation_ci.py calling [free_calling ...]
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent.parent
for p in (str(_ROOT), str(_ROOT / "tests")):
    if p not in sys.path:
        sys.path.insert(0, p)

from analysis.controller_ci import (  # noqa: E402
    BASELINE_PATH,
    CANONICAL_DAYS,
    GATED,
    _worst,
    classify,
    measure,
)
from analysis.inflation_policy import ARMS  # noqa: E402


def main() -> int:
    arms = sys.argv[1:]
    if not arms or any(a not in ARMS or ARMS[a] is None for a in arms):
        print(f"usage: inflation_ci.py <arm> [...]; arms = "
              f"{[a for a, p in ARMS.items() if p is not None]}", file=sys.stderr)
        return 2
    baseline = json.loads(BASELINE_PATH.read_text())["metrics"]

    any_failed = False
    for arm in arms:
        current = measure({"actrl.min_airflow_inflation": ARMS[arm]})
        print(f"\n=== {arm} vs standing CI baseline "
              f"(worst canonical day of {CANONICAL_DAYS}) ===")
        print(f"{'metric':<20}{'worst-day':>12}{'base':>9}{'current':>9}"
              f"{'delta':>9}  status")
        failed = False
        for k, (worse_dir, thresh) in GATED.items():
            day, b, c, reg = _worst(baseline, current, k, worse_dir)
            status = classify(c - b, reg, thresh)
            failed = failed or status == "FAIL"
            print(f"{k:<20}{day:>12}{b:>9.3f}{c:>9.3f}{c - b:>+9.3f}  {status}")
        print(f"{arm}: {'REGRESSION' if failed else 'PASS'}")
        any_failed = any_failed or failed
    return 1 if any_failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
