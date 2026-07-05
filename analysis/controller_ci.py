"""Pre-deploy controller regression gate (ideas.md #8).

Replays the current *production* actrl/control (no overrides) over three
canonical June days and diffs comfort/cost metrics against a committed
standing baseline (analysis/ci_baseline.json). Catches "the new logic
cycles the compressor 2x" / "warmups got sluggish" regressions before a
deploy for ~a few minutes of compute -- the tuning harness already does
the replay + scoring, this just pins a baseline and applies pass/fail
directions.

Local gate, not cloud CI: data/ is gitignored (rolling archive), so the
gate runs against the local parquet. The canonical days are fixed
historical dates, so the baseline is reproducible against the same
processed snapshot. Re-baseline with --update-baseline whenever an
*intended* behaviour change lands (and say so in the commit).

Usage:
    uv run python analysis/controller_ci.py            # gate (exit 1 on regression)
    uv run python analysis/controller_ci.py --update-baseline
    uv run python analysis/controller_ci.py --set control.global_deadband_ki=0.025
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent.parent
for p in (str(_ROOT), str(_ROOT / "tests")):
    if p not in sys.path:
        sys.path.insert(0, p)

from analysis.tune import parse_overrides, score_one_day  # noqa: E402

CANONICAL_DAYS = ["2026-06-22", "2026-06-30", "2026-06-09"]  # cold / overcast / mild
BASELINE_PATH = _ROOT / "analysis" / "ci_baseline.json"
PARQUET = _ROOT / "data" / "processed" / "june.parquet"

# metric -> (worse_direction, regress_threshold on the median).
# worse_direction: +1 = a rise is a regression, -1 = a fall is a regression.
# `None` threshold => informational only (reported, never fails the gate;
# energy is intentionally traded by tuning changes, so it never gates).
GATED = {
    "time_in_band":       (-1, 0.01),
    "deg_min_below":      (+1, 2.0),
    "deg_min_above":      (+1, 0.5),
    "overshoot_max":      (+1, 0.3),
    "overshoot_rise_max": (+1, 0.3),
    "osc_per_h":          (+1, 0.3),
    "starts":             (+1, 1.0),
    "rise_time_med":      (+1, 8.0),
    "energy_kwh":         (+1, None),
    "abovemin_frac":      (+1, None),
}
EPS = 1e-6  # below this a metric is "unchanged" (deterministic replay)


def measure(overrides: dict) -> dict[str, dict]:
    """Per-day metrics for the given controller config: {day: {metric: v}}."""
    return {d: score_one_day(PARQUET, d, overrides) for d in CANONICAL_DAYS}


def _worst(baseline: dict, current: dict, metric: str, worse_dir: int):
    """The single worst per-day regression for `metric`: (day, base, cur,
    signed_regression) where signed_regression > 0 means degraded. Worst =
    largest degradation in the worse direction (so a cold-day-only cycling
    spike is caught even if other days improve)."""
    worst = None
    for d in CANONICAL_DAYS:
        b = baseline[d][metric]
        c = current[d][metric]
        reg = worse_dir * (c - b)  # >0 == worse
        if worst is None or reg > worst[3]:
            worst = (d, b, c, reg)
    return worst


def classify(delta: float, reg: float, thresh: float | None) -> str:
    """Pure status decision for one metric's worst-day movement. `delta` =
    current - baseline (signed), `reg` = worse-direction magnitude (>0 ==
    degraded), `thresh` = fail threshold (None => never gates). Returns one
    of ok/improved/info/warn/FAIL (only FAIL fails the gate)."""
    if abs(delta) <= EPS:
        return "ok"
    if reg <= 0:
        return "improved"
    if thresh is None:
        return "info"  # traded metric (energy); report, never gate
    return "FAIL" if reg > thresh else "warn"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--update-baseline", action="store_true",
                    help="write current metrics as the new baseline and exit")
    ap.add_argument("--set", dest="overrides", action="append", default=[],
                    metavar="KEY=VALUE", help="controller override (repeatable)")
    args = ap.parse_args()
    overrides = parse_overrides(args.overrides)

    current = measure(overrides)

    if args.update_baseline:
        payload = {"days": CANONICAL_DAYS, "metrics": current}
        BASELINE_PATH.write_text(json.dumps(payload, indent=2) + "\n")
        print(f"wrote baseline -> {BASELINE_PATH.relative_to(_ROOT)}")
        for d in CANONICAL_DAYS:
            print(f"  {d}: " + " ".join(f"{k}={current[d][k]:.3f}" for k in GATED))
        return 0

    if not BASELINE_PATH.exists():
        print("no baseline -- run with --update-baseline first", file=sys.stderr)
        return 2
    baseline = json.loads(BASELINE_PATH.read_text())["metrics"]

    # Worst-case per-day gate: fail if ANY canonical day regresses beyond
    # threshold (median would hide a cold-day-only cycling regression).
    print(f"{'metric':<20}{'worst-day':>12}{'base':>9}{'current':>9}{'delta':>9}  status")
    failed = False
    for k, (worse_dir, thresh) in GATED.items():
        day, b, c, reg = _worst(baseline, current, k, worse_dir)
        status = classify(c - b, reg, thresh)
        failed = failed or status == "FAIL"
        print(f"{k:<20}{day:>12}{b:>9.3f}{c:>9.3f}{c - b:>+9.3f}  {status}")

    print()
    if failed:
        print("REGRESSION: at least one gated metric degraded beyond threshold "
              "on a canonical day.")
        return 1
    print("PASS: no gated metric regressed beyond threshold on any canonical day.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
