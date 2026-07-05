"""Unit tests for the pre-deploy gate's pure logic (ideas.md #8).

No data/ access -- exercises classify() and _worst() on hand-built dicts.
"""
from __future__ import annotations

from analysis.controller_ci import EPS, _worst, classify


def test_classify_unchanged_is_ok():
    assert classify(0.0, 0.0, 1.0) == "ok"
    assert classify(EPS / 2, EPS / 2, 1.0) == "ok"


def test_classify_improvement_never_fails():
    # moved in the good direction (reg <= 0) even if large
    assert classify(-5.0, -5.0, 1.0) == "improved"


def test_classify_warn_below_threshold_fail_above():
    assert classify(0.5, 0.5, 1.0) == "warn"
    assert classify(1.5, 1.5, 1.0) == "FAIL"


def test_classify_info_metric_never_gates():
    # thresh None (energy/abovemin): a real regression is only "info"
    assert classify(9.0, 9.0, None) == "info"


def test_worst_picks_largest_regression_across_days():
    # deg_min_below style: worse_dir=+1. Cold day spikes, others improve --
    # the worst-case gate must still surface the cold-day regression.
    days = ["cold", "overcast", "mild"]
    import analysis.controller_ci as ci
    orig = ci.CANONICAL_DAYS
    ci.CANONICAL_DAYS = days
    try:
        baseline = {"cold": {"m": 26.0}, "overcast": {"m": 0.2}, "mild": {"m": 2.6}}
        current = {"cold": {"m": 40.0}, "overcast": {"m": 0.1}, "mild": {"m": 2.5}}
        day, b, c, reg = _worst(baseline, current, "m", +1)
        assert day == "cold"
        assert reg == 14.0  # 40 - 26, the worst degradation
    finally:
        ci.CANONICAL_DAYS = orig


def test_worst_returns_best_day_when_all_improve():
    days = ["a", "b"]
    import analysis.controller_ci as ci
    orig = ci.CANONICAL_DAYS
    ci.CANONICAL_DAYS = days
    try:
        baseline = {"a": {"m": 10.0}, "b": {"m": 5.0}}
        current = {"a": {"m": 8.0}, "b": {"m": 4.0}}
        _, _, _, reg = _worst(baseline, current, "m", +1)
        assert reg < 0  # every day improved -> no regression
    finally:
        ci.CANONICAL_DAYS = orig
