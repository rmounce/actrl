"""Unit tests for analysis/ctrl_overrides.py (docs/tasks/014).

Exercises the real `actrl`/`control` modules' existing constants (never
mutating production source -- only setattr/restore at runtime).
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

# Stub hassapi before `import actrl` (same dance as tests/hvac_harness.py /
# sim/closed_loop.py) -- actrl.py imports the real AppDaemon `hassapi`
# module, which only exists inside the AppDaemon runtime.
sys.path.insert(0, str(Path(__file__).resolve().parent))
import hvac_harness  # noqa: E402,F401  (side effect: stubs sys.modules["hassapi"])
import actrl  # noqa: E402
import control  # noqa: E402
from analysis.ctrl_overrides import ctrl_overrides  # noqa: E402
from analysis.tune import parse_overrides  # noqa: E402


def test_override_applies_and_restores():
    original = actrl.global_ki
    with ctrl_overrides({"actrl.global_ki": 0.0005}):
        assert actrl.global_ki == 0.0005
    assert actrl.global_ki == original


def test_override_restores_across_exception():
    original = control.global_deadband_ki
    with pytest.raises(RuntimeError):
        with ctrl_overrides({"control.global_deadband_ki": 0.02}):
            assert control.global_deadband_ki == 0.02
            raise RuntimeError("boom")
    assert control.global_deadband_ki == original


def test_multiple_overrides_both_restore():
    orig_actrl = actrl.global_ki
    orig_control = control.global_deadband_ki
    with ctrl_overrides(
        {"actrl.global_ki": 0.0009, "control.global_deadband_ki": 0.03}
    ):
        assert actrl.global_ki == 0.0009
        assert control.global_deadband_ki == 0.03
    assert actrl.global_ki == orig_actrl
    assert control.global_deadband_ki == orig_control


def test_unknown_module_prefix_raises_value_error():
    with pytest.raises(ValueError):
        with ctrl_overrides({"statctrl.some_attr": 1.0}):
            pass


def test_missing_attr_raises_attribute_error():
    with pytest.raises(AttributeError):
        with ctrl_overrides({"actrl.definitely_not_a_real_attr_xyz": 1.0}):
            pass


def test_missing_attr_restores_earlier_overrides():
    original = actrl.global_ki
    with pytest.raises(AttributeError):
        with ctrl_overrides(
            {"actrl.global_ki": 0.0007, "actrl.definitely_not_a_real_attr_xyz": 1.0}
        ):
            pass
    assert actrl.global_ki == original


def test_type_preservation_int_constant():
    original = actrl.global_temp_deriv_window
    assert isinstance(original, int)
    with ctrl_overrides({"actrl.global_temp_deriv_window": 3}):
        assert actrl.global_temp_deriv_window == 3
        assert isinstance(actrl.global_temp_deriv_window, int)
    assert actrl.global_temp_deriv_window == original


def test_set_string_type_preservation_int_constant():
    # analysis/tune.py's --set parsing (analysis.tune.parse_overrides):
    # overriding an int constant with the literal string "3" must yield
    # int 3, not float 3.0, so it round-trips through ctrl_overrides
    # cleanly (docs/tasks/014 section 3).
    assert isinstance(actrl.global_temp_deriv_window, int)
    parsed = parse_overrides(["actrl.global_temp_deriv_window=3"])
    value = parsed["actrl.global_temp_deriv_window"]
    assert value == 3
    assert isinstance(value, int)

    parsed_float = parse_overrides(["actrl.global_ki=0.0005"])
    assert parsed_float["actrl.global_ki"] == pytest.approx(0.0005)
    assert isinstance(parsed_float["actrl.global_ki"], float)
