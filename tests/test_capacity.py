"""Golden tests for control.MideaCapacityController.

Replays every fixture captured in tests/fixtures/capacity/*.json (recorded
from the pre-extraction actrl.py, see tests/capture_capacity_fixtures.py)
against the extracted control.MideaCapacityController, asserting the
returned rval and all eight owned state values match at every step.

No actrl import, no hassapi stub: this exercises control.py in isolation.
"""

import json
from pathlib import Path

import pytest

from control import MideaCapacityController

FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures" / "capacity"

STATE_ATTRS = [
    "on_counter",
    "min_power_counter",
    "max_power_counter",
    "prev_step",
    "guesstimated_comp_speed",
    "compressor_totally_off",
    "prev_unsigned_compressed_error",
]


def load(name):
    with open(FIXTURES_DIR / name) as f:
        return json.load(f)


def assert_state(capacity, expected):
    for attr in STATE_ATTRS:
        got = getattr(capacity, attr)
        want = expected[attr]
        if isinstance(want, bool) or isinstance(want, int):
            assert got == want, f"{attr}: {got!r} != {want!r}"
        else:
            assert got == pytest.approx(want, rel=1e-12), f"{attr}: {got!r} != {want!r}"
    assert capacity.deadband_integrator.get() == pytest.approx(
        expected["deadband_integral"], rel=1e-12
    )


def replay(case):
    capacity = MideaCapacityController(log=lambda msg: None)
    for step in case["steps"]:
        if step["op"] == "compress":
            rval = capacity.compress(step["error"], step["deriv"])
            assert rval == step["rval"], f"rval: {rval!r} != {step['rval']!r}"
        elif step["op"] == "set_attr":
            setattr(capacity, step["attr"], step["value"])
        else:
            raise AssertionError(f"unknown op {step['op']!r}")
        assert_state(capacity, step["state"])


FIXTURE_FILES = sorted(p.name for p in FIXTURES_DIR.glob("*.json"))


@pytest.mark.parametrize("filename", FIXTURE_FILES)
def test_capacity_fixture(filename):
    cases = load(filename)
    for case in cases:
        replay(case)
