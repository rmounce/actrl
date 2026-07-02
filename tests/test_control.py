import json
from pathlib import Path

import pytest

from control import DeadbandIntegrator, MyDeriv, MyPID, MyWMA, WindowStateHandler

FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures"


def load(name):
    with open(FIXTURES_DIR / name) as f:
        return json.load(f)


def test_my_wma():
    for case in load("my_wma.json"):
        wma = MyWMA(window=case["window"])
        for value, expected in zip(case["inputs"], case["outputs"]):
            wma.set(value)
            assert wma.get() == pytest.approx(expected, rel=1e-12)


def test_my_deriv():
    for case in load("my_deriv.json"):
        deriv = MyDeriv(window=30, factor=60.0)
        for step in case["steps"]:
            if step.get("op") == "clear":
                deriv.clear()
            else:
                deriv.set(error=step["error"], target=step["target"])
            assert deriv.get() == pytest.approx(step["output"], rel=1e-12)


def test_my_pid():
    for case in load("my_pid.json"):
        pid = MyPID(kp=1.0, ki=0.01, kd=12.0, window=60)
        for step in case["steps"]:
            op = step["op"]
            if op == "update":
                pid.update(step["error"], step["setpoint"])
            elif op == "set_integral":
                pid.set_integral(step["value"])
            elif op == "adjust_integral":
                pid.adjust_integral(step["adjustment"])
            elif op == "clear":
                pid.clear()
            assert pid.get_output() == pytest.approx(step["output"], rel=1e-12)


def test_deadband_integrator():
    for case in load("deadband_integrator.json"):
        di = DeadbandIntegrator(ki=0.125)
        for step in case["steps"]:
            if step.get("op") == "clear":
                di.clear()
                assert di.get() == pytest.approx(step["integral"], rel=1e-12)
            else:
                rval = di.set(step["error"])
                assert rval == step["rval"]
                assert di.get() == pytest.approx(step["integral"], rel=1e-12)


def test_window_state_handler():
    cases = load("window_state_handler.json")

    two_rooms = next(c for c in cases if c["pattern"] == "two_rooms")
    wsh = WindowStateHandler()
    for step in two_rooms["steps"]:
        wsh.update(step["room"], step["window_open"])
        assert wsh.get_offset("room_a") == pytest.approx(
            step["offset_room_a"], rel=1e-12
        )
        assert wsh.get_offset("room_b") == pytest.approx(
            step["offset_room_b"], rel=1e-12
        )
        assert wsh.get_offset("room_c") == pytest.approx(
            step["offset_room_never_updated"], rel=1e-12
        )

    default_rate = next(
        c for c in cases if c["pattern"] == "default_accumulation_rate"
    )
    wsh_default = WindowStateHandler()
    assert wsh_default.accumulation_rate == pytest.approx(
        default_rate["value"], rel=1e-12
    )
