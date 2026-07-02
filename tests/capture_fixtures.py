"""Capture golden fixtures from the unmodified actrl.py before extracting
control.py, by driving each pure control class through deterministic
scripted sequences and recording inputs + outputs to tests/fixtures/*.json.

Run once against the pre-move actrl.py: `python3 tests/capture_fixtures.py`
"""

import json
import sys
import types
from pathlib import Path

# Stub out hassapi so `import actrl` works outside the AppDaemon runtime.
_hassapi = types.ModuleType("hassapi")


class _Hass:
    pass


_hassapi.Hass = _Hass
sys.modules["hassapi"] = _hassapi

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import actrl  # noqa: E402

FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures"


def capture_wma():
    cases = []

    for window in (5, 30):
        # ramp
        wma = actrl.MyWMA(window=window)
        seq = [float(i) for i in range(11)]
        outputs = []
        for v in seq:
            wma.set(v)
            outputs.append(wma.get())
        cases.append(
            {"window": window, "pattern": "ramp", "inputs": seq, "outputs": outputs}
        )

        # step
        wma = actrl.MyWMA(window=window)
        seq = [0.0] * 3 + [10.0] * 5 + [-5.0] * 3
        outputs = []
        for v in seq:
            wma.set(v)
            outputs.append(wma.get())
        cases.append(
            {"window": window, "pattern": "step", "inputs": seq, "outputs": outputs}
        )

        # alternating sign
        wma = actrl.MyWMA(window=window)
        seq = []
        for i in range(1, 11):
            seq.append(float(i) if i % 2 == 1 else -float(i))
        outputs = []
        for v in seq:
            wma.set(v)
            outputs.append(wma.get())
        cases.append(
            {
                "window": window,
                "pattern": "alternating",
                "inputs": seq,
                "outputs": outputs,
            }
        )

    with open(FIXTURES_DIR / "my_wma.json", "w") as f:
        json.dump(cases, f, indent=2)


def capture_deriv():
    cases = []

    # constant error
    deriv = actrl.MyDeriv(window=30, factor=60.0)
    steps = []
    for _ in range(15):
        deriv.set(error=1.0, target=0.0)
        steps.append({"error": 1.0, "target": 0.0, "output": deriv.get()})
    cases.append({"pattern": "constant_error", "steps": steps})

    # ramping error
    deriv = actrl.MyDeriv(window=30, factor=60.0)
    steps = []
    for i in range(20):
        error = i * 0.25
        deriv.set(error=error, target=0.0)
        steps.append({"error": error, "target": 0.0, "output": deriv.get()})
    cases.append({"pattern": "ramping_error", "steps": steps})

    # target step (must show target-change compensation)
    deriv = actrl.MyDeriv(window=30, factor=60.0)
    steps = []
    for i in range(10):
        deriv.set(error=2.0, target=0.0)
        steps.append({"error": 2.0, "target": 0.0, "output": deriv.get()})
    for i in range(10):
        deriv.set(error=2.0, target=5.0)
        steps.append({"error": 2.0, "target": 5.0, "output": deriv.get()})
    for i in range(5):
        deriv.set(error=2.0, target=5.0)
        steps.append({"error": 2.0, "target": 5.0, "output": deriv.get()})
    cases.append({"pattern": "target_step", "steps": steps})

    # post-clear
    deriv = actrl.MyDeriv(window=30, factor=60.0)
    steps = []
    for i in range(5):
        deriv.set(error=1.0, target=0.0)
        steps.append({"op": "set", "error": 1.0, "target": 0.0, "output": deriv.get()})
    deriv.clear()
    steps.append({"op": "clear", "output": deriv.get()})
    for i in range(5):
        deriv.set(error=3.0, target=1.0)
        steps.append({"op": "set", "error": 3.0, "target": 1.0, "output": deriv.get()})
    cases.append({"pattern": "post_clear", "steps": steps})

    with open(FIXTURES_DIR / "my_deriv.json", "w") as f:
        json.dump(cases, f, indent=2)


def capture_pid():
    cases = []

    def make_pid():
        return actrl.MyPID(kp=1.0, ki=0.01, kd=12.0, window=60)

    # basic update/get_output sequence
    pid = make_pid()
    steps = []
    errors = [0.5, 0.4, 0.3, 0.2, 0.1, 0.0, -0.1, -0.2, 0.3, 0.6, 1.0, 0.5, -0.5, -1.0, 0.0]
    setpoint = 21.0
    for error in errors:
        pid.update(error, setpoint)
        steps.append(
            {
                "op": "update",
                "error": error,
                "setpoint": setpoint,
                "output": pid.get_output(),
            }
        )
    cases.append({"pattern": "basic_sequence", "steps": steps})

    # interleaved with set_integral / adjust_integral
    pid = make_pid()
    steps = []
    ops = [
        ("update", 0.5, 21.0),
        ("update", 0.4, 21.0),
        ("set_integral", 0.2),
        ("update", 0.3, 21.0),
        ("adjust_integral", -0.05),
        ("update", 0.2, 21.5),
        ("update", 0.1, 22.0),
        ("adjust_integral", 0.15),
        ("update", -0.2, 22.0),
        ("set_integral", -0.1),
        ("update", -0.3, 22.0),
        ("update", 0.0, 22.0),
        ("adjust_integral", 0.5),
        ("update", 0.05, 22.0),
    ]
    for op in ops:
        if op[0] == "update":
            _, error, setpoint = op
            pid.update(error, setpoint)
            steps.append(
                {
                    "op": "update",
                    "error": error,
                    "setpoint": setpoint,
                    "output": pid.get_output(),
                }
            )
        elif op[0] == "set_integral":
            _, value = op
            pid.set_integral(value)
            steps.append({"op": "set_integral", "value": value, "output": pid.get_output()})
        elif op[0] == "adjust_integral":
            _, adjustment = op
            pid.adjust_integral(adjustment)
            steps.append(
                {"op": "adjust_integral", "adjustment": adjustment, "output": pid.get_output()}
            )
    cases.append({"pattern": "interleaved_integral_ops", "steps": steps})

    # clear mid-sequence
    pid = make_pid()
    steps = []
    for error in [0.5, 0.4, 0.3]:
        pid.update(error, 21.0)
        steps.append({"op": "update", "error": error, "setpoint": 21.0, "output": pid.get_output()})
    pid.clear()
    steps.append({"op": "clear", "output": pid.get_output()})
    for error in [1.0, 0.5, -0.5]:
        pid.update(error, 21.0)
        steps.append({"op": "update", "error": error, "setpoint": 21.0, "output": pid.get_output()})
    cases.append({"pattern": "clear_mid_sequence", "steps": steps})

    with open(FIXTURES_DIR / "my_pid.json", "w") as f:
        json.dump(cases, f, indent=2)


def capture_deadband_integrator():
    cases = []

    def make():
        return actrl.DeadbandIntegrator(ki=0.125)

    # small positive error until it emits +1
    di = make()
    steps = []
    for _ in range(20):
        rval = di.set(0.1)
        steps.append({"error": 0.1, "rval": rval, "integral": di.get()})
    cases.append({"pattern": "small_positive_until_plus_one", "steps": steps})

    # sign flips, exercising +/-0.75 preload
    di = make()
    steps = []
    for error in [0.2, 0.2, -0.2, -0.2, 0.2, -0.2, 0.2, 0.2, 0.2, -0.2, -0.2, -0.2]:
        rval = di.set(error)
        steps.append({"error": error, "rval": rval, "integral": di.get()})
    cases.append({"pattern": "sign_flips", "steps": steps})

    # large errors
    di = make()
    steps = []
    for error in [5.0, -5.0, 3.0, -3.0, 10.0, -10.0]:
        rval = di.set(error)
        steps.append({"error": error, "rval": rval, "integral": di.get()})
    cases.append({"pattern": "large_errors", "steps": steps})

    # clear()
    di = make()
    steps = []
    for error in [0.3, 0.3, 0.3]:
        rval = di.set(error)
        steps.append({"op": "set", "error": error, "rval": rval, "integral": di.get()})
    di.clear()
    steps.append({"op": "clear", "integral": di.get()})
    for error in [-0.3, -0.3, -0.3]:
        rval = di.set(error)
        steps.append({"op": "set", "error": error, "rval": rval, "integral": di.get()})
    cases.append({"pattern": "clear", "steps": steps})

    with open(FIXTURES_DIR / "deadband_integrator.json", "w") as f:
        json.dump(cases, f, indent=2)


def capture_window_state_handler():
    cases = []

    wsh = actrl.WindowStateHandler()
    steps = []
    # room_a: open, open, open, close, open, open
    # room_b: closed the whole time initially, then opens later
    sequence = [
        ("room_a", True),
        ("room_b", False),
        ("room_a", True),
        ("room_b", False),
        ("room_a", True),
        ("room_b", True),
        ("room_a", False),
        ("room_b", True),
        ("room_a", True),
        ("room_b", True),
        ("room_a", True),
        ("room_b", False),
    ]
    for room, window_open in sequence:
        wsh.update(room, window_open)
        steps.append(
            {
                "room": room,
                "window_open": window_open,
                "offset_room_a": wsh.get_offset("room_a"),
                "offset_room_b": wsh.get_offset("room_b"),
                "offset_room_never_updated": wsh.get_offset("room_c"),
            }
        )
    cases.append({"pattern": "two_rooms", "steps": steps})

    # default accumulation rate value, recorded directly
    wsh_default = actrl.WindowStateHandler()
    cases.append(
        {"pattern": "default_accumulation_rate", "value": wsh_default.accumulation_rate}
    )

    with open(FIXTURES_DIR / "window_state_handler.json", "w") as f:
        json.dump(cases, f, indent=2)


def main():
    FIXTURES_DIR.mkdir(parents=True, exist_ok=True)
    capture_wma()
    capture_deriv()
    capture_pid()
    capture_deadband_integrator()
    capture_window_state_handler()
    print(f"Fixtures written to {FIXTURES_DIR}")


if __name__ == "__main__":
    main()
