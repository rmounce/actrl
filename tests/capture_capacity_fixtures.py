"""Capture golden fixtures for the compressor capacity-control logic
(`compress` / `midea_reset_quirks` / `midea_runtime_quirks`) from the
unmodified actrl.py, before it is extracted into `control.MideaCapacityController`
(task 003).

Drives the pre-move logic on a bare `actrl.Actrl` instance (constructed via
`object.__new__`, bypassing AppDaemon's `__init__`/`initialize`) with only the
eight state attributes owned by the capacity controller set up manually, plus
a stubbed `log`. Each scenario is a deterministic, hardcoded sequence of
`compress(error, deriv)` calls, interleaved with the same bookkeeping
`main()` performs between calls (`on_counter += 1`,
`prev_unsigned_compressed_error = rval`), and occasionally an explicit
external state mutation (recorded as its own step) such as the defrost
override or the ac_min_power on_counter clamp.

Every mutation -- whether from `compress()` itself or from the driving
harness below -- is recorded as a "step" with the full 8-value state snapshot
taken immediately after the mutation. This makes replay in
`tests/test_capacity.py` mechanical: apply each step's operation, then assert
the recorded rval/state matches.

Run once against the pre-move actrl.py:
    python3 tests/capture_capacity_fixtures.py
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


def make_instance():
    """A bare Actrl with only the capacity-controller state initialised,
    matching the defaults `initialize()` sets for a cold start."""
    inst = object.__new__(actrl.Actrl)
    inst.logs = []
    inst.log = lambda msg, level=None, **kwargs: inst.logs.append(str(msg))
    inst.on_counter = 0
    inst.min_power_counter = 0
    inst.max_power_counter = 0
    inst.prev_step = 0
    inst.guesstimated_comp_speed = 0
    inst.compressor_totally_off = True
    inst.prev_unsigned_compressed_error = 0
    inst.deadband_integrator = actrl.DeadbandIntegrator(
        ki=(actrl.global_deadband_ki * 60.0 * actrl.interval)
    )
    return inst


def snapshot(inst):
    state = {attr: getattr(inst, attr) for attr in STATE_ATTRS}
    state["deadband_integral"] = inst.deadband_integrator.get()
    return state


def compress_step(inst, steps, error, deriv):
    """Call compress(), record it, then apply + record the same bookkeeping
    main() performs between calls."""
    rval = inst.compress(error, deriv)
    steps.append(
        {
            "op": "compress",
            "error": error,
            "deriv": deriv,
            "rval": rval,
            "state": snapshot(inst),
        }
    )
    inst.prev_unsigned_compressed_error = rval
    steps.append(
        {
            "op": "set_attr",
            "attr": "prev_unsigned_compressed_error",
            "value": rval,
            "state": snapshot(inst),
        }
    )
    inst.on_counter += 1
    steps.append(
        {
            "op": "set_attr",
            "attr": "on_counter",
            "value": inst.on_counter,
            "state": snapshot(inst),
        }
    )
    return rval


def set_attr_step(inst, steps, attr, value):
    """Record an explicit external mutation (defrost override, ac_min_power
    on_counter clamp) the same way main() applies it outside compress()."""
    setattr(inst, attr, value)
    steps.append(
        {
            "op": "set_attr",
            "attr": attr,
            "value": value,
            "state": snapshot(inst),
        }
    )


def run_constant(inst, steps, error, deriv, count):
    for _ in range(count):
        compress_step(inst, steps, error, deriv)


def capture_cold_start():
    inst = make_instance()
    steps = []
    # off -> on-blip -> soft start (soft_delay=45 cycles) -> ramp (soft_ramp=15) -> step-ups
    run_constant(inst, steps, error=1.0, deriv=0.0, count=55)
    run_constant(inst, steps, error=1.8, deriv=0.1, count=30)
    with open(FIXTURES_DIR / "cold_start.json", "w") as f:
        json.dump([{"pattern": "cold_start", "steps": steps}], f, indent=2)


def capture_equilibrium():
    inst = make_instance()
    steps = []
    # warm up to a stable, running state without large deadband swings
    run_constant(inst, steps, error=0.0, deriv=0.0, count=50)
    # slow oscillation, long enough to fully emit step-up and step-down
    # sequences via the deadband integrator several times over
    for _ in range(3):
        run_constant(inst, steps, error=0.05, deriv=0.0, count=20)
        run_constant(inst, steps, error=-0.05, deriv=0.0, count=20)
    with open(FIXTURES_DIR / "equilibrium.json", "w") as f:
        json.dump([{"pattern": "equilibrium", "steps": steps}], f, indent=2)


def capture_faithful_hysteresis():
    inst = make_instance()
    steps = []
    run_constant(inst, steps, error=0.0, deriv=0.0, count=50)
    # entry + latch
    run_constant(inst, steps, error=3.0, deriv=0.0, count=5)
    # decay back down through the hysteresis band to exit
    for error in [2.5, 2.0, 1.5, 1.0, 0.5, 0.0, -0.5, -1.0]:
        compress_step(inst, steps, error=error, deriv=0.0)
    with open(FIXTURES_DIR / "faithful_hysteresis.json", "w") as f:
        json.dump([{"pattern": "faithful_hysteresis", "steps": steps}], f, indent=2)


def capture_min_power():
    inst = make_instance()
    steps = []
    run_constant(inst, steps, error=0.0, deriv=0.0, count=50)
    # 600 cycles > min_power_delay (270), min_power_time (270), purge_delay
    # (540): covers the eventual_off ramp, the min_power_time 'blip', and
    # purge wraparound.
    run_constant(inst, steps, error=-0.3, deriv=0.0, count=600)
    with open(FIXTURES_DIR / "min_power.json", "w") as f:
        json.dump([{"pattern": "min_power", "steps": steps}], f, indent=2)


def capture_immediate_off():
    inst = make_instance()
    steps = []
    run_constant(inst, steps, error=0.0, deriv=0.0, count=50)
    # ramp guesstimated_comp_speed above 0 via repeated step-ups
    run_constant(inst, steps, error=1.9, deriv=0.1, count=30)
    assert inst.guesstimated_comp_speed > 0, "precondition: speed must be > 0"
    # immediate_off_threshold = -1.5
    run_constant(inst, steps, error=-1.6, deriv=0.0, count=5)
    with open(FIXTURES_DIR / "immediate_off.json", "w") as f:
        json.dump([{"pattern": "immediate_off", "steps": steps}], f, indent=2)


def capture_min_power_threshold_cut():
    cases = []
    for deriv, label in [(0.0, "deriv_zero"), (0.5, "deriv_plus"), (-0.5, "deriv_minus")]:
        inst = make_instance()
        steps = []
        run_constant(inst, steps, error=0.0, deriv=0.0, count=50)
        run_constant(inst, steps, error=-2.5, deriv=deriv, count=8)
        cases.append({"pattern": f"min_power_threshold_cut_{label}", "steps": steps})
    with open(FIXTURES_DIR / "min_power_threshold_cut.json", "w") as f:
        json.dump(cases, f, indent=2)


def capture_external_defrost_override():
    inst = make_instance()
    steps = []
    run_constant(inst, steps, error=0.0, deriv=0.0, count=50)
    run_constant(inst, steps, error=0.5, deriv=0.0, count=5)
    # simulate main()'s defrost detection forcing max speed between cycles
    set_attr_step(
        inst,
        steps,
        "guesstimated_comp_speed",
        actrl.compressor_power_increments + actrl.compressor_power_safety_margin,
    )
    run_constant(inst, steps, error=0.5, deriv=0.0, count=10)
    with open(FIXTURES_DIR / "external_defrost_override.json", "w") as f:
        json.dump([{"pattern": "external_defrost_override", "steps": steps}], f, indent=2)


def capture_on_counter_clamp():
    inst = make_instance()
    steps = []
    # run past soft_delay so on_counter is well beyond the soft-start window
    run_constant(inst, steps, error=1.0, deriv=0.0, count=60)
    # simulate main()'s input_boolean.ac_min_power clamp
    set_attr_step(inst, steps, "on_counter", min(inst.on_counter, actrl.soft_delay - 1))
    run_constant(inst, steps, error=1.0, deriv=0.0, count=10)
    with open(FIXTURES_DIR / "on_counter_clamp.json", "w") as f:
        json.dump([{"pattern": "on_counter_clamp", "steps": steps}], f, indent=2)


def main():
    FIXTURES_DIR.mkdir(parents=True, exist_ok=True)
    capture_cold_start()
    capture_equilibrium()
    capture_faithful_hysteresis()
    capture_min_power()
    capture_immediate_off()
    capture_min_power_threshold_cut()
    capture_external_defrost_override()
    capture_on_counter_clamp()
    print(f"Fixtures written to {FIXTURES_DIR}")


if __name__ == "__main__":
    main()
