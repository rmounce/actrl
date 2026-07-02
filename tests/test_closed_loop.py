"""Smoke tests for the assembled closed-loop simulator (sim/closed_loop.py).

Not golden tests — just invariants: heat mode heats and consumes energy,
off mode does neither, and the heat-mode sign convention reaches the unit
correctly (the 2026-07-03 bug: below-setpoint reports must mean "more
compressor" in heat mode).
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from scenarios import base_world, room_climates  # noqa: E402
from sim.closed_loop import ClosedLoop  # noqa: E402

TARGETS = {"bed_1": 21.0, "bed_2": 20.5, "bed_3": 21.0, "kitchen": 21.5, "study": 20.0}


def make_loop(deficit=2.5):
    start = {r: t - deficit for r, t in TARGETS.items()}
    world = base_world(setpoint=24.0)
    room_climates(world, "heat", TARGETS, start)
    world["binary_sensor.m5atom_outdoor_fan"]["state"] = "on"
    world["binary_sensor.m5atom_compressor"]["state"] = "on"
    loop = ClosedLoop(world, dict(start), setpoint=24.0)
    loop.unit.mode = "heat"
    loop.unit.mode_sign = -1.0
    loop.unit.reset()
    return loop, start


def test_heat_mode_heats_and_consumes():
    loop, start = make_loop()
    rows = [loop.step(t_out=8.0) for _ in range(360)]  # 1 h
    loop.close()
    assert rows[-1]["T_bed_1"] > start["bed_1"] + 0.5
    assert sum(r["p_kw"] for r in rows) > 0
    # The unit must have stepped up beyond min power at some point
    # (heat-mode sign convention regression guard).
    assert max(r["increment"] for r in rows) > 0


def test_compressor_spins_up_not_step():
    # Power must approach the increment-0 steady draw (~0.665 kW) through
    # the ~20 s first-order lag, not jump there on the first cycle.
    loop, _ = make_loop()
    rows = [loop.step(t_out=8.0) for _ in range(24)]  # 4 min
    loop.close()
    steady_min = loop.hvac.power_kw(0)
    first = next(r for r in rows if r["p_kw"] > 0)
    assert first["p_kw"] < 0.6 * steady_min
    assert rows[-1]["p_kw"] >= 0.95 * loop.hvac.power_kw(rows[-1]["increment"])


def test_satisfied_house_stays_off():
    loop, start = make_loop(deficit=-1.0)  # rooms 1 C above target
    rows = [loop.step(t_out=8.0) for _ in range(180)]
    loop.close()
    assert all(r["q_kw"] == 0.0 for r in rows[6:])  # after startup settles
    # Free-running: house drifts, no heat input
    assert abs(rows[-1]["T_bed_1"] - (start["bed_1"])) < 0.5
