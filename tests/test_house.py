"""Offline, synthetic analytic checks for sim/house.py's RC integrator.

No pandas, no real data -- pure stdlib math against closed-form solutions
of the ODE dT_i/dt = a_i*(T_out-T_i) + c_i*(T_others_i-T_i) + g_i + q_i(t).
"""
from __future__ import annotations

import math

import pytest

from sim.house import ROOMS, House, HouseParams, RoomParams, max_stable_dt_h


def _symmetric_params(tau_out: float, tau_cpl: float, gain: float) -> HouseParams:
    """All five rooms share the same RoomParams (keeps the coupled system
    reducible to a single scalar ODE for analytic comparison)."""
    return HouseParams(
        rooms={room: RoomParams(tau_out=tau_out, tau_cpl=tau_cpl, gain=gain) for room in ROOMS}
    )


def _run(house: House, t_out: float, hours: float, dt_s: float, q=None):
    steps = round(hours * 3600 / dt_s)
    for _ in range(steps):
        house.step(t_out=t_out, q=q, dt_s=dt_s)
    return house.temps


def test_single_room_decay_matches_exponential():
    """Coupling and gains zeroed: each room decays toward T_out on its own
    tau_out, matching T(t) = T_out + (T0-T_out)*exp(-t/tau) within 1%."""
    tau = 20.0  # hours
    t_out = 5.0
    t0 = 25.0
    params = _symmetric_params(tau_out=tau, tau_cpl=math.inf, gain=0.0)
    house = House(params, {room: t0 for room in ROOMS}, dt_s=10)

    dt_s = 10.0
    hours_elapsed = 0.0
    for _ in range(int(6 * 3600 / dt_s)):  # 6 simulated hours
        house.step(t_out=t_out, dt_s=dt_s)
        hours_elapsed += dt_s / 3600.0
        expected = t_out + (t0 - t_out) * math.exp(-hours_elapsed / tau)
        for room in ROOMS:
            assert house.temps[room] == pytest.approx(expected, rel=0.01)


def test_equilibrium_converges_to_analytic_fixed_point():
    """Constant t_out drives all (symmetric) rooms to T = T_out + gain*tau."""
    tau = 15.0
    gain = 0.1  # K/h
    t_out = 8.0
    params = _symmetric_params(tau_out=tau, tau_cpl=5.0, gain=gain)
    house = House(params, {room: 18.0 for room in ROOMS}, dt_s=10)

    final = _run(house, t_out=t_out, hours=8 * tau, dt_s=30)  # 8 time constants, <0.1% residual
    expected = t_out + gain * tau
    for room in ROOMS:
        assert final[room] == pytest.approx(expected, abs=0.05)


def test_coupling_pulls_rooms_together_faster_than_toward_outdoor():
    """Two rooms initialised apart, weak outdoor coupling / strong inter-room
    coupling: the pair converges to each other much faster than the house
    average converges toward T_out."""
    params = _symmetric_params(tau_out=500.0, tau_cpl=2.0, gain=0.0)
    initial = {room: 20.0 for room in ROOMS}
    initial["bed_1"] = 30.0
    initial["bed_2"] = 10.0
    house = House(params, initial, dt_s=10)

    house_avg_start = sum(initial.values()) / len(initial)
    gap_start = abs(initial["bed_1"] - initial["bed_2"])

    _run(house, t_out=0.0, hours=6.0, dt_s=10)

    gap_end = abs(house.temps["bed_1"] - house.temps["bed_2"])
    house_avg_end = sum(house.temps.values()) / len(house.temps)

    assert gap_end < 0.1 * gap_start  # pair has essentially equalised
    # house average barely moved toward outdoor (tau_out=500h >> 3h window)
    assert abs(house_avg_end - house_avg_start) < 0.5


def test_constant_q_produces_analytic_steady_state_offset():
    """With zero baked-in gain, a constant external q [K/h] produces the
    same T = T_out + q*tau fixed point as the gain does."""
    tau = 12.0
    q_val = 0.15
    t_out = 10.0
    params = _symmetric_params(tau_out=tau, tau_cpl=4.0, gain=0.0)
    house = House(params, {room: 22.0 for room in ROOMS}, dt_s=10)

    q = {room: q_val for room in ROOMS}
    final = _run(house, t_out=t_out, hours=8 * tau, dt_s=30, q=q)  # 8 time constants
    expected = t_out + q_val * tau
    for room in ROOMS:
        assert final[room] == pytest.approx(expected, abs=0.05)


def test_stability_guard_rejects_excessive_dt_at_construction():
    tau = 0.05  # hours (3 min) -- deliberately tiny to trigger the guard
    params = _symmetric_params(tau_out=tau, tau_cpl=math.inf, gain=0.0)
    limit_h = max_stable_dt_h(params)
    assert limit_h < 1  # sanity: this tau really is small

    too_big_dt_s = (limit_h + 0.01) * 3600
    with pytest.raises(ValueError):
        House(params, {room: 20.0 for room in ROOMS}, dt_s=too_big_dt_s)

    ok_dt_s = (limit_h * 0.5) * 3600
    house = House(params, {room: 20.0 for room in ROOMS}, dt_s=ok_dt_s)
    house.step(t_out=15.0, dt_s=ok_dt_s)  # should not raise


def test_stability_guard_rejects_excessive_dt_on_step():
    params = HouseParams()  # default calibration.md params
    house = House(params, {room: 20.0 for room in ROOMS}, dt_s=10)
    limit_h = max_stable_dt_h(params)
    too_big_dt_s = (limit_h + 1.0) * 3600
    with pytest.raises(ValueError):
        house.step(t_out=15.0, dt_s=too_big_dt_s)


def test_default_params_cover_all_rooms():
    params = HouseParams()
    assert set(params.rooms) == set(ROOMS)


def test_missing_room_in_initial_temps_raises():
    params = HouseParams()
    incomplete = {room: 20.0 for room in ROOMS if room != "study"}
    with pytest.raises(ValueError):
        House(params, incomplete, dt_s=10)
