"""Offline, synthetic analytic checks for sim/house.py's RC integrator.

No pandas, no real data -- pure stdlib math against closed-form solutions
of the ODE dT_i/dt = a_i*(T_out-T_i) + c_i*(T_others_i-T_i) + g_i + q_i(t).
"""
from __future__ import annotations

import math
from dataclasses import replace

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


# --- Measured-air lead node (docs/tasks/009-measured-air-node.md) ---------


def _symmetric_params_meas(
    tau_out: float, tau_cpl: float, gain: float, tau_meas_h: float = 0.0, lead_h: float = 0.0
) -> HouseParams:
    """Like _symmetric_params, with the measured-node fields exposed."""
    return HouseParams(
        rooms={
            room: RoomParams(
                tau_out=tau_out, tau_cpl=tau_cpl, gain=gain, tau_meas_h=tau_meas_h, lead_h=lead_h
            )
            for room in ROOMS
        }
    )


def test_disabled_measured_node_is_exact_passthrough():
    """tau_meas_h == 0.0 disables the node: temps_measured must equal temps
    exactly (bit-identical), even under many mixed steps with nonzero,
    time-varying q. (Defaults now carry fitted enabled values, so disable
    explicitly.)"""
    params = HouseParams(
        rooms={
            room: replace(p, tau_meas_h=0.0, lead_h=0.0)
            for room, p in HouseParams().rooms.items()
        }
    )
    house = House(params, {room: 20.0 for room in ROOMS}, dt_s=10)

    for i in range(500):
        q = {room: 0.05 * ((i + hash(room)) % 7 - 3) for room in ROOMS}  # varying, some negative
        t_out = 5.0 + (i % 11)
        pv = 0.3 if i % 4 == 0 else 0.0
        house.step(t_out=t_out, q=q, dt_s=10, pv_kw=pv)
        assert house.temps_measured == house.temps


def test_measured_node_steady_state_offset():
    """Constant q, enabled node: Tm - T converges to lead_h * q within 1%
    once both the (much faster) measured node and the bulk state have
    settled. tau_meas_h is chosen much shorter than tau_out so "T" itself
    is genuinely at its own fixed point (dT/dt == 0) by the time we check
    the offset -- otherwise Tm is chasing a still-moving target and never
    reaches the textbook Tm - T == lead_h * q relation."""
    tau_meas_h = 0.1  # hours (6 min) -- short so the test runs fast
    tau_out = 20.0  # hours -- >> tau_meas_h, so it's the binding settling time
    lead_h = 2.0
    q_val = 0.5  # K/h
    t_out = 10.0
    params = _symmetric_params_meas(
        tau_out=tau_out, tau_cpl=8.0, gain=0.0, tau_meas_h=tau_meas_h, lead_h=lead_h
    )
    house = House(params, {room: 18.0 for room in ROOMS}, dt_s=10)

    q = {room: q_val for room in ROOMS}
    steps = round(8 * tau_out * 3600 / 10)  # 8 bulk time constants -- settles both states
    for _ in range(steps):
        house.step(t_out=t_out, q=q, dt_s=10)

    expected_offset = lead_h * q_val
    for room in ROOMS:
        offset = house.temps_measured[room] - house.temps[room]
        assert offset == pytest.approx(expected_offset, rel=0.01)


def test_measured_node_decays_exponentially_after_stop():
    """After q returns to 0, (Tm - T) decays toward 0 as
    exp(-t/tau_meas_h), matched within 1% over a 2-tau window.

    tau_out is chosen much longer than tau_meas_h so the bulk state is
    essentially static over the short (2 * tau_meas_h) decay window -- the
    node's own decay dominates, isolating exp(-t/tau_meas_h) from the
    bulk's own (much slower) relaxation."""
    tau_meas_h = 0.1  # hours
    tau_out = 20.0  # hours -- >> tau_meas_h, keeps T ~static during the decay window
    # lead_h large relative to q_val*tau_meas_h: switching q off perturbs
    # the bulk's own derivative by -q_val instantly (a*(t_out-T) alone,
    # independent of tau_out), which leaks a ~q_val*tau_meas_h contaminant
    # into (Tm - T)'s decay. Keeping lead_h*q_val >> q_val*tau_meas_h (i.e.
    # lead_h >> tau_meas_h) makes that leak << 1% of the signal so the
    # decay reads as the clean single exponential the node's own ODE
    # predicts in isolation.
    lead_h = 50.0
    q_val = 0.5
    t_out = 10.0
    dt_s = 10.0
    params = _symmetric_params_meas(
        tau_out=tau_out, tau_cpl=8.0, gain=0.0, tau_meas_h=tau_meas_h, lead_h=lead_h
    )
    house = House(params, {room: 18.0 for room in ROOMS}, dt_s=dt_s)

    q_on = {room: q_val for room in ROOMS}
    # Run to (near) full steady state first (bulk + node) so the decay
    # starts from a known, settled offset.
    for _ in range(round(8 * tau_out * 3600 / dt_s)):
        house.step(t_out=t_out, q=q_on, dt_s=dt_s)

    start_offset = {room: house.temps_measured[room] - house.temps[room] for room in ROOMS}

    q_off = {room: 0.0 for room in ROOMS}
    hours_elapsed = 0.0
    for _ in range(round(2 * tau_meas_h * 3600 / dt_s)):
        house.step(t_out=t_out, q=q_off, dt_s=dt_s)
        hours_elapsed += dt_s / 3600.0
        for room in ROOMS:
            expected_offset = start_offset[room] * math.exp(-hours_elapsed / tau_meas_h)
            actual_offset = house.temps_measured[room] - house.temps[room]
            assert actual_offset == pytest.approx(expected_offset, abs=0.01 * abs(start_offset[room]))


def test_measured_node_stability_guard_rejects_excessive_dt():
    """The Euler stability guard is extended to reject dt_h > tau_meas_h/2
    when the measured node is enabled, even if the bulk tau_out/tau_cpl
    would otherwise allow a larger step."""
    tau_meas_h = 0.01  # hours (36 s) -- deliberately tiny to trigger the guard
    params = _symmetric_params_meas(
        tau_out=500.0, tau_cpl=500.0, gain=0.0, tau_meas_h=tau_meas_h, lead_h=1.0
    )
    limit_h = max_stable_dt_h(params)
    assert limit_h == pytest.approx(tau_meas_h / 2.0)  # measured node is the binding constraint

    too_big_dt_s = (limit_h + 0.001) * 3600
    with pytest.raises(ValueError):
        House(params, {room: 20.0 for room in ROOMS}, dt_s=too_big_dt_s)

    ok_dt_s = (limit_h * 0.5) * 3600
    house = House(params, {room: 20.0 for room in ROOMS}, dt_s=ok_dt_s)
    house.step(t_out=15.0, dt_s=ok_dt_s)  # should not raise


# --- Orientation-resolved solar gains (docs/tasks/011-orientation-solar.md) -


def test_constant_sun_ne_produces_analytic_steady_state_offset():
    """With s_ne enabled on every room (s_nw at 0) and a constant sun_ne
    factor, the house settles to T = T_out + s_ne*sun_ne*tau -- the same
    constant-forcing fixed point as the existing constant-q test, just
    driven through the s_ne*sun_ne term instead of q."""
    tau = 12.0
    s_ne = 0.25
    sun_ne_val = 0.6
    t_out = 10.0
    params = HouseParams(
        rooms={
            room: RoomParams(tau_out=tau, tau_cpl=4.0, gain=0.0, s_ne=s_ne, s_nw=0.0)
            for room in ROOMS
        }
    )
    house = House(params, {room: 22.0 for room in ROOMS}, dt_s=10)

    steps = round(8 * tau * 3600 / 30)  # 8 time constants
    for _ in range(steps):
        house.step(t_out=t_out, dt_s=30, sun_ne=sun_ne_val, sun_nw=0.0)

    expected = t_out + s_ne * sun_ne_val * tau
    for room in ROOMS:
        assert house.temps[room] == pytest.approx(expected, abs=0.05)


def test_zero_s_ne_s_nw_ignores_sun_kwargs():
    """Regression: with every room's s_ne/s_nw explicitly zeroed, supplying
    nonzero sun_ne/sun_nw to step() must not perturb the trajectory at all
    -- it must match a run without those kwargs, bit for bit."""
    params = HouseParams(
        rooms={
            room: replace(p, s_ne=0.0, s_nw=0.0) for room, p in HouseParams().rooms.items()
        }
    )
    initial = {room: 18.0 + i for i, room in enumerate(ROOMS)}
    house_with_sun = House(params, dict(initial), dt_s=10)
    house_without_sun = House(params, dict(initial), dt_s=10)

    for i in range(500):
        q = {room: 0.05 * ((i + hash(room)) % 7 - 3) for room in ROOMS}
        t_out = 5.0 + (i % 11)
        house_with_sun.step(t_out=t_out, q=q, dt_s=10, sun_ne=0.7, sun_nw=0.4)
        house_without_sun.step(t_out=t_out, q=q, dt_s=10)
        assert house_with_sun.temps == house_without_sun.temps
