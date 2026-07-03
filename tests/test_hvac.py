"""Offline tests for sim.hvac (calibration-anchored HVAC model)."""

import pytest

from sim.hvac import DeadTimeLag, Defrost, FirstOrderLag, Hvac, HvacParams


@pytest.fixture
def hvac():
    return Hvac()


def test_power_endpoints(hvac):
    # Anchors: ~720 W total at min (665 W outdoor mean at speed<=1 incl.
    # purge/blip amortisation, 2026-07-03 energy refit + 55 W fan),
    # ~3055 W total at max (defaults).
    assert hvac.power_kw(0) == pytest.approx(0.720, abs=0.001)
    assert hvac.power_kw(14) == pytest.approx(3.055, abs=0.001)


def test_power_monotonic_and_clamped(hvac):
    powers = [hvac.power_kw(n) for n in range(15)]
    assert all(b > a for a, b in zip(powers, powers[1:]))
    assert hvac.power_kw(-3) == hvac.power_kw(0)
    assert hvac.power_kw(99) == hvac.power_kw(14)


def test_cop_matches_calibration_point(hvac):
    # At the fit reference (P=0.7 kW, Tout=10 C): e = e0 = 1.045 * 0.80
    # (open-loop fit level scaled by the 2026-07-03 closed-loop energy
    # refit), COP = e0 * 4.8 ~ 4.0.
    assert hvac.cop(0.7, 10.0) == pytest.approx(1.045 * 0.80 * 4.8, rel=1e-9)


def test_cop_falls_with_power_and_cold(hvac):
    assert hvac.cop(3.0, 10.0) < hvac.cop(0.7, 10.0)
    assert hvac.cop(0.7, 2.0) < hvac.cop(0.7, 10.0)


def test_efficiency_floor():
    hvac = Hvac(HvacParams(e_floor=0.2))
    # Absurd extrapolation stays at the floor, keeping COP positive.
    assert hvac.efficiency(10.0, -30.0) == 0.2


def test_house_q_units(hvac):
    # Min power at 10 C: P=0.665 kW, COP~5.02 -> Q~3.34 kW ->
    # q = Q/C ~ 0.70 K/h. Sanity: heats the ~30 h-tau house noticeably.
    q = hvac.house_q(0, 10.0)
    assert 0.5 < q < 0.9


def test_output_tuple_consistent(hvac):
    p_kw, q_kw = hvac.heating_output(7, 5.0)
    assert q_kw == pytest.approx(hvac.cop(p_kw, 5.0) * p_kw, rel=1e-12)


def test_first_order_lag_tracks_tau():
    # tau=20 s at dt=10 s: 1 - exp(-0.5) ~ 0.393 per step; ~95% by 60 s.
    lag = FirstOrderLag(20.0)
    assert lag.step(1.0, 10.0) == pytest.approx(0.393, abs=0.001)
    for _ in range(5):
        lag.step(1.0, 10.0)
    assert lag.value == pytest.approx(0.95, abs=0.03)
    lag.reset()
    assert lag.value == 0.0


def test_first_order_lag_passthrough_when_disabled():
    lag = FirstOrderLag(0.0, initial=5.0)
    assert lag.step(2.5, 10.0) == 2.5


def test_dead_time_lag_holds_during_dead_time():
    # dead=20s at cycle=10s -> 2 cycles of zero movement, then tau kicks in.
    lag = DeadTimeLag(dead_s=20.0, tau_s=20.0, cycle_s=10.0)
    assert lag.step(1.0) == 0.0
    assert lag.step(1.0) == 0.0
    assert lag.step(1.0) == pytest.approx(0.393, abs=0.001)


def test_dead_time_lag_zero_dead_time_is_plain_lag():
    lag = DeadTimeLag(dead_s=0.0, tau_s=20.0, cycle_s=10.0)
    assert lag.step(1.0) == pytest.approx(0.393, abs=0.001)


def test_dead_time_lag_reset():
    lag = DeadTimeLag(dead_s=20.0, tau_s=20.0, cycle_s=10.0)
    lag.step(1.0)
    lag.reset()
    assert lag.step(1.0) == 0.0
    assert lag.step(1.0) == 0.0


def test_defrost_triggers_after_cold_running_time():
    params = HvacParams(defrost_trigger_s=100.0, defrost_duration_s=30.0)
    dfr = Defrost(params)
    triggered = [dfr.step(True, 5.0, 10.0) for _ in range(10)]
    assert not any(triggered[:9])
    assert triggered[9]
    assert dfr.active


def test_defrost_does_not_trigger_when_warm_or_off():
    params = HvacParams(defrost_trigger_s=50.0, defrost_duration_s=30.0)
    dfr = Defrost(params)
    for _ in range(20):
        assert not dfr.step(True, 20.0, 10.0)  # too warm
    for _ in range(20):
        assert not dfr.step(False, 2.0, 10.0)  # not running


def test_defrost_ends_after_duration_and_resets_accumulator():
    params = HvacParams(defrost_trigger_s=20.0, defrost_duration_s=20.0)
    dfr = Defrost(params)
    dfr.step(True, 5.0, 10.0)
    assert dfr.step(True, 5.0, 10.0)  # triggers on the 2nd cycle
    assert dfr.active
    dfr.step(True, 5.0, 10.0)  # remaining 20s -> 10s
    assert dfr.active
    dfr.step(True, 5.0, 10.0)  # remaining 10s -> 0s
    assert not dfr.active
    # accumulator reset: needs a fresh full run before triggering again
    assert not dfr.step(True, 5.0, 10.0)


def test_defrost_disabled_when_trigger_s_not_positive():
    params = HvacParams(defrost_trigger_s=0.0)
    dfr = Defrost(params)
    for _ in range(1000):
        assert not dfr.step(True, 2.0, 10.0)
