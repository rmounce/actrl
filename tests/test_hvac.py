"""Offline tests for sim.hvac (calibration-anchored HVAC model)."""

import pytest

from sim.hvac import FirstOrderLag, Hvac, HvacParams


@pytest.fixture
def hvac():
    return Hvac()


def test_power_endpoints(hvac):
    # Anchors: ~665 W total at min, ~3055 W total at max (defaults).
    assert hvac.power_kw(0) == pytest.approx(0.665, abs=0.001)
    assert hvac.power_kw(14) == pytest.approx(3.055, abs=0.001)


def test_power_monotonic_and_clamped(hvac):
    powers = [hvac.power_kw(n) for n in range(15)]
    assert all(b > a for a, b in zip(powers, powers[1:]))
    assert hvac.power_kw(-3) == hvac.power_kw(0)
    assert hvac.power_kw(99) == hvac.power_kw(14)


def test_cop_matches_calibration_point(hvac):
    # At the fit reference (P=0.7 kW, Tout=10 C): e = e0 = 1.045,
    # COP = 1.045 * 4.8 ~ 5.0 (docs/calibration.md).
    assert hvac.cop(0.7, 10.0) == pytest.approx(1.045 * 4.8, rel=1e-9)


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
