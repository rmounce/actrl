"""HVAC performance model: compressor increment -> electrical power and
delivered heat, for the closed-loop simulator (docs/ideas.md #3).

Calibration provenance (2026-07-02, June archive + Actron rated tables —
see docs/calibration.md and analysis/actron_tables.py):

- Electrical power vs increment: linear between two solid anchors —
  observed min-power outdoor draw ~610 W at n=0 and ~2950 W near rated
  (table rated input 3.08 kW at 4 C). Mid-range is sparsely observed
  (the controller deliberately sits at min power 93% of steady time), so
  the linear interpolation is an assumption; refine when more mid-speed
  data accumulates. Indoor fan adds ~55-105 W (fan mode follows speed).
- COP: the fitted efficiency shape e(P_kW, T_out) [K/h per kW] from
  docs/calibration.md (no-sun, defrost-excised: e = 1.045
  - 0.101*(P-0.7) + 0.0522*(Tout-10)) times the effective house thermal
  capacity C_eff ~ 4.8 kWh/K. COP here is referenced to heat delivered
  INTO THE ROOMS (duct losses folded in), which is what the house model
  wants. Winter/heating fit only — cooling uses the rated tables as a
  placeholder until summer data exists.
- Defrost is NOT modelled here (the emulator doesn't model it either);
  the ~6% cold-morning energy overhead (docs/calibration.md) must be
  added externally when comparing simulated vs recorded energy for cold
  pre-dawn runs.
- Time delay (electrical): power follows an increment command as a single
  first-order lag with tau ~ 20 s (analysis/lag_fit.py + superposed-epoch
  average of 84 clean +/-1 steps in June: 67% at 10 s, ~90% at 60 s,
  settled by ~3 min). Shutdowns are instant cuts in the recorded traces,
  so the lag applies to running changes only — the caller resets it on
  power-off.
- Time delay (heat delivery): the "elbow" a compressor-speed change
  produces in room temperature, minutes after the power step, lags
  further behind (analysis/heat_lag_fit.py, 2026-07-03 — Ryan's
  observation). Fit as dead time + first-order lag on delivered heat Q
  from the same clean +/-1 steps, using kitchen (the open zone, best
  house-average proxy, cleanest fit: r2=0.95): dead time ~15 s, tau
  ~180 s. This bundles refrigerant/coil dynamics, duct transport, and
  average_temperature sensor smoothing — not separately identifiable
  from this data, so it's applied as one lumped lag on Q, downstream of
  the electrical lag. Per-room fits vary a lot (bed_2 essentially flat,
  r2=0.12 — consistent with its known weak/uncertain heat-split share)
  so the model uses the pooled/kitchen number rather than per room.

Stdlib only.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class HvacParams:
    max_increment: int = 14
    # Electrical power [W]: outdoor unit linear in increment.
    p_outdoor_min_w: float = 610.0
    p_outdoor_max_w: float = 2950.0
    # Indoor fan [W], linear in increment (fan mode follows comp speed).
    p_indoor_min_w: float = 55.0
    p_indoor_max_w: float = 105.0
    # Heating efficiency proxy e(P_kW, Tout) [K/h per kW], docs/calibration.md.
    e0: float = 1.045
    e_per_kw: float = -0.101
    e_per_k: float = 0.0522
    e_ref_p_kw: float = 0.7
    e_ref_tout: float = 10.0
    # Effective house thermal capacity [kWh/K], docs/calibration.md anchor.
    c_eff_kwh_per_k: float = 4.8
    # Floor for the efficiency proxy: keeps COP positive if extrapolated
    # far outside the fitted envelope (e.g. very cold + max power).
    e_floor: float = 0.2
    # Compressor spin-up: first-order lag of electrical power behind the
    # commanded increment [s] (analysis/lag_fit.py). 0 disables.
    lag_tau_s: float = 20.0
    # Heat-delivery lag: dead time + first-order lag on Q, downstream of the
    # electrical lag (analysis/heat_lag_fit.py). 0 tau disables the lag part;
    # 0 dead time disables the delay part.
    q_lag_dead_s: float = 15.0
    q_lag_tau_s: float = 180.0


class FirstOrderLag:
    """y follows target with time constant tau_s; exact discrete update so
    any dt_s is stable. tau_s <= 0 means pass-through."""

    def __init__(self, tau_s: float, initial: float = 0.0):
        self.tau_s = tau_s
        self.value = initial

    def step(self, target: float, dt_s: float) -> float:
        if self.tau_s <= 0:
            self.value = target
        else:
            alpha = 1.0 - math.exp(-dt_s / self.tau_s)
            self.value += alpha * (target - self.value)
        return self.value

    def reset(self, value: float = 0.0) -> None:
        self.value = value


class DeadTimeLag:
    """Fixed dead time followed by a first-order lag: y tracks target
    delayed by dead_s and then smoothed with time constant tau_s. Assumes a
    constant step interval (cycle_s), matching the closed loop's fixed 10 s
    cycle — the dead time is quantised to whole cycles."""

    def __init__(self, dead_s: float, tau_s: float, cycle_s: float, initial: float = 0.0):
        self.dead_s = dead_s
        self.tau_s = tau_s
        self.cycle_s = cycle_s
        n_delay = max(0, round(dead_s / cycle_s)) if cycle_s > 0 else 0
        self._buffer: list[float] = [initial] * n_delay
        self._lag = FirstOrderLag(tau_s, initial)

    def step(self, target: float) -> float:
        if self._buffer:
            self._buffer.append(target)
            delayed = self._buffer.pop(0)
        else:
            delayed = target
        return self._lag.step(delayed, self.cycle_s)

    def reset(self, value: float = 0.0) -> None:
        self._buffer = [value] * len(self._buffer)
        self._lag.reset(value)


class Hvac:
    """Maps compressor increment + outdoor temp to (P_elec_kW, Q_kW).

    Heating only for now (the calibrated season). Q is heat delivered to
    the rooms (duct losses folded into the calibration), so
    q_house [K/h] = Q_kW / c_eff_kwh_per_k feeds sim.house directly.
    """

    def __init__(self, params: HvacParams = HvacParams()):
        self.params = params

    def power_kw(self, increment: float) -> float:
        p = self.params
        n = max(0.0, min(float(increment), float(p.max_increment)))
        frac = n / p.max_increment
        outdoor = p.p_outdoor_min_w + frac * (p.p_outdoor_max_w - p.p_outdoor_min_w)
        indoor = p.p_indoor_min_w + frac * (p.p_indoor_max_w - p.p_indoor_min_w)
        return (outdoor + indoor) / 1000.0

    def efficiency(self, p_kw: float, t_out: float) -> float:
        """Fitted efficiency proxy e [K/h per kW] (heating)."""
        p = self.params
        e = (
            p.e0
            + p.e_per_kw * (p_kw - p.e_ref_p_kw)
            + p.e_per_k * (t_out - p.e_ref_tout)
        )
        return max(p.e_floor, e)

    def cop(self, p_kw: float, t_out: float) -> float:
        return self.efficiency(p_kw, t_out) * self.params.c_eff_kwh_per_k

    def heating_output(self, increment: float, t_out: float) -> tuple[float, float]:
        """(electrical power kW, delivered heat kW) when compressor runs at
        `increment` with outdoor temperature `t_out`. increment <= 0 or the
        unit being off should be handled by the caller (returns min-power
        numbers for increment 0 because the compressor is still running)."""
        p_kw = self.power_kw(increment)
        q_kw = self.cop(p_kw, t_out) * p_kw
        return p_kw, q_kw

    def house_q(self, increment: float, t_out: float) -> float:
        """Delivered heat as a house-average temperature rate [K/h] —
        directly usable as the total q for sim.house."""
        _, q_kw = self.heating_output(increment, t_out)
        return q_kw / self.params.c_eff_kwh_per_k
