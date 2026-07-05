"""Per-room RC thermal model of the house.

Parameterised from the June system-identification results in
docs/calibration.md (read it first; reproduced with analysis/sysid_june.py).
This is the plant model the simulator (docs/ideas.md #3) drives with the
controller + Midea unit emulator.

Model, per room i (rooms: bed_1, bed_2, bed_3, kitchen, study)::

    dT_i/dt = a_i * (T_out - T_i) + c_i * (T_others_i - T_i) + g_i + q_i(t)

- ``T_others_i`` = mean of the other four rooms' temperatures.
- ``a_i = 1 / tau_out_i``, ``c_i = 1 / tau_cpl_i`` [1/h], ``g_i`` [K/h] are
  the per-room table values from docs/calibration.md, baked into
  ``HouseParams`` as defaults (every value overridable).
- ``q_i(t)`` [K/h] is externally supplied heat input per unit room thermal
  mass -- zero when free-running; the HVAC model supplies it later.

Unit convention: all rates (``a``, ``c``, ``g``, ``q``) are in units of
K per **hour**. Time internally is tracked in hours. ``step()`` takes a
wall-clock step in **seconds** (default 10 s, the control interval) and
converts to hours before integrating, so callers work in seconds/degrees C
throughout -- only the internal rate constants are per-hour.

Integration is simple forward Euler:

    T_i(t + dt) = T_i(t) + dt_h * dT_i/dt(t)

Pure Python + stdlib only; no pandas here (analysis scripts may use it).
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace

ROOMS = ("bed_1", "bed_2", "bed_3", "kitchen", "study")

# Per-room two-node fit defaults from docs/calibration.md (2026-07-02,
# June 2026 archive, winter/heating season):
#   room     tau_out(h)  tau_cpl(h)  gains(K/h)
#   bed_1        24.7        9.0       +0.04
#   bed_2        30.5       45.1       +0.00
#   bed_3        26.4        9.8       -0.04
#   kitchen     118.6        2.9       +0.20
#   study        33.9       19.3       +0.02
_DEFAULT_TAU_OUT = {
    "bed_1": 24.7,
    "bed_2": 30.5,
    "bed_3": 26.4,
    "kitchen": 118.6,
    "study": 33.9,
}
_DEFAULT_TAU_CPL = {
    "bed_1": 9.0,
    "bed_2": 45.1,
    "bed_3": 9.8,
    "kitchen": 2.9,
    "study": 19.3,
}
_DEFAULT_GAIN = {
    "bed_1": 0.04,
    "bed_2": 0.00,
    "bed_3": -0.04,
    "kitchen": 0.20,
    "study": 0.02,
}
# Solar gain [K/h per kW of PV-output proxy], originally fitted by
# analysis/solar_fit.py (2026-07-03) on unit-off residuals against Solcast
# power_pv_5m. Superseded by the orientation-resolved s_ne/s_nw terms below
# (docs/tasks/011-orientation-solar.md, 2026-07-03): the whole-house PV
# proxy peaks at noon and cannot represent NE-window morning beam vs
# NW-window afternoon beam, which the per-room replay-bias profiles showed
# mattered (bed_2/bed_3 ran 3-4.6 K cold at 09:00-12:00). Zeroed here; the
# `solar`/`pv_kw` machinery is kept in place (inert by default) rather than
# removed, in case a whole-house-scale diffuse/internal-gain term is wanted
# later.
_DEFAULT_SOLAR = {
    "bed_1": 0.0,
    "bed_2": 0.0,
    "bed_3": 0.0,
    "kitchen": 0.0,
    "study": 0.0,
}
# Orientation-resolved clear-sky solar terms [K/h at clear-sky full sun on
# that face], fitted by analysis/solar_orient_fit.py (trajectory-space fit
# against unit-off residuals, 2026-07-03; see docs/tasks/011-orientation-
# solar.md and docs/calibration.md). Driven by House.step's sun_ne/sun_nw
# kwargs (cloud-scaled irradiance factors, 0..1) rather than pv_kw.
_DEFAULT_S_NE = {
    "bed_1": 0.265,
    "bed_2": 0.672,
    "bed_3": 1.277,
    "kitchen": 0.355,
    "study": 0.259,
}
_DEFAULT_S_NW = {
    "bed_1": 0.024,
    "bed_2": 0.0,
    "bed_3": 0.0,
    "kitchen": 0.0,
    "study": 0.084,
}
# Measured-air lead node. tau_meas and the bedroom leads fitted by
# analysis/fast_node_fit.py (2026-07-03) from superposed no-sun unit-stop
# transients (sag amplitude/decay of the measured temp above the RC
# trend); bed_3's own fit is solar-contaminated (wrong sign, n=4) and
# study has no clean events — both take the pooled bedroom prior.
# The leads are refit in closed loop (grid search on 2026-06-22/27
# replays, matching recorded min-power cycle counts/on-fractions, morning
# warmup behaviour and RMSE): the open-loop stop-fit is biased ~2-5x low —
# its slow-trend subtraction assumes the node settles inside the 45 min
# window, and its q_pre denominator rides the COP model. Kitchen 0.45
# reproduces the recorded midday cycling exactly (4/4 starts) where 0.40
# does not (3/4); bedroom leads at 5x their stop-fit values (~0.35-0.40,
# consistent with the kitchen — same ducts/sensors) fix the 06-27 morning
# warmup (early-window on-fraction 1.00 vs recorded 0.79, peak increment
# 2 vs recorded 3-5, where the stop-fit values gave 0.44 and 11) and pull
# the 22nd's energy match to -1%. Both transitions are sharp (bed scale
# 4x behaves like 3x, 5x flips) — these are loop thresholds, treat the
# values as a set. Cycle/warmup texture is prioritised over exact energy:
# it is what the comfort/tuning work needs the sim to get right.
_DEFAULT_TAU_MEAS = {
    "bed_1": 0.25,
    "bed_2": 0.28,
    "bed_3": 0.25,
    "kitchen": 0.21,
    "study": 0.25,
}
_DEFAULT_LEAD = {
    "bed_1": 0.35,
    "bed_2": 0.40,
    "bed_3": 0.40,
    "kitchen": 0.45,
    "study": 0.40,
}
# q-sensitivity of the lead amplitude: lead_eff = lead_h * (1 + lead_q_h * q).
# The constant leads above were fitted on midday MIN-POWER cycling; recorded
# cold-morning warmup onsets show the kitchen sensor running 0.3-1 K/h ahead
# of the sim's — a lead that grows with delivered q (docs/calibration.md
# "kitchen cold-morning warmup onset"). 0.0 = the pre-existing linear model.
_DEFAULT_LEAD_Q = {
    "bed_1": 0.0,
    "bed_2": 0.0,
    "bed_3": 0.0,
    # 0.2 fitted 2026-07-06 (analysis/kitchen_onset_refit.py) against the
    # cold-morning warmup onsets: kitchen day RMSE 0.356->0.334, onset gap
    # -25%, mild-day control and the 06-22 cycling windows unchanged.
    # Larger values trade mild-day fidelity away without closing the rest
    # of the gap -- the remainder is controller integral-state divergence,
    # not sensor physics (docs/calibration.md "kitchen cold-morning
    # warmup onset", 2026-07-06 update).
    "kitchen": 0.2,
    "study": 0.0,
}
# q-sensitivity of the sensor-pocket coupling speed (see RoomParams.tau_q_h).
_DEFAULT_TAU_Q = {
    "bed_1": 0.0,
    "bed_2": 0.0,
    "bed_3": 0.0,
    "kitchen": 0.0,
    "study": 0.0,
}


@dataclass(frozen=True)
class RoomParams:
    """One room's RC parameters. tau_out/tau_cpl in hours, gain in K/h."""

    tau_out: float
    tau_cpl: float
    gain: float
    solar: float = 0.0  # K/h per kW of PV-output proxy (analysis/solar_fit.py); superseded, kept inert
    # Orientation-resolved clear-sky solar terms [K/h at clear-sky full sun],
    # driven by House.step's sun_ne/sun_nw kwargs (docs/tasks/011).
    s_ne: float = 0.0
    s_nw: float = 0.0
    # Measured-air lead node (docs/tasks/009-measured-air-node.md): the
    # room sensor sits in a small fast air mass that leads the bulk room
    # temperature while HVAC heat is being delivered, then relaxes back to
    # the bulk temperature once the unit stops. tau_meas_h == 0.0 disables
    # the node (Tm == T identically, exact passthrough). Not fitted here --
    # defaults stay 0.0 until a separate calibration commit lands values.
    tau_meas_h: float = 0.0
    lead_h: float = 0.0
    # lead_eff = lead_h * (1 + lead_q_h * q_i): the sensor's lead amplitude
    # grows with delivered heat (0.0 = plain linear lead, exact pre-existing
    # behaviour). Fitted only where warmup-onset data demanded it.
    lead_q_h: float = 0.0
    # tau_eff = tau_meas_h / (1 + tau_q_h * q_i): the sensor pocket couples
    # faster the harder air is being delivered (high fan). 0.0 = constant
    # tau, exact pre-existing behaviour.
    tau_q_h: float = 0.0

    @property
    def a(self) -> float:
        """Outdoor-coupling rate 1/tau_out [1/h]."""
        return 1.0 / self.tau_out

    @property
    def c(self) -> float:
        """Inter-room coupling rate 1/tau_cpl [1/h]."""
        return 1.0 / self.tau_cpl

    @property
    def tau_eff(self) -> float:
        """Combined effective time constant 1/(a+c) [h]."""
        return 1.0 / (self.a + self.c)


def _default_rooms() -> dict[str, RoomParams]:
    return {
        room: RoomParams(
            tau_out=_DEFAULT_TAU_OUT[room],
            tau_cpl=_DEFAULT_TAU_CPL[room],
            gain=_DEFAULT_GAIN[room],
            solar=_DEFAULT_SOLAR[room],
            s_ne=_DEFAULT_S_NE[room],
            s_nw=_DEFAULT_S_NW[room],
            tau_meas_h=_DEFAULT_TAU_MEAS[room],
            lead_h=_DEFAULT_LEAD[room],
            lead_q_h=_DEFAULT_LEAD_Q[room],
            tau_q_h=_DEFAULT_TAU_Q[room],
        )
        for room in ROOMS
    }


@dataclass(frozen=True)
class HouseParams:
    """Full house parameterisation: one RoomParams per room in ROOMS.

    Defaults are the docs/calibration.md fit. Override individual rooms via
    ``replace_room`` or by constructing with a custom ``rooms`` dict.
    """

    rooms: dict[str, RoomParams] = field(default_factory=_default_rooms)

    def __post_init__(self) -> None:
        missing = set(ROOMS) - set(self.rooms)
        if missing:
            raise ValueError(f"HouseParams missing rooms: {sorted(missing)}")

    def replace_room(self, room: str, **kwargs: float) -> "HouseParams":
        """Return a new HouseParams with one room's parameters overridden."""
        if room not in self.rooms:
            raise KeyError(room)
        new_rooms = dict(self.rooms)
        new_rooms[room] = replace(new_rooms[room], **kwargs)
        return HouseParams(rooms=new_rooms)

    def min_tau_eff(self) -> float:
        """Smallest combined effective time constant across rooms [h]."""
        return min(p.tau_eff for p in self.rooms.values())

    def min_tau_meas(self) -> float | None:
        """Smallest enabled measured-node time constant across rooms [h],
        or None if every room has the node disabled (tau_meas_h == 0.0)."""
        active = [p.tau_meas_h for p in self.rooms.values() if p.tau_meas_h > 0.0]
        return min(active) if active else None


def max_stable_dt_h(params: HouseParams) -> float:
    """Largest step (hours) that satisfies this module's stability guard.

    Forward Euler for dT/dt = -k*T + const is numerically stable for
    dt < 2/k = 2*tau_eff. We guard well inside that with a dt <= tau_eff/2
    threshold (per room, tightest room wins) so a caller has real margin
    rather than sitting on the stability boundary. The measured-air lead
    node (tau_meas_h, when enabled) is integrated forward-Euler alongside
    the bulk state at the same dt, so its own tau/2 threshold is folded in
    too (rooms with the node disabled, tau_meas_h == 0.0, are excluded --
    they contribute an exact passthrough, not a fast ODE).
    """
    limit_h = params.min_tau_eff() / 2.0
    min_tau_meas = params.min_tau_meas()
    if min_tau_meas is not None:
        limit_h = min(limit_h, min_tau_meas / 2.0)
    return limit_h


class House:
    """Deterministic per-room RC thermal model, forward-Euler integrated."""

    def __init__(
        self,
        params: HouseParams,
        initial_temps: dict[str, float],
        dt_s: float = 10.0,
    ) -> None:
        missing = set(ROOMS) - set(initial_temps)
        if missing:
            raise ValueError(f"initial_temps missing rooms: {sorted(missing)}")
        self.params = params
        self._check_dt(dt_s)
        self.temps: dict[str, float] = {room: float(initial_temps[room]) for room in ROOMS}
        # Measured-air lead node state, initialised equal to the bulk temp
        # (docs/tasks/009-measured-air-node.md). Disabled rooms
        # (tau_meas_h == 0.0) are kept exactly equal to self.temps by
        # step() below -- an exact passthrough, not a small-tau
        # approximation.
        self.temps_measured: dict[str, float] = dict(self.temps)

    def _check_dt(self, dt_s: float) -> None:
        if dt_s <= 0:
            raise ValueError(f"dt_s must be positive, got {dt_s}")
        dt_h = dt_s / 3600.0
        limit_h = max_stable_dt_h(self.params)
        if dt_h > limit_h:
            raise ValueError(
                f"dt_s={dt_s}s ({dt_h:.4f} h) exceeds the Euler stability "
                f"guard of tau_eff/2={limit_h:.4f} h for these parameters "
                "(tightest room's combined tau_out/tau_cpl is too short "
                "for this step size)"
            )

    def step(
        self,
        t_out: float,
        q: dict[str, float] | None = None,
        dt_s: float = 10.0,
        pv_kw: float = 0.0,
        sun_ne: float = 0.0,
        sun_nw: float = 0.0,
    ) -> dict[str, float]:
        """Advance the model by dt_s seconds; returns the updated temps dict.

        q: optional per-room heat input [K/h], defaults to 0 for all rooms
        (free-running).
        pv_kw: PV-output irradiance proxy [kW] driving per-room solar gain
        (0 = night / not modelled, reproducing the pre-solar behaviour);
        superseded by sun_ne/sun_nw (see RoomParams.s_ne/s_nw), kept inert
        by default (_DEFAULT_SOLAR == 0).
        sun_ne, sun_nw: cloud-scaled clear-sky irradiance factors (0..1) on
        the house's NE/NW vertical faces (sim/solar.py), driving each
        room's s_ne/s_nw orientation-resolved solar gain. Defaults of 0
        reproduce the pre-orientation-solar behaviour exactly.
        """
        self._check_dt(dt_s)
        dt_h = dt_s / 3600.0
        q = q or {}
        current = self.temps
        current_measured = self.temps_measured
        new_temps: dict[str, float] = {}
        new_temps_measured: dict[str, float] = {}
        for room in ROOMS:
            p = self.params.rooms[room]
            t_i = current[room]
            q_i = q.get(room, 0.0)
            others_mean = sum(current[r] for r in ROOMS if r != room) / (len(ROOMS) - 1)
            dTdt = (
                p.a * (t_out - t_i)
                + p.c * (others_mean - t_i)
                + p.gain
                + p.solar * pv_kw
                + p.s_ne * sun_ne + p.s_nw * sun_nw
                + q_i
            )
            new_temps[room] = t_i + dt_h * dTdt

            # Measured-air lead node: a sensor model layered on top of the
            # (unchanged) bulk state above -- carries no energy, doesn't
            # feed back into the bulk ODE. tau_meas_h == 0.0 disables it:
            # exact passthrough (Tm == T identically), not a small-tau
            # approximation.
            if p.tau_meas_h > 0.0:
                tm_i = current_measured[room]
                lead_eff = p.lead_h * (1.0 + p.lead_q_h * q_i)
                tau_eff = p.tau_meas_h / (1.0 + p.tau_q_h * q_i)
                dTmdt = ((t_i + lead_eff * q_i) - tm_i) / tau_eff
                new_temps_measured[room] = tm_i + dt_h * dTmdt
            else:
                new_temps_measured[room] = new_temps[room]
        self.temps = new_temps
        self.temps_measured = new_temps_measured
        return self.temps
