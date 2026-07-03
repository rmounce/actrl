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
# Solar gain [K/h per kW of PV-output proxy], fitted by analysis/solar_fit.py
# (2026-07-03) on unit-off residuals against Solcast power_pv_5m. Kitchen's
# raw fit came out slightly negative (coupling-term cross-talk from sunlit
# bed_3, not physics) and it already replays at ~0.4 C RMSE, so it is
# clamped to zero. Zero PV input (the default) reproduces the night fit.
_DEFAULT_SOLAR = {
    "bed_1": 0.031,
    "bed_2": 0.029,
    "bed_3": 0.133,
    "kitchen": 0.0,
    "study": 0.042,
}


@dataclass(frozen=True)
class RoomParams:
    """One room's RC parameters. tau_out/tau_cpl in hours, gain in K/h."""

    tau_out: float
    tau_cpl: float
    gain: float
    solar: float = 0.0  # K/h per kW of PV-output proxy (analysis/solar_fit.py)
    # Measured-air lead node (docs/tasks/009-measured-air-node.md): the
    # room sensor sits in a small fast air mass that leads the bulk room
    # temperature while HVAC heat is being delivered, then relaxes back to
    # the bulk temperature once the unit stops. tau_meas_h == 0.0 disables
    # the node (Tm == T identically, exact passthrough). Not fitted here --
    # defaults stay 0.0 until a separate calibration commit lands values.
    tau_meas_h: float = 0.0
    lead_h: float = 0.0

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
    ) -> dict[str, float]:
        """Advance the model by dt_s seconds; returns the updated temps dict.

        q: optional per-room heat input [K/h], defaults to 0 for all rooms
        (free-running).
        pv_kw: PV-output irradiance proxy [kW] driving per-room solar gain
        (0 = night / not modelled, reproducing the pre-solar behaviour).
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
                dTmdt = ((t_i + p.lead_h * q_i) - tm_i) / p.tau_meas_h
                new_temps_measured[room] = tm_i + dt_h * dTmdt
            else:
                new_temps_measured[room] = new_temps[room]
        self.temps = new_temps
        self.temps_measured = new_temps_measured
        return self.temps
