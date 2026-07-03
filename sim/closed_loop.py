"""Closed-loop simulator: real actrl control logic driving the simulated
house through the simulated Midea unit (docs/ideas.md #3 "Harness").

Wiring, once per 10 s control cycle:

    actrl (HarnessActrl in a FakeWorld)
      reads room temps  <-------------------------.
      writes follow-me report + damper positions   |
        |                                          |
        v                                          |
    sim.midea_unit.MideaUnit.step(report)          |
        -> compressor increment                    |
        v                                          |
    sim.hvac.Hvac -> (P_elec kW, Q kW)             |
        Q split per room by damper x airflow share |
        v                                          |
    sim.house.House.step(t_out, q) -> room temps --'

Heat split assumption: delivered airflow share and room thermal mass are
two separate knobs, q_i = q_house * flow_share_i / mass_share_i:

- flow_share_i (AIRFLOW_WEIGHTS): proportional to duct count (kitchen 2.0 =
  two ducts, others 1.0) — confirmed against the real duct layout
  (2026-07-03).
- mass_share_i (MASS_WEIGHTS): each room's fraction of the house's
  aggregate thermal capacity C_eff, used only to convert delivered kW into
  that room's own K/h forcing (the room's *dynamics* — how fast a given
  K/h moves its temperature — are separately, independently fit per room
  in sim/house.py's tau_out/tau_cpl; this weight only affects the split of
  incoming heat). For bed_1/bed_2/bed_3/study, set from real floor areas
  (Energy and Outdoor Design report, EOD-3341, 2020) normalised so their
  average is 1.0 — previously all four were assumed equal (1.0), which
  overstated study's and bed_2's mass relative to bed_1/bed_3 by ~2x
  (docs/calibration.md "Whole-day closed-loop replay"). Kitchen is left at
  2.0 (its RC fit is separately anchored and already close to the
  living-area floor-area ratio, ~2.3x a bedroom).

This module imports the test harness (tests/hvac_harness.py) for the
FakeWorld/HarnessActrl shell — a deliberate reuse of the golden-test
infrastructure; the sys.path insertion below is the price of keeping the
harness where the tests live.
"""
from __future__ import annotations

import sys
from pathlib import Path
from unittest import mock

_ROOT = Path(__file__).resolve().parent.parent
for p in (str(_ROOT), str(_ROOT / "tests")):
    if p not in sys.path:
        sys.path.insert(0, p)

import hvac_harness  # noqa: E402  (tests/hvac_harness.py)
import actrl  # noqa: E402
from sim.house import House, HouseParams, ROOMS  # noqa: E402
from sim.hvac import DeadTimeLag, Defrost, FirstOrderLag, Hvac  # noqa: E402
from sim.midea_unit import MideaUnit  # noqa: E402

AIRFLOW_WEIGHTS = {"bed_1": 1.0, "bed_2": 1.0, "bed_3": 1.0, "study": 1.0, "kitchen": 2.0}
# Floor areas [m2] from EOD-3341 (Energy and Outdoor Design, 2020), zones
# "Bedroom 1 + WIR", "Bedroom 2", "Bedroom 3", "Study"; normalised so the
# four rooms' average weight is 1.0 (matching the old equal-weight scale).
MASS_WEIGHTS = {"bed_1": 1.395, "bed_2": 0.893, "bed_3": 1.070, "study": 0.642, "kitchen": 2.0}
FOLLOW_ME_SERVICE = "esphome/m5atom_send_follow_me"
UNIT_CLIMATE = "climate.m5atom_climate"


def _entity_float(world, entity_id: str) -> float:
    """State of a numeric world entity, NaN when unset/non-numeric."""
    try:
        return float(world.entities[entity_id]["state"])
    except (KeyError, TypeError, ValueError):
        return float("nan")


class ClosedLoop:
    """One assembled simulation run."""

    def __init__(
        self,
        initial_world: dict,
        initial_temps: dict[str, float],
        setpoint: float,
        house_params: HouseParams | None = None,
        hvac: Hvac | None = None,
        unit: MideaUnit | None = None,
    ):
        self.world = hvac_harness.FakeWorld(initial_world)
        self.app = hvac_harness.HarnessActrl(self.world)
        self.house = House(house_params or HouseParams(), initial_temps)
        self.hvac = hvac or Hvac()
        self.unit = unit or MideaUnit(setpoint=setpoint)
        # Compressor spin-up: power tracks the commanded increment with a
        # ~20 s lag (sim/hvac.py docstring); shutdown is an instant cut.
        self._p_lag = FirstOrderLag(self.hvac.params.lag_tau_s)
        # Heat-delivery lag: delivered Q lags the (already-lagged) power by
        # a further dead time + first-order lag (sim/hvac.py docstring).
        self._q_lag = DeadTimeLag(
            self.hvac.params.q_lag_dead_s, self.hvac.params.q_lag_tau_s, cycle_s=10.0
        )
        # Defrost: periodic zero-heat episodes at cold outdoor temps
        # (sim/hvac.py docstring); not reset alongside the lags below since
        # frost accumulation is a slower, independent process.
        self._defrost = Defrost(self.hvac.params)
        self.cycle = -1
        # Controller-input offsets: production actrl actuates on per-room
        # "feels like" (apparent temperature, packages/aircon.yaml: T +
        # 0.33*wvp - 4.0), not raw average_temperature — and the recorded
        # room targets are in feels-like units too. The house model
        # simulates physical temperature; this per-room additive offset
        # (typically ~ -0.4..-0.5 K at winter indoor humidity) converts it
        # to what the controller sees. Set via step(ctrl_offsets=...);
        # empty (the default) reproduces the raw-temperature behaviour.
        self._ctrl_offsets: dict[str, float] = {}
        # Telemetry: one row per cycle.
        self.history: list[dict] = []

        self._sleep_patch = mock.patch.object(actrl.time, "sleep", lambda s: None)
        self._sleep_patch.start()
        self.app.initialize()

    def _write_room_temps(self):
        # The real controller reads the real sensors, which are the
        # measured-air lead node (sim/house.py, docs/tasks/009), not the
        # bulk RC temperature -- equal when the node is disabled.
        for room in ROOMS:
            t = self.house.temps_measured[room] + self._ctrl_offsets.get(room, 0.0)
            self.world.update(
                f"sensor.{room}_average_temperature", {"state": f"{t:.2f}"}
            )

    def _follow_me_reports(self, journal_slice):
        return [
            e["data"]["temperature"]
            for e in journal_slice
            if e.get("service") == FOLLOW_ME_SERVICE
        ]

    def _damper_positions(self):
        out = {}
        for room in ROOMS:
            attrs = self.world.entities.get(f"cover.{room}", {}).get("attributes", {})
            out[room] = float(attrs.get("current_position", 0.0)) / 100.0
        return out

    def _room_q(self, q_house: float) -> dict[str, float]:
        dampers = self._damper_positions()
        sum_mass = sum(MASS_WEIGHTS.values())
        flows = {r: dampers[r] * AIRFLOW_WEIGHTS[r] for r in ROOMS}
        total_flow = sum(flows.values())
        if total_flow <= 0:
            return {r: 0.0 for r in ROOMS}
        return {
            r: q_house * (flows[r] / total_flow) / (MASS_WEIGHTS[r] / sum_mass)
            for r in ROOMS
        }

    def step(
        self,
        t_out: float,
        updates: dict | None = None,
        pv_kw: float = 0.0,
        ctrl_offsets: dict[str, float] | None = None,
    ) -> dict:
        """Advance one 10 s cycle. `updates` = extra world-entity updates
        applied before actrl runs (setpoint changes etc.); `pv_kw` = PV
        irradiance proxy for the house solar-gain term; `ctrl_offsets` =
        per-room feels-like minus physical-temperature offsets applied to
        what the controller reads (see __init__)."""
        self.cycle += 1
        self.world.cycle = self.cycle
        if ctrl_offsets is not None:
            self._ctrl_offsets = ctrl_offsets
        self._write_room_temps()
        for eid, entry in (updates or {}).items():
            self.world.update(eid, entry)

        j0 = len(self.world.journal)
        self.app.main({})
        journal_slice = self.world.journal[j0:]

        # Unit side: mode from the climate entity actrl controls; feed every
        # follow-me packet emitted this cycle (power-on retransmits included).
        unit_entity = self.world.entities.get(UNIT_CLIMATE, {})
        unit_mode = unit_entity.get("state")
        sp = unit_entity.get("attributes", {}).get("temperature")
        if sp is not None:
            self.unit.setpoint = float(sp)  # real unit learns setpoint changes
        reports = self._follow_me_reports(journal_slice)
        if unit_mode in ("heat", "cool"):
            if self.unit.mode != unit_mode:
                self.unit.mode = unit_mode
                self.unit.mode_sign = 1.0 if unit_mode == "cool" else -1.0
                self.unit.reset()
                self._p_lag.reset()
                self._q_lag.reset()
            if not self.unit.running:
                self.unit.reset()  # power cycle via climate entity clears state
                self._p_lag.reset()  # compressor was cut; spin up from zero
                self._q_lag.reset()
            for r in reports:
                self.unit.step(r)
            increment = self.unit.comp_speed
            if self.unit.running:
                p_target = self.hvac.power_kw(increment)
                p_kw = self._p_lag.step(p_target, 10.0)
            else:
                self._p_lag.reset()  # instant cut, per recorded shutdowns
                self._q_lag.reset()
                p_kw = 0.0
            q_target = self.hvac.cop(p_kw, t_out) * p_kw if p_kw > 0 else 0.0
            q_kw = self._q_lag.step(q_target)
            if unit_mode == "heat" and self._defrost.step(self.unit.running, t_out, 10.0):
                p_kw = self.hvac.params.defrost_power_kw
                q_kw = 0.0
                self._p_lag.reset(p_kw)  # resume post-defrost from this level
                self._q_lag.reset(0.0)
            if unit_mode == "cool":
                q_kw = -q_kw  # placeholder until a cooling calibration exists
        else:
            self.unit.running = False
            self.unit.comp_speed = 0
            self._p_lag.reset()
            self._q_lag.reset()
            increment = 0
            p_kw, q_kw = 0.0, 0.0

        q_house = q_kw / self.hvac.params.c_eff_kwh_per_k
        q_rooms = self._room_q(q_house) if q_kw else {r: 0.0 for r in ROOMS}
        self.house.step(t_out, q_rooms, dt_s=10.0, pv_kw=pv_kw)

        row = {
            "cycle": self.cycle,
            "t_out": t_out,
            "mode": unit_mode,
            "increment": increment,
            "p_kw": p_kw,
            "q_kw": q_kw,
            **{f"T_{r}": self.house.temps[r] for r in ROOMS},
            **{f"Tm_{r}": self.house.temps_measured[r] for r in ROOMS},
            **{f"damper_{r}": self._damper_positions()[r] for r in ROOMS},
            "weighted_error": _entity_float(
                self.world, "input_number.aircon_weighted_error"
            ),
        }
        self.history.append(row)
        return row

    def close(self):
        self._sleep_patch.stop()
