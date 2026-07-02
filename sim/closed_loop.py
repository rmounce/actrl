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

Heat split assumption: room thermal mass is proportional to its airflow
weight (kitchen 2.0 = two ducts = biggest room; others 1.0), so
q_i = q_house * share_i / (w_i / sum_w) where share_i is the room's
airflow-weighted damper share. With all dampers equal this collapses to
q_i = q_house for every room.

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
from sim.hvac import FirstOrderLag, Hvac  # noqa: E402
from sim.midea_unit import MideaUnit  # noqa: E402

AIRFLOW_WEIGHTS = {"bed_1": 1.0, "bed_2": 1.0, "bed_3": 1.0, "study": 1.0, "kitchen": 2.0}
FOLLOW_ME_SERVICE = "esphome/m5atom_send_follow_me"
UNIT_CLIMATE = "climate.m5atom_climate"


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
        self.cycle = -1
        # Telemetry: one row per cycle.
        self.history: list[dict] = []

        self._sleep_patch = mock.patch.object(actrl.time, "sleep", lambda s: None)
        self._sleep_patch.start()
        self.app.initialize()

    def _write_room_temps(self):
        for room in ROOMS:
            t = self.house.temps[room]
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
        sum_w = sum(AIRFLOW_WEIGHTS.values())
        flows = {r: dampers[r] * AIRFLOW_WEIGHTS[r] for r in ROOMS}
        total_flow = sum(flows.values())
        if total_flow <= 0:
            return {r: 0.0 for r in ROOMS}
        return {
            r: q_house * (flows[r] / total_flow) / (AIRFLOW_WEIGHTS[r] / sum_w)
            for r in ROOMS
        }

    def step(self, t_out: float, updates: dict | None = None) -> dict:
        """Advance one 10 s cycle. `updates` = extra world-entity updates
        applied before actrl runs (setpoint changes etc.)."""
        self.cycle += 1
        self.world.cycle = self.cycle
        self._write_room_temps()
        for eid, entry in (updates or {}).items():
            self.world.update(eid, entry)

        j0 = len(self.world.journal)
        self.app.main({})
        journal_slice = self.world.journal[j0:]

        # Unit side: mode from the climate entity actrl controls; feed every
        # follow-me packet emitted this cycle (power-on retransmits included).
        unit_mode = self.world.entities.get(UNIT_CLIMATE, {}).get("state")
        reports = self._follow_me_reports(journal_slice)
        if unit_mode in ("heat", "cool"):
            if self.unit.mode != unit_mode:
                self.unit.mode = unit_mode
                self.unit.mode_sign = 1.0 if unit_mode == "cool" else -1.0
                self.unit.reset()
                self._p_lag.reset()
            if not self.unit.running:
                self.unit.reset()  # power cycle via climate entity clears state
                self._p_lag.reset()  # compressor was cut; spin up from zero
            for r in reports:
                self.unit.step(r)
            increment = self.unit.comp_speed
            if self.unit.running:
                p_target = self.hvac.power_kw(increment)
                p_kw = self._p_lag.step(p_target, 10.0)
            else:
                self._p_lag.reset()  # instant cut, per recorded shutdowns
                p_kw = 0.0
            q_kw = self.hvac.cop(p_kw, t_out) * p_kw if p_kw > 0 else 0.0
            if unit_mode == "cool":
                q_kw = -q_kw  # placeholder until a cooling calibration exists
        else:
            self.unit.running = False
            self.unit.comp_speed = 0
            self._p_lag.reset()
            increment = 0
            p_kw, q_kw = 0.0, 0.0

        q_house = q_kw / self.hvac.params.c_eff_kwh_per_k
        q_rooms = self._room_q(q_house) if q_kw else {r: 0.0 for r in ROOMS}
        self.house.step(t_out, q_rooms, dt_s=10.0)

        row = {
            "cycle": self.cycle,
            "t_out": t_out,
            "mode": unit_mode,
            "increment": increment,
            "p_kw": p_kw,
            "q_kw": q_kw,
            **{f"T_{r}": self.house.temps[r] for r in ROOMS},
        }
        self.history.append(row)
        return row

    def close(self):
        self._sleep_patch.stop()
