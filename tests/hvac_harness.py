"""Headless harness for running actrl.Actrl without Home Assistant.

Provides a FakeHass entity world implementing the subset of the AppDaemon
API that actrl uses, plus a scenario runner that drives main() cycles and
journals every service call and entity write for golden comparison.
"""

import sys
import types
from pathlib import Path
from unittest import mock

# Stub out hassapi so `import actrl` works outside the AppDaemon runtime.
_hassapi = types.ModuleType("hassapi")


class _Hass:
    pass


_hassapi.Hass = _Hass
sys.modules["hassapi"] = _hassapi

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import actrl  # noqa: E402


class FakeEntity:
    def __init__(self, world, entity_id):
        self.world = world
        self.entity_id = entity_id

    def get_state(self, attribute=None):
        entry = self.world.entities.get(self.entity_id)
        if entry is None:
            return None
        if attribute is None or attribute == "state":
            return entry["state"]
        if attribute == "all":
            return entry
        return entry["attributes"].get(attribute)

    def set_state(self, state=None, **kwargs):
        self.world.ensure(self.entity_id)
        self.world.entities[self.entity_id]["state"] = state
        self.world.journal.append(
            {"cycle": self.world.cycle, "write": [self.entity_id, state]}
        )


class FakeWorld:
    def __init__(self, initial):
        # initial: {entity_id: {"state": s, "attributes": {...}}}
        self.entities = {}
        for eid, entry in initial.items():
            self.entities[eid] = {
                "state": entry.get("state"),
                "attributes": dict(entry.get("attributes", {})),
            }
        self.journal = []
        self.cycle = -1  # -1 during initialize()

    def ensure(self, eid):
        if eid not in self.entities:
            self.entities[eid] = {"state": None, "attributes": {}}

    def update(self, eid, entry):
        self.ensure(eid)
        if "state" in entry:
            self.entities[eid]["state"] = entry["state"]
        for k, v in entry.get("attributes", {}).items():
            self.entities[eid]["attributes"][k] = v


class HarnessActrl(actrl.Actrl):
    """actrl.Actrl wired to a FakeWorld instead of AppDaemon."""

    def __init__(self, world):
        self.world = world
        self.logs = []

    # --- AppDaemon API subset used by actrl ---

    def get_state(self, entity_id, attribute=None):
        if "." not in entity_id:
            domain = entity_id
            return {
                eid: entry
                for eid, entry in self.world.entities.items()
                if eid.split(".")[0] == domain
            }
        entity = FakeEntity(self.world, entity_id)
        return entity.get_state(attribute=attribute)

    def get_entity(self, entity_id):
        return FakeEntity(self.world, entity_id)

    def call_service(self, service, **kwargs):
        self.world.journal.append(
            {"cycle": self.world.cycle, "service": service, "data": kwargs}
        )
        # Emulate instant device actuation for services whose effects
        # actrl reads back within the same or later cycles.
        if service == "climate/set_hvac_mode":
            self.world.update(kwargs["entity_id"], {"state": kwargs["hvac_mode"]})
        elif service == "climate/set_fan_mode":
            self.world.update(
                kwargs["entity_id"],
                {"attributes": {"fan_mode": kwargs["fan_mode"]}},
            )
        elif service == "cover/set_cover_position":
            self.world.update(
                kwargs["entity_id"],
                {"attributes": {"current_position": float(kwargs["position"])}},
            )
        elif service == "number/set_value":
            self.world.update(kwargs["entity_id"], {"state": str(kwargs["value"])})

    def run_every(self, callback, start, interval, **kwargs):
        self._run_every = (callback, start, interval)

    def log(self, msg, level=None, **kwargs):
        self.logs.append(str(msg))


def run_scenario(scenario):
    """Run a scenario dict and return the journal.

    scenario = {
        "name": str,
        "initial": {eid: {"state": ..., "attributes": {...}}},
        "cycles": int,
        "updates": {cycle_index: {eid: {"state": ...} | {"attributes": {...}}}},
    }
    """
    world = FakeWorld(scenario["initial"])
    app = HarnessActrl(world)
    with mock.patch.object(actrl.time, "sleep", lambda s: None):
        app.initialize()
        for cycle in range(scenario["cycles"]):
            world.cycle = cycle
            for eid, entry in scenario.get("updates", {}).get(cycle, {}).items():
                world.update(eid, entry)
            app.main({})
    return world.journal
