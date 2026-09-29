"""Cooling surplus decay responds to the room actually setting demand."""

import pytest

from hvac_harness import FakeWorld, HarnessActrl
from scenarios import ROOMS, base_world, room_climates


TARGETS = {
    "bed_1": (16.0, 28.0),
    "bed_2": (12.0, 30.0),
    "bed_3": (12.0, 30.0),
    "kitchen": (20.0, 25.0),
    "study": (16.0, 28.0),
}


def controller(surplus_w=0):
    forecasts = [{"mpc_p_pv_curtailment": surplus_w}] * 12
    world = base_world(
        climate_state="cool", grid_surplus_integral="4.0", forecasts=forecasts
    )
    world["input_boolean.ac_use_grid_surplus_cool"]["state"] = "on"
    room_climates(world, "heat_cool", TARGETS, {r: 24.0 for r in ROOMS})
    app = HarnessActrl(FakeWorld(world))
    app.initialize()
    app.targets = {
        "heat": {r: low for r, (low, _) in TARGETS.items()},
        "cool": {r: high for r, (_, high) in TARGETS.items()},
    }
    return app


def test_decay_doubles_when_lower_offset_room_leads():
    app = controller()
    temps = {r: 24.0 for r in ROOMS}
    temps["kitchen"] = 24.5  # kitchen offset 1.75; bedrooms offset 4.0
    assert app._surplus_decay_gain(temps) == 2.0
    app._add_grid_surplus(temps)
    assert app.grid_surplus_integral == pytest.approx(3.9)


def test_decay_keeps_original_rate_when_high_offset_room_leads():
    app = controller()
    temps = {r: 24.0 for r in ROOMS}
    temps["bed_2"] = 30.0
    assert app._surplus_decay_gain(temps) == 1.0
    app._add_grid_surplus(temps)
    assert app.grid_surplus_integral == pytest.approx(3.95)


def test_gain_does_not_accelerate_growth_or_disabled_surplus():
    app = controller(surplus_w=1000)
    temps = {r: 24.0 for r in ROOMS}
    temps["kitchen"] = 24.5
    app._add_grid_surplus(temps)
    assert app.grid_surplus_integral == pytest.approx(4.1)

    app.world.entities["input_boolean.ac_use_grid_surplus_cool"]["state"] = "off"
    assert app._surplus_decay_gain(temps) == 1.0
    app.mode = "off"
    app.world.entities["input_boolean.ac_use_grid_surplus_cool"]["state"] = "on"
    assert app._surplus_decay_gain(temps) == 1.0
