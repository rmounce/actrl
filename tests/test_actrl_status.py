import pytest

from hvac_harness import actrl, run_scenario_world
from scenarios import heat_approach, manual_mode


def status_from(world):
    return world.entities[actrl.actrl_status_entity]


def test_active_status_describes_actrl_intent_and_lead_room():
    scenario = heat_approach()
    scenario["cycles"] = 2

    world = run_scenario_world(scenario)
    status = status_from(world)

    assert status["state"] == "heating"
    assert status["attributes"]["mode"] == "heat"
    assert status["attributes"]["lead_room"] == "bed_1"
    assert status["attributes"]["lead_temperature"] == pytest.approx(20.015)
    assert status["attributes"]["lead_target"] == pytest.approx(21.0)
    assert status["attributes"]["demand"] == pytest.approx(-0.985)
    assert status["attributes"]["active_rooms"] == 5
    assert status["attributes"]["capacity_max"] == 14
    assert status["attributes"]["heartbeat"] > 0


def test_manual_mode_publishes_pause_state():
    world = run_scenario_world(manual_mode())
    status = status_from(world)

    assert status["state"] == "manual"
    assert status["attributes"]["mode"] == "off"
    assert status["attributes"]["capacity_step"] == 0


def test_local_inhibit_pauses_without_issuing_plant_commands():
    scenario = heat_approach()
    scenario["cycles"] = 2
    scenario["initial"][actrl.local_inhibit_entity]["state"] = "on"

    world = run_scenario_world(scenario)
    status = status_from(world)

    assert status["state"] == "inhibited"
    assert status["attributes"]["mode"] == "off"
    assert not any("service" in event for event in world.journal)


def test_control_resumes_after_inhibit_is_released():
    scenario = heat_approach()
    scenario["cycles"] = 3
    scenario["initial"][actrl.local_inhibit_entity]["state"] = "on"
    scenario["updates"] = dict(scenario["updates"])
    scenario["updates"][1] = dict(scenario["updates"][1])
    scenario["updates"][1][actrl.local_inhibit_entity] = {"state": "off"}

    world = run_scenario_world(scenario)
    status = status_from(world)

    assert status["state"] == "heating"
    assert any(
        event.get("service") == "climate/set_hvac_mode"
        for event in world.journal
    )
