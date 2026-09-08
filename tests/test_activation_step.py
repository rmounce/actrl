"""Target-step catch-up through the real target and PID passes."""
from unittest.mock import patch

import pytest

from hvac_harness import FakeWorld, HarnessActrl, actrl
from scenarios import base_world, ROOMS


def controller(mode="heat"):
    app = HarnessActrl(FakeWorld(base_world(climate_state=mode)))
    app.initialize()
    app.mode = mode
    for room in ROOMS:
        app.rooms_enabled[room] = True
    return app


def targets(app, study, kitchen=20):
    requested = {"heat": {}, "cool": {}}
    requested[app.mode] = {"study": study, "kitchen": kitchen}
    app._update_room_targets({room: 20 for room in ROOMS}, requested)


def outputs(app, error=1.3):
    return app._calculate_pid_outputs({app.mode: {"study": error, "kitchen": -0.1}})


@pytest.mark.parametrize("mode,start,end", [("heat", 16, 19.5), ("cool", 28, 24.5)])
def test_activation_matches_leader_once_then_hands_over(mode, start, end):
    app = controller(mode)
    targets(app, start)
    app.pids["kitchen"].set_integral(2.1)
    targets(app, end)
    result = outputs(app)
    assert result["study"] == pytest.approx(2)
    assert result["kitchen"] == pytest.approx(2)
    assert actrl.damper_share(result["study"]) == pytest.approx(1)
    result = outputs(app)
    assert result["study"] == pytest.approx(2)
    assert result["kitchen"] < 2
    assert sum("Target-step catch-up" in msg for _, msg in app.log_records) == 1


@pytest.mark.parametrize("step,qualifies", [(0.1, False), (0.5, False), (0.9, False), (1.0, True), (-2, False)])
def test_step_boundary_and_direction(step, qualifies):
    app = controller()
    targets(app, 18)
    app.pids["kitchen"].set_integral(2.1)
    targets(app, 18 + step)
    result = outputs(app, error=0.5)
    assert (result["study"] == pytest.approx(2)) == qualifies


def test_small_steps_never_accumulate_and_match_original_pid_path():
    app, baseline = controller(), controller()
    for instance in [app, baseline]:
        targets(instance, 16)
        instance.pids["kitchen"].set_integral(2.1)
    for i in range(1, 41):
        for instance in [app, baseline]:
            targets(instance, 16 + i * 0.1)
        baseline.activation_steps = {"heat": set(), "cool": set()}
        assert outputs(app) == outputs(baseline)
    assert not any("Target-step catch-up" in msg for _, msg in app.log_records)


@pytest.mark.parametrize("error", [0.49, 0, -1])
def test_effective_demand_gate_does_not_defer_boost(error):
    app = controller()
    targets(app, 16)
    app.pids["kitchen"].set_integral(2.1)
    targets(app, 19.5)
    outputs(app, error)
    outputs(app, 1.3)
    assert not any("Target-step catch-up" in msg for _, msg in app.log_records)


def test_already_leading_zone_is_not_boosted():
    app = controller()
    targets(app, 16)
    targets(app, 19.5)
    app.pids["study"].set_integral(2)
    outputs(app)
    assert not any("Target-step catch-up" in msg for _, msg in app.log_records)


def test_startup_pause_and_missing_target_do_not_create_steps():
    app = controller()
    targets(app, 19.5)
    assert not app.activation_steps["heat"]
    app._reset_internal_state()
    app.mode = "heat"
    targets(app, 22)
    assert not app.activation_steps["heat"]
    app._update_room_targets({room: 20 for room in ROOMS}, {"heat": {}, "cool": {}})
    targets(app, 24)
    assert not app.activation_steps["heat"]


def test_mode_transition_discards_pending_step():
    app = controller()
    targets(app, 16)
    targets(app, 19.5)
    with patch.object(actrl.time, "sleep", lambda seconds: None):
        app._handle_mode_change("cool", 24)
    assert not app.activation_steps["heat"]


def test_timer_cancellation_closes_satisfied_zone():
    app = controller()
    targets(app, 16)
    app.pids["kitchen"].set_integral(2.1)
    targets(app, 19.5)
    outputs(app)
    targets(app, 16)
    result = outputs(app, -2.2)
    assert actrl.damper_share(result["study"]) == 0


def test_main_uses_effective_error_and_commands_full_opening():
    from scenarios import room_climates

    temperatures = {room: 18.2 for room in ROOMS}
    temperatures['kitchen'] = 19.9
    requested = {room: 16 for room in ROOMS}
    requested['kitchen'] = 20
    world = base_world(climate_state='heat')
    room_climates(world, 'heat', requested, temperatures)
    app = HarnessActrl(FakeWorld(world))
    with patch.object(actrl.time, 'sleep', lambda seconds: None):
        app.initialize()
        app.main({})
        app.pids['kitchen'].set_integral(2.1)
        app.pids['study'].set_integral(0)
        app.world.update('sensor.kitchen_average_temperature', {'state': '20.1'})
        app.world.update('climate.study_aircon', {'attributes': {'temperature': 19.5}})
        app.main({})
    assert float(app.world.entities['input_number.study_pid']['state']) == pytest.approx(2)
    assert any('Target-step catch-up study' in msg for _, msg in app.log_records)


def test_simultaneous_steps_share_leader_without_amplifying_it():
    app = controller()
    targets(app, 16, 18)
    app.pids['kitchen'].set_integral(2.1)
    targets(app, 19.5, 20)
    result = app._calculate_pid_outputs({'heat': {'study': 1.3, 'kitchen': 0.8}})
    assert result['study'] == pytest.approx(2)
    assert result['kitchen'] == pytest.approx(2)
    assert sum('Target-step catch-up' in msg for _, msg in app.log_records) == 1
