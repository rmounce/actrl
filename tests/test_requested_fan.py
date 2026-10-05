"""Fan feedback must not change requested-speed airflow protection."""
from unittest.mock import patch

import pytest

from hvac_harness import FakeWorld, HarnessActrl, actrl
from scenarios import base_world


def controller(fan="low", speed="0"):
    app = HarnessActrl(FakeWorld(base_world(
        climate_state="heat", fan_mode=fan, comp_speed=speed,
    )))
    app.initialize()
    app.targets = {"heat": {"bed_1": 22, "kitchen": 20}}
    app.activation_steps = {"heat": set(), "cool": set()}
    return app


def airflow(app):
    return app._calculate_pid_outputs({"heat": {"bed_1": 1, "kitchen": -1}})


@pytest.mark.parametrize("requested", ["low", "medium", "high"])
def test_off_feedback_preserves_airflow_and_hysteresis(requested):
    speed = str(actrl.compressor_power_safety_margin / 2)
    baseline, feedback = controller(requested, speed), controller(requested, speed)
    feedback.world.update(actrl.climate_entity, {"attributes": {"fan_mode": "off"}})
    assert feedback._determine_fan_mode() == baseline._determine_fan_mode()
    assert airflow(feedback) == pytest.approx(airflow(baseline))


@pytest.mark.parametrize("reported", ["off", "auto", "unknown", None])
def test_reload_with_unusable_feedback_reconstructs_request(reported):
    app = controller(reported)
    assert app.requested_fan_mode == "low"
    assert airflow(app)["kitchen"] <= 0


def test_requested_high_retains_extra_airflow_until_low_command():
    app = controller("high")
    assert airflow(app)["kitchen"] == pytest.approx(0.77725)
    with patch.object(actrl.time, "sleep"):
        app.try_set_fan_mode("low")
    app.world.update(actrl.climate_entity, {"attributes": {"fan_mode": "off"}})
    assert app.requested_fan_mode == "low"
    assert airflow(app)["kitchen"] <= 0


def test_imminent_high_request_prepares_airflow():
    app = controller("low", str(actrl.compressor_power_increments))
    assert app._determine_fan_mode() == "high"
    assert airflow(app)["kitchen"] == pytest.approx(0.77725)
