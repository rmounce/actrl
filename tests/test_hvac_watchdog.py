"""Offline unit tests for hvac_watchdog's pure decision logic.

No hassapi, no network. Only exercises the module-level functions and the
`evaluate_watchdog` suppression state machine.
"""

import datetime
import sys
import types

import pytest

if "hassapi" not in sys.modules:
    _hassapi_stub = types.ModuleType("hassapi")
    _hassapi_stub.Hass = object
    sys.modules["hassapi"] = _hassapi_stub

from hvac_watchdog import (
    CLIMATE_ENTITY,
    DEFAULT_ENTITIES,
    DEFAULT_INTERVAL_S,
    DEFAULT_LOOP_STALE_MINUTES,
    DEFAULT_NOTIFY_SERVICE,
    DEFAULT_REALERT_HOURS,
    LOOP_ENTITY,
    LOOP_LIVENESS_KEY,
    check_entity_staleness,
    check_loop_liveness,
    evaluate_watchdog,
)

NOW = datetime.datetime(2026, 7, 2, 12, 0, 0, tzinfo=datetime.timezone.utc)


def iso(minutes_ago):
    return (NOW - datetime.timedelta(minutes=minutes_ago)).isoformat()


# --- config defaults -----------------------------------------------------


def test_config_defaults():
    assert DEFAULT_NOTIFY_SERVICE == "persistent_notification/create"
    assert DEFAULT_INTERVAL_S == 60
    assert DEFAULT_LOOP_STALE_MINUTES == 5
    assert DEFAULT_REALERT_HOURS == 12
    assert CLIMATE_ENTITY == "climate.m5atom_climate"
    assert LOOP_ENTITY == "input_number.aircon_weighted_error"
    assert DEFAULT_ENTITIES["climate.m5atom_climate"] == 60
    assert DEFAULT_ENTITIES["sensor.bed_1_average_temperature"] == 15


# --- entity staleness edges -----------------------------------------------


@pytest.mark.parametrize("state", ["unavailable", "unknown", None])
def test_staleness_unavailable_states_always_alert(state):
    is_active, reason = check_entity_staleness(
        NOW, "sensor.x", state, iso(0), threshold_minutes=15
    )
    assert is_active
    assert "sensor.x" in reason


def test_staleness_naive_timestamp_treated_as_utc():
    naive = (NOW - datetime.timedelta(minutes=3)).replace(tzinfo=None).isoformat()
    is_active, _ = check_entity_staleness(NOW, "sensor.x", "21.0", naive, threshold_minutes=15)
    assert not is_active


def test_staleness_missing_last_updated_alerts():
    is_active, reason = check_entity_staleness(
        NOW, "sensor.x", "21.0", None, threshold_minutes=15
    )
    assert is_active
    assert "no last_updated" in reason


def test_staleness_within_threshold_ok():
    is_active, _ = check_entity_staleness(
        NOW, "sensor.x", "21.0", iso(14.9), threshold_minutes=15
    )
    assert not is_active


def test_staleness_at_exact_threshold_ok():
    is_active, _ = check_entity_staleness(
        NOW, "sensor.x", "21.0", iso(15.0), threshold_minutes=15
    )
    assert not is_active


def test_staleness_just_over_threshold_alerts():
    is_active, reason = check_entity_staleness(
        NOW, "sensor.x", "21.0", iso(15.01), threshold_minutes=15
    )
    assert is_active
    assert "sensor.x" in reason


# --- control-loop liveness -------------------------------------------------


@pytest.mark.parametrize("climate_state", ["off", "fan_only", None, "idle"])
def test_liveness_not_gated_when_not_heat_or_cool(climate_state):
    is_active, _ = check_loop_liveness(
        NOW, climate_state, iso(999), loop_stale_minutes=5
    )
    assert not is_active


@pytest.mark.parametrize("climate_state", ["heat", "cool"])
def test_liveness_fresh_ok(climate_state):
    is_active, _ = check_loop_liveness(
        NOW, climate_state, iso(4.9), loop_stale_minutes=5
    )
    assert not is_active


@pytest.mark.parametrize("climate_state", ["heat", "cool"])
def test_liveness_stale_alerts(climate_state):
    is_active, reason = check_loop_liveness(
        NOW, climate_state, iso(5.01), loop_stale_minutes=5
    )
    assert is_active
    assert LOOP_ENTITY in reason


def test_liveness_missing_last_updated_while_active_alerts():
    is_active, reason = check_loop_liveness(
        NOW, "heat", None, loop_stale_minutes=5
    )
    assert is_active
    assert "no last_updated" in reason


# --- evaluate_watchdog: suppression sequencing ----------------------------


def base_readings(entity_state="21.0", entity_last_updated=None, climate_state="off",
                   loop_last_updated=None):
    return {
        "entities": {
            "sensor.only": {
                "state": entity_state,
                "last_updated": entity_last_updated or iso(0),
            }
        },
        "climate_state": climate_state,
        "loop_last_updated": loop_last_updated or iso(0),
    }


def base_config():
    return {
        "entities": {"sensor.only": 15},
        "loop_stale_minutes": 5,
        "realert_hours": 12,
    }


def test_first_trigger_alerts_once():
    readings = base_readings(entity_state="unavailable")
    actions, state = evaluate_watchdog(NOW, readings, base_config(), {})
    keys = {a["key"]: a["kind"] for a in actions}
    assert keys["entity:sensor.only"] == "alert"
    assert state["entity:sensor.only"]["alerted"] is True
    assert state["entity:sensor.only"]["last_alert"] == NOW


def test_persisting_condition_within_realert_window_no_repeat():
    readings = base_readings(entity_state="unavailable")
    config = base_config()
    actions1, state1 = evaluate_watchdog(NOW, readings, config, {})
    later = NOW + datetime.timedelta(hours=11)
    actions2, state2 = evaluate_watchdog(later, readings, config, state1)
    assert not any(a["key"] == "entity:sensor.only" for a in actions2)
    # suppression state carries forward unchanged
    assert state2["entity:sensor.only"]["last_alert"] == NOW


def test_persisting_condition_realerts_after_window():
    readings = base_readings(entity_state="unavailable")
    config = base_config()
    _, state1 = evaluate_watchdog(NOW, readings, config, {})
    later = NOW + datetime.timedelta(hours=12, minutes=1)
    actions2, state2 = evaluate_watchdog(later, readings, config, state1)
    assert any(
        a["key"] == "entity:sensor.only" and a["kind"] == "alert" for a in actions2
    )
    assert state2["entity:sensor.only"]["last_alert"] == later


def test_realert_at_exact_boundary_fires():
    readings = base_readings(entity_state="unavailable")
    config = base_config()
    _, state1 = evaluate_watchdog(NOW, readings, config, {})
    later = NOW + datetime.timedelta(hours=12)
    actions2, _ = evaluate_watchdog(later, readings, config, state1)
    assert any(
        a["key"] == "entity:sensor.only" and a["kind"] == "alert" for a in actions2
    )


def test_recovery_sends_all_clear_once():
    bad_readings = base_readings(entity_state="unavailable")
    config = base_config()
    _, state1 = evaluate_watchdog(NOW, bad_readings, config, {})

    good_readings = base_readings(entity_state="21.0", entity_last_updated=iso(0))
    later = NOW + datetime.timedelta(minutes=1)
    actions2, state2 = evaluate_watchdog(later, good_readings, config, state1)
    keys = {a["key"]: a["kind"] for a in actions2}
    assert keys["entity:sensor.only"] == "recover"
    assert state2["entity:sensor.only"]["alerted"] is False
    assert state2["entity:sensor.only"]["last_alert"] is None

    # Staying good produces no further actions.
    even_later = later + datetime.timedelta(minutes=1)
    actions3, _ = evaluate_watchdog(even_later, good_readings, config, state2)
    assert not any(a["key"] == "entity:sensor.only" for a in actions3)


def test_no_action_when_never_triggered():
    readings = base_readings(entity_state="21.0")
    actions, state = evaluate_watchdog(NOW, readings, base_config(), {})
    assert actions == []
    assert state["entity:sensor.only"] == {"alerted": False, "last_alert": None}


def test_loop_liveness_gated_on_climate_mode_end_to_end():
    config = base_config()
    # Unit off: stale weighted_error is irrelevant.
    readings_off = base_readings(
        entity_state="21.0", climate_state="off", loop_last_updated=iso(999)
    )
    actions, state = evaluate_watchdog(NOW, readings_off, config, {})
    assert not any(a["key"] == LOOP_LIVENESS_KEY for a in actions)
    assert state[LOOP_LIVENESS_KEY]["alerted"] is False

    # Unit heating with stale weighted_error: alert.
    readings_heat_stale = base_readings(
        entity_state="21.0", climate_state="heat", loop_last_updated=iso(999)
    )
    actions2, state2 = evaluate_watchdog(NOW, readings_heat_stale, config, state)
    assert any(
        a["key"] == LOOP_LIVENESS_KEY and a["kind"] == "alert" for a in actions2
    )

    # Loop recovers.
    readings_heat_fresh = base_readings(
        entity_state="21.0", climate_state="heat", loop_last_updated=iso(0)
    )
    later = NOW + datetime.timedelta(minutes=1)
    actions3, state3 = evaluate_watchdog(later, readings_heat_fresh, config, state2)
    assert any(
        a["key"] == LOOP_LIVENESS_KEY and a["kind"] == "recover" for a in actions3
    )
    assert state3[LOOP_LIVENESS_KEY]["alerted"] is False


def test_independent_conditions_tracked_separately():
    config = {
        "entities": {"sensor.a": 15, "sensor.b": 15},
        "loop_stale_minutes": 5,
        "realert_hours": 12,
    }
    readings = {
        "entities": {
            "sensor.a": {"state": "unavailable", "last_updated": iso(0)},
            "sensor.b": {"state": "21.0", "last_updated": iso(0)},
        },
        "climate_state": "off",
        "loop_last_updated": iso(0),
    }
    actions, state = evaluate_watchdog(NOW, readings, config, {})
    keys = {a["key"]: a["kind"] for a in actions}
    assert keys == {"entity:sensor.a": "alert"}
    assert state["entity:sensor.a"]["alerted"] is True
    assert state["entity:sensor.b"]["alerted"] is False
