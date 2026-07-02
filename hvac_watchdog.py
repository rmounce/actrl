"""Standalone AppDaemon watchdog: alerts when HVAC monitoring/control goes
silent.

Two independent checks, evaluated on a run_every tick:

1. Entity staleness/unavailability: each configured entity must have a
   state other than unavailable/unknown, and its `last_updated` must be
   no older than the entity's configured threshold.
2. Control-loop liveness: while `climate.m5atom_climate` is in `heat` or
   `cool`, `input_number.aircon_weighted_error` (rewritten every ~10 s by
   actrl.py's main cycle) must have updated within `loop_stale_minutes`.
   A stale value while the unit is on means the control loop is wedged.

This app never imports or calls into actrl.py/control.py/statctrl.py; it
only reads entities and sends notifications.

Pure decision logic (evaluate_watchdog and friends) has no hassapi
dependency and is unit tested directly. `Watchdog` is a thin AppDaemon
shell: it reads entity state, calls the pure logic, and dispatches
notifications + suppression bookkeeping.

Example apps.yaml block (add manually to the live config; not deployed by
this repo):

```yaml
watchdog:
  module: hvac_watchdog
  class: Watchdog
  # optional overrides (defaults shown):
  notify_service: persistent_notification/create
  interval_s: 60
  loop_stale_minutes: 5
  realert_hours: 12
  entities:            # entity -> max age minutes
    sensor.bed_1_average_temperature: 15
    sensor.bed_2_average_temperature: 15
    sensor.bed_3_average_temperature: 15
    sensor.kitchen_average_temperature: 15
    sensor.study_average_temperature: 15
    sensor.shellyem_ec64c9c6932b_channel_1_power: 15
    sensor.shellyem_ec64c9c6932b_channel_2_power: 15
    climate.m5atom_climate: 60
```

Module is named `hvac_watchdog`, not `watchdog`: AppDaemon itself uses
the PyPI `watchdog` package for app auto-reload, and app dirs land on
sys.path — a `watchdog.py` would shadow it.
"""

import datetime

import hassapi as hass  # type: ignore

# --- Defaults (also the argless-`watchdog:`-block behaviour) -----------

DEFAULT_NOTIFY_SERVICE = "persistent_notification/create"
DEFAULT_INTERVAL_S = 60
DEFAULT_LOOP_STALE_MINUTES = 5
DEFAULT_REALERT_HOURS = 12

DEFAULT_ENTITIES = {
    "sensor.bed_1_average_temperature": 15,
    "sensor.bed_2_average_temperature": 15,
    "sensor.bed_3_average_temperature": 15,
    "sensor.kitchen_average_temperature": 15,
    "sensor.study_average_temperature": 15,
    "sensor.shellyem_ec64c9c6932b_channel_1_power": 15,
    "sensor.shellyem_ec64c9c6932b_channel_2_power": 15,
    "climate.m5atom_climate": 60,
}

CLIMATE_ENTITY = "climate.m5atom_climate"
LOOP_ENTITY = "input_number.aircon_weighted_error"
LOOP_ACTIVE_MODES = ("heat", "cool")

UNAVAILABLE_STATES = ("unavailable", "unknown", None)

LOOP_LIVENESS_KEY = "loop_liveness"


# --- Pure decision logic -------------------------------------------------


def _parse_last_updated(last_updated):
    """Parse an ISO timestamp string; return None if missing/unparseable."""
    if not last_updated:
        return None
    try:
        return datetime.datetime.fromisoformat(last_updated)
    except (TypeError, ValueError):
        return None


def _age_minutes(now, last_updated):
    """Minutes between last_updated and now, or None if unknown."""
    parsed = _parse_last_updated(last_updated)
    if parsed is None:
        return None
    # HA last_updated is tz-aware; guard against naive values so an odd
    # timestamp degrades to an alert (None) instead of crashing the check.
    if (parsed.tzinfo is None) != (now.tzinfo is None):
        if parsed.tzinfo is None:
            parsed = parsed.replace(tzinfo=datetime.timezone.utc)
        else:
            return None
    delta = now - parsed
    return delta.total_seconds() / 60.0


def check_entity_staleness(now, entity_id, state, last_updated, threshold_minutes):
    """Return (is_active, reason) for one entity's staleness condition."""
    if state in UNAVAILABLE_STATES:
        return True, f"{entity_id} is {state}"
    age = _age_minutes(now, last_updated)
    if age is None:
        return True, f"{entity_id} has no last_updated timestamp"
    if age > threshold_minutes:
        return True, (
            f"{entity_id} last updated {age:.1f} min ago "
            f"(threshold {threshold_minutes} min)"
        )
    return False, ""


def check_loop_liveness(now, climate_state, loop_last_updated, loop_stale_minutes):
    """Return (is_active, reason) for the control-loop liveness condition."""
    if climate_state not in LOOP_ACTIVE_MODES:
        return False, ""
    age = _age_minutes(now, loop_last_updated)
    if age is None:
        return True, (
            f"{LOOP_ENTITY} has no last_updated timestamp while "
            f"{CLIMATE_ENTITY} is {climate_state}"
        )
    if age > loop_stale_minutes:
        return True, (
            f"{LOOP_ENTITY} stale for {age:.1f} min while "
            f"{CLIMATE_ENTITY} is {climate_state} "
            f"(threshold {loop_stale_minutes} min)"
        )
    return False, ""


def evaluate_conditions(now, readings, config):
    """Evaluate all configured conditions.

    readings = {
        "entities": {entity_id: {"state": ..., "last_updated": iso_str_or_None}},
        "climate_state": str_or_None,
        "loop_last_updated": iso_str_or_None,
    }
    config = {
        "entities": {entity_id: threshold_minutes},
        "loop_stale_minutes": float,
    }

    Returns {key: (is_active, reason)}.
    """
    results = {}
    entity_readings = readings.get("entities", {})
    for entity_id, threshold_minutes in config.get("entities", {}).items():
        reading = entity_readings.get(entity_id, {})
        results[f"entity:{entity_id}"] = check_entity_staleness(
            now,
            entity_id,
            reading.get("state"),
            reading.get("last_updated"),
            threshold_minutes,
        )

    results[LOOP_LIVENESS_KEY] = check_loop_liveness(
        now,
        readings.get("climate_state"),
        readings.get("loop_last_updated"),
        config.get("loop_stale_minutes", DEFAULT_LOOP_STALE_MINUTES),
    )
    return results


def evaluate_watchdog(now, readings, config, suppression_state):
    """Evaluate conditions and apply alert suppression bookkeeping.

    suppression_state: {key: {"alerted": bool, "last_alert": datetime_or_None}}
    (opaque to callers; pass back what this function returns).

    Returns (actions, new_suppression_state):
        actions: list of {"key": str, "kind": "alert"|"recover", "message": str}
    """
    realert_hours = config.get("realert_hours", DEFAULT_REALERT_HOURS)
    realert_delta = datetime.timedelta(hours=realert_hours)

    conditions = evaluate_conditions(now, readings, config)

    actions = []
    new_state = {}

    for key, (is_active, reason) in conditions.items():
        prior = suppression_state.get(key, {"alerted": False, "last_alert": None})

        if is_active:
            last_alert = prior["last_alert"]
            should_alert = not prior["alerted"] or (
                last_alert is not None and (now - last_alert) >= realert_delta
            )
            if should_alert:
                actions.append(
                    {"key": key, "kind": "alert", "message": reason}
                )
                new_state[key] = {"alerted": True, "last_alert": now}
            else:
                new_state[key] = prior
        else:
            if prior["alerted"]:
                actions.append(
                    {
                        "key": key,
                        "kind": "recover",
                        "message": f"{key} recovered",
                    }
                )
            new_state[key] = {"alerted": False, "last_alert": None}

    return actions, new_state


# --- AppDaemon shell -------------------------------------------------


class Watchdog(hass.Hass):
    def initialize(self):
        self.notify_service = self.args.get("notify_service", DEFAULT_NOTIFY_SERVICE)
        self.interval_s = float(self.args.get("interval_s", DEFAULT_INTERVAL_S))
        self.loop_stale_minutes = float(
            self.args.get("loop_stale_minutes", DEFAULT_LOOP_STALE_MINUTES)
        )
        self.realert_hours = float(
            self.args.get("realert_hours", DEFAULT_REALERT_HOURS)
        )
        self.watched_entities = dict(self.args.get("entities", DEFAULT_ENTITIES))
        self.suppression_state = {}
        self.run_every(self.check, "now", self.interval_s)

    def check(self, kwargs):
        now = self.get_now()
        readings = self._read_entities()
        config = {
            "entities": self.watched_entities,
            "loop_stale_minutes": self.loop_stale_minutes,
            "realert_hours": self.realert_hours,
        }
        actions, self.suppression_state = evaluate_watchdog(
            now, readings, config, self.suppression_state
        )
        for action in actions:
            self._handle_action(action)

    def _read_entities(self):
        entities = {}
        for entity_id in self.watched_entities:
            entities[entity_id] = {
                "state": self.get_state(entity_id),
                "last_updated": self.get_state(entity_id, attribute="last_updated"),
            }
        return {
            "entities": entities,
            "climate_state": self.get_state(CLIMATE_ENTITY),
            "loop_last_updated": self.get_state(LOOP_ENTITY, attribute="last_updated"),
        }

    def _handle_action(self, action):
        if action["kind"] == "alert":
            title = "HVAC watchdog alert"
            level = "WARNING"
        else:
            title = "HVAC watchdog: recovered"
            level = "INFO"
        message = action["message"]
        self.log(f"{title}: {message}", level=level)
        self._notify(title, message)

    def _notify(self, title, message):
        # Both persistent_notification/create and notify/* services accept
        # title + message keyword args.
        self.call_service(self.notify_service, title=title, message=message)
