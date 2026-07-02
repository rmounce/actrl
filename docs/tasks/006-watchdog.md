# 006: Watchdog app — alert on stale sensors and dead control loop

Status: done
Branch: task/006-watchdog

## Goal

A new standalone AppDaemon app `hvac_watchdog.py` (repo root, deployed alongside
the others) that raises Home Assistant notifications when the HVAC control
stack silently degrades. Motivating incident: `sensor.m5atom_current`
stopped reporting 2026-06-29 and nothing noticed (docs/data.md). Silent
service-call failures are a known risk (docs/actrl.md "Known issues").

This app must NOT modify or import actrl.py/control.py/statctrl.py — it
observes entities only.

## Behaviour

Two check types, evaluated on a `run_every` tick (default 60 s):

1. **Entity staleness/unavailability**: for each configured entity, alert
   if state is `unavailable`/`unknown`, or if `last_updated` is older than
   the entity's threshold. Use AppDaemon's
   `get_state(entity, attribute="last_updated")` (ISO timestamp) and parse
   with `datetime.fromisoformat`; compare against `self.get_now()`
   (AppDaemon-aware time, works under time-travel in tests).
2. **Control-loop liveness**: when `climate.m5atom_climate` is in `heat`
   or `cool`, `input_number.aircon_weighted_error` must have updated
   within `loop_stale_minutes` (default 5). actrl rewrites it every 10 s
   cycle while a mode is active, so a stale value with the unit on means
   the app/loop is wedged.

Alerting:

- Send via `self.call_service` to a configurable `notify_service`
  (default: `persistent_notification/create` with a title+message; if the
  arg is e.g. `notify/mobile_app_foo`, send `{"title":..., "message":...}`).
- **Suppression**: one alert when a condition first triggers, then repeat
  at most every `realert_hours` (default 12) while it persists. One
  all-clear message when a previously-alerted condition recovers. Track
  per-condition state in a dict keyed by entity/check name.
- Log every alert/recovery via `self.log`.

## Configuration

Via `self.args` with these defaults (document an `apps.yaml` example block
in the module docstring; the live apps.yaml is out of scope):

```yaml
watchdog:
  module: hvac_watchdog
  class: Watchdog
  # optional overrides:
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

Defaults live in hvac_watchdog.py so an argless `watchdog:` block works.
(Module is named `hvac_watchdog`, not `watchdog`: AppDaemon itself uses
the PyPI `watchdog` package for app auto-reload, and app dirs land on
sys.path — a `watchdog.py` would shadow it.)

## Design constraints

- Pure decision logic as module-level functions/classes (no hassapi
  dependency): given (now, per-entity state+last_updated, config,
  suppression state) → list of alert/recover/none actions + new
  suppression state. The `hass.Hass` subclass is a thin shell that reads
  entities, calls the pure logic, and sends notifications — same
  philosophy as control.py vs actrl.py.
- `hvac_watchdog.py` must import cleanly with the hassapi stub trick used in
  tests/hvac_harness.py.
- No blocking sleeps. No writes to any entity other than notifications.

## Deliverables

1. `hvac_watchdog.py` — app + pure logic + docstring apps.yaml example.
2. `tests/test_hvac_watchdog.py` — offline unit tests for the pure logic:
   staleness threshold edges, unavailable/unknown states, liveness check
   gated on climate mode, first-alert/suppression/re-alert/recovery
   sequencing, config defaults. No hassapi, no network.
3. `deploy.sh`: add `hvac_watchdog.py` to `FILES` (do NOT run deploy.sh).
4. `docs/watchdog.md` — caveman-style: what it checks, config reference,
   what a deployment needs (apps.yaml block added manually in prod).
5. Task file Log updated; Status → review.

## Out of scope / do not modify

- `actrl.py`, `control.py`, `statctrl.py`, `calib.py`, `analysis/`,
  `tools/`, `systemd/`, `appdaemon/` (NEVER deploy or run deploy.sh),
  existing tests/fixtures.
- The live apps.yaml (prod config) — documentation only.
- No new dependencies (stdlib only; pandas et al. are for analysis, not
  apps).
- No credentials (public repo).

## Acceptance criteria (runnable)

```bash
uv run pytest                              # all pass, offline
python3 - <<'EOF'
import sys, types
m = types.ModuleType("hassapi"); m.Hass = object
sys.modules["hassapi"] = m
import hvac_watchdog
print("hvac_watchdog import OK")
EOF
grep -c "hvac_watchdog.py" deploy.sh           # expect 1
git diff master -- actrl.py control.py statctrl.py | wc -l   # expect 0
```

## Questions

(none yet)

## Log

- 2026-07-02: spec written (Claude Fable), status ready.
- 2026-07-02: implementation started by a Sonnet subagent, which was
  terminated mid-task by an account spend limit after drafting
  hvac_watchdog.py and tests (uncommitted). Claude Fable completed the
  task directly in the same worktree: fixed the tests' missing hassapi
  stub (module import crashed collection), added tz-aware/naive
  timestamp robustness to `_age_minutes` (+ test), added
  hvac_watchdog.py to deploy.sh FILES, wrote docs/watchdog.md.
  Acceptance: 107 passed / 1 skipped offline; hassapi-stub import OK;
  deploy.sh grep = 1; actrl/control/statctrl diff vs master = 0 lines.
  Not deployed; live apps.yaml block is a manual prod step
  (docs/watchdog.md). Status done.
