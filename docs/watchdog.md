# hvac_watchdog.py — stale-sensor and dead-loop alerting

Standalone AppDaemon app. Reads entities, never writes anything except
notifications; never imports actrl/control/statctrl. Motivating incident:
`sensor.m5atom_current` stopped reporting 2026-06-29 and nothing noticed
for days (docs/data.md).

## What it checks (every `interval_s`, default 60 s)

1. **Entity staleness**: each entity in `entities` must not be
   `unavailable`/`unknown` and must have `last_updated` newer than its
   per-entity threshold (minutes). Defaults cover the five room average
   temperatures (15 min), both Shelly EM power channels (15 min) and
   `climate.m5atom_climate` (60 min).
2. **Control-loop liveness**: while `climate.m5atom_climate` is in
   `heat`/`cool`, `input_number.aircon_weighted_error` must have updated
   within `loop_stale_minutes` (default 5). actrl rewrites it every 10 s
   cycle, so staleness with the unit on = wedged app/loop.

## Alerting

- Sent via `notify_service` (default `persistent_notification/create`;
  set e.g. `notify/mobile_app_<phone>` for push). Title + message args.
- One alert on first trigger, re-alert at most every `realert_hours`
  (default 12) while the condition persists, one recovery message when it
  clears. Everything also goes to the AppDaemon log (alerts at WARNING).

## Deploying

1. `./deploy.sh` (hvac_watchdog.py is in FILES).
2. Add to the live apps.yaml (an argless block uses all defaults):

```yaml
watchdog:
  module: hvac_watchdog
  class: Watchdog
```

Full config reference: the module docstring in `hvac_watchdog.py`
(notify_service, interval_s, loop_stale_minutes, realert_hours, entities).

## Notes

- The module is `hvac_watchdog`, not `watchdog`: AppDaemon uses the PyPI
  `watchdog` package for app auto-reload and the apps dir is on sys.path —
  a `watchdog.py` would shadow it and break AppDaemon.
- The watchdog cannot alert about its own death. If that matters, pair it
  with an external heartbeat later (ideas.md §4 territory).
- Pure decision logic (`evaluate_watchdog` and friends) is hassapi-free
  and unit tested offline in `tests/test_hvac_watchdog.py`.
