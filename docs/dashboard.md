# Climate dashboard prototype

- Added 2026-09-25: `/dashboard-advanced/climate-prototype`, **Climate** tab
  immediately after Weather. Existing views preserved.
- Source: `dashboards/climate.yaml` — one Lovelace view, not a whole dashboard.
- Published through HA WebSocket `lovelace/config/save`; verified by reading back.
  No HA restart or AppDaemon deployment.
- Existing dependencies: button-card, ApexCharts, card-mod. No new HACS components.
- Responsive HA Sections layout: three compact columns for system/signals,
  rooms/preferences and trends; stacked on narrow screens.
- Overview uses `sensor.actrl_status`; heartbeat older than 90 seconds displays
  a warning. Its heartbeat attribute is authoritative, not nested timestamp attrs.
- Room headline and temperature chart follow `input_boolean.ac_use_feels_like`:
  feels-like when on, measured average when off. Secondary line labels the basis,
  requested climate target band and reported damper.
  Target band is not actrl's internal smoothed/offset target.
- Room tap opens native climate controls. Scheduler may change targets unless
  room manual mode is on; hold switches provided below.
- Plant capacity is explicitly labelled an estimate. Retain slightly negative
  standby power readings rather than disguising meter noise as measured zero.
- Three overview charts: temperatures, electrical power, dampers. Six hours,
  two-minute averages, one-minute refresh, animations off, explicit entity lists.
  Aggregation smooths brief events.
- Original Weather: ten history charts, nine configured with ten-second refresh;
  several auto-entities lists. Potential load source, not measured proof of heating.
- Rollback: delete only the `climate-prototype` view via HA dashboard editor/API.
  Initial local full-dashboard backup: `.git/climate-dashboard-before.json`.
- Update: load latest dashboard, replace only this path, preserve other views,
  check for intervening edits, save, read back. Do not overwrite live `.storage` files.
- Performance intent: reduce history rendering and dynamic discovery. No claim
  of measured phone CPU, battery or temperature improvement; compare on device.
- Browser verification uses a disposable Playwright Docker container. No host
  browser system packages installed; initial host browser download removed.
- Diagnostic numbers render read-only; controller signals rounded to two decimals.
  Plant overview uses reported activity/fan mode, not the spoofed follow-me
  thermostat's `current_temperature`.
- Chromium checks at 1440px desktop and 390px phone: five room cards, three
  overview charts, no page errors or HA error cards; phone document width 390px
  (no page overflow). Detailed toggle rendered eleven charts, then returned to
  three after switching off.
  Screenshots retained locally under `.git/hvac-desktop.png` and `.git/hvac-phone.png`.

## Diagnostics expansion · 2026-09-25

- Climate now includes direct damper sliders, plant thermostat, controller inhibit,
  debug logging, room reset script, room PID and damper targets, humidity,
  controller health and plant error/protection flags. Weather link removed.
- Detailed histories: room PID, room humidity, controller demand/trend/integral,
  estimated capacity, follow-me temperature, solar offset, air-path temperatures,
  current Sigen grid import/export. The old Fronius grid entity no longer exists.
- `input_boolean.climate_detailed_history` is a dedicated HA helper. Off by
  default: only the three overview charts mount. Switch **Load detailed charts**
  at the start of Room diagnostics to render the eight additional charts;
  switch off to release them. Its state is shared across devices and restored by HA.
- HA view published via WebSocket with concurrent-edit check and exact readback.
  No HVAC service calls or AppDaemon deployment.

## Density revision · 2026-09-25

- Removed explanatory cards; status reduced to state, leading room and temperatures.
- Room padding 18 → 10px; title row 42 → 30px; tighter secondary text.
- Charts 215 → 175px; entity rows tightened; smaller card headings and corners.
- Signals and preferences follow their own columns, avoiding a second section row.
- Expert-facing dashboard: concise labels and readings; explanations live here.

## Temperature basis revision · 2026-09-25

- Feels-like is primary while enabled; measured temperatures shown only when off.
- Conditional temperature charts instantiate only the selected basis.
- Room cards already subscribe to both sensors and the selector; no polling added.
- Missing selected temperature displays a dash, not a misleading numeric zero.
