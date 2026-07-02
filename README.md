# actrl

AppDaemon apps that turn a single-zone Midea ducted reverse-cycle aircon into a
multi-room, variable-capacity climate control system in Home Assistant.

The stock unit exposes only a dumb thermostat. These apps gain incremental
compressor speed control by spoofing the "follow me" remote temperature
feature, and per-room control by driving zone dampers from per-room PIDs.
Solar surplus (via EMHASS forecasts) opportunistically widens room targets to
soak up energy that would otherwise be curtailed.

## Apps

- **`actrl.py`** — the whole-of-house control loop (10 s cycle). Per-room PIDs
  drive damper positions; the weighted demand is compressed into fake
  "follow me" temperatures that step the compressor speed up/down through
  reverse-engineered Midea controller behaviour. Handles soft start, defrost,
  purge cycles, minimum airflow, static pressure and fan speed selection, and
  grid-surplus target offsets. See [docs/actrl.md](docs/actrl.md).
- **`statctrl.py`** — per-room setpoint scheduler (one app instance per room).
  Moves each room's `climate.<room>_aircon` setpoints between
  turbo/active/inactive/window-open levels with rate-limited slewing, driven
  by HA scheduler switches and timers. Includes adaptive optimum start that
  learns each room's warm-up/cool-down rate. See
  [docs/statctrl.md](docs/statctrl.md).

## Layout

- `actrl.py`, `statctrl.py` — source of truth for the active apps.
- `control.py` — pure (HA-independent) control classes used by actrl;
  covered by golden tests in `tests/` (`uv run pytest`).
- `appdaemon/` — gitignored deployed copies; a diff vs top level means
  undeployed changes.
- `archive/` — retired experiments (EMHASS/MPC-style planner), kept for
  reference.
- `docs/` — implementation analysis and [ideas/roadmap](docs/ideas.md).

## Requirements

Home Assistant with AppDaemon, an ESPHome device (`m5atom`) bridging the Midea
unit (climate entity, follow-me service, static pressure number, compressor /
outdoor fan binary sensors), zone dampers as `cover.<room>` entities, per-room
temperature sensors, and assorted `input_number` / `input_boolean` helpers.
Entity names are currently hardcoded in the apps.
