# Ideas / roadmap

Proposals, roughly ordered. Status markers: `[ ]` proposed, `[~]` in
progress, `[x]` done.

## 1. Near-term fixes (small, low risk)

- `[x]` statctrl `save_adaptive_model`: merge only this room's keys to fix
  the cross-instance overwrite race (docs/statctrl.md).
- `[x]` actrl `_set_static_pressure`: bound the retry loop (5 attempts →
  warn and continue) so a dead ESPHome device can't wedge the app.
- `[x]` actrl: guard `avg_deriv = deriv_sum / weight_sum` against
  `weight_sum == 0`.
- `[x]` actrl: replace stray `print()` with `self.log()`; rename the local
  that shadows module-level `climate_entity` in `_get_current_targets`.
- `[ ]` Reduce/eliminate blocking `time.sleep` in callbacks (AppDaemon
  `run_in` continuations, or at minimum document why each sleep exists).

## 2. Testability refactor

Progress: task 001 (pure-class extraction, done), task 002 (headless
whole-cycle harness + golden scenarios, done), task 003 (capacity controller
extraction, ready for implementation), task 004 (calibration data export,
draft). See `docs/tasks/`.

Extract the remaining control logic from the `hass.Hass` classes so it can
be unit-tested and later driven by a simulator:
- `compress`/`midea_reset_quirks`/`midea_runtime_quirks` — refactor into a
  `MideaCapacityController` object whose inputs are (error, deriv, mode,
  defrost_detected) and whose outputs are (fake temp offset, speed estimate).
  This encodes the irreplaceable reverse-engineered behaviour; it deserves
  the densest test coverage.
- The PID integral-adjustment passes in `_calculate_pid_outputs` (top-zone
  anti-runaway, normalisation, minimum airflow, negative clamp) — pure given
  (pid states, errors, airflow weights, door states, fan power).
- Keep the AppDaemon class as a thin I/O shell: read entities → call pure
  logic → write entities/services. Config (rooms, entity names, constants)
  becomes a dataclass so tests/simulator can construct variants.
- Tooling: `uv`-managed dev venv, `pytest`. Tests must not import `hassapi`.

Test ideas once extracted:

- Step-sequence invariants: any sequence of demands leaves `prev_step`
  returning to 0; emitted values never set the ramp-up flag unintentionally
  (never ≥ stable+2 outside faithful/hysteresis mode); speed estimate stays
  in [0, increments+margin].
- Deadband integrator: symmetric behaviour, ±1 emission rate vs error size,
  preload on sign change.
- Scenario replays: synthetic error trajectories (approach, overshoot,
  purge-window, defrost) asserting on/off decisions match current behaviour
  (golden tests to protect against regressions during refactoring).
- statctrl: slew step arithmetic, deadline-driven step intervals, adaptive
  EWMA update maths, model merge.

## 3. House thermal simulator (the big one)

Goal: a model of the house + HVAC accurate enough that controller changes can
be evaluated in simulation instead of live trial on the family.

### Components

- **Thermal model**: per-room lumped RC network (room air node, envelope
  node, inter-room coupling, common return-air path). Inputs: outdoor
  temp/humidity, solar gain (can reuse EMHASS PV/irradiance data as a proxy),
  internal gains, window/door states, delivered airflow × supply-air temp per
  room. Low thermal mass (per archive/README.md) suggests a 1R1C or 2R2C per
  room may suffice.
- **HVAC model**: compressor increment → capacity and electrical power as a
  function of indoor/outdoor conditions (heating COP degrades with outdoor
  temp; defrost cycles below ~7 °C outdoor); fan power already measured
  (table in actrl.py `_calculate_pid_outputs`); duct/damper model mapping
  damper positions + static pressure + fan speed → per-room airflow split.
- **Midea controller emulator**: the flip side of
  `MideaCapacityController` — given fake temperature reports, emulate the
  observed stepping/flags/purge/shutdown behaviour so actrl's protocol
  handling is exercised end-to-end.
- **Harness**: runs extracted control logic against the simulator with a
  fake entity layer; produces comfort metrics (time-in-band, overshoot),
  energy use, compressor cycling counts.

### Calibration / cross-check

- Source recorded data from HA (recorder DB and/or InfluxDB — see
  ai-energy-forecast-slop repo notes) : room temps, damper positions, comp
  speed estimate, fan mode, static pressure, outdoor conditions, unit power.
- Fit RC parameters per room via system identification on periods of known
  input (nightly free-running decay is ideal for envelope parameters; step
  changes in damper position for airflow split).
- Validation: replay a held-out day's inputs through the simulator, compare
  simulated vs recorded room temperature trajectories; report per-room RMSE.
  Continuously re-check as seasons change.

### Payoff

- Regression-test controller changes (does the new deadband logic still
  avoid the purge cycle? does it cycle the compressor more?).
- Tune constants (PID gains, thresholds) offline against comfort/energy
  objectives.
- Quantify grid-surplus strategy value in kWh shifted vs comfort cost.

## 4. Smaller ideas

- `[ ]` Config as data: move rooms/entities/constants into `apps.yaml` args
  or a dataclass config so a second house / test rig is possible.
- `[ ]` Structured metrics: publish PID terms and controller state as JSON
  attributes on one entity rather than many `input_number`s (fewer recorder
  rows, easier analysis).
- `[ ]` Watchdog: alert if the main loop stops running or a service call
  repeatedly fails (currently failures are silent in logs).
- `[x]` Deploy script: `./deploy.sh` copies the deployable files into
  `appdaemon/`.
- Won't do (for now): anything building on statctrl adaptive optimum start
  (e.g. exposing learned minutes-per-degree as sensors) — feature trialed
  and deliberately left inert, not a focus area (docs/statctrl.md).
