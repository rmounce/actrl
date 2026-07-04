# actrl.py — implementation analysis

Analysis of the implementation as of 2026-07-02. Line references are
approximate and will drift.

## Purpose

Single AppDaemon app (`Actrl`) providing multi-room climate control on top of
a one-zone Midea ducted unit. Two control problems are solved simultaneously:

1. **Room balance** — per-room PIDs map temperature error to damper position.
2. **Capacity control** — aggregate demand is converted into fake "follow me"
   temperature reports that trick the Midea controller into stepping its
   compressor speed up/down one increment at a time.

## Hardware / entity surface

- `climate.m5atom_climate` — the Midea unit via ESPHome.
- `esphome/m5atom_send_follow_me` — service used to report a fake ambient
  temperature (the entire capacity-control mechanism).
- `number.m5atom_static_pressure` — duct static pressure setting 1–4.
- `binary_sensor.m5atom_compressor`, `binary_sensor.m5atom_outdoor_fan` —
  used to detect defrost cycles (compressor on + outdoor fan off in heat mode).
- `cover.<room>` — zone dampers (5% position steps, matching a Zone10e).
- `sensor.<room>_average_temperature` / `sensor.<room>_feels_like` — inputs.
- `climate.<room>_aircon` — virtual per-room thermostats (targets set by
  statctrl or the user); `heat_cool` mode enables grid-surplus play.
- `binary_sensor.<room>_window`, `binary_sensor.<room>_door` — comfort and
  airflow adjustments.
- Various `input_number.*` entities used both as dashboards metrics and as
  persistence across app restarts (`aircon_comp_speed`,
  `grid_surplus_integral`).
- Rooms: bed_1, bed_2, bed_3, study (airflow weight 1.0), kitchen (2.0 — two
  ducts).

## Main cycle (10 s, `main()`)

1. Manual-mode escape hatch: `input_boolean.ac_manual_mode` resets internal
   state and skips the cycle.
2. Read temperatures (feels-like optional w/ fallback), update window-open
   offsets.
3. Read per-room targets from `climate.<room>_aircon` (heat/cool/heat_cool);
   ramp internal targets toward them (`_update_room_target`) — step at most
   0.1 °C plus proportional/linear smoothing so setpoint changes don't shock
   the PIDs' derivative terms (MyDeriv also compensates for target deltas
   directly).
4. `_add_grid_surplus`: average EMHASS curtailment forecast over the next hour
   (`sensor.mpc_p_pv_curtailment` attr `forecasts`) feeds an integral that can
   widen room targets by up to 1.0 °C (bounded by 21 °C min cooling / max
   heating and the room's heat/cool band midpoint). Window-open offsets erode
   it; overshoot is bled off to prevent wind-up.
5. `_calculate_demand`: per-room signed errors for both modes; demand = max
   room error per mode. If demand exceeds `grid_surplus_max_offset` the
   surplus integral is trimmed and errors recomputed.
6. `_determine_new_mode`: hysteresis via `immediate_off_threshold` (−1.5) so
   an active mode is sticky; mode change or None resets PIDs and turns off.
7. `_calculate_pid_outputs` (see below) → damper values
   (`100 * output / 2.0`, i.e. full damper travel across 2 °C of PID output).
8. Compressor demand: `weighted_error` = demand of the *worst* zone (not the
   damper-weighted average — deliberate change, the weighted average is still
   computed for the derivative); `compress()` turns error + predicted
   derivative into a small integer offset from the setpoint; the fake
   temperature `setpoint + offset` is transmitted.
9. Off handling: `off_fan_running_counter` gives cooling mode a 2.5 min fan
   run-on; dampers set open-only while stopping.
10. Static pressure "gear change": after 30 min saturated at max power, bump
    static pressure to 4 (requires power-off to change). Fan speed follows
    guesstimated compressor speed with hysteresis (low/medium/high).
11. On power-on transitions, extra follow-me packets with 1 s spacing work
    around ordering/processing races in the unit.

## PID machinery

- `MyWMA` — linearly-weighted moving average over a fixed window.
- `MyDeriv` — WMA of deltas × lookahead factor; compensates target changes so
  setpoint steps don't spike D. Room PIDs predict 2 min ahead over a 10 min
  window; the global temp derivative predicts 10 min ahead over 5 min.
- `MyPID` — textbook P+I+D with externally adjustable integral. That external
  adjustment is the interesting part:
  - **Top-zone anti-runaway**: if the top two zones' outputs differ by more
    than `normalised_damper_range - room_pid_minimum` (2.1), the top zone's
    integral is trimmed so it can't starve the next zone.
  - **Normalisation**: all integrals are shifted equally each cycle so
    `max(output) == normalised_damper_range` (2.0) — the hungriest room always
    pins its damper fully open and the compressor, not the dampers, modulates
    total capacity.
  - **Minimum airflow**: measured fan power table (static pressure × fan
    speed, baseline SP2/low ≈ one open duct) sets a minimum sum of
    airflow-weighted damper outputs; integrals are nudged up in a loop until
    satisfied. Closed doors derate a room's airflow weight to 0.25.
  - **Negative clamp**: integral may only wind down to −0.1
    (`room_pid_minimum`) so a satisfied room hovers just below opening rather
    than accumulating unbounded wind-down.

## Capacity control (`compress` + `midea_runtime_quirks`)

The Midea controller, fed follow-me temperatures, behaves like a stepper:

- Each 1 °C change of reported error steps compressor speed by ±1 increment
  (~14 increments to saturation). A report exactly at setpoint holds speed
  (no step): this is what makes the step-down sequence net −1 rather than
  −2 (observed via closed-loop replay vs the 2026-06-22 recorded taper,
  2026-07-04).
- `reported >= setpoint + 2` sets a **ramp-up flag**. Closed-loop replay
  against recorded data (2026-06-22 06:20–09:20 taper, 2026-07-04) shows it
  clears on a decrement-demand report (`reported <= setpoint − 1`, i.e. the
  step-down sequence's leading elements) — the real unit follows a
  step-down sequence back down instead of redlining forever. The latched
  climb itself is slow, ~1 increment per 3–6 min (not every 10 s cycle).
- `reported <= setpoint − 1` sets a **ramp-down flag**; `>= +1` clears it.
- Step sequences are crafted to end with no flags set:
  step up = `[+1, +2, 0]`, step down = `[−1, −2, +1, 0]` (offsets from
  stable). One sequence element is emitted per 10 s cycle via `prev_step`.
- After 90 min of continuous low-speed running the unit does a ~1 min
  full-speed **purge**. `compress()` tightens the off-threshold with runtime
  (`immediate_off_threshold` → `eventual_off_threshold` over 45 min) so the
  system prefers to turn off before paying for a purge.
- At the setpoint-reached temp for ~1 h the unit shuts down entirely; a
  periodic "blip" (`min_power_time`) resets its internal timer.
- **Defrost detection** (heat, compressor on, outdoor fan off) forces the
  speed estimate to max, since the unit restarts at full speed after defrost.
- Soft start: first 7.5 min report "stable − 1" (min power) to cover
  compressor start + possible defrost, then ramp to true demand over 2.5 min.
  Errors > 2 °C (`faithful_threshold`) bypass soft start and hand control back
  to the Midea's own logic ("faithful" mode) with simple max-power hysteresis.
- `guesstimated_comp_speed` is open-loop dead reckoning of the unit's current
  increment, persisted in `input_number.aircon_comp_speed`; it saturates with
  a ±2 safety margin and is corrected to extremes on faithful/min-power
  events.
- `DeadbandIntegrator` provides the fine control signal in the stable region:
  it accumulates error and emits ±1 step requests, with a ±0.75 preload when
  the error changes sign so small errors act quickly.

## State persistence

Restart-safe state lives in HA entities: comp speed estimate, grid surplus
integral; mode is inferred from the climate entity ("assuming aircon already
running"). Everything else (PIDs, counters) restarts cold.

Warm start (app reload while the unit is running, observed 2026-07-02):
`on_counter` is set to `soft_delay + soft_ramp`, so a restart mid-soft-start
bypasses the remaining soft start; `min_power_counter` resets to 0,
restarting the tightening-off-threshold/purge clock; PID integrals cold-start
and are re-normalised within one cycle (dampers held by the deadband). All
accepted by design — warm starts are rare and don't need perfect continuity,
only a consistent state, which the persisted speed estimate provides. The
weak spot would be a restart mid-ramp-up with a stale
`input_number.aircon_comp_speed`.

## Known issues / risks

- **Blocking sleeps in callbacks**: up to ~2.5 s of `time.sleep` per cycle in
  a 10 s `run_every` callback (AppDaemon thread-pool pressure; warnings on
  overrun). `_set_static_pressure` retries are bounded (5 × 1 s) but still
  block while active.
- Tested: the pure classes (task 001) and the capacity-control logic
  (task 003, `MideaCapacityController`) live in `control.py` with golden
  unit tests, plus whole-cycle golden scenarios via the headless harness
  (task 002). The remaining untested logic is the PID
  integral-adjustment passes and I/O plumbing inside `Actrl` itself,
  covered only at cycle level.
- All configuration (rooms, entities, thresholds) is module-level constants;
  fine for one house, hostile to testing/simulation.
