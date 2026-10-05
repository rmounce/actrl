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
- `esphome/hvac_xye_send_follow_me` — service used to report a fake ambient
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
- `sensor.actrl_status` — deliberate monitoring/display contract. State is
  `initializing`, `manual`, `inhibited`, `switching`, `idle`, `heating`, or
  `cooling`; attributes carry mode, lead room/temperature/target, signed
  demand, active-room count, capacity step/max, grid offset, and heartbeat.
  Capacity step is encoded as a numeric string because AppDaemon's HA REST
  adapter otherwise prunes numeric zero as falsey.
- Rooms: bed_1, bed_2, bed_3, study (airflow weight 1.0), kitchen (2.0 — two
  ducts).

## Main cycle (10 s, `main()`)

1. Pause escape hatches: `input_boolean.ac_manual_mode` and the ATOM S3 local
   inhibit switch reset internal state, publish their pause state, and
   skip control. Pause entry and release log once rather than every ten-second
   cycle. Releasing either resumes from reset state. Local inhibit is
   still enforced independently in ESPHome if AppDaemon or HA is unavailable.
   `input_boolean.actrl_debug_logging` changes this app's log threshold between
   INFO and DEBUG immediately; it does not reload or reset the controller.
2. Read temperatures (feels-like optional w/ fallback), update window-open
   offsets.
3. Read per-room targets from `climate.<room>_aircon` (heat/cool/heat_cool);
   ramp internal targets toward them (`_update_room_target`) — step at most
   0.1 °C plus proportional/linear smoothing so setpoint changes don't shock
   the PIDs' derivative terms (MyDeriv also compensates for target deltas
   directly).
4. `_add_grid_surplus`: average EMHASS curtailment forecast over the next hour
   (`sensor.mpc_p_pv_curtailment` attr `forecasts`) feeds a shared integral.
   Applied room offsets are bounded by 21 °C min cooling / max heating and
   the room's heat/cool band midpoint; they can exceed 1.0 °C. The 1.0 °C
   constant limits resulting demand by trimming the integral. Window-open
   offsets erode applied offsets; overshoot is bled off to prevent wind-up.
   During cooling decay, the integral rate doubles when the demand-leading
   room has less applied surplus offset than another room. Growth and
   matched-leader decay retain the original rate.
5. `_calculate_demand`: per-room signed errors for both modes; demand = max
   room error per mode. If demand exceeds `grid_surplus_max_offset` the
   surplus integral is trimmed and errors recomputed.
5b. `_apply_price_pressure` (docs/pricing.md): a continuous K offset from
   live vs forecast retail prices (`control.price_pressure_offset`; k=2,
   tau=20 h, clamps −0.75/+1.5 K) is added to room errors AFTER
   `_calculate_demand` so the grid-surplus integral bookkeeping never sees
   price demand. Positive (bank = pre-heat/pre-cool ahead of dear hours) is
   capped per room at the same 21 °C/midpoint/window bounds as grid
   surplus, net of any surplus offset already applied; negative (shave
   during the dear hour) applies as-is. Gated by
   `input_boolean.ac_use_price_pressure`; boolean or price entities missing
   ⇒ offset 0 and bit-identical behaviour. Reads
   `sensor.amber_5min_current_general_price` (live retail) and
   `sensor.dh_unit_load_cost` attr `unit_load_cost_forecasts` (EMHASS-
   published tariffed forecast, follows the configured price source — the
   HWC planner's feed). Metric: `input_number.aircon_price_pressure`
   (only written while the boolean is on).
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
  - **Target-step catch-up** (deployed 2026-10-06): a requested
    heating target rise / cooling target fall of ≥1.0°C in one observed
    cycle copies the current raw PID leader's integral into that room when it
    is an increase, preserving the room's own P+D response. This can make a
    newly cold room the immediate leader. Effective demand must be ≥0.5°C.
    Runs once before anti-runaway, normalisation and airflow protection;
    no global gain change. Small steps never accumulate into a trigger.
    Trigger uses requested targets; demand gate uses smoothed targets plus
    feels-like/window/surplus/price effects. Offset changes alone cannot
    trigger it. First observations, absent targets, mode transitions and
    pause/resume do not seed. A failed demand gate is not deferred.
    Multiple qualifying rooms use the same pre-seeding leader's integral, so
    processing order cannot cascade boosts. Integral memory persists afterward;
    normal PID operation unwinds it. A missed
    observation can combine several external changes into one observed
    step; no timer-source discrimination is attempted.
  - **Top-zone anti-runaway**: if the top two zones' outputs differ by more
    than `normalised_damper_range - room_pid_minimum` (2.1), the top zone's
    integral is trimmed so it can't starve the next zone.
  - **Normalisation**: all integrals are shifted equally each cycle so
    `max(output) == normalised_damper_range` (2.0) — the hungriest room always
    pins its damper fully open and the compressor, not the dampers, modulates
    total capacity.
  - **Negative clamp**: integral may only wind down to −0.1
    (`room_pid_minimum`) so a satisfied room hovers just below opening rather
    than accumulating unbounded wind-down. Runs before the minimum-airflow
    top-up (which no longer props raw outputs up — the clamp must see raw
    outputs).
  - **Minimum airflow** (`min_airflow_inflation`, module-level): measured fan
    power table (static pressure × fan speed, baseline SP2/low ≈ one open
    duct) sets a minimum sum of airflow-weighted damper outputs; this cycle's
    outputs are topped up by an equal increment until satisfied. Stateless
    since 2026-07-06 — integrals are never written, so the top-up carries no
    memory and a satisfied zone hands airflow back the moment other zones
    cover the minimum (was: integral nudges that ratcheted satisfied zones
    up for tens of minutes; study in docs/tuning.md "Min-airflow inflation
    policy"). Closed doors derate a room's airflow weight to 0.25.

### Zone activation: capacity / minimum-airflow feedback

- 2026-10-06 Adelaide 06:41–06:44, confirmed from raw InfluxDB climate /
  zone PID history and AppDaemon logs: reported `m5atom_climate.fan_mode`
  alternated `off`/`low` after the 06:39:28 start; held `low` from
  06:41:18.443 to 06:43:49.630, then resumed alternating. PID calculation
  precedes the cycle's fan command. `off` is absent from the airflow power
  table, so it falls back to `high`. At SP2-equivalent minimum, bed 1 fully
  open plus kitchen 24.227% supplies 144/97 duct-equivalents; kitchen's
  published PID is 0.7773. At 06:41:28 the lower minimum removed the
  stateless top-up: kitchen PID fell to -0.0543 and its damper closed;
  bed 2/3/study fell roughly 0.8 while bed 1 stayed 2.0. At 06:43:58
  top-up returned and kitchen reopened. Published zone PID includes airflow
  inflation, so these jumps do not establish integral resets. Cause of the
  device's `off` reports remains unverified; static pressure / all door
  states were not independently recovered from this Influx window.
  Follow-up: built ESPHome `midea_xye/air_conditioner.cpp` immediately
  publishes commanded fan mode in `control()`, then C0 polls overwrite it
  from the reported fan-speed nibble, including `FAN_MODE_OFF`. Actrl
  reissues `low` every 10s when it sees `off`; observed low publications
  coincide with those cycles. HVAC state stayed `heat`. Shelly 30s means:
  outdoor power rose ~98W at 06:39:30 → 2.00kW at 06:40:30, then
  settled ~575W; indoor power rose ~7W → 63–71W at 06:41:30–06:43:00,
  then ~51W after 06:44. Thus command/feedback alternation is confirmed;
  physical on/off cycling is unsupported. Initial fan delay is consistent
  with heating warm-up, but later `off` reports persisted with indoor draw
  ~51W: do not equate reported fan `off` with physical fan stopped.
- Same morning 07:03:11: raw climate history changed from alternating
  `off`/`low`, `hvac_action=idle`, to stable `low`, `hvac_action=heating`;
  HVAC mode remained heat. At 07:03:18 kitchen PID 0.777250 → 0.554720,
  damper command 24.227% → 14.607% (AppDaemon log). Bed 1 stayed 2.0;
  bed 2/3/study dropped ~0.23–0.33. Same pre-fix airflow fallback removal,
  despite ongoing unit operation; smaller jump because kitchen's underlying
  output was already positive. Compressor estimate and surplus integral
  remained zero. No app reload in the event window. The requested-speed fix
  deployed at 07:35 removes this reported-state dependency too.

### Indoor fan feedback: temperature-gated speed, 2026-10-06

- Correction to the initial diagnosis: 07:03 includes a real sustained
  indoor electrical-power increase, not just command/report alternation.
  Raw Shelly indoor channel 2, 06:53–07:13: minute means ~50–51W before
  the event, ~70W afterward. Individual samples: 07:03:07 50.15W,
  :14 52.85W, :15 56.67W, :16 59.76W, :17 63.51W, :19 73.02W,
  :24 69.27W. Report switched to stable low at 07:03:11, before the
  kitchen damper command at :18. The fan change cannot be attributed
  solely to that later damper movement. Outdoor minute means rose
  gradually ~626W → 691W beforehand, then settled ~671–675W.
- Coil inlet sensor: initial low-report transition at 06:41:18 bracketed
  by 31°C at :17 and 32°C at :21; off-report transition at 06:43:49
  coincided with 27.5°C. Coil then warmed slowly; second low transition
  at 07:03:11 coincided with 32°C. Consistent with heating cold-draft
  protection selecting a reduced fan regime below a coil-temperature
  threshold, with hysteresis. Hypothesis, not a proven threshold or RPM
  mapping. ~50W versus ~70W suggests two running regimes; motor RPM and
  actual duct flow were not measured. Static pressure was not recovered.
- Cached built ESPHome `midea_xye/air_conditioner.cpp` / `.h`: C0 RX byte
  9 low nibble maps 0=off, 4=low, 2=medium, 1=high; bit 0x80 selects auto.
  `hvac_action` in heat is derived from whether that same nibble is zero,
  so idle/heating is not independent compressor evidence. Both labels
  misdescribe this running reduced-power regime if zero is intentional.
  Command `control()` publishes fan_mode optimistically; C0 overwrites it.
  C3 TX uses mutable fan_mode when the queued command is sent; off maps
  to auto in its default branch. Actual TX bytes were not captured, so
  command/feedback races and auto fallback remain unquantified.
- Deployed actrl fix removes artificial high-minimum inflation and report
  driven hysteresis changes. It does not explain/measure physical fan
  regulation, prove the low-speed airflow table for this reduced regime,
  or separate request from feedback inside ESPHome.
- Next evidence: passively log timestamped C0 byte 9 (full byte), C3 TX
  fan byte 7, coil temperature, protect flags, static pressure and indoor
  watts over natural transitions. Compare requested low/medium/high with
  actual reports/power and temperature hysteresis. Preserve raw codes;
  do not rename zero to stopped or assign an RPM without measurement.
  No live fan experiments or firmware changes made in this investigation.

- Hypothesis raised during the 2026-09-09 study timer review: an incumbent
  zone's integral advantage delays airflow redistribution. The newly cold
  zone retains a large error; capacity demand uses the maximum room error
  (despite the `weighted_error` name), so compressor demand can rise before
  that zone receives its intended airflow. Temperature derivative feedback
  is airflow-weighted and also changes with allocation.
- `_determine_fan_mode` raises requested fan speed with estimated compressor
  steps. Fix deployed 2026-10-06 07:35 Adelaide: fan hysteresis uses the
  last requested speed, independent of reported `off` / command echoes.
  `_calculate_pid_outputs` uses static pressure and the higher of the last
  requested speed and the currently planned speed to set minimum airflow.
  Increases prepare airflow before the command; decreases retain protection
  until the lower request has been sent. Startup seeds the request from a
  valid low/medium/high report, otherwise from the restored compressor
  estimate. Unknown requested speeds retain the conservative high fallback.
  Reported feedback still triggers retransmission of the requested command.
  A higher minimum can require satisfied rooms
  to remain open even after the new room reaches full opening, reducing its
  share of total airflow and delivering unwanted heat elsewhere. This is a
  dynamic feedback path; the airflow top-up itself is stateless.
- Example at SP2, doors open: low requires 1.0 duct-equivalent; medium
  requires 115/97 ≈ 1.186; high requires 144/97 ≈ 1.485. Study supplies
  at most 1.0. If kitchen alone supplies the remainder, its two-duct weight
  requires ~9.3% opening at medium or ~24.2% at high. These are illustrative
  command-space minima, not reconstructed actual flows or this event's
  confirmed static-pressure setting.
- Earlier aggressive redistribution could reduce later capacity/fan demand
  and avoid that constraint becoming binding. Candidate comparison:
  original PID, match-leader seeding, proportional-error-relative seeding
  (optionally retaining D). Preserve airflow protection; test whether the
  need for top-up falls, rather than weakening the protection.
- Evidence, Adelaide 2026-09-09: study requested 47.645% at 08:33:10 and
  100% at 08:39:00; kitchen first requested closing at 08:39:50. InfluxDB
  `climate.m5atom_climate` fan reports first show medium at 08:33:31;
  medium is continuous from 08:42:29 until low at 09:07:11. Kitchen average
  temperature peaked at 21.02°C at 09:02:17 against an unchanged 20°C
  target (review window ends 10:16). This supports the proposed sequence,
  but does not establish how much overshoot minimum airflow caused.
- First live proportional-relative activation, Adelaide 2026-09-09 22:56:
  bed 1 heat target changed 16 → 19.5°C at effective 18.5°C. On the next
  control cycle bed 1 PID changed about -.098 → 2.0 and its damper target to
  100%; kitchen changed 2.0 → .77725 and 24.227%. The latter matches the
  configured convex damper mapping exactly. Compressor estimate moved 0 → 2
  within 20s and to 3 after 3m30s. This confirms coherent policy execution;
  it does not establish comparative energy savings.
- Reported-state quirk: before continuous medium, fan reports alternate
  requested low/medium with off while `hvac_action=idle`, then medium/low
  while heating. The pre-fix version treated `off` as outside the airflow
  table and fell back to high; the requested-speed fix above removes
  that dependency. Do not equate every report with physical fan speed.
  Need cycle-aligned fan/pressure/door inputs and pre/post-top-up PID outputs
  to establish when the constraint bound. Pressure query returned no rows;
  P/I/D components were not logged at INFO. Preserve this uncertainty.

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

Deployment 2026-10-06 07:35:25 Adelaide: `./deploy.sh`; AppDaemon reloaded
actrl and room schedulers, inferred the AC already running, and entered the
first control cycle at 07:35:35 without logged errors. Scheduler initialization
warned about missing optional price/adaptive-start input booleans. Direct HA
REST verification using the documented config token returned HTTP 401.

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
