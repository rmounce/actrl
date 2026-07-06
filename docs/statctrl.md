# statctrl.py — implementation analysis

Analysis as of 2026-07-02, including the adaptive optimum start work.

## Purpose

Per-room setpoint scheduler. One AppDaemon app instance per room (configured
via `args`: `room`, optional `type` limiting to heat or cool). It moves the
room's `climate.<room>_aircon` heat/cool setpoints between comfort levels, and
rate-limits how fast setpoints change so actrl's PIDs see ramps rather than
steps. It only acts when the room thermostat is in `heat_cool` mode and
`input_boolean.<room>_manual_ac` is off.

## State machine

Room state per mode, priority order (`get_current_state`):

1. `window_open` — `binary_sensor.<room>_window` on; target = inactive
   setpoint ∓ 2 °C (no dedicated helpers, derived).
2. `turbo` — `timer.<room>_timed_turbo` active.
3. `active` — `timer.<room>_timed_active` active or
   `input_boolean.<room>_scheduled_<mode>` on (driven by HA scheduler
   switches).
4. `inactive` — otherwise.

Setpoints per state come from `input_number.<room>_setpoint_<state>_<low|high>`
(low = heat, high = cool).

## Slewing

- **Toward comfort ("slew on")**: steps of the climate entity's
  `target_temp_step` (default 0.1 °C), rate from
  `input_number.statctrl_slew_on` (°C/h); with a deadline (scheduled start)
  the interval is computed so the ramp lands on time.
- **Away from comfort ("slew off")**: same stepping, rate from
  `input_number.statctrl_slew_off`.
- Immediate jump when currently *outside* the target bound in the comfort
  direction (e.g. setpoint below a rising heat target) and adaptive mode is
  off.
- Timers per mode (`active_timers`) drive the next step; a 60 s
  `periodic_check` and `listen_state` on all relevant entities catch
  everything else. actrl ramps its internal targets at half the rate statctrl
  slews, so the two don't cancel (see comment on
  `grid_surplus_open_window_rate` in actrl.py).
- Pre-start ramping (non-adaptive): while inactive, if the active setpoint
  couldn't be reached by the next scheduled start at the slew-on rate, start
  ramping early. Next start time is discovered by inspecting HA scheduler
  (`switch.schedule_*`) attributes for schedules that turn on
  `input_boolean.<room>_scheduled_<mode>`.

## Adaptive optimum start

**Status: deliberately inert (2026-07-02).** Trialed, results not liked,
disabled; not a focus area for future work. The code remains but does
nothing unless explicitly enabled. Enabling requires HA helpers that
intentionally do not exist:

- `input_boolean.statctrl_adaptive_optimum_start` — global switch.
- `input_boolean.<room>_adaptive_optimum_start` — per-room override.
- Or `adaptive_optimum_start: true` app arg in `apps.yaml`.

With none present, `adaptive_enabled()` falls back to the arg default
(false). The "Entity ... not found" warnings from each statctrl instance at
init are the `listen_state` subscriptions to these absent helpers — expected
and harmless.

Learns each room's actual minutes-per-degree and uses it (× safety factor
1.2, capped at 180 min lead) to decide when to begin the pre-start ramp,
instead of the naive slew-rate heuristic.

- Enable via `input_boolean.<room>_adaptive_optimum_start` (per-room), falling
  back to `input_boolean.statctrl_adaptive_optimum_start` (global), falling
  back to app arg `adaptive_optimum_start` (default off).
- Learning sessions start at pre-start or when a schedule flips to active with
  ≥ 0.4 °C comfort error; they end when the room is within 0.2 °C of target.
  Samples are clamped to 10–180 min/°C and folded into an EWMA (α = 0.25) per
  `room:mode` key.
- Model persisted to `statctrl_adaptive.json` next to the app (path
  overridable via `adaptive_model_path` arg); written atomically via a
  per-room tmp file + `os.replace`. Saves reload the file and merge only this
  room's keys, so concurrent per-room instances can't revert each other.
- When adaptive is enabled, "active with error" transitions also slew instead
  of jumping (`update_setpoint` → `slew_on_step`), so learned rates reflect
  the same mechanism used for pre-start.

## Price-aware warmup start

Implemented 2026-07-07 (docs/pricing.md "Warmup-start chooser" — the
spike-morning money, validated in sim at −36% on the 06-22 spike with
better comfort). Heat mode only.

- Gated by `input_boolean.statctrl_price_aware`; missing/off, or any
  missing price entity, means shift 0.0 = behaviour unchanged.
- `get_price_shift`: within 8.5 h of the next scheduled start, asks
  `control.warmup_decide` (coarse expected-cost model: warm at capacity,
  then pay MARGINAL hold vs the free-floating counterfactual; forecast
  valued at g_lambda = fc + upside knots) whether starting earlier than
  just-in-time buys the same warmup energy cheaper. Re-decided every
  check with the freshest forecast; frozen once the shifted start time
  arrives so a ramp in progress never flip-flops.
- Applied by moving the whole pre-start machinery earlier:
  `effective_start = next_start − shift` replaces `next_start` in the
  adaptive call, the slew-rate trigger, and the slew deadline. Inside the
  shift window the setpoint ramps at the slew-on rate.
- Hold guard: between the shifted start and the real schedule the
  slew-off branch is suppressed (a committed session only — an
  uncommitted evening decision must not freeze the normal slew-off).
- Inputs: `sensor.dh_unit_load_cost` attr `unit_load_cost_forecasts`
  (EMHASS-published tariffed price per 30 min), `sensor.temperature_adelaide`
  (held constant over the horizon), current room temp, active setpoint
  as the day target, hours-to-start as the deadline. Entity names
  overridable via app args `price_forecast_entity` /
  `outdoor_temp_entity`.
- Per-room decisions from house-level constants: rooms sharing a
  schedule reach the same answer; the model steers WHEN energy is
  bought, not what temperature anything is driven to.
- Deadline = schedule start (the study's deadline was the end of the
  recorded target ramp, slightly later) — errs a touch early, comfort-
  safe.

## Known issues / risks

- `handle_adaptive_start` returns `True` for both "too early, deliberately
  waiting" and "started pre-start ramp", which makes `update_setpoint`'s
  control flow hard to trace.
- `get_next_scheduled_start`: `actions`/`timeslots` locals unused; assumes
  the HA `scheduler` custom component's attribute schema (`next_slot`,
  `next_trigger`, `actions`, `entities`) — document/verify against the
  installed version if it misbehaves.
- `get_current_setpoint` does a redundant `get_state` (fetches, ignores,
  fetches again) and will `TypeError` on `float(None)` if the climate entity
  is briefly unavailable (partly guarded by the `heat_cool` check upstream).
- `set_climate` writes both `target_temp_low` and `target_temp_high` each
  call, reading the other mode's current value — benign, but means heat and
  cool updates can race across the two mode loops within one app.
