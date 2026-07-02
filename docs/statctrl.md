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
