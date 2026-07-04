# Task 012: Clearable ramp-up latch + slow latched climb in MideaUnit

## Status
done

## Background

`sim/midea_unit.py` implements the unit's ramp-up flag per control.py's old
comment "no known way to clear": once `reported_error >= +2` for 3
consecutive cycles, the emulated unit ramps to max at 1 increment per 10 s
cycle and ignores every subsequent report until power-off.

Recorded data disproves both parts (closed-loop replay investigation,
2026-07-04, sessions logged in docs/tasks/010 baselines):

- On 2026-06-22 06:20–09:20 the real outdoor unit peaked at ~2.5 kW
  (mid-speed) and then tapered smoothly in lockstep with actrl's step-down
  sequences (`[-1, -2, +1, 0]` offsets) from 07:15 to 09:20. The step-down
  sequence's leading elements reach reported error −1/−2, and the real unit
  demonstrably follows them back down — so a decrement demand clears the
  latch in practice.
- The real climb rate is ~1 increment per 3–6 min, not 1 per 10 s.

In closed-loop replays the unclearable latch pins the sim at increment 14
for hours, producing a hard cutoff instead of a taper and +0.7–1.3 K
morning overshoot (2026-06-21/22). A scratch fix (clear on decrement
demand + slow latched climb) cut kitchen RMSE 0.394→0.364 (06-22) and
0.470→0.433 (06-21) and halved the overshoot. This task productionises it.

## Changes

All in `sim/midea_unit.py`, `tests/test_midea_unit.py`, `docs/actrl.md`.
Do NOT touch `appdaemon/`, `actrl.py`, `control.py`, `statctrl.py`,
`tests/fixtures/`, or anything under `data/`.

### 1. sim/midea_unit.py

a. New constructor parameter `ramp_up_period_cycles` (module default
   `DEFAULT_RAMP_UP_PERIOD_CYCLES = 18`, i.e. one latched increment per
   ~3 min at 10 s cycles). While the ramp-up flag is latched, the unit
   climbs by `ramp_flag_rate` only once every `ramp_up_period_cycles`
   intervals (an internal counter, reset by `reset()` and whenever the
   flag is not latched). On the other intervals while latched, the
   ordinary delta-stepping rule applies (in the observed regime actrl
   holds its report constant during deep demand, so delta is 0 and speed
   holds — do not suppress ordinary stepping, just don't add the latch
   climb). The ramp-DOWN flag path keeps its existing every-cycle rate:
   it is actrl-driven and self-clearing by design, and the recorded taper
   constrains only the up direction.

b. Clear rule for the ramp-up flag: at the top of `step()`, before the
   flag logic, `reported_error <= ramp_down_set_threshold` (the same
   threshold that sets the ramp-down flag, default −1) clears
   `ramp_up_flag` and zeroes the debounce streak. The ramp-down flag
   set/clear logic is unchanged.

c. Update the module docstring's Assumptions table: rewrite row 3's
   "Choice made" to describe the debounced latch + clear-on-decrement +
   slow climb, citing the 2026-06-22 recorded taper as the observational
   source (real unit follows step-down sequences back down; climb ~1
   increment per 3–6 min). Add the two new parameters to the Parameter
   column. Keep the table formatting style.

### 2. tests/test_midea_unit.py

a. Replace `test_ramp_up_flag_latches_and_never_clears` with two tests:
   - `test_ramp_up_flag_latches_and_ignores_holds`: latch the flag (3
     consecutive reports at setpoint+2 in cool mode), then feed reports
     that hold/step within the 0..+1 band; the flag stays latched and
     speed never decreases (climbs at the slow period).
   - `test_ramp_up_flag_clears_on_decrement_demand`: latch the flag,
     then feed one report at setpoint−1 (cool mode); the flag clears,
     and subsequent ordinary stepping can reduce speed.

b. New test `test_latched_climb_rate_is_slow`: latch the flag with speed
   near 0, then hold the report constant at setpoint+2 for
   `2 * ramp_up_period_cycles` intervals; comp_speed rises by exactly 2
   (one increment per period), not by one per interval.

c. `test_ramp_down_flag_ramps_speed_toward_zero_regardless_of_reports`
   currently steps a report at setpoint+2 while ramp-down is latched —
   check it still passes; adjust only if the new clear rule interacts
   (it should not: +2 does not hit the clear threshold for ramp-down).

d. Do not modify `test_closed_loop_tracking` fixtures. If it fails, STOP
   and report the failure verbatim instead of adjusting tolerances or
   fixtures.

### 3. docs/actrl.md

The bullet `reported >= setpoint + 2 sets a **ramp-up flag** (unknown how
to clear)` → update to note the observed behaviour: cleared by a
decrement-demand report (≤ setpoint − 1, i.e. the step-down sequence's
leading elements), and the latched climb is slow (~1 increment per
3–6 min) — observed from the 2026-06-22 recorded taper, 2026-07-04.

## Acceptance (runnable, from repo root)

1. `uv run pytest tests/test_midea_unit.py -q` — all pass.
2. `uv run pytest -q` — full suite passes (golden fixtures untouched).
3. `git diff --stat` touches only: sim/midea_unit.py,
   tests/test_midea_unit.py, docs/actrl.md, docs/tasks/012-clearable-ramp-latch.md.

The June-wide scorecard regression gate (analysis/scorecard.py) is run by
the orchestrator after merge — the worktree has no data/ archive; do not
attempt it.

## Log

- 2026-07-04 (implementer): Implemented as specced in `sim/midea_unit.py`:
  added `ramp_up_period_cycles` (default 18) constructor param + internal
  `_ramp_up_period_counter`; added a clear rule at the top of `step()` that
  drops `ramp_up_flag` and the debounce streak/counter when
  `reported_error <= ramp_down_set_threshold`; while latched, the ordinary
  delta-stepping rule now runs on every cycle except the one every
  `ramp_up_period_cycles` where the latch climb (`ramp_flag_rate`) fires
  instead. Rewrote Assumptions table row 3 with the new choice + sources.
  Updated `docs/actrl.md`'s ramp-up bullet to describe the clear-on-decrement
  + slow-climb behaviour.
  Tests: replaced `test_ramp_up_flag_latches_and_never_clears` with
  `test_ramp_up_flag_latches_and_ignores_holds` and
  `test_ramp_up_flag_clears_on_decrement_demand`; added
  `test_latched_climb_rate_is_slow`. Also had to adjust two pre-existing
  tests whose numbers assumed the old fast (every-cycle) latched climb:
  `test_speed_bounds_0_and_max` (increased the iteration count so the
  latch has enough cycles to actually reach max_speed under the new slow
  climb) and `test_ramp_up_flag_clears_on_decrement_demand`'s setup (now
  builds speed up via non-latching bursts first, like the neighbouring
  ramp-down test, so there is meaningful speed for the post-clear
  decrement to reduce — the original ordering left speed at 0 both before
  and after the clear, making the "speed can now reduce" assertion
  vacuously true/false). `test_ramp_down_flag_ramps_speed_toward_zero_regardless_of_reports`
  needed no changes (confirmed the new clear rule doesn't interact: +2
  never hits the ramp-down clear threshold).
  Results: `uv run pytest tests/test_midea_unit.py -q` → 23 passed.
  `uv run pytest -q` → 166 passed, 1 skipped (full suite, golden fixtures
  untouched, `test_closed_loop_tracking` passed without modification).
  `git diff --stat` against base 9dac191 touches only sim/midea_unit.py,
  tests/test_midea_unit.py, docs/actrl.md, and this spec file's Log/Status.
