# Task 013: Hold-at-setpoint stepping rule in MideaUnit

## Status
Spec ready for implementation.

## Background

Assumption 1 in `sim/midea_unit.py` documents that under the sign-of-delta
ordinary stepping rule, actrl's step-down sequence (`[-1, -2, +1, 0]`
offsets from stable, i.e. reported errors `[0, -1, 2, 1]`) nets **-2**
where the real unit nets -1. The note dismissed this as inside the ±2
tracking tolerance — but in closed-loop replay it compounds: a long
morning taper is a chain of step-down sequences, so the emulated unit
descends at exactly 2x the real rate (2026-06-22: sim reaches 0 at 08:15
where the real unit idles down until 09:20; actrl's dead-reckoned guess
and the emulated speed diverge by ~1 per sequence, guess 9 vs unit 3 at
08:00). Investigation 2026-07-04 (docs/tasks/010 log context, task 012
follow-up).

The spurious extra decrement comes solely from the sequence's `stable -> 0`
transition — a report exactly at setpoint. Fix rule: **a report at
setpoint means "hold"** — the ordinary delta-stepping rule is skipped on
any cycle where `reported_error == 0` (the unit already treats
`== setpoint` specially for its shutdown timer, so this is consistent
thermostat logic, not sequence pattern-matching). Effects, verified in
scratch:

- step-down sequence nets -1 (was -2); step-up sequence still nets +1.
- 06-22 taper reaches 0 at 08:50 (was 08:15; recorded 09:20), kitchen
  RMSE 0.364 -> 0.361, energy -2% -> -1%.
- 06-21 peak increment 13 -> 11 (recorded ~10) and the late ramp rolls
  off 10 -> 8 -> 3 -> 0 nearly on top of the recording.
- Golden closed-loop tracking fixtures pass 8/8 with the rule (the
  dead-reckoner always assumed -1 per sequence, so tracking improves).

## Changes

Only in `sim/midea_unit.py`, `tests/test_midea_unit.py`, `docs/actrl.md`,
and this spec's Log. Do NOT touch `appdaemon/`, `actrl.py`, `control.py`,
`statctrl.py`, `tests/fixtures/`, or anything under `data/`.

### 1. sim/midea_unit.py

a. In `step()`, the ordinary delta-stepping branch (the `else` after the
   ramp-flag branches) must not step when `reported_error == 0`: treat an
   at-setpoint report as "hold" (step 0) regardless of delta. Implement it
   in that branch (e.g. compute step only when `reported_error != 0`), NOT
   by suppressing the delta bookkeeping — `_prev_reported_error` must
   still update every cycle as now, so the transition OUT of an
   at-setpoint report still contributes its delta on the next cycle
   unless that cycle is itself at setpoint. (Scratch validation used an
   equivalent formulation; either is fine as long as the two sequence
   nets below hold.)
b. Rewrite Assumption 1's "Choice made" cell: sign-of-delta stepping,
   PLUS at-setpoint reports hold (no ordinary step). Cite: step-down
   sequence nets -1 matching `guesstimated_comp_speed`'s dead-reckoning
   (previously -2, which doubled closed-loop taper rates — 2026-06-22
   recorded taper, task 013); step-up sequence unchanged at +1; the
   at-setpoint-is-special reading is consistent with rule 9's shutdown
   timer keying on `reported_error == 0`.

### 2. tests/test_midea_unit.py

a. New test `test_step_down_sequence_nets_minus_one`: cool mode, speed 8,
   `_prev_reported_error` primed to 1 (stable), feed reported temps for
   rvals `[0, -1, 2, 1]`; final comp_speed == 7. Mirror-image test
   `test_step_up_sequence_nets_plus_one`: rvals `[2, 3, 1]` from the same
   priming; final comp_speed == 9. (Priming via one warm-up `step()` at
   setpoint+1 from a known speed is also acceptable — just make the
   intent explicit in a comment.)
b. New test `test_at_setpoint_report_holds_speed`: at speed N with
   `_prev_reported_error` == 1, one step at exactly setpoint leaves
   comp_speed == N (old rule would have stepped -1).
c. Check the existing ordinary-stepping tests
   (`test_ordinary_step_changes_speed_by_delta`,
   `test_ordinary_step_caps_at_one_regardless_of_jump_magnitude`) — if
   they feed at-setpoint reports expecting a step, adjust the report
   values to non-zero errors so they still test what they intend; do not
   weaken assertions.
d. Do not modify `test_closed_loop_tracking` or its fixtures. If it
   fails, STOP and report the failure verbatim.

### 3. docs/actrl.md

In the capacity-control section, the bullet "Each 1 °C change of reported
error steps compressor speed by ±1 increment" — append a note: reports
exactly at setpoint hold speed (no step); this is what makes the
step-down sequence net −1 (observed via closed-loop replay vs the
2026-06-22 recorded taper, 2026-07-04).

## Acceptance (runnable, from repo root)

1. `uv run pytest tests/test_midea_unit.py -q` — all pass.
2. `uv run pytest -q` — full suite passes (golden fixtures untouched).
3. `git diff --stat` touches only: sim/midea_unit.py,
   tests/test_midea_unit.py, docs/actrl.md,
   docs/tasks/013-hold-at-setpoint-step.md.

The June-wide scorecard gate (analysis/scorecard.py) is run by the
orchestrator after merge — the worktree has no data/ archive; do not
attempt it.

## Log

(implementer: append a short completion note here — what changed, test results)
