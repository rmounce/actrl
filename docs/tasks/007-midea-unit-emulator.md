# 007: Midea unit emulator — the flip side of MideaCapacityController

Status: ready
Branch: task/007-midea-unit-emulator

## Goal

`sim/midea_unit.py`: a deterministic emulator of the Midea ducted unit's
observed follow-me behaviour, so actrl's protocol handling can be exercised
in closed loop (ideas.md §3 "Midea controller emulator"). The behaviour
rules are the reverse-engineered knowledge encoded in **control.py's
comments and docstrings** (`MideaCapacityController`, `compress`,
`midea_runtime_quirks`) and **docs/actrl.md "Capacity control"**. Those two
sources are the ONLY specification — do not invent behaviour beyond them.

## Behavioural rules to implement (cross-check each against the sources)

The unit is fed one reported temperature per control interval (10 s) and
holds a fixed setpoint. Work in offsets: `reported_error = reported −
setpoint` (integer degrees).

1. Compressor speed is an integer 0..`max_speed` (default 14 increments).
2. Each ±1 °C *change* of reported error steps speed by ±1 increment
   (bounded at 0 and max).
3. `reported_error >= +2` sets a latching **ramp-up flag** (no known way
   to clear): while set, the unit ramps speed toward max regardless of
   subsequent reports.
4. `reported_error <= −1` sets a **ramp-down flag**: while set, the unit
   ramps speed toward 0. `reported_error >= +1` clears it.
5. After 90 min (`purge_delay`) of continuous running below a low-speed
   threshold, a ~1 min full-speed **purge**, then return to prior speed;
   the 90 min clock restarts.
6. Holding at the setpoint-reached report for ~1 h shuts the unit down
   entirely; any report change resets that internal timer (the
   `min_power_time` "blip" in compress() exists to trigger exactly this
   reset — make sure the emulator's timer semantics make that blip
   effective).
7. Expose (read-only) for assertions/telemetry: `comp_speed`,
   `ramp_up_flag`, `ramp_down_flag`, `running`, minutes-at-min-power,
   purge state. A `step(reported_temp) -> None` (or small result object)
   advances one 10 s interval.

Where the sources are silent (e.g. exact ramp rate while a flag is set,
purge trigger speed threshold), choose the simplest plausible behaviour,
make it a constructor parameter, and record every such choice in an
**Assumptions** table in the module docstring AND in this task file's Log.
These will be reviewed and later tuned against recorded data.

## Closed-loop acceptance test (the point of the task)

`tests/test_midea_unit.py`:

1. Unit tests for each rule above (flags latch/clear, step bounds, purge
   timing, shutdown timer + blip reset).
2. **Closed-loop tracking test**: instantiate
   `control.MideaCapacityController` and a `MideaUnit`; for each of the
   error trajectories in `tests/fixtures/capacity/*.json` (read the
   `compress` op inputs only — do NOT modify the fixtures), run:
   controller `compress(error, deriv)` → rval → emulator
   `step(setpoint + rval)` each cycle, mirroring main()'s bookkeeping
   (increment `on_counter`, assign `prev_unsigned_compressed_error`).
   Assert that whenever the controller's `guesstimated_comp_speed` is in
   [0, 14] (not saturated by the ±2 margin) and outside soft-start,
   faithful mode, and external-override steps, the emulator's actual
   speed is within ±2 increments of the controller's estimate. Scenarios
   where the fixture applies external mutations (defrost override,
   on_counter clamp) may be skipped for the tracking assertion — say so
   in the test.
3. The tracking test documents (in comments) any scenario where tracking
   only holds because of an Assumption — that's expected and useful.

## Out of scope / do not modify

- `actrl.py`, `control.py`, `statctrl.py`, `hvac_watchdog.py`, `calib.py`,
  `tools/`, `analysis/`, `systemd/`, `appdaemon/` (NEVER deploy),
  existing tests and fixtures (`tests/fixtures/**` read-only).
- No defrost emulation in this task (unit-side defrost needs outdoor
  conditions — future task).
- No new dependencies (stdlib only).
- Do not create docs/ files other than this task file — module docstring
  carries the documentation for now.

## Acceptance criteria (runnable)

```bash
uv run pytest                                   # all pass incl. closed loop
python3 -c "import sim.midea_unit"              # hassapi-free, stdlib-only
git diff master -- tests/fixtures | wc -l       # expect 0
git diff master -- control.py actrl.py | wc -l  # expect 0
```

## Questions

(none yet)

## Log

- 2026-07-02: spec written (Claude Fable), status ready.
