# 003: Extract MideaCapacityController from actrl.py

Status: ready
Branch: task/003-extract-capacity-controller

## Goal

Move the compressor capacity-control logic — `compress`,
`midea_reset_quirks`, `midea_runtime_quirks` and the state they own — out of
the `Actrl` class into a `MideaCapacityController` class in `control.py`,
with unit-level golden fixtures captured before the move. **Zero behaviour
change.** This code encodes irreplaceable reverse-engineered Midea behaviour
(see docs/actrl.md "Capacity control"): move it verbatim, never "improve" it.

## Safety net

Task 002's whole-cycle golden tests (`tests/test_cycles.py`) already cover
this code end-to-end. They must pass **unchanged** (do not regenerate cycle
fixtures). Any cycle-test failure means the refactor changed behaviour —
fix the refactor, not the fixture.

## Design (follow exactly)

Add to `control.py`:

```python
class MideaCapacityController:
    def __init__(self, log=lambda msg: None):
        self.log = log
        self.on_counter = 0
        self.min_power_counter = 0
        self.max_power_counter = 0
        self.prev_step = 0
        self.guesstimated_comp_speed = 0
        self.compressor_totally_off = True
        self.prev_unsigned_compressed_error = 0
        self.deadband_integrator = DeadbandIntegrator(
            ki=(global_deadband_ki * 60.0 * interval)
        )

    def compress(self, error, deriv): ...        # moved verbatim
    def midea_reset_quirks(self, rval): ...      # moved verbatim
    def midea_runtime_quirks(self, rval): ...    # moved verbatim
```

- The three methods move **verbatim**; the only permitted edit is
  `self.log(...)` calls now go to the injected `log` callable (same message
  strings).
- Attribute names stay identical so `Actrl` call sites become mechanical
  `self.X` → `self.capacity.X` rewrites.
- Constants the moved code needs migrate to `control.py` (keep their
  comments): `interval`, `global_deadband_ki`, `soft_delay`, `soft_ramp`,
  `minimum_temp_intervals`, `compressor_power_increments`,
  `compressor_power_safety_margin`, `min_power_delay`, `purge_delay`,
  `min_power_time`, `faithful_threshold`, `desired_on_threshold`,
  `eventual_off_threshold`, `min_power_threshold`,
  `immediate_off_threshold`, `ac_on_threshold`, `ac_stable_threshold`,
  `ac_off_threshold`. `actrl.py` re-imports from `control` the ones it
  still uses elsewhere (`interval` is used to derive several actrl-only
  constants — those derivations stay in actrl.py; grep before deciding
  each constant's home).

In `actrl.py`:

- `initialize()`: construct `self.capacity = MideaCapacityController(log=self.log)`,
  then apply the existing persistence/startup logic to `self.capacity.*`
  (comp speed from `input_number.aircon_comp_speed`; `compressor_totally_off`
  / `on_counter` from the already-running check).
- Rewrite every remaining reference to the eight owned attributes to
  `self.capacity.<attr>`. Complete touchpoint list (verify with grep;
  line numbers as of commit 1a43bf7): `initialize` (~178–201),
  `main` (~293–400: on_counter increment, ac_min_power clamp, defrost
  override, comp-speed persistence write, state logging, off handling
  setting on_counter=0, max_power_counter static-pressure check,
  prev_unsigned_compressed_error assignment, meta_integral write),
  `_reset_internal_state`, `_handle_mode_change`, `_determine_fan_mode`.
- `self.compress(...)` becomes `self.capacity.compress(...)`.
- Nothing else changes. `print`-diffable rule: apart from the constants
  block, the `actrl.py` diff must consist only of deletions (moved code),
  the import line(s), the constructor line, and `self.X` → `self.capacity.X`
  substitutions.

## Steps

1. **Capture unit fixtures BEFORE refactoring** (`tests/capture_capacity_fixtures.py`):
   instantiate the pre-move logic via `object.__new__(actrl.Actrl)` with the
   eight attributes set manually and `log` stubbed (reuse the hassapi-stub
   pattern from `tests/hvac_harness.py`). Drive `compress(error, deriv)`
   sequences; after each call record rval plus all eight state values
   (deadband integral via `.get()`). Scenario coverage (hardcoded,
   deterministic — mimic how `main()` mutates state between calls, i.e.
   increment `on_counter` each cycle and assign
   `prev_unsigned_compressed_error = rval` after each call):
   - cold start: off → on-blip → soft start (45+ cycles) → ramp → step-ups.
   - equilibrium: errors oscillating ±0.05 long enough to emit complete
     step-up and step-down sequences via the deadband integrator.
   - faithful/hysteresis: error 3.0, then decay — entry, latch, exit.
   - min power: error −0.3 for 300+ cycles — min_power_counter, the
     min_power_time blip, eventual_off ramp crossing, purge wraparound.
   - immediate off: speed > 0, error −1.6.
   - min_power_threshold cut: error −2.5 with deriv 0 and with deriv ±0.5.
   - external defrost override: mid-run, set guesstimated_comp_speed = 16
     between calls (record it as an explicit step in the fixture).
   - on_counter clamp: mid-run, clamp on_counter to soft_delay − 1
     (the ac_min_power path).
   Write `tests/fixtures/capacity/*.json`; commit before touching actrl.py.
2. Perform the move per Design above.
3. `tests/test_capacity.py`: replay every fixture against
   `control.MideaCapacityController` (no actrl import, no hassapi stub),
   asserting rval and all eight state values each step
   (`pytest.approx(rel=1e-12)` for floats, exact for ints/bools).
4. Update this file's Log; set Status to `review`.

## Out of scope / do not modify

- `tests/fixtures/cycles/*` — never regenerate.
- `tests/hvac_harness.py`, `tests/scenarios.py`, `tests/test_cycles.py`,
  `tests/test_control.py`, `tests/capture_fixtures.py`.
- `statctrl.py`, `appdaemon/` (never deploy), `archive/`, docs other than
  this file.
- No behaviour change; no cleanup of the moved code (including its
  comments — they are the documentation of record for Midea quirks).

## Acceptance criteria (runnable)

```bash
uv run pytest                       # all tests incl. unchanged cycle goldens
python3 -c "import control"         # still hassapi-free
python3 - <<'EOF'
import sys, types
m = types.ModuleType("hassapi"); m.Hass = object
sys.modules["hassapi"] = m
import actrl
print("actrl import OK")
EOF
git diff master -- tests/fixtures/cycles | wc -l   # expect 0
grep -c "def compress\|def midea" actrl.py         # expect 0
grep -c "def compress\|def midea" control.py       # expect 3
git diff master -- actrl.py         # manual review per Design "diffable rule"
```

## Questions

(none yet)

## Log

- 2026-07-02: spec written (Claude Fable), status ready.
