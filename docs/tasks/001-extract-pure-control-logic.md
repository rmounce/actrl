# 001: Capture golden fixtures and extract pure control logic

Status: ready
Branch: task/001-extract-pure-control-logic

## Goal

Move the pure (HA-independent) control classes out of `actrl.py` into a new
`control.py` module, with characterization ("golden") tests proving the move
changed nothing. This is a **pure code move** — zero behaviour change, zero
cleanup, zero renaming.

Classes to move: `MyWMA`, `MyDeriv`, `MyPID`, `DeadbandIntegrator`,
`WindowStateHandler`.

## Background

- `actrl.py` runs a real house's HVAC under AppDaemon. Read `AGENTS.md` and
  `docs/actrl.md` first.
- `actrl.py` imports `hassapi`, which only exists inside the AppDaemon
  runtime. That is why fixtures must be captured with a stub (step 1) and why
  tests must import from `control.py`, never from `actrl.py`.
- `WindowStateHandler`'s default constructor arg references the module-level
  constant `grid_surplus_open_window_rate` (0.2 * interval). When moving,
  keep the numeric behaviour identical: move the constants it needs
  (`grid_surplus_open_window_rate` and its inputs) or inline the computed
  default — either is fine as long as the default value is unchanged and
  `actrl.py` still works.

## Steps

1. **Capture fixtures BEFORE moving anything.** Write
   `tests/capture_fixtures.py` that:
   - stubs `hassapi` (insert a dummy module with a no-op `Hass` class into
     `sys.modules`) so `import actrl` works outside AppDaemon;
   - drives each class through deterministic scripted input sequences and
     records inputs + outputs to `tests/fixtures/*.json`. Coverage to
     include, at minimum:
     - `MyWMA(window=5,30)`: ramp, step, alternating-sign sequences; `get()`
       after each `set()`.
     - `MyDeriv(window=30, factor=60.0)`: constant error, ramping error,
       target step (must show target-change compensation), post-`clear()`.
     - `MyPID(kp=1.0, ki=0.01, kd=12.0, window=60)` (the real actrl
       construction with `interval = 10/60`, see `initialize()`): sequences of
       `update(error, setpoint)` + `get_output()`, plus `set_integral` /
       `adjust_integral` calls interleaved.
     - `DeadbandIntegrator(ki=0.125)`: small positive error until it emits
       +1, sign flips (exercise the ±0.75 preload), large errors, `clear()`.
     - `WindowStateHandler()`: open/close sequences across two rooms,
       `get_offset` for a room never updated.
   - Sequences must be hardcoded (no randomness without a fixed seed).
   - Run it once against the *unmodified* `actrl.py`; commit the fixtures.
2. **Create `control.py`** at repo top level. Move the five classes and any
   module-level constants they need, verbatim. Stdlib imports only
   (`collections.deque` etc.). Module docstring: one line noting it is pure
   logic shared by AppDaemon apps and tests.
3. **Update `actrl.py`** to `from control import MyWMA, MyDeriv, MyPID,
   DeadbandIntegrator, WindowStateHandler`. Delete the moved definitions.
   No other edits.
4. **Write `tests/test_control.py`**: pytest tests that import from
   `control` (NOT `actrl`, no hassapi stubbing needed) and replay every
   fixture, asserting outputs match to `pytest.approx(rel=1e-12)`.
5. **Tooling**: add a minimal `pyproject.toml` (project name `actrl`,
   `requires-python >= 3.11`, pytest as a dev dependency group), and add
   `.venv/` to `.gitignore`. Use `uv` (`uv sync`, `uv run pytest`). Do not
   add runtime dependencies.
6. Update the `Log` section below and set Status to `review`.

## Out of scope / do not modify

- `compress` / `midea_reset_quirks` / `midea_runtime_quirks` and everything
  else inside the `Actrl` class body — do not move, do not touch.
- `statctrl.py`, `appdaemon/` (never deploy), `archive/`, `docs/*.md` other
  than this file.
- No behaviour changes, no refactoring/renaming/reformatting of moved code
  beyond what the move itself requires. Resist cleanup urges.

## Acceptance criteria (runnable)

```bash
# 1. All tests pass
uv run pytest

# 2. control.py is hassapi-free and importable standalone
python3 -c "import control"

# 3. actrl.py still compiles (hassapi stubbed the same way as capture script)
python3 - <<'EOF'
import sys, types
m = types.ModuleType("hassapi"); m.Hass = object
sys.modules["hassapi"] = m
import actrl
print("actrl import OK")
EOF

# 4. actrl.py diff is subtractive + one import: no edits inside remaining code
git diff master -- actrl.py   # manual review: only deletions + import line(s)

# 5. Deployed copies untouched
git status --short appdaemon/ | wc -l   # expect 0 (dir is gitignored; also: no file mtime changes)
```

## Questions

(none yet)

## Log

- 2026-07-02: spec written (Claude Fable), status ready.
