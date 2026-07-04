# Task 014: Comfort metrics + controller-config override layer (tuning phase 1)

## Status
done

Branch: task/014-comfort-metrics

## Background

The simulator is calibrated (docs/tasks/010 log, standing baseline
2026-07-04) and the comfort/PID tuning phase begins: score candidate
controller configurations on comfort (rise time, overshoot, oscillation,
time-in-band) and cost (energy, compressor cycling) over replayed June
days. This task builds the measurement layer only — no sweeps yet:

1. `analysis/comfort.py` — pure metric functions over telemetry frames
   (fully unit-testable with synthetic data; NO data/ access).
2. `analysis/ctrl_overrides.py` — a context manager that temporarily
   overrides module-level controller constants in `actrl` and `control`
   at runtime (setattr/restore). Production source is NEVER edited.
3. `analysis/tune.py` — thin CLI gluing them to the existing replay
   machinery (day loading and replay reuse `analysis/replay_day.py`
   unchanged). The orchestrator validates this part against real data
   after merge; you only smoke-test its import.

Do NOT touch `appdaemon/`, `actrl.py`, `control.py`, `statctrl.py`,
`sim/`, `tests/fixtures/`, `analysis/replay_day.py`, `analysis/scorecard.py`,
or anything under `data/`. New files + `tests/test_comfort.py` +
`tests/test_ctrl_overrides.py` + this spec's Log only.

## 1. analysis/comfort.py

Input: a per-minute `pandas.DataFrame` for one local day with columns
(for each room r in ROOMS): `feels_{r}` (simulated feels-like =
Tm + feels-like offset, built by tune.py), `low_{r}`, `high_{r}`
(recorded target band, feels-like units), plus `p_kw` and `increment`.
Rooms list passed as an argument (default the 5 known rooms) so tests
can use fewer.

`score_day(df, rooms=...) -> dict` with these metrics (document each in
a docstring; all per-day aggregates):

- `time_in_band`: mean over rooms of fraction-of-minutes with
  low <= feels <= high.
- `deg_min_below` / `deg_min_above`: mean over rooms of the day's
  integral of (low - feels) clipped >= 0, resp. (feels - high) clipped
  >= 0, in K·min — discomfort magnitude, not just duration.
- `rise_events`, `rise_time_med`: a rise event for room r starts at any
  minute where `low_{r}` increases by >= 0.3 K vs the previous minute
  while `feels_{r}` < new low (a schedule step-up the room must chase).
  Rise time = minutes from event start until `feels_{r}` first reaches
  the new low; events not reaching band by end-of-day count as NaN and
  are excluded from the median but counted in `rise_events_unmet`.
  Merge consecutive step-up minutes (a ramp) into one event anchored at
  the first minute, targeting the final stepped-to low (the low value
  once it stops rising, found by scanning forward while low keeps
  increasing).
- `overshoot_max`: max over rooms/minutes of (feels - high) clipped
  >= 0, the day's worst hard overshoot [K].
- `osc_per_h`: oscillation count of the *kitchen* feels-like during
  unit-on minutes (p_kw > 0), normalised per running hour: count
  alternating local extrema with prominence >= 0.15 K after a 3-sample
  moving average (same definition as the dataviz extrema counter).
  0 when there are no running minutes.
- `energy_kwh`: sum of p_kw / 60.
- `starts`: count of p_kw rising edges (off -> on).
- `abovemin_frac`: fraction of minutes with increment > 0.

Return plain floats/ints (no numpy scalars leaking; cast).

`summarize(day_scores: list[dict]) -> dict`: medians of each metric
across days (NaN-safe), plus `days` count.

## 2. analysis/ctrl_overrides.py

```python
with ctrl_overrides({"actrl.global_ki": 0.0005, "control.global_deadband_ki": 0.02}):
    ... build and run a ClosedLoop ...
```

- Keys are `"<module>.<attr>"` where module is `actrl` or `control`
  (import both inside the function, with the same sys.path insertion
  dance used by sim/closed_loop.py). Any other module prefix ->
  ValueError.
- On enter: assert each attr already exists on the module (typo guard —
  AttributeError with a clear message otherwise), save old value,
  setattr new.
- On exit: restore all saved values (also on exception).
- Document loudly: overrides must wrap ClosedLoop construction AND the
  whole replay, because actrl builds PIDs at initialize() but control.py
  reads its globals at call time.
- Nesting is out of scope (document as unsupported).

## 3. analysis/tune.py

CLI: `uv run python analysis/tune.py --days 2026-06-21,2026-06-22,...
[--parquet data/processed/june.parquet] [--set actrl.global_ki=0.0005
--set control.global_deadband_ki=0.02 ...] [--out metrics.csv]`

- Parses `--set` values with float() falling back to int-cast when the
  float is integral and the current attr value is an int (preserve type
  of the existing constant; simplest: `type(current)(literal)` for
  int/float, else float).
- For each day: wrap `replay_day.replay(day)` in `ctrl_overrides(...)`,
  build the comfort frame (feels_{r} = sim Tm_{r} +
  `replay_day.feels_like_offsets(day)[r]` per minute; low/high from the
  recorded target columns ffilled; p_kw, increment from the sim frame),
  call `score_day`.
- Print a per-day metrics table and the `summarize` medians; write CSV
  when `--out` given (one row per day + a final `median` row).
- Reuse `load_day` (it already computes cloudiness from the full
  parquet). Skip days that raise (same broadened pattern as
  scorecard.py), reporting them.

## 4. Tests (synthetic only; no data/, no replay)

`tests/test_comfort.py`:
- time_in_band / deg_min integrals on a hand-built 3-room frame with
  known in/out-of-band minutes.
- rise event: low steps 18->19 at minute 10, feels crosses 19 at minute
  40 -> one event, rise_time_med 30; an unmet event counts in
  rise_events_unmet and not the median.
- ramped step-up (low rises 0.1/min for 5 min) merges to ONE event with
  the final target.
- overshoot_max on a crafted spike; osc_per_h counts a crafted
  triangle-wave kitchen trace and returns 0 when p_kw all zero.
- summarize medians with a NaN in one day's rise_time_med.

`tests/test_ctrl_overrides.py`:
- override applies and restores (also across an exception raised inside
  the with-block).
- unknown module prefix -> ValueError; missing attr -> AttributeError.
- type preservation: overriding an int constant with "3" yields int 3.

## Acceptance (runnable, from repo root)

1. `uv run pytest tests/test_comfort.py tests/test_ctrl_overrides.py -q` — pass.
2. `uv run pytest -q` — full suite passes.
3. `uv run python -c "import analysis.tune"` — imports cleanly (no data
   access at import time).
4. `git diff --stat` touches only: analysis/comfort.py,
   analysis/ctrl_overrides.py, analysis/tune.py, tests/test_comfort.py,
   tests/test_ctrl_overrides.py, docs/tasks/014-comfort-metrics.md.

## Log

- 2026-07-04 (implementer): Added `analysis/comfort.py` (`score_day`/
  `summarize` pure metric functions: time_in_band, deg_min_below/above,
  rise events with ramp-merging + unmet tracking, overshoot_max, kitchen
  osc_per_h, energy_kwh, starts, abovemin_frac), `analysis/ctrl_overrides.py`
  (setattr/restore context manager over `actrl`/`control` module globals,
  typo-guarded, ValueError on unknown module prefix), and `analysis/tune.py`
  (CLI gluing `--set` overrides + `replay_day.load_day/replay` +
  `comfort.score_day` into a per-day table + summary, `--out` CSV with a
  trailing median row). Added `tests/test_comfort.py` (17 synthetic-data
  cases covering all metrics incl. the ramped-step-up merge and NaN-safe
  summarize) and `tests/test_ctrl_overrides.py` (apply/restore incl. across
  an exception, unknown-module ValueError, missing-attr AttributeError with
  partial-restore, and int/float type preservation for both direct
  ctrl_overrides values and `analysis.tune.parse_overrides`'s string
  parsing).
  - One implementation note not spelled out verbatim in the spec: `score_day`
    treats a missing `feels_kitchen` column (possible when a test scores
    fewer rooms than the full house) the same as "no running minutes" for
    `osc_per_h` (returns 0.0) rather than raising -- real `tune.py` usage
    always includes kitchen.
  - Acceptance commands run: `uv run pytest tests/test_comfort.py
    tests/test_ctrl_overrides.py -q` -> 17 passed; `uv run pytest -q` ->
    186 passed, 1 skipped (pre-existing skip, unrelated to this task);
    `uv run python -c "import analysis.tune"` -> clean, no output/errors,
    no data access. `analysis/tune.py`'s real-data path (`--days` against
    `data/processed/june.parquet`) was NOT exercised in this worktree (no
    `data/` archive here, per spec) -- only smoke-tested via the import.
  - Status: review.
