# 005: Calibration data loader — align data/raw/ into analysis-ready series

Status: done
Branch: task/005-calibration-data-loader

## Goal

A library (`calib.py`, repo root) + small CLI that loads the per-day
long-format archives produced by `tools/export_history.py` (see
`docs/data.md`) into a single uniform-grid pandas DataFrame, applying the
correct per-series resampling semantics. This is the foundation every
simulator-calibration experiment (ideas.md §3) will sit on — correctness of
the gap/fill rules matters more than speed.

## Dependencies

Add **dev** dependencies via `uv add --dev pandas pyarrow`. The deployed
apps (actrl.py, control.py, statctrl.py) must NOT import calib.py or
pandas — this is offline analysis tooling only.

## Input format (from docs/data.md — re-read it first)

`data/raw/<YYYY-MM-DD>/<measurement>__<entity_id>.csv.gz`, columns
`time,field,value`; one row per (timestamp, field); timestamps are UTC
ISO8601 with variable sub-second precision. A series/day file may be absent
(no points that day — for state-change-recorded series this means "did not
change", NOT "missing").

## API

```python
import calib

df = calib.load_range(
    start="2026-06-01", end="2026-06-30",   # inclusive UTC dates
    data_dir="data",                          # data/raw/ underneath
    freq="1min",                              # uniform output grid
    seed_days=14,                             # look-back for state seeds
)
```

Returns a DataFrame indexed by UTC `DatetimeIndex` at `freq`, one column per
(series, field) with flat string names, e.g. `bed_1_average_temperature`,
`climate.bed_1_aircon.temperature`, `damper.bed_1`,
`power.outdoor_unit`, `power.indoor_unit`. Exact naming: use a
COLUMN_SPECS table in calib.py mapping (measurement, entity_id, field) →
(column_name, kind); unmapped fields are skipped with a debug note. Give
the Shelly channels the semantic names above (channel 1 = outdoor unit,
channel 2 = indoor unit).

## Resampling rules by kind (the heart of the task — implement exactly)

- `temperature` (room averages, m5atom coil/inside/outside temps):
  per-bin mean; bridge gaps ≤ 15 min by time-linear interpolation; longer
  gaps stay NaN.
- `state` (cover__damper positions, climate setpoints/modes-as-numbers,
  target_temp_high/low): last-observation-carried-forward with **unlimited**
  ffill (recorded on change only). Seed the initial value by scanning up to
  `seed_days` days before `start` for the most recent point (a damper can
  sit still for days — see docs/data.md bed_2/bed_3 note). If no seed found,
  NaN until first observation.
- `power` (Shelly channels): clamp raw values at ≥ 0 **before** binning
  (channel 1 idles ~-7 W, CT offset), then per-bin mean; no interpolation
  (gaps = NaN).
- `controller` (input_number PID terms, comp_speed, weighted_error,
  avg_deriv, meta_integral): per-bin mean, then ffill up to 2 min only;
  longer gaps NaN (they mean the app wasn't running — that fact matters).
- `slow` (BOM temperature_adelaide/humidity_adelaide `mean_value`, EMHASS
  power_load_5m/power_pv_5m `mean_value`): time-linear interpolation onto
  the grid, no extrapolation beyond first/last point.

## CLI

`uv run python calib.py --start 2026-06-01 --end 2026-06-30 [--report] [--out data/processed/june.parquet]`

- `--report`: print a per-column coverage table (non-NaN %, first/last
  timestamp, count of gap runs > 30 min) — this is the operator's data-QA
  view; make it readable.
- `--out`: write the aligned DataFrame to parquet under `data/processed/`
  (already covered by the `data/` gitignore — verify).

## Deliverables

1. `calib.py` — API + CLI above. Pure functions for everything testable
   (parsing, binning, each fill rule, seed scan) with thin I/O wrappers,
   same style as tools/export_history.py.
2. `tests/test_calib.py` — offline tests using small synthetic `.csv.gz`
   fixtures written to tmp_path by the tests themselves (do NOT commit
   real exported data; data/ stays gitignored). Cover: each resampling
   kind's rules (incl. the 15-min interpolation limit, unlimited ffill,
   clamp-before-mean, 2-min controller ffill limit), cross-day seeding
   (seed found N days back / not found), absent-file-means-unchanged for
   state series, mixed sub-second timestamp parsing, UTC index correctness.
3. `docs/data.md`: append a "Loading for analysis" section — API example,
   the resampling-rules table, column naming convention.
4. `pyproject.toml`: pandas + pyarrow as dev deps (uv add --dev).
5. Task file Log updated; Status → review.

## Out of scope / do not modify

- `actrl.py`, `control.py`, `statctrl.py`, `appdaemon/` (never deploy),
  `tools/export_history.py`, `tools/export_entities.json`, existing tests
  and fixtures, `systemd/`.
- No credentials anywhere (repo is public); this task needs no DB access.
- The real archive lives in the MAIN checkout at
  `/home/saltspork/actrl/data/` — a worktree won't have it (gitignored).
  The CLI must accept `--data-dir`; for live acceptance runs read from
  `--data-dir /home/saltspork/actrl/data` (treat it as read-only) and
  write `--out` under your own worktree's `data/processed/`.
- Do not commit anything under `data/`; do not write anything into the
  main checkout.

## Acceptance criteria (runnable)

```bash
uv run pytest                                    # all pass, offline
uv run python calib.py --data-dir /home/saltspork/actrl/data --start 2026-06-05 --end 2026-06-12 --report
uv run python calib.py --data-dir /home/saltspork/actrl/data --start 2026-06-01 --end 2026-06-30 --out data/processed/june.parquet
python3 -c "import pandas as pd; df = pd.read_parquet('data/processed/june.parquet'); print(df.shape); print(df['bed_1_average_temperature'].notna().mean())"
git status --short                               # data/ not listed
python3 -c "import actrl" 2>&1 | grep -q pandas && echo FAIL || echo "apps still pandas-free"
```

The `--report` run must show sane coverage for room temperatures (>95%
during June) and unlimited-ffill dampers at 100% after their seed point.

## Questions

(none yet)

## Log

- 2026-07-02: spec written (Claude Fable), status ready.
- 2026-07-02: implemented `calib.py` (COLUMN_SPECS covering 45 columns:
  room + m5atom temperatures, dampers, climate setpoints, Shelly power
  channels, input_number controller series, BOM/EMHASS slow series) with
  the five resampling kinds (temperature/state/power/controller/slow) as
  pure functions plus thin I/O wrappers, `tests/test_calib.py` (28 offline
  tests, synthetic `.csv.gz` fixtures via tmp_path), `docs/data.md` "Loading
  for analysis" section, `pandas`+`pyarrow` dev deps. All runnable
  acceptance criteria pass, incl. the live-archive `--report` and
  `--out`/parquet runs against `/home/saltspork/actrl/data` (read-only) and
  the apps-stay-pandas-free check. Notable fix during implementation: the
  temperature interpolation cap needed all-or-nothing gap semantics (a run
  either fully bridges if ≤15 min or stays entirely NaN) rather than
  pandas' native `interpolate(limit=...)`, which partially fills the front
  of longer gaps instead of leaving them alone — added a `long_gap_mask`
  helper to implement this correctly. Status -> review.
- 2026-07-02: reviewed and merged to master (Claude Fable, supervising).
  Implemented by a Sonnet 5 subagent. Verified: main checkout untouched
  (the subagent's stray `uv add` into the main checkout was fully
  reverted — status/stash/.venv all clean), field `current_position`
  confirmed present in real cover__damper data (alongside `value`; the
  docs/data.md field list from task 004 was non-exhaustive), all
  acceptance criteria re-run independently including the June parquet
  export (43200×45, bed_1 coverage 96.5%) and the apps-pandas-free
  check. m5atom raw temp sparsity (~11–19% coverage) looks like
  ESPHome on-change reporting — dense under load, silent when flat —
  acceptable for calibration. Status done.
