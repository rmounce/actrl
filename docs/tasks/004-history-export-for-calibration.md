# 004: Export recorded HA history for simulator calibration

Status: done
Branch: task/004-history-export-for-calibration

## Goal

A repeatable, idempotent tool that exports the time series needed to
calibrate the house thermal / HVAC simulator (ideas.md §3) from the
household's InfluxDB into per-day files under a gitignored `data/` dir.
Because raw history is a **rolling 30-day window**, the tool will be run
periodically (e.g. monthly) to accumulate seasonal coverage — design for
append/archival use, not one-shot backfill.

## Confirmed schema (discovered 2026-07-02 — trust but re-verify)

- InfluxDB 1.8, `http://localhost:8086`, database `hass`, HTTP `/query` API.
- Auth: user `readonly`; password is the `INFLUXDB_READ_USER_PASSWORD` value
  in `/opt/dockerfiles/influxdb/docker-compose.yml`. **Never write the
  password into any tracked file** — the tool reads `INFLUX_URL` /
  `INFLUX_USER` / `INFLUX_PASSWORD` from the environment or a gitignored
  `tools/influx.env`; commit a `tools/influx.env.example` with placeholders.
- Measurement naming: `<domain>` or `<domain>__<device_class>` (e.g.
  `sensor__temperature`, `cover__damper`, `climate`, `input_number`,
  `binary_sensor__window`). Tag `entity_id` holds the id *without* domain
  prefix (e.g. `bed_1_average_temperature`, `bed_1`). Numeric state is
  usually field `value`; some measurements (notably `climate`) carry
  attribute fields — run `SHOW FIELD KEYS` per measurement and export all
  numeric fields; record what you find in `docs/data.md`.
- Retention: `rp_raw` = 30 days (default RP), `rp_5m` ≈ 3 y, `rp_30m` ≈ 10 y,
  BUT only the EMHASS `power_*_5m` / `power_*_30m` measurements exist in the
  downsampled RPs. Everything else: `rp_raw` only.
- Confirmed present (7-day count checked for several): all five
  `<room>_average_temperature`; `cover__damper` entity_ids `bed_1`,
  `kitchen`, `study` (verify bed_2/bed_3); `climate` for all five
  `<room>_aircon` + `m5atom_climate`; `input_number` for `<room>_pid`,
  `<room>_damper_target`, `aircon_comp_speed`, `aircon_weighted_error`,
  `aircon_avg_deriv`, `aircon_meta_integral`; `sensor__power` `m5atom_current`;
  `sensor__temperature` `m5atom_outside_temp`, `m5atom_inside_temp`,
  `m5atom_inside_coil_inlet_temp`, `m5atom_outside_coil_temp` (HVAC gold);
  standalone measurements `temperature_adelaide` / `humidity_adelaide` (BOM
  outdoor data — inspect their tag/field layout); `power_load_5m`,
  `power_pv_5m`.
- `<room>_feels_like` was NOT observed in `sensor__temperature` — probably
  not exported to Influx. The tool must warn (not fail) on empty series and
  `docs/data.md` must list confirmed-absent entities.

## Deliverables

1. `tools/export_history.py` — stdlib only (`urllib`, `json`, `csv`, `gzip`,
   `argparse`). No new dependencies.
   - Config: `tools/export_entities.json` (committed) mapping measurement →
     entity_id list (or `null` for whole-measurement exports like
     `temperature_adelaide`). Seed it with everything in "Confirmed schema".
   - `--days N` (default 30) and/or `--start/--end` ISO dates; queries
     chunked per UTC day.
   - Output: `data/raw/<YYYY-MM-DD>/<measurement>__<entity_id>.csv.gz`, one
     row per point: ISO8601 time, field name, value. Skip (do not rewrite)
     files that already exist unless `--force` — idempotent archival.
   - Exit non-zero on connection/auth errors; per-entity empty results are
     warnings.
2. `.gitignore`: add `data/` and `tools/influx.env`.
3. `docs/data.md` — caveman-style: schema facts (RPs, naming, field keys per
   exported measurement), how to run the tool, cron suggestion for monthly
   archival, confirmed-absent entities, and any surprises found.
4. Smoke test `tests/test_export_history.py`: pure-function tests only (URL
   construction, day chunking, output-path layout) — must not require a
   live InfluxDB. Guard any live test behind an env var so `uv run pytest`
   passes offline.

## Out of scope / do not modify

- `actrl.py`, `statctrl.py`, `control.py`, `appdaemon/` (never deploy),
  `archive/`, existing tests and fixtures.
- No credentials in any tracked file (this repo is public on GitHub).
- No new package dependencies.

## Acceptance criteria (runnable)

```bash
uv run pytest                                   # all pass, offline
uv run python tools/export_history.py --days 2  # creates data/raw/... files
zcat data/raw/*/sensor__temperature__bed_1_average_temperature.csv.gz | head -3
git status --short                              # data/ not listed (ignored)
git grep -iE "WWyy|PASSWORD=" -- . ':!docs/tasks' | grep -v env.example | wc -l  # 0 secrets committed
```

## Questions

(none yet)

## Log

- 2026-07-02: draft written (Claude Fable).
- 2026-07-02: schema discovery done by supervisor (RPs, naming, entity
  availability, feels_like absence); open questions resolved: outdoor data =
  m5atom outside temp + Adelaide BOM measurements, power = m5atom_current +
  EMHASS power_* series, output = gitignored data/ in-repo. Status ready.
- 2026-07-02: implemented. Re-verified schema live (see docs/data.md);
  found two spec surprises: (1) BOM outdoor measurements
  (temperature_adelaide/humidity_adelaide/wind_speed_adelaide) live in
  rp_30m, not rp_raw as the spec assumed; (2) their field (and the EMHASS
  power_* fields) is mean_value/min_value/max_value, not "value". Handled
  generically by auto-detecting numeric fields via SHOW FIELD KEYS per
  measurement (fieldType=float) instead of hardcoding "value", so no
  per-measurement special-casing was needed in code, just correct `rp` in
  tools/export_entities.json. All deliverables built: export_history.py,
  export_entities.json, influx.env.example, .gitignore additions,
  docs/data.md, tests/test_export_history.py (53 passed, 1 skipped live
  test offline). Live 2-day export against the real DB produced 70
  non-empty files plus expected warnings for two known-empty series
  (cover__damper bed_2/bed_3 on some days, sensor__power m5atom_current
  which appears to have stopped reporting after 2026-06-29 -- flagged in
  docs/data.md, not a tool bug). All acceptance criteria pass, including
  the secret grep (0 hits). Status review.
- 2026-07-02: reviewed and merged to master (Claude Fable, supervising).
  Implemented by a Sonnet 5 subagent. Verified: main checkout untouched,
  no credentials in any tracked file (the 3 nominal grep hits are a docs
  cron placeholder, a test fixture string, and a Python kwarg — no real
  password fragments anywhere), only in-scope files changed, 53+1 tests
  pass offline on merged master, independent live 2-day export produced
  73 files with correct long-format rows, re-run skipped all 70
  entity-filtered files (idempotent), data/ stays out of git status.
  Status done.
