# Calibration data export (InfluxDB history)

Tool: `tools/export_history.py`. Exports HA history from InfluxDB into
gitignored `data/raw/<YYYY-MM-DD>/<measurement>__<entity>.csv.gz` files for
simulator calibration (`docs/ideas.md` #3). Stdlib only.

## Running it

```bash
cp tools/influx.env.example tools/influx.env   # once, fill in the real password
# password lives in /opt/dockerfiles/influxdb/docker-compose.yml, key
# INFLUXDB_READ_USER_PASSWORD -- never commit it
uv run python tools/export_history.py --days 30       # rolling 30-day export
uv run python tools/export_history.py --start 2026-06-01 --end 2026-06-30
uv run python tools/export_history.py --days 30 --force  # re-fetch existing files
```

Idempotent: existing output files are skipped unless `--force`. Because the
raw retention policy is a rolling 30-day window, run this **monthly** to
accumulate seasonal coverage (see cron suggestion below) -- do not treat it
as a one-shot backfill.

Config: `tools/export_entities.json` (committed) maps measurement name to
`{"rp": "<retention policy>", "entities": [...] | null}`. Numeric fields are
always auto-detected per measurement via `SHOW FIELD KEYS` (all
`fieldType=float` columns) -- no per-measurement field list needed, this
picks up e.g. `climate`'s `temperature`/`current_temperature`/etc and the
`mean_value`/`min_value`/`max_value` fields of the downsampled BOM/EMHASS
series automatically.

`entities: null` means "no `entity_id` filter, take whatever series come
back" -- used for `temperature_adelaide` etc. In practice these still carry
an `entity_id` tag in the data, so the output filename still gets a
meaningful `__<entity_id>` suffix (see `series_suffix()` in the tool).

Output row format: long/tall CSV, one row per (time, field) pair --
`time,field,value`. A `climate` entity with 5 numeric fields produces 5 rows
per timestamp, not 5 columns.

### Monthly systemd timer (installed 2026-07-02)

`systemd/actrl-export-history.{service,timer}` — user units, same pattern as
ai-energy-forecast-slop: tracked in the repo, symlinked into
`~/.config/systemd/user/`, edits take effect after
`systemctl --user daemon-reload`. Runs `--days 32` on the 1st of each month
at 06:10 with `Persistent=true` (a missed month = raw data gone forever, so
it catches up on next boot). Overlap with the previous run is skipped
automatically since those files already exist. Credentials come from the
gitignored `tools/influx.env` (mode 600).

```bash
# setup on a new machine (linger already enabled on this one)
ln -s /home/saltspork/actrl/systemd/actrl-export-history.* ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now actrl-export-history.timer
# logs / status
journalctl --user -u actrl-export-history.service
systemctl --user list-timers actrl-export-history.timer
```

## Confirmed schema facts (re-verified 2026-07-02 against the live DB)

- InfluxDB 1.8, `http://localhost:8086`, database `hass`, HTTP `/query`.
- Retention policies: `rp_raw` (720h = 30d, **default**), `rp_5m` (26280h ≈
  3y), `rp_30m` (89040h ≈ 10y), plus an unused legacy `autogen` (168h).
  Because `rp_raw` is the default RP, unqualified measurement names resolve
  to `rp_raw` -- **downsampled series must be queried with an explicit
  `"rp_5m"."<measurement>"` / `"rp_30m"."<measurement>"` prefix**, or you
  silently get zero rows back (no error).
- Measurement naming: `<domain>` or `<domain>__<device_class>`. Tag
  `entity_id` holds the id without domain prefix. Tags present on the
  `<domain>__<device_class>` measurements checked: `domain`, `entity_id`,
  `source` (constant per measurement in practice, so filtering by
  `entity_id` + `GROUP BY *` yields one series).
- Field keys per exported measurement (`SHOW FIELD KEYS`, float only):
  - `sensor__temperature`, `cover__damper`, `input_number`, `sensor__power`:
    field `value` (plus various string/attribute fields, ignored).
  - `climate`: multiple numeric fields -- `current_temperature`,
    `current_humidity`, `temperature`, `target_temp_high`,
    `target_temp_low`, `target_temp_step`, `max_temp`, `min_temp`,
    `supported_features`, plus `value`. The tool exports all of them.
  - `temperature_adelaide` / `humidity_adelaide` / `wind_speed_adelaide`:
    field `mean_value` (**not** `value`).
  - `power_load_5m` / `power_pv_5m` (and the `_30m` / dump-load /
    without-deferrable variants): fields `min_value`, `mean_value`,
    `max_value` (**not** `value`).

## Surprises vs. the task spec's initial assumptions

- **BOM outdoor data (`temperature_adelaide`, `humidity_adelaide`,
  `wind_speed_adelaide`) lives in `rp_30m`, not `rp_raw`.** The spec assumed
  "everything except EMHASS `power_*` is `rp_raw` only" -- false for these
  three. `SHOW FIELD KEYS FROM "temperature_adelaide"` (unqualified, i.e.
  against the default `rp_raw`) returns nothing; you have to know to qualify
  with `"rp_30m"."temperature_adelaide"`. Confirmed no data at all currently
  sits in `rp_raw` or `rp_5m` for these three measurements.
- **Field name is not always `value`.** The BOM and EMHASS downsampled
  series use `mean_value`/`min_value`/`max_value`. Rather than special-case
  this per measurement in the config, the tool auto-detects numeric fields
  via `SHOW FIELD KEYS` for every measurement, uniformly.
- `temperature_adelaide` has two InfluxDB *series* under the hood: one
  tagged `entity_id=adelaide_west_terrace_ngayirdapira_temp` (has data) and
  one tagged only `location=Adelaide,original_resolution=1h,...` (no data
  found in any RP as of 2026-07-02 -- likely stale/legacy metadata from an
  earlier integration version). The tool's `GROUP BY *` + per-series export
  handles this correctly (empty series produce no output rows).
- The Midea/m5atom power and current entities exist but report no useful
  data on this system (per the user, 2026-07-02; `m5atom_current` had no
  points after 2026-06-29 anyway). The real HVAC power source of truth is a
  separate Shelly EM meter: `sensor__power` entity
  `shellyem_ec64c9c6932b_channel_1_power` = **outdoor unit** (compressor +
  outdoor fan), `..._channel_2_power` = **indoor unit** (indoor fan). Both
  report ~every 13 s. Channel 1 reads slightly negative at idle (~-7 W, CT
  offset) -- clamp at 0 before integrating energy.
- `cover__damper` `bed_2` / `bed_3`: got "empty result" warnings on some of
  the two sample days during the acceptance test -- these dampers may not
  be reporting position changes as often as `bed_1`/`kitchen`/`study` (or
  were idle at a position that doesn't emit new points); not investigated
  further, flagging for whoever calibrates against damper position data.

## Confirmed-absent entities (do not add to `export_entities.json`)

- `<room>_feels_like` (e.g. `bed_1_feels_like`) is **not** present in
  `sensor__temperature` -- only `adelaide_airport_temp_feels_like` and
  `adelaide_west_terrace_ngayirdapira_temp_feels_like` (BOM feels-like, not
  per-room) exist. Matches the spec's expectation.

## Output format

`data/raw/<YYYY-MM-DD>/<measurement>__<entity_id>.csv.gz`, columns
`time,field,value`, one row per (timestamp, field). Gitignored (`data/`).

## Verified end-to-end (2026-07-02)

`uv run python tools/export_history.py --days 2` against the live DB
produced 70+ non-empty files (temperatures, dampers, climate, input_number
PID/damper-target/comp-speed/error/deriv/integral state, BOM outdoor
temp/humidity, EMHASS power load/PV) plus warnings (not failures) for the
known-empty series above. Re-running with the same range skipped every file
that already existed (idempotent); `--force` re-fetches.

## Loading for analysis

Tool: `calib.py` (repo root). Loads `data/raw/` into a single uniform-grid
`pandas` DataFrame for simulator-calibration work (`docs/ideas.md` #3).
Dev-only tooling (`pandas`/`pyarrow` are dev dependencies) — the deployed
apps never import it or pandas.

```python
import calib

df = calib.load_range(
    start="2026-06-01", end="2026-06-30",   # inclusive UTC dates
    data_dir="data",                          # data/raw/ underneath
    freq="1min",                              # uniform output grid
    seed_days=14,                             # look-back for state seeds
)
```

Returns a DataFrame indexed by a UTC `DatetimeIndex` at `freq`, one column
per (series, field) per the static `COLUMN_SPECS` table in `calib.py`
(unmapped fields are silently skipped — extend the table to pick up more
series).

CLI:

```bash
uv run python calib.py --start 2026-06-01 --end 2026-06-30 \
    [--data-dir data] [--freq 1min] [--seed-days 14] \
    [--report] [--out data/processed/june.parquet]
```

`--report` prints a per-column coverage table (non-NaN %, first/last
timestamp, count of gap runs > 30 min) — the data-QA view. `--out` writes
parquet under `data/processed/` (gitignored, same as the rest of `data/`).

### Column naming convention

- Room/M5Atom temperature sensors: the bare `sensor__temperature` entity id,
  e.g. `bed_1_average_temperature`, `m5atom_outside_coil_temp`.
- Cover damper positions: `damper.<entity_id>`, e.g. `damper.bed_1`.
- Climate setpoints: `climate.<entity_id>.<field>`, e.g.
  `climate.bed_1_aircon.target_temp_high`,
  `climate.m5atom_climate.temperature`.
- Shelly EM channels: semantic names `power.outdoor_unit` (channel 1,
  compressor + outdoor fan) / `power.indoor_unit` (channel 2, indoor fan).
- `input_number` controller series (PID terms, comp_speed, damper targets,
  weighted_error, avg_deriv, meta_integral): the bare entity id, e.g.
  `bed_1_pid`, `aircon_comp_speed`.
- BOM/EMHASS slow series: the bare measurement name, e.g.
  `temperature_adelaide`, `power_load_5m`.

### Resampling rules by kind

| kind | rule |
| --- | --- |
| `temperature` (room averages, m5atom temps) | per-bin mean; bridge gaps ≤ 15 min by time-linear interpolation; longer gaps stay entirely `NaN` (not partially filled) |
| `state` (dampers, climate setpoints) | last-observation-carried-forward, **unlimited** ffill (recorded on change only); seeded by scanning up to `seed_days` days before `start` for the most recent point; an absent per-day file means "did not change", not "missing" |
| `power` (Shelly channels) | clamp raw values at ≥ 0 **before** binning (channel 1 idles ~-7 W, CT offset), then per-bin mean; no fill of empty bins |
| `controller` (`input_number` PID terms, comp_speed, weighted_error, avg_deriv, meta_integral) | per-bin mean, then ffill up to 2 min only; longer gaps stay `NaN` (the app wasn't running — that fact matters for calibration) |
| `slow` (BOM `temperature_adelaide`/`humidity_adelaide`, EMHASS `power_load_5m`/`power_pv_5m`, all via `mean_value`) | time-linear interpolation directly onto the grid; no extrapolation beyond the first/last raw point |

### Observed coverage (2026-06-05 .. 2026-06-12, `--report`)

Room temperatures and dampers: 100% (dampers unlimited-ffill from a seed
before the window). `power.outdoor_unit` 99.9%. BOM/EMHASS slow series
~99.7-100%. The four raw `m5atom_*` temperature sensors (coil/inside/outside,
distinct from the per-room `sensor__temperature` averages) were much lower
(11-19% coverage, 100+ gap runs > 30 min) — that sensor appears to report
sparsely/unreliably over this window; not investigated further, flagging for
whoever calibrates against it. `aircon_meta_integral` similarly had large
gaps (11.6% coverage) despite the app clearly running throughout, worth a
look before using it for calibration.
