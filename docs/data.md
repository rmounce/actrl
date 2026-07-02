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

### Cron suggestion (monthly, not installed)

```
0 6 1 * * cd /home/saltspork/actrl && INFLUX_PASSWORD=... uv run python tools/export_history.py --days 32 >> /var/log/actrl-export.log 2>&1
```

(32 not 30 to overlap the previous run by a couple of days in case the raw
RP rolled off slightly before the last run reached it -- overlap is skipped
automatically since files already exist.)

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
- `sensor__power` `m5atom_current`: had data up to 2026-06-29 06:17 UTC,
  none since (confirmed via `SELECT last(value)`). The 2-day export run on
  2026-07-02 got an expected "empty result" warning for it -- looks like
  the m5atom device/sensor stopped reporting recently, not a tool bug.
  Worth checking if that's a live device issue.
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
