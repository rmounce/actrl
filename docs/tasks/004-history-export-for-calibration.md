# 004: Export recorded HA history for simulator calibration

Status: draft (needs schema discovery + user confirmation before `ready`)
Branch: task/004-history-export-for-calibration

## Goal

A repeatable tool that exports the time series needed to calibrate the house
thermal / HVAC simulator (ideas.md §3) from the household's InfluxDB into
tidy per-day files the simulator work can consume.

## Background

- InfluxDB runs on this host (`docker ps`: `influxdb`; config under
  `/opt/dockerfiles/influxdb/`, sudo may be required — see the
  ai-energy-forecast-slop repo notes). HA also has its own recorder DB.
- Schema (org/bucket/measurement naming, retention, whether all needed
  entities are exported to Influx) is **unconfirmed** — hence draft status.

## Entities to export (from actrl.py / docs/actrl.md)

- Per room: `sensor.<room>_average_temperature`, `sensor.<room>_feels_like`,
  `cover.<room>` position, `binary_sensor.<room>_{window,door}`,
  `input_number.<room>_pid`, `input_number.<room>_damper_target`,
  room climate setpoints.
- Whole system: `climate.m5atom_climate` (mode, setpoint, fan_mode),
  `number.m5atom_static_pressure`, `binary_sensor.m5atom_{compressor,outdoor_fan}`,
  `input_number.aircon_comp_speed`, `input_number.aircon_weighted_error`,
  `input_number.aircon_avg_deriv`, `input_number.fake_temperature`,
  `input_number.grid_surplus_integral`.
- Environment/energy: outdoor temperature + humidity (entity TBC), unit
  power consumption (entity TBC), PV curtailment
  (`sensor.mpc_p_pv_curtailment`), grid import/export (entity TBC).

## Steps (to firm up before ready)

1. Discovery: query InfluxDB for available measurements/entities from the
   list above; record findings (schema, retention, gaps) in `docs/data.md`.
2. `tools/export_history.py`: entity list + date range in a small config;
   output one Parquet (or CSV) file per day under a data dir (gitignored);
   resample-on-read left to consumers — export raw points.
3. Document usage in `docs/data.md`; add a `uv` dependency group for the
   Influx client if needed (dev-only).

## Open questions for the user

- Are all the listed entities actually exported to InfluxDB, and how far
  back does retention go?
- Preferred output location for exported data (repo-adjacent dir? NAS?).
- Outdoor temp/humidity + power entity names.

## Log

- 2026-07-02: draft written (Claude Fable).
