# 011: Orientation-resolved solar gains in the simulator

Status: ready
Branch: task/011-orientation-solar

## Background (read first)

docs/calibration.md and `analysis/solar_orient_fit.py`'s docstring. The
per-room solar term driven by the whole-house PV proxy
(`RoomParams.solar` × `pv_kw`) cannot represent window-orientation
timing: NE windows get a large *morning* beam while the PV proxy peaks
at noon. Fitted orientation-resolved terms (trajectory space, 2026-07-03):

| room    | s_ne  | s_nw  |
|---------|-------|-------|
| bed_1   | 0.298 | 0.010 |
| bed_2   | 0.577 | 0.000 |
| bed_3   | 1.354 | 0.000 |
| kitchen | 0.403 | 0.000 |
| study   | 0.271 | 0.082 |

[K/h at clear-sky full sun on that face.] These supersede
`_DEFAULT_SOLAR`, which must be zeroed (field kept for compatibility).

## Implementation (exactly this structure)

1. **`sim/solar.py` (NEW, stdlib `math`/`datetime` only — no numpy/pandas
   in `sim/`)**: port `sun_position` and `vertical_irradiance` from
   `analysis/solar_orient_fit.py` (which stays unchanged), operating on a
   single timezone-aware `datetime` and returning floats. Constants
   `LAT = -34.93`, `LON = 138.60`, `AZ_NE = 45.0`, `AZ_NW = 315.0`,
   `MIN_ELEV_DEG = 3.0`. Keep the same formulas (NOAA-style approximation)
   so analysis and sim agree.
2. **`sim/house.py`**: `RoomParams` gains `s_ne: float = 0.0` and
   `s_nw: float = 0.0`; new module dicts `_DEFAULT_S_NE`/`_DEFAULT_S_NW`
   with the table above wired into `_default_rooms()`; `_DEFAULT_SOLAR`
   set to 0.0 for every room (leave the `solar` field and `pv_kw`
   machinery in place, now inert by default — comment why).
   `House.step` gains keyword args `sun_ne: float = 0.0,
   sun_nw: float = 0.0` (cloud-scaled irradiance factors, 0..1) and adds
   `p.s_ne * sun_ne + p.s_nw * sun_nw` [K/h] to each room's forcing
   alongside the existing solar term. Defaults of 0 reproduce current
   behaviour exactly.
3. **`sim/closed_loop.py`**: `ClosedLoop.step` gains passthrough kwargs
   `sun_ne: float = 0.0, sun_nw: float = 0.0`, forwarded to `House.step`.
4. **`analysis/replay_day.py`**: compute the two drivers and feed them
   per minute:
   - cloudiness `c(t)` = recorded PV / per-minute-of-day PV *envelope*
     (max over the whole parquet, same as
     `analysis/solar_orient_fit.cloudiness`) clipped [0,1] — note the
     envelope must be computed from the FULL parquet before the day is
     sliced (add it in `load_day` as an extra returned column, or
     restructure minimally; document the choice);
   - `sun_ne(t)`/`sun_nw(t)` = `sim.solar.vertical_irradiance` at each
     minute × `c(t)`;
   - pass to `loop.step(..., sun_ne=..., sun_nw=...)`.

## Tests (`tests/test_house.py` + new `tests/test_solar.py`)

- `sim/solar.py` sanity: at 2026-06-21 12:00 UTC+9:30 in Adelaide, solar
  noon azimuth within ±10° of north and elevation ~31–33°; NE irradiance
  factor peaks in the local morning (larger at 09:00 than at 15:00);
  a SW-facing (225°) surface gets ~0 at winter noon.
- `House.step` with a single enabled `s_ne` room: constant `sun_ne`
  produces the analytic steady-state offset (same pattern as the
  existing constant-q test).
- Regression: all `s_ne = s_nw = 0` (explicitly constructed) with
  `sun_ne/sun_nw` supplied → trajectories identical to a run without the
  kwargs.

## Out of scope / do not modify

- `analysis/solar_orient_fit.py`, `analysis/solar_fit.py`,
  `analysis/solar_refit_openloop.py`, `analysis/scorecard.py`, `actrl.py`,
  `control.py`, `calib.py`, `tools/`, `appdaemon/` (NEVER deploy),
  `sim/midea_unit.py`, `sim/hvac.py`, golden fixtures
  (`tests/fixtures/` — never regenerate).
- No refitting; use the table's values verbatim.
- `/home/saltspork/actrl/data/` is READ-ONLY; your worktree has no
  `data/` — pass `--parquet /home/saltspork/actrl/data/processed/june.parquet`.
- No new dependencies; `git -C <worktree>` / cwd inside the worktree only.

## Acceptance criteria (runnable)

```bash
uv run pytest          # all pass (new tests included), no golden regen
python3 -c "import sim.solar, sim.house"     # stdlib-only
# before/after comparison — run each of these on your BASE commit first,
# save output, then after your change; record both in the Log:
for d in 2026-06-08 2026-06-15 2026-06-18 2026-06-22 2026-06-27; do
  uv run python analysis/replay_day.py --date $d \
      --parquet /home/saltspork/actrl/data/processed/june.parquet
done
git diff master -- tests/fixtures | wc -l    # 0
```

Expected direction (not a hard gate — record what you get): mild days
(06-08, 06-15, 06-18) energy over-prediction and bedroom daytime cold
bias should drop substantially; heavy days (06-22, 06-27) should not
regress badly. The reviewer judges acceptability.

## Questions

(none yet)

## Log

- 2026-07-03: spec written (Claude Fable), status ready.
