# 008: House thermal model — per-room RC simulator

Status: done
Branch: task/008-house-thermal-model

## Goal

`sim/house.py`: a deterministic per-room thermal model of the house,
parameterised from the June system-identification results in
**docs/calibration.md** (read it first), validated by replaying recorded
free-running nights. This is the plant model the simulator (ideas.md §3)
will drive with the controller + unit emulator.

## Model (implement exactly this structure)

Per room i (rooms: bed_1, bed_2, bed_3, kitchen, study):

```
dT_i/dt = a_i * (T_out - T_i) + c_i * (T_others_i - T_i) + g_i + q_i(t)
```

- `T_others_i` = mean of the other four rooms' temperatures.
- `a_i = 1/tau_out_i`, `c_i = 1/tau_cpl_i`, `g_i` [K/h] from the
  docs/calibration.md per-room table (defaults baked into a
  `HouseParams` dataclass; every value overridable).
- `q_i(t)` [K/h] = externally supplied heat input per unit room thermal
  mass (0 when free-running; the HVAC model supplies it later).
- Integrate with simple forward Euler at a configurable step (default
  10 s = the control interval; times in hours internally, document the
  unit convention prominently).

API sketch:

```python
sim = House(params: HouseParams, initial_temps: dict[str, float])
sim.step(t_out: float, q: dict[str, float], dt_s: float = 10) -> dict[str, float]
sim.temps  # current dict
```

Pure Python + stdlib. No pandas in `sim/` (analysis scripts may use it).

## Deliverables

1. `sim/house.py` (+ `sim/__init__.py` if not already present from a
   parallel task — create it minimal/empty; do not put logic in it).
2. `tests/test_house.py` — offline, synthetic:
   - single-room analytic check: with coupling and gains zeroed, decay
     from T0 toward constant T_out matches exp(-t/tau) within 1% over
     several hours at 10 s steps;
   - equilibrium: constant inputs converge to the analytic fixed point;
   - coupling symmetry: two rooms initialised apart converge toward each
     other faster than toward outdoor;
   - q input: constant q produces the analytic steady-state offset;
   - Euler stability guard: constructor/step must reject dt that makes
     any a_i+c_i term unstable (dt > tau/2 style check).
3. `analysis/validate_house_sim.py` — replay validation against the real
   archive (pandas allowed here): load `data/processed/june.parquet` via
   `calib.load_range`-style columns (the parquet may need regenerating:
   `uv run python calib.py --data-dir /home/saltspork/actrl/data --start
   2026-06-01 --end 2026-06-30 --out data/processed/june.parquet`).
   Reuse the free-running night-window detection from
   `analysis/sysid_june.py` (import it or lift the function — do not
   modify that file). For each window: initialise the model from recorded
   room temps, drive with recorded `temperature_adelaide`, q=0; report
   per-room and house-average RMSE per window and the medians across
   windows. Print a table. **Flag (do not fail) any house-average median
   RMSE > 0.5 °C** — the reviewer judges acceptability.
4. Task file Log updated with the validation numbers; Status → review.

## Out of scope / do not modify

- `actrl.py`, `control.py`, `statctrl.py`, `hvac_watchdog.py`, `calib.py`,
  `analysis/sysid_june.py`, `tools/`, `systemd/`, `appdaemon/` (NEVER
  deploy), existing tests and fixtures.
- `sim/midea_unit.py` (a parallel task owns it — do not touch even if
  present in your checkout).
- No HVAC/heat-input modelling beyond accepting the q dict.
- The real archive at `/home/saltspork/actrl/data/` is READ-ONLY for the
  validation run; write parquet/output under your own worktree's `data/`.
- No new dependencies.

## Acceptance criteria (runnable)

```bash
uv run pytest                                   # all pass, offline
python3 -c "import sim.house"                   # stdlib-only import
uv run python analysis/validate_house_sim.py    # prints per-window RMSE table
git diff master -- analysis/sysid_june.py calib.py | wc -l   # expect 0
```

## Questions

(none yet)

## Log

- 2026-07-02: spec written (Claude Fable), status ready.
- 2026-07-02: implemented `sim/house.py` (`HouseParams`/`RoomParams`
  dataclasses with docs/calibration.md defaults baked in, forward-Euler
  `House` with a stability guard rejecting `dt_s` where any room's
  `dt_h > tau_eff/2`, `tau_eff = 1/(a+c)`). `tests/test_house.py`: 8 offline
  analytic tests (single-room exponential decay within 1%, symmetric-room
  equilibrium fixed point, coupling-pulls-together-faster-than-outdoor,
  constant-`q` steady-state offset, stability guard at construction and at
  `step()`, default-params room coverage, missing-room validation) — all
  pass, plus full existing suite (115 passed, 1 skipped, unrelated
  pre-existing skip).
  Regenerated `data/processed/june.parquet` under this worktree (gitignored,
  read-only source archive untouched) via `calib.py`.
  `analysis/validate_house_sim.py` imports `night_windows` from
  `analysis/sysid_june.py` unmodified (`git diff master -- analysis/sysid_june.py
  calib.py` = 0 lines), replays all 24 free-running June night windows
  free-running (q=0) from recorded initial room temps driven by recorded
  outdoor temp, at 1-min steps (native data grid). Result:

  ```
  24 windows replayed.
  per-room medians (C): bed_1 0.126  bed_2 0.128  bed_3 0.142  kitchen 0.253  study 0.138
  house-average median RMSE: 0.141 C
  ```

  All well under the 0.5 C flag threshold — no flag raised. Kitchen has the
  highest median RMSE (0.253 C), consistent with docs/calibration.md's note
  that it's the noisiest/least-identifiable room (tau_out ≈ 118.6 h,
  effectively unidentifiable outdoor path, r² = 0.50 in the original fit).
  All acceptance criteria pass. Status → review.
- 2026-07-02: reviewed and merged to master (Claude Fable, supervising).
  Implemented by a Sonnet 5 subagent. Verified: main checkout untouched,
  only in-scope files changed, tests and the 24-night validation replay
  re-run independently in the worktree (identical numbers), model
  structure and defaults match docs/calibration.md exactly, stability
  guard is conservative (tau_eff/2 vs the true 2*tau_eff limit).
  Status done.
