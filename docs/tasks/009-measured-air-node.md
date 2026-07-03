# 009: Measured-air lead node — fast sensor dynamics per room

Status: ready
Branch: task/009-measured-air-node

## Background (read first)

docs/calibration.md, especially "Whole-day closed-loop replay" and the
feels-like section. The room sensors (`{room}_average_temperature`) do not
read the bulk thermal mass the RC model simulates: superposed unit-stop
events (12 clean events, June, kitchen damper >50%) show the measured
temperature sagging at ~2 K/h for the first ~10 min after the compressor
cuts, relaxing to the single-RC decay rate (~0.5 K/h) by ~40 min. The
sensors sit in a small fast air mass that leads the bulk room temperature
while HVAC heat is being delivered. This is why the closed-loop sim shows
"lag then overshoot" on morning ramps (controller pushes to increment 11
where the real unit peaked at 9) and cycles too slowly/smoothly at midday.

## Model (implement exactly this structure)

Add a per-room *measured temperature* state `Tm_i` on top of the existing
(unchanged) bulk state `T_i` in `sim/house.py`:

```
dTm_i/dt = ((T_i + lead_h_i * q_i) - Tm_i) / tau_meas_h_i     [K/h]
```

- `q_i` [K/h] is the same per-room HVAC forcing already passed to
  `House.step` — the measured node is a *sensor model*, it carries no
  energy and must not feed back into the bulk ODE, which stays exactly
  as-is (all existing tau_out/tau_cpl/gain/solar behaviour untouched).
- Steady state: `Tm = T + lead_h * q` (supply-air offset while heating).
- `q -> 0`: `Tm` decays toward `T` with time constant `tau_meas_h`
  (the observed post-stop sag).
- New `RoomParams` fields: `tau_meas_h: float = 0.0`, `lead_h: float = 0.0`.
  **`tau_meas_h == 0` disables the node: `Tm ≡ T` identically** (exact
  passthrough, not a small-tau approximation). Keep 0.0 defaults — the
  fitted values land in a separate calibration commit (analysis running in
  parallel; not your task).
- Integrate forward-Euler alongside the bulk state at the same `dt`;
  initialise `Tm = T` initial. Extend the existing Euler stability guard
  to also reject `dt_h > tau_meas_h / 2` when the node is enabled.
- Expose `House.temps_measured -> dict[str, float]` (equals `temps` when
  disabled). `House.step` return value stays the bulk temps (unchanged
  signature/semantics).

## Wiring

- `sim/closed_loop.py`: `_write_room_temps` must present
  `house.temps_measured[room] + ctrl_offset` to the controller (the real
  controller reads the real sensors, which are the measured node).
  Telemetry rows gain `Tm_{room}` columns alongside the existing
  `T_{room}` (equal when disabled).
- `analysis/replay_day.py`: the per-room RMSE/bias comparison must compare
  recorded `{room}_average_temperature` against **`Tm_{room}`** (recorded
  data *is* the measured signal). Keep the CSV output containing both
  column sets. With the default (disabled) params every reported number
  must be bit-identical to master — that is the regression gate.

## Deliverables

1. `sim/house.py` changes as above (stdlib only, no pandas in `sim/`).
2. `sim/closed_loop.py` + `analysis/replay_day.py` wiring as above.
3. `tests/test_house.py` additions (offline, synthetic):
   - disabled default (`tau_meas_h=0`): `temps_measured == temps` exactly
     after many mixed steps with nonzero q;
   - steady state: constant q, enabled node → `Tm - T` converges to
     `lead_h * q` within 1% after ~5 tau;
   - stop decay: after q returns to 0, `(Tm - T)` decays as
     `exp(-t/tau_meas_h)` within 1% over 2 tau;
   - stability guard rejects `dt_h > tau_meas_h / 2` when enabled.
4. Task file Log updated (numbers from the acceptance runs); Status → review.

## Out of scope / do not modify

- `actrl.py`, `control.py`, `statctrl.py`, `hvac_watchdog.py`, `calib.py`,
  `tools/`, `systemd/`, `appdaemon/` (NEVER deploy), `sim/midea_unit.py`,
  `sim/hvac.py`, existing tests other than `tests/test_house.py`, and
  **never regenerate any golden fixture** (`tests/fixtures/`).
- No parameter fitting — defaults stay 0.0/disabled.
- The real archive at `/home/saltspork/actrl/data/` is READ-ONLY. Your
  worktree has no `data/`; pass the main checkout's parquet path
  explicitly (see acceptance) and write any `--out` CSVs under your
  worktree or /tmp, never under `/home/saltspork/actrl/data/`.
- No new dependencies. Do not run anything against the main checkout
  working tree: all git/uv commands as `git -C <your-worktree>` /
  `uv run --directory <your-worktree>` or with cwd inside the worktree.

## Acceptance criteria (runnable)

```bash
uv run pytest                          # all pass, no golden regen
python3 -c "import sim.house"          # stdlib-only import
# regression gate: identical output to master with disabled defaults
uv run python analysis/replay_day.py --date 2026-06-22 \
    --parquet /home/saltspork/actrl/data/processed/june.parquet
# -> per-room RMSE/bias, energy %, on-fractions all identical to the same
#    command run on master (capture master's output first for the diff)
git diff master -- tests/fixtures | wc -l    # expect 0
```

## Questions

(none yet)

## Log

- 2026-07-03: spec written (Claude Fable), status ready.
