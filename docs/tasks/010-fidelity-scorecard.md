# 010: June-wide simulator fidelity scorecard

Status: ready
Branch: task/010-fidelity-scorecard

## Background (read first)

docs/calibration.md "Whole-day closed-loop replay" and the measured-air
sections. The closed-loop simulator has so far been validated by eye and
by per-day RMSE on four hand-picked June days, and the measured-air lead
refit was tuned against 2026-06-22 specifically. Before the sim is used
to score controller-tuning experiments we need the same metrics computed
over *every* archived June day — both to check the recent fixes
generalise beyond the days they were fitted/validated on, and to serve as
the standing regression gate / scoring harness for the tuning phase.

## Deliverable

`analysis/scorecard.py`: replays every complete local day in the June
parquet through the existing closed loop and prints one row per day plus
a median summary. Reuse `analysis/replay_day.py`'s `load_day` and
`replay` functions by import — do not duplicate or modify them.

Per-day metrics (columns):

- `kit_rmse`, `kit_bias` — kitchen Tm vs recorded average_temperature
  [°C] (kitchen is the cycling driver and the best-anchored room).
- `rms_all` — mean of the five rooms' RMSEs [°C].
- `energy_pct` — (sim − recorded)/recorded HVAC energy, recorded =
  outdoor+indoor unit power clipped at 0 (same as replay_day.report).
- `on_frac_sim` / `on_frac_rec` — fraction of minutes unit running
  (sim `p_kw > 0`; recorded outdoor power > 300 W).
- `abovemin_sim` / `abovemin_rec` — sim `increment > 0`; recorded
  outdoor power > 800 W.
- `starts_sim` / `starts_rec` — number of off→on transitions per day.
- `on_min_sim` / `on_min_rec`, `off_min_sim` / `off_min_rec` — mean
  contiguous on-run and off-run duration in minutes (cycle texture; use
  the same on/off definitions as the starts columns).

After the per-day table, print a summary block: median (and, for
`energy_pct`, also mean) of each metric over the days run, plus
sim-vs-rec medians side by side for the paired columns.

CLI:

```
uv run python analysis/scorecard.py [--parquet PATH] [--out csv_path]
    [--days 2026-06-22,2026-06-27]   # optional subset for quick runs
```

Days where `load_day` raises (incomplete archive coverage) are skipped
with a one-line notice, not fatal. Full-June runtime will be tens of
minutes (each replay is ~8640 controller cycles) — that's acceptable;
print each row as it completes (flush) so progress is visible.

## Out of scope / do not modify

- Everything except the new `analysis/scorecard.py` and the task file:
  in particular `analysis/replay_day.py`, `sim/`, `actrl.py`,
  `control.py`, `calib.py`, `tools/`, `appdaemon/` (NEVER deploy),
  existing tests, and **never regenerate any golden fixture**
  (`tests/fixtures/`).
- No model or parameter changes; this is measurement only.
- The real archive at `/home/saltspork/actrl/data/` is READ-ONLY. Your
  worktree has no `data/`; pass
  `--parquet /home/saltspork/actrl/data/processed/june.parquet`. Write
  any `--out` CSV under your worktree or /tmp.
- No new dependencies. All git/uv commands via `git -C <worktree>` /
  cwd inside the worktree; never operate on the main checkout.

## Acceptance criteria (runnable)

```bash
uv run pytest                       # unchanged, all pass
# spot check: subset run matches analysis/replay_day.py's numbers for the
# same days (kitchen RMSE/bias, energy %, on-fractions)
uv run python analysis/scorecard.py --days 2026-06-22,2026-06-27 \
    --parquet /home/saltspork/actrl/data/processed/june.parquet
uv run python analysis/replay_day.py --date 2026-06-22 \
    --parquet /home/saltspork/actrl/data/processed/june.parquet
# full June run completes, skipping incomplete days gracefully
uv run python analysis/scorecard.py \
    --parquet /home/saltspork/actrl/data/processed/june.parquet \
    --out /tmp/scorecard_june.csv
git diff master -- tests/fixtures analysis/replay_day.py sim | wc -l  # 0
```

Record the full-June summary block in the task Log (it is this task's
main output — the baseline the tuning phase will be scored against).

## Questions

(none yet)

## Log

- 2026-07-03: spec written (Claude Fable), status ready.
