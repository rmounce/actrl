# 010: June-wide simulator fidelity scorecard

Status: review
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
- 2026-07-03 (post-merge, Claude Fable): re-ran the full scorecard on
  master AFTER the bedroom-lead refit (8633ef7) — this is the standing
  tuning-phase baseline (per-day CSV kept out of the repo; medians):
  kit_rmse 0.407, kit_bias −0.017, rms_all 1.057, energy_pct median
  +0.002 / mean +0.107, on_frac 0.246 vs 0.184, abovemin 0.006 vs 0.038,
  starts 9.5 vs 6.0, on_min 45.1 vs 55.7, off_min 98.0 vs 164.4.
  The bedroom refit barely moves June-wide medians (its wins are on the
  heavy days: 22nd energy −7%→−1%); the dominant remaining error is
  mild-day over-heating (sim rooms read cold without solar/internal
  gains → extra cycling: sim ~9-10 starts vs recorded 3-6 on light
  days, energy up to +86%). Next calibration target: daytime
  solar/internal gains, incl. revisiting kitchen's clamped-to-zero solar
  term.
- 2026-07-03 (after task 011 + hour-angle fix, commit 229c006): full
  scorecard re-run, the new standing baseline. Medians: kit_rmse 0.342,
  kit_bias +0.023, rms_all 0.677 (was 1.057), energy_pct −0.253 median /
  −0.241 mean (was ~0/+0.107 — sign flipped, see below), on_frac 0.165
  vs 0.184, abovemin 0.002 vs 0.038, starts 5.5 vs 6.0 (was 9.5 vs 6.0),
  on_min 54.2 vs 55.7, off_min 181.2 vs 164.4. Temperatures and cycle
  texture are now well-matched June-wide; the remaining error is a
  strikingly *uniform* −24% energy under-read: with room temps right,
  the sim delivers ~1.3x too much heat per kWh. Candidates: the
  efficiency-proxy level e (r²=0.28 fit) is too high, and/or unmodelled
  electrical overheads (90-min min-power purge spikes, start blips —
  visible as abovemin_rec 3.8% vs sim 0.2%). Next calibration target.
- 2026-07-03 (after energy refit, commit 4a40f51 — e level x0.80, min
  draw 665 W): standing baseline (superseded 2026-07-04). Medians: kit_rmse
  0.344, kit_bias −0.001, rms_all 0.679, energy_pct −0.036 median /
  −0.054 mean, on_frac 0.182 vs 0.184, abovemin 0.027 vs 0.038, starts
  5.0 vs 6.0, on_min 67.5 vs 55.7, off_min 188.7 vs 164.4. Known
  residuals: mild-day energy ~−30% (~0.6 kWh/day, run-time-decision
  degeneracy, not efficiency); sim on-runs ~20% longer / off-runs ~15%
  longer than recorded.
- 2026-07-04 (after task 012 — clearable ramp-up latch + slow latched
  climb, merge 5202a57): standing baseline (superseded same day). Medians:
  kit_rmse 0.332 (was 0.344), kit_bias −0.001, rms_all 0.679,
  energy_pct −0.036 median / −0.063 mean, on_frac 0.182 vs 0.184,
  abovemin 0.027 vs 0.038, starts 5.0 vs 6.0, on_min 68.1 vs 55.7,
  off_min 188.7 vs 164.4. No day's kit_rmse regressed >0.05. The fix
  removes the pinned-at-max/hard-cutoff artefact on deep-demand
  mornings (06-21/06-22 overshoot halved: +1.33→+0.63 K, +0.74→+0.41 K).
  Known residuals unchanged, plus: sim tapers ~2x faster than the real
  unit after a deep-demand ramp (reaches 0 ~1 h early on 06-22) — next
  candidate, likely deadband-integrator pacing vs the real unit's.
- 2026-07-04 (after task 013 — hold-at-setpoint stepping rule, merge
  e59a807): THE STANDING TUNING-PHASE BASELINE. The 2x-fast taper was
  Assumption 1's step-down sequence netting -2 vs the real unit's -1
  (actrl guess and emulated speed diverged ~1 per sequence); at-setpoint
  reports now hold, sequence nets -1. Medians: kit_rmse 0.340, rms_all
  0.654 (was 0.679), energy_pct -0.025 median / -0.057 mean, abovemin
  0.049 vs 0.038, on_frac 0.178 vs 0.184, starts 5 vs 6. No day's
  kit_rmse regressed >0.05. 06-22 taper now ends 08:50 (was 08:15,
  recorded 09:20); 06-21 peak increment 11 (was 13, recorded ~10),
  morning overshoot vs recorded peak: 06-22 +0.33 K, 06-21 +0.51 K.
  Remaining residuals: mild-day energy ~-30% (run-decision degeneracy);
  sim taper still ends ~30 min early on 06-22 (plant-side, rooms warm
  slightly faster through the descent).
- 2026-07-04 NEGATIVE RESULT (do not re-chase): hypothesis that the
  early taper end came from e(P) overcrediting heat at low power was
  tested by pivoting the e slope at 1.8 kW (b in {-0.04, 0, +0.04,
  +0.08} vs current -0.081) over 06-22/06-21/06-30/06-09 closed-loop.
  Rejected: 06-30 (overcast low-speed control day) energy blew out
  +3% -> +14..+28% with the morning run stretching 08:00 -> ~10:50 at
  every rotation, while the taper days' morning end barely moved
  (09:12 -> ~09:10) — so min-power heat is NOT globally overestimated,
  and the 06-22 late-morning "sim warms at min power where real house
  held flat" signature is most likely a day-specific unmodelled loss
  (school-morning occupancy/ventilation). Params left unchanged.
- 2026-07-03: implemented `analysis/scorecard.py` (import-reuses
  `load_day`/`replay` from `analysis/replay_day.py`, unmodified). All
  existing tests pass (`uv run pytest`: 159 passed, 1 skipped, unchanged).

  Subset spot-check (`--days 2026-06-22,2026-06-27`) matches
  `analysis/replay_day.py --date <day>` exactly for kitchen RMSE/bias,
  rms_all (mean of the 5 rooms' RMSEs), energy_pct, on_frac, abovemin —
  e.g. 06-22: kit_rmse 0.36/bias +0.08, energy sim 10.77 vs rec 11.59
  kWh (-7%), on_frac sim 48%/rec 51%, abovemin sim 10%/rec 13%, all
  reproduced by the scorecard row.

  While building the full-June run, found `load_day` only checks row
  *count* (>= 1440), not column completeness. Several June days have
  full row counts but mid-day archive outages (setpoint/humidity
  columns entirely NaN for hours) that `ffill` can't repair — these
  either silently produced garbage (non-finite) metrics or crashed the
  replay loop on a later day (NaN propagating into the emulator's
  setpoint arithmetic). Added a broader per-day try/except in
  `scorecard.py` (still a one-line notice + skip, same graceful-skip
  contract as `load_day`'s SystemExit) that also catches replay
  exceptions and rejects non-finite metrics — `analysis/replay_day.py`
  itself was not touched.

  Full June run (`uv run python analysis/scorecard.py --parquet
  /home/saltspork/actrl/data/processed/june.parquet --out
  /tmp/scorecard_june.csv`), ~30 min runtime, 24/31 days scored
  (2026-06-01, 02, 03, 12, 13, 17 skipped for the archive-gap reasons
  above; 2026-07-01 skipped as an out-of-range partial day). Per-day
  rows and summary block below (verbatim from the run):

  ```
  date            kit_rmse    kit_bias     rms_all  energy_pct on_frac_sim on_frac_rec abovemin_sim abovemin_rec  starts_sim  starts_rec  on_min_sim  on_min_rec off_min_sim off_min_rec
  2026-06-01  skipped: incomplete day: 870 minutes in archive
  2026-06-02  skipped: non-finite metric (archive gap not caught by load_day)
  2026-06-03  skipped: cannot convert float NaN to integer
  2026-06-04  0.37 -0.01 0.96 +0.52 0.31 0.18 0.02 0.02 10 6 44.90 43.50 90.09 168.43
  2026-06-05  0.38 +0.03 0.98 +0.27 0.33 0.22 0.05 0.06 9 6 52.56 52.83 96.70 160.43
  2026-06-06  0.60 -0.33 1.28 +0.36 0.23 0.14 0.00 0.01 9 3 37.56 69.33 110.20 308.00
  2026-06-07  0.61 -0.27 1.27 +0.04 0.24 0.19 0.00 0.05 6 3 56.50 89.67 157.29 292.75
  2026-06-08  0.54 -0.25 1.10 +0.22 0.16 0.12 0.00 0.01 6 3 38.17 55.67 173.00 318.25
  2026-06-09  0.35 -0.04 1.01 -0.23 0.13 0.14 0.02 0.03 4 3 45.75 66.00 251.40 310.50
  2026-06-10  0.65 -0.32 0.96 +0.22 0.09 0.06 0.00 0.00 4 3 33.75 30.33 261.00 337.25
  2026-06-11  0.57 -0.32 1.21 -0.04 0.15 0.16 0.08 0.07 2 2 111.50 112.50 405.67 405.00
  2026-06-12  skipped: non-finite metric (archive gap not caught by load_day)
  2026-06-13  skipped: non-finite metric (archive gap not caught by load_day)
  2026-06-14  0.42 -0.09 1.27 -0.14 0.31 0.29 0.06 0.09 9 7 49.67 59.71 99.30 127.75
  2026-06-15  0.34 -0.01 0.68 +0.01 0.17 0.15 0.01 0.01 7 4 35.86 55.75 148.62 243.40
  2026-06-16  0.67 -0.51 0.53 -0.08 0.03 0.02 0.00 0.00 1 1 39.00 28.00 700.50 706.00
  2026-06-17  skipped: non-finite metric (archive gap not caught by load_day)
  2026-06-18  0.52 -0.16 1.06 +0.97 0.14 0.06 0.00 0.00 7 2 28.14 40.50 155.38 453.00
  2026-06-19  0.33 -0.05 1.04 +0.52 0.24 0.14 0.00 0.01 9 6 38.00 34.00 109.80 176.57
  2026-06-20  0.33 -0.03 1.25 +0.34 0.38 0.26 0.00 0.01 10 8 55.40 46.38 88.60 118.78
  2026-06-21  0.46 +0.17 1.13 -0.01 0.47 0.38 0.11 0.15 10 8 67.50 69.00 76.50 98.67
  2026-06-22  0.36 +0.08 1.12 -0.07 0.48 0.51 0.10 0.13 11 16 63.00 45.75 67.91 41.65
  2026-06-23  0.36 +0.12 0.87 -0.08 0.42 0.39 0.06 0.10 9 7 67.78 80.57 92.22 109.50
  2026-06-24  0.38 -0.02 1.05 -0.12 0.39 0.42 0.09 0.14 9 9 63.11 67.22 87.20 83.50
  2026-06-25  0.41 -0.00 1.09 -0.07 0.36 0.38 0.08 0.12 9 7 57.22 77.57 102.78 112.12
  2026-06-26  0.33 +0.01 1.23 -0.04 0.36 0.33 0.08 0.12 8 9 65.12 52.56 102.11 96.70
  2026-06-27  0.34 +0.07 1.05 -0.08 0.38 0.42 0.13 0.16 8 9 67.50 67.11 100.00 83.60
  2026-06-28  0.37 +0.07 1.25 -0.11 0.26 0.24 0.00 0.06 10 6 37.40 57.17 96.91 156.71
  2026-06-29  0.36 +0.10 0.79 +0.18 0.22 0.16 0.00 0.01 10 6 31.40 39.50 102.36 171.86
  2026-06-30  0.34 +0.07 0.78 +0.20 0.23 0.17 0.00 0.01 10 9 33.00 27.22 100.91 119.50
  2026-07-01  skipped: incomplete day: 570 minutes in archive

  24 day(s) scored

  summary (median over scored days):
    kit_rmse     0.371
    kit_bias     -0.020
    rms_all      1.057
    energy_pct   -0.000
    on_frac_sim  0.249
    on_frac_rec  0.184
    abovemin_sim 0.017
    abovemin_rec 0.038
    starts_sim   9.000
    starts_rec   6.000
    on_min_sim   47.708
    on_min_rec   55.708
    off_min_sim  102.237
    off_min_rec  164.429
    energy_pct (mean) +0.116

  sim vs recorded (median):
    on_frac    sim 0.249  rec 0.184
    abovemin   sim 0.017  rec 0.038
    starts     sim 9.000  rec 6.000
    on_min     sim 47.708  rec 55.708
    off_min    sim 102.237  rec 164.429
  ```

  Takeaway vs the 4-day hand-picked validation in docs/calibration.md:
  kitchen RMSE/bias generalise well across the month (median 0.37 °C,
  -0.02 bias — line up with the 0.34-0.40 °C the 4-day set showed).
  Median energy_pct is ~0 but the mean is +12% and swings widely day to
  day (-23% to +97%) — the mild-day over-prediction noted in
  docs/calibration.md (unmodelled solar/internal gains making the sim's
  unlit rooms call for more heat than the real house needed) shows up
  across most of June, not just the two mild days originally checked.
  Cycle texture also diverges: sim runs shorter/more frequent on-cycles
  (median on_min 48 vs recorded 56) and shorter off-cycles (102 vs 164)
  with more starts (9 vs 6) — a modulation/hysteresis difference to
  watch when this scorecard is used to score controller tuning.

  `git diff a5ffaad -- tests/fixtures analysis/replay_day.py sim` is
  empty (0 lines): this branch touches only `analysis/scorecard.py` and
  this task file. `git diff master -- ...` is nonzero only because
  master advanced past this branch's base (a5ffaad) with an unrelated
  commit, 8633ef7 "Refit bedroom measured-air leads in closed loop",
  touching `sim/` — not anything from this task.

  Status: review.
