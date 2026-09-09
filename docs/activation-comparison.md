# 2026-09-09 activation comparison — checkpoint

Status: INITIAL THREE-WAY COMPARISON COMPLETE. No deployment.

## Objective / agreed policies

- Review actual study timer activation at 08:33:10 Adelaide, through 10:16.
- Compare original controller, committed match-leader seeding, and stronger
  proportional-relative seeding. User favours investigating early airflow
  redistribution to avoid subsequent capacity/fan escalation.
- Selected third variant: transfer incumbent raw leader's I to
  the qualifying room if that increases I. Difference becomes P+D, retaining
  derivative feedback. Same >=1.0°C requested step / >=0.5°C effective error
  gates. This can boost a newly activated room already leading on raw output
  if its I is lower; unlike match-leader, it need not be below the raw leader.
- Production implementation now uses the selected relative policy, NOT
  deployed. The analysis retains the historical match-leader arm.

## Files / resume commands

- `analysis/activation_comparison.py`: offline loader, policy context manager,
  three-hour warmup, event anchoring, 18-arm sensitivity matrix, CSV metrics.
- `tests/test_activation_comparison.py`: 7 focused tests passed; policy
  identity, gates, relative P+D, context restoration. Full suite not yet run
  for this comparison (previous production change: 230 passed, 1 skipped).
- Numeric input snapshot: `data/activation_review/raw/2026-09-{08,09}/`
  (gitignored copy of `/tmp/actrl-review-history`; includes preceding history).
- Results and raw metadata: `analysis/out/activation_20260909/` (gitignored).
  `observed.csv`, `summary.csv`, 18 trajectory CSVs, `metadata.json`.
- Historical metadata fetch scratch: `/tmp/activation_metadata.py`.
  Earlier probe files: `/tmp/actrl_activation_*probe.py` (scratch, untracked).
- Run: `.venv/bin/python analysis/activation_comparison.py --data-dir data/activation_review`
- Tests: `.venv/bin/pytest -q tests/test_activation_comparison.py`
- Latest edit gates experimental policies until EVENT (`comparison_enabled`);
  rerun the matrix after resuming. Existing CSVs predate this final gating
  edit. All numerical results below are provisional, not a final report.

## Inputs confirmed / assumptions

- Local HA `/opt/dockerfiles/hass/config/packages/aircon.yaml`: climate
  current_temperature selects feels-like when enabled; formula is
  T + 0.33 * RH/100 * 6.105 * exp(17.27*T/(237.7+T)) - 4.
- Reconstructed feels-like matches recorded study/kitchen climate temperature
  with ~0.03°C RMSE, near-zero bias, over the replay window. Therefore use
  reconstructed feels-like for control/target-crossing metrics, Tm for raw
  sensor comparison, T for model bulk. Prior conversation mixed these;
  corrected in docs/ideas.md.
- Metadata query: last recorded static pressure 2; no changes in window.
  All room climates heat_cool. Recorded windows off, bed_2 door on. Other
  door states not in retrieved history; simulator assumes open. Missing
  history does NOT establish that the entity is absent or the door open.
- Only ac_manual_mode returned among selected booleans (off); surplus
  integral zero. Surplus and price effects disabled in replay; other toggle
  histories unavailable. Room targets/outdoor/RH use recorded series.
- Solar not measured: clear-sky geometry scaled 0, 0.5, 1 for sensitivity.
- Causal forward-fill 10s grid; finite history, no future backfill. Loader
  allows 15min tail staleness (target/setpoint states exempt). Step time
  remains 08:33:10. Actual sampled peaks can miss sub-10s raw peaks.

## Replay design / limitations

- Unanchored arm: free-running full plant/controller from 05:33:10. It drifts
  enough that study is already fully open at activation; policies identical.
  This is a model-validation failure, not evidence policies are equivalent.
- Anchored arms: recorded effective temperatures drive controller warmup
  until activation, while thermal/plant model runs. At activation, measured
  temperatures are set to last recorded pre-event values; model thermal lead
  is retained (sensitivity 0x, 1x, 2x). Bulk state remains an assumption.
- Reconstruct P/D from recorded preceding temps/targets; I = last published
  PID - P - D. This assumes airflow top-up was inactive at the boundary.
  Set room target history, enable flags, mode heat, estimated capacity and
  unit increment from record; initialise damper positions from published
  PID curve, and electrical lag from preceding recorded power.
- Unit ramp flags, heat-delivery lag, capacity counters and internal state
  otherwise come from warmup. Not fully reconstructed from actual device.
- After activation, no forcing of recorded temperatures, power or dampers.
  Recorded targets/weather/moisture offsets remain exogenous.
- Minimum-airflow wrapper measures extra duct-equivalents added and duration.
  All existing runs report ZERO top-up after activation. Fan-report override
  quirks from actual HA are not modelled; simulator actuates services instantly.
- Main validation gap persists: baseline capacity too low / recovery too fast.
  Do not present energy/comfort improvements as calibrated predictions.

## Initial conclusion

- The anchored replay consistently ranks the policies in this order for study
  recovery: scale relative to leading PID, match leading PID, original.
- Match leading PID is the conservative improvement. Across the five anchored
  assumption sets it reaches the study target 1.17–4.0min before original and
  reduces study deficit by 2.25–2.67K·min. Its simulated study peak remains
  within 0.05°C of original.
- Scale relative to leading PID is much stronger. It reaches target
  5.67–7.83min before original and reduces study deficit by 6.18–6.63K·min,
  but raises the simulated study peak by 0.10–0.20°C and diverts the kitchen's
  first damper command from 100% to 20%.
- The stronger policy's lower simulated peak power and energy are not yet
  credible forecasts. The baseline reaches target too early and uses much
  less compressor power and energy than the house did. Its worse temperature
  RMSE is further evidence that this replay cannot determine real effect size.
- No arm invokes minimum-airflow top-up. The benefit comes from changing
  relative room demand and the resulting capacity trajectory, not from
  escaping a simulated minimum-airflow feedback loop.

## Central run (anchor, solar .5, lead 1)

Window: activation to 10:16 (~103min). Effective temperature metrics.

| Metric | Original | Match leading PID | Scale relative |
|---|---:|---:|---:|
| First study opening | 50% | 100% | 100% |
| First kitchen opening | 100% | 100% | 20% |
| Time to study target | 17.5min | 15.5min | 11.17min |
| Peak electrical power | 1.225kW | 1.058kW | .887kW |
| Window energy | .607kWh | .545kWh | .488kWh |
| Study raw-sensor RMSE against actual | .316°C | .368°C | .529°C |
| Minimum-airflow top-up duration | 0 | 0 | 0 |

- Actual sampled: target crossing ~23.67min (exact rounded climate crossing
  08:56:09); peak power ~1.992kW (raw ~1.999); energy ~1.106kWh;
  effective study peak ~20.761°C at +63.83min. Check end-of-interval timestamp
  convention; reported simulated times have ~10s resolution/phase uncertainty.
- Across anchored solar/lead sensitivity, proportional catches target first.
  Baseline mismatch prevents confident effect-size claims. Inspect peaks,
  overshoot integral, energy and cancellation before choosing policy.

## Airflow hypothesis — new evidence

- SP2 floors: low 1.0, medium 115/97=1.186, high 144/97=1.485 duct-equivalents.
- From recorded published PIDs: sum(weight * min(1,(max(0,PID)/2)^1.5))
  never falls below ~1.514 during 08:33:10–09:07:10, assuming open doors.
  During continuous medium fan (08:42:29–09:07:11), this is comfortably above
  the medium floor. Thus minimum-airflow binding is NOT supported under
  these assumptions. Still inspect door derating and cycle alignment before
  ruling it out. More aggressive priority can help via integral redistribution
  even when the hypothesised airflow feedback did not occur this morning.
- Fan reports alternate requested medium with off/low early, then continuous
  medium 08:42:29–09:07:11. off falls back to high in airflow code; published
  airflow above even that floor weakens the hypothesis at open-door SP2.

## Verification

- Final gated matrix rerun 2026-09-09. For each anchored sensitivity set, all
  three trajectory CSVs are exactly identical for the 1,080 pre-event rows.
- `tests/test_activation_comparison.py`: 7 passed. Covers policy identity,
  activation/error gates, relative P+D preservation and context restoration.
- Five anchored assumption sets cover solar scale 0/.5/1 and retained thermal
  lead scale 0/1/2. Policy ranking is unchanged across them.
- Unanchored policies are identical because warmup drift leaves the study
  already fully open. This is a failed validation arm, not contrary evidence.
- Central anchored cancellation replay: cancelling study at +10min closes its
  damper in the same simulated cycle under all three policies. It remains
  closed for the rest of the window; proportional uses .370kWh versus .372kWh
  original, so no cancellation energy claim is warranted.
- Focused simultaneous activation: study and bed 2 share one pre-seeding
  integral reference without cascading. For illustrative P+D values 1.3/.8
  against kitchen -.1, normalized outputs are 2/1.5/.6 and damper targets are
  100%/65%/16.4%. Cancellation and a fresh later activation also behave once
  each as intended.

## Possible follow-up

1. Inspect later effects on all rooms for a simultaneous real-world activation
   once suitable recorded data exists.
2. Investigate baseline compressor mismatch without fitting to variant gains.
   Warmup/device internal state and measured/bulk model mismatch are candidates.
   If unresolved, report comparative results as conditional, not validated.
3. Revisit the stronger policy only after cancellation and multi-room tradeoffs
   are scored. Do not deploy either policy as a result of this initial replay.
