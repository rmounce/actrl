# Comfort/PID tuning — findings log

Method: candidate controller configs run through the calibrated closed-loop
simulator (`analysis/tune.py`, task 014) with constants overridden at
runtime (`analysis/ctrl_overrides.py`) — production actrl.py/control.py
untouched. Scored by `analysis/comfort.py` over replayed June days;
exogenous inputs (weather, solar, schedule, feels-like offsets) identical
across candidates, so comparisons are apples-to-apples A/B.

Metrics: time_in_band, deg_min_below (K·min underheating), rise_time_med
(minutes from schedule-ramp start to reaching the final target; the
statctrl ramp itself takes ~105 min), overshoot_rise_max (peak past the
stepped-to low within 2 h of reaching it), osc_per_h (kitchen extrema
>=0.15 K per running hour), energy_kwh, starts, abovemin_frac.

## Sweep 1 — single knobs, 6-day set (2026-07-04)

Days: 06-21, 06-22, 06-27 (heavy/taper), 06-30 (overcast), 06-14
(medium), 06-09 (mild). Medians:

| candidate | overshoot | rise | deg_min_below | in_band | osc/h | kWh |
|---|---|---|---|---|---|---|
| baseline | 1.19 | 116 | 12.9 | .965 | 1.43 | 7.02 |
| deadband_ki 0.025 (x2) | 0.99 | 113 | 10.9 | .967 | 1.43 | 7.14 |
| deadband_ki 0.00625 (x0.5) | 1.41 | 125 | 15.5 | .960 | 1.43 | 6.95 |
| deriv_factor 15 | 1.17 | 123 | 16.4 | .963 | 1.37 | 6.77 |
| deriv_factor 20 | 1.03 | 126 | 20.0 | .960 | 1.61 | 6.82 |
| room_ki 0.002 (x2) | 1.08 | 111 | 14.6 | .954 | 1.40 | 6.86 |
| global_ki 0.0005 (x2) | no effect | | | | | |
| min_power_threshold -1.25 | no effect | | | | | |

Findings: `control.global_deadband_ki` is the dominant comfort knob and
monotonic in this range — faster integrator improves every comfort axis
at once (it paces both step-up AND step-down demand, so warmups engage
harder and tapers back off sooner). Longer derivative horizon trades
overshoot for sluggish warmups (bad deal). `actrl.global_ki` (meta
integral) and `min_power_threshold` don't bind on these days.

## Sweep 2 — dose + combinations, 6-day set

| candidate | overshoot | rise | deg_min_below | in_band | osc/h | kWh | starts |
|---|---|---|---|---|---|---|---|
| baseline | 1.19 | 116 | 12.9 | .965 | 1.43 | 7.02 | 5.5 |
| deadband_ki 0.05 (x4) | 0.98 | 111 | 7.9 | .975 | 1.41 | 7.02 | 6.5 |
| x2 + room_ki x2 | 0.89 | 111 | 11.5 | .962 | 1.49 | 7.01 | 6.0 |
| x2 + deriv 15 | 0.97 | 117 | 14.3 | .966 | 1.46 | 6.97 | 6.0 |
| **x4 + room_ki x2** | **0.84** | **106** | 10.7 | .962 | 1.52 | 7.00 | 6.0 |

## Full-June validation — 24 days (the scoreable set)

| candidate | overshoot | rise | deg_min_below | in_band | osc/h | June kWh | June starts |
|---|---|---|---|---|---|---|---|
| baseline | 0.91 | 110 | 3.10 | .987 | 1.43 | 112.1 | 111 |
| deadband_ki x4 | 0.86 | 110 | 2.63 | .987 | 1.52 | 111.4 | 119 |
| x4 + room_ki x2 | 0.82 | 108 | 2.28 | .988 | 1.52 | 111.5 | 120 |

June-wide medians are more muted than the heavy-day subset (mild days
have small/no events); the gains concentrate on cold mornings — which is
when comfort matters. Robustness scan (any day worse than baseline by
>0.2 K overshoot / >5 K·min underheating / >1.0 osc/h):

- deadband_ki x4: one day (06-28: overshoot 0.77 -> 1.20).
- x4 + room_ki x2: one day (06-22, the coldest morning: overshoot
  actually improves 1.10 -> 0.76 but deg_min_below 25.9 -> 36.3 — the
  faster integrator redistributes damper authority during the deepest
  warmup).

Both cost ~zero energy June-wide; both add ~8 starts/June (+7%);
oscillation +6%.

## Recommendation (pending Ryan review — NOT deployed)

`control.global_deadband_ki` 0.0125 -> 0.05, `actrl.room_ki`
0.001 -> 0.002 ("x4 + room_ki x2"): overshoot −10% June-wide (−30% on
cold mornings), rise ~5–10 min faster, underheating −27%, energy flat.

Caveats before real-world rollout:

1. **Sensor noise**: the sim's temperature traces are noise-free; a 4x
   deadband integrator also integrates real sensor noise 4x faster.
   Real-world hunting may be slightly worse than simulated. Suggest a
   staged trial: x2 (0.025) first — it captures over half the benefit —
   then x4 if the recorded behaviour stays clean.
2. Winter-only, June-only evidence; cooling season unvalidated.
3. The x4+room_ki combo deepens the single coldest morning's
   underheating trough (06-22); if that trade reads badly in practice,
   plain deadband_ki x4 is the safer variant (best underheating profile,
   one overshoot regression day).

Reproduce: `uv run python analysis/tune.py --days <days> --set
control.global_deadband_ki=0.05 --set actrl.room_ki=0.002`. Raw per-day
CSVs from these sweeps are session-scratch; the tables above are the
record.

## Sensor-noise check (2026-07-05)

Measured from quiet archive nights (HVAC off 30+ min, 01:00-05:00, n=6072
min): all five average_temperature sensors show 0.005-0.007 K residual
std, white (lag-1 rho ~ 0); humidity noise contributes < 0.001 K to
feels-like. Two orders of magnitude below control-relevant errors — NOT
worth modelling in the sim; the noise caveat on the deadband_ki
recommendation is withdrawn. (The 1-min archive can't see sub-minute
noise, but the controller's 5-min WMA derivative and minutes-scale
integrators filter that band regardless.)

## Pre-heat ramp strategy study (2026-07-05, analysis/preheat.py)

Question: statctrl's gentle morning ramp (+0.1 K/3 min, riding near min
power for its COP) vs starting later at higher power (less time
accumulating envelope losses). Strategies scored against the identical
comfort requirement (full target by the recorded ramp-end deadline) over
00:00-12:00; late = worst room's minutes past deadline.

Cold mornings (06-21 / 06-22 / 06-27), baseline efficiency curve:

| strategy | AM kWh | late min | notes |
|---|---|---|---|
| recorded (statctrl) | 7.3 / 8.6 / 8.4 | 27 / 29 / 18 | cheapest everywhere, but late |
| step150 (faithful blast) | 10.0 / 10.0 / 11.4 | 0 / 0 / 7 | +19-36% energy |
| step120 | 10.0 / 10.0 / 11.2 | 17 / 16 / 41 | +16-34%, still late |
| step60 | 8.6 / 9.1 / 9.5 | 56 / 78 / 79 | cheaper but badly late — capacity-bound |
| ramp0.05x90 | 7.8 / 8.8 / 9.1 | 0 / 31 / 2 | ~on-time for +0.4-0.75 kWh |

Mild/medium days (06-14, 06-20): same ordering, recorded cheapest, late
steps +40-70% AM energy.

Efficiency-curve bracket (the min-power-COP advantage is the weakly
identified part of the e(P) fit, r2=0.28): re-ran recorded vs
step120/step90 on the three cold days with e_per_kw rotated to 0.0 and
+0.04 (min-power advantage removed / reversed). The energy gap narrows
from ~+30% to +3-12% but NEVER flips — recorded stays cheapest under
every plausible curve.

Conclusions:

1. **The current gentle-ramp approach is validated** — late-and-hard
   never wins on energy, even with the COP curve rotated against min
   power. The physics: a hard warmup pays lower COP at high speed AND
   still takes 1.5-2.5 h on cold mornings (capacity-bound), so the
   avoided-losses window is much smaller than intuition suggests, while
   distribution overshoot during the blast wastes the rest.
2. **Ryan's cold-morning caveat is real and visible**: aggressive late
   starts (step60, ramp0.1x60) miss the deadline by 46-79 min on the
   coldest days — there is no capacity margin for defrost + deep warmup.
   (Late starts do save ~10 defrost min on 06-22 — less cold running —
   but nowhere near enough to matter.)
3. **The actionable tweak is the opposite of the hypothesis**: statctrl
   is slightly too gentle/late — the worst room lands 18-29 min past the
   schedule on cold mornings. A modestly faster ramp (~0.05 K/min
   starting ~90 min before deadline, vs the current ~0.033 K/min) buys
   punctuality for +0.4-0.75 kWh on cold days; on mild days it costs
   ~nothing. Alternatively keep the current rate but start it ~30 min
   earlier on forecast-cold mornings (statctrl adaptive-start territory —
   NOTE: Ryan trialled and disliked adaptive optimum start before; treat
   as a suggestion only).

Raw CSVs session-scratch (preheat_*.csv); tables above are the record.

## Noise robustness matrix (2026-07-05, after Ryan's pushback)

Ryan (who hand-tuned the current gains) challenged the "noise is
irrelevant" call — correctly: the derivative path (5-min WMA slope x10
horizon), the integer rounding of the transmitted follow-me report, and
on/off/emission threshold crossings all amplify noise nonlinearly with
gain, and the 1-min archive can't see the 10 s band the controller
samples. Synthetic AR(1) controller-read noise (tau 60 s, per-day seeds)
now injectable via `analysis/tune.py --noise-sigma` (replay_day.replay
ctrl_noise). Matrix over the 6-day set:

| candidate | sigma K | deg_min_below | overshoot | osc/h | starts | kWh |
|---|---|---|---|---|---|---|
| baseline  | 0 / .006 / .02 / .05 | 12.9 / 12.2 / 10.8 / 9.4 | 1.19 / 1.19 / 1.16 / 1.14 | 1.43 / 1.42 / 1.36 / 1.44 | 5.5 / 5.5 / 6 / 7 | 7.02 / 7.01 / 7.04 / 7.01 |
| dbki_x4   | 0 / .006 / .02 / .05 | 7.9 / 8.9 / 6.9 / 7.5 | 0.98 / 0.97 / 0.94 / 0.95 | 1.41 / 1.41 / 1.34 / **1.75** | 6.5 / 6.5 / 6.5 / 7.5 | 7.02 / 7.02 / 7.06 / 7.16 |
| combo_xdr | 0 / .006 / .02 / .05 | 10.7 / 10.5 / 9.1 / 6.2 | 0.84 / 0.84 / 0.80 / 0.88 | 1.52 / 1.51 / 1.58 / **1.70** | 6 / 6 / 7 / 7.5 | 7.00 / 6.98 / 7.05 / 7.06 |

Findings:

1. At the measured archive floor (0.006 K) noise changes nothing for any
   candidate.
2. Ryan's mechanism is real and visible: at 0.05 K (~8x the floor) the
   high-gain candidates' oscillation jumps +20-25% (1.41->1.75,
   1.52->1.70) while baseline's stays flat (1.43->1.44) — the hand-tuned
   0.0125 deadband_ki is indeed implicitly noise-hardened, and gain
   increases do buy noise sensitivity. Starts +1-1.5/day at 0.05 for all.
3. The ranking never flips: the recommended configs beat baseline on
   every comfort metric at every noise level tested (e.g. combo overshoot
   0.88 vs baseline 1.14 at sigma 0.05).
4. Curiosity: mild noise slightly *improves* several metrics for all
   candidates (deg_min_below falls monotonically for baseline) — dither
   through the integer report rounding.

Net: recommendation unchanged at plausible noise (<= ~0.02 K), with a
sharpened caveat — if the real 10 s-band noise is much above ~0.03 K, the
x4 gain starts paying an oscillation tax. The staged rollout (x2 first,
watch recorded oscillation) is the right control for exactly this
unknown. Sub-minute noise could be measured directly by logging one
room's raw sensor at 10 s for a day before deploying.

## Sub-minute noise band measured directly (2026-07-05)

Ryan asked whether recent live HA data could characterise the noise; it
turned out the monthly raw export already carries it — HA's InfluxDB
integration records every state change, and data/raw/ keeps the
un-resampled stream (~8,900 changes/day per room sensor, median ~8 s
spacing = the native cadence the controller samples; HA recorder would
return the identical series).

Controller-eye measurement (10 s ffill grid, 5-min detrend, quiet HVAC-off
nights, 6 days): sigma_10s = 0.010 (kitchen) to 0.014 K (bed_1/2),
lag-1 rho 0.1-0.4 (correlation time ~10-20 s), decorrelated by 1 min.
About 2x the 1-min-archive floor (resampling had averaged it down), and
2-3x BELOW the ~0.03 K level where the matrix shows high gains paying an
oscillation tax.

Confirmation run at matched parameters (sigma 0.014, tau 15 s, 6-day set):

| | baseline | combo_xdr |
|---|---|---|
| overshoot | 1.16 (noiseless 1.19) | 0.84 (0.84) |
| rise med | 114 (116) | 105 (106) |
| deg_min_below | 9.7 (12.9) | 9.4 (10.7) |
| osc/h | 1.36 (1.43) | 1.57 (1.52) |
| kWh / starts | 7.02 / 6 (7.02 / 5.5) | 7.09 / 7 (7.00 / 6) |

At real measured noise the recommendation's margins are intact (combo
osc +3%, +1 start/day; every comfort advantage preserved). The
sub-minute-band unknown is CLOSED with data: staged rollout is now
ordinary prudence, not a necessary control.

## Bulk-state estimator — oracle A/B (2026-07-05, ideas.md #1)

The room sensors read the fast measured-air lead node `Tm`, not the bulk
mass `T` (task 009): while heating `Tm = T + lead·q` (over-reads
progress), and after a stop `Tm` sags ~2 K/h toward `T` for ~10 min.
Idea #1: control on estimated bulk `T` instead of the raw sensor to cut
cycling and premature satisfaction.

Test is an **oracle upper bound**: feed the controller the true bulk `T`
(a perfect de-leaded estimator), while comfort is *still* scored on
`Tm + offset` (the experienced/sensor signal) in both arms — so this
bounds the best achievable payoff. Only what `_write_room_temps` presents
to actrl changes; plant/HVAC/scoring untouched. 5 days (06-09 mild, 06-21,
06-22 cold, 06-27 warmup, 06-30 overcast). `scratchpad`-style driver kept
at `analysis/oracle_bulk_estimator.py`. Medians:

| metric | baseline | oracle | delta |
|---|---|---|---|
| time_in_band | .954 | .984 | +.030 |
| deg_min_below | 19.6 | 8.3 | **−58%** |
| deg_min_above / overshoot_max | 0.0 | 0.0 | 0 (never breaches high band) |
| overshoot_rise_max | 1.11 | 1.62 | +0.52 (within band) |
| osc_per_h | 1.45 | 1.00 | −31% |
| starts | 6 | 5 | −1 (−2 on cold days) |
| abovemin_frac | .158 | .113 | −29% |
| rise_time_med | 117 | 110 | −7 min |
| energy_kwh | 9.46 | 10.28 | +9% |

Reading — the idea is directionally right but the framing needs
correcting. It is **not** "cut cycling AND overshoot for free":

- The real, large win is **cold-time**: `deg_min_below` −58%, concentrated
  in the high-load rooms (06-22: bed_1 55→33, kitchen 70→48 K·min; the
  three small rooms were already ~0). The baseline was *chronically
  under-heating* — the in-run lead faked "arrived" and the controller cut
  early. Above-band discomfort (`deg_min_above`, `overshoot_max`) is 0 in
  BOTH arms, so the +0.5 K `overshoot_rise` is just warmer settling inside
  the wide heat_cool band, not discomfort. In heating season the overshoot
  worry is a non-issue.
- **Cycling does improve**: −1–2 starts/day, −31% oscillation, −29% time
  above min power (→ better COP per delivered kWh).
- The **+9% energy is real, previously-missing heat**, not inefficiency.
  Attribution on 06-22: the extra energy lands in the morning warmup
  window (00–12) while *midday* energy drops (12–18: 0.66→0.46 kWh) as
  cycling falls. So it's a comfort/energy **rebalance** (less cold, fewer
  cycles, more kWh), not a Pareto free lunch. A fair equal-comfort
  comparison would drop the oracle-arm targets to spend the saved cold-time
  back as energy — untested, the obvious next step.

Realizability caveat (important): the literal "invert the lead model"
(`T_est = Tm − lead·q`) fixes the *in-run* lead but NOT the *post-stop
sag* — once the compressor cuts, `q→0` so the subtraction is inert and
`T_est = Tm`, which still sags. Recovering the full oracle needs a
*dynamic* observer, `T_est = Tm + tau_meas·dṪm − lead·q`, i.e. it must use
the measured derivative (actrl already computes temp derivatives) plus a
`q` estimate from compressor-speed + damper state. Whether an observer
built only from controller-observable `q` proxies captures the oracle
benefit is the key open question before any deploy. (Answered YES —
next section.)

## Bulk-state estimator — realizable observer + equal-comfort (2026-07-05, ideas.md #1)

Both open questions from the oracle A/B above are now answered, same 5
days, same A/B channel.

**(a) Equal-comfort re-run** (`analysis/equal_comfort_bulk.py`): the
oracle arm additionally reads every room `delta` K warm (equivalent to
lowering its targets by delta; scoring bands untouched), swept
delta ∈ {0.1..0.4}. At **delta = +0.2 K the oracle returns to baseline
energy** (35.86 vs 35.14 kWh summed, +2%) with **~equal comfort**
(deg_min_below sum 72.4 vs 68.3 K·min — mild days give back a few
K·min while 06-21 improves; medians alone said −22%, a median artifact)
and **strictly better cycling** (starts 21 vs 29, osc −24%). Corrected
framing: at matched energy *and* comfort, the estimator's free benefit
is the **cycling reduction** (compressor wear + COP-per-kWh), not
comfort. The comfort win is bought with energy, and the trim delta is
the dial. (+0.3 is past the knee: comfort degrades with no energy gain.)

**(b) Realizable observer** (`analysis/observer_bulk_estimator.py`):
`T_est = Tm + tau_meas·dṪm − lead·q̂` from controller-observable signals
only — Tm quantized to 0.01 K at the 10 s cycle, derivative from a
first-order-filtered finite difference (tau 120 s; 60/240 both worse,
sensitivity mild), q̂ from the *previous* cycle's electrical power
(production reads `power.outdoor_unit`) through the fitted COP model and
commanded damper shares, with **no knowledge of the q-lag or defrost**.
tau_meas/lead are the fitted calibration constants.

5-day results (sums; medians in the scripts' output):

| arm | deg_min_below K·min | starts | osc sum | kWh | ΔE |
|---|---|---|---|---|---|
| baseline | 68.3 | 29 | 6.65 | 35.14 | — |
| oracle (true bulk T) | 36.1 | 24 | 5.00 | 38.43 | +9.4% |
| observer | 36.2 | 20 | 4.68 | 42.45 | +20.8% |
| **observer +0.2 K trim** | **37.8** | **16** | **4.59** | **39.18** | **+11.5%** |

- The raw observer **fully recovers the oracle's comfort** (36.2 vs 36.1
  K·min) and beats it on cycling, but runs ~+11% hotter than the oracle:
  a systematic *cold* bias (filtered derivative lags warmup ramps;
  defrost-blind q̂ infers heat that was never delivered) makes it heat
  more.
- The bias is trimmable with the same warm offset from (a): **observer
  +0.2 K matches oracle comfort at +2% more energy than the oracle, with
  the fewest starts of any arm (16 vs baseline 29, −45%)**. It also
  dominates the raw observer outright.
- So the answer to the oracle's key open question is **yes**: an observer
  built only from what production actrl can see captures the full
  benefit. The estimator idea survives realizability.

Deploy considerations, in order:

1. The comfort/energy split is a policy dial (the trim / effective
   targets), not a property of the observer — pick the point at deploy
   time; equal-energy gets the cycling win for free, +11% energy buys
   cold-time −45%.
2. Residual +2% vs oracle is mostly the defrost-blind q̂; production
   could detect defrost from the power signature (~1 kW draw, zero heat)
   and zero q̂ during it. Untested refinement.
3. The observer needs per-room `tau_meas`/`lead` constants in actrl and
   a q̂ path (power sensor + damper shares + COP curve). The sim's
   fitted values transfer, but sim-fitted lead constants are loop
   thresholds (sim/house.py) — a staged rollout with the controller CI
   gate (analysis/controller_ci.py) plus a day of shadow-mode logging
   (log T_est alongside Tm, act on Tm) is the prudent first step.

## Widened off-thresholds vs the observer (2026-07-05, Ryan's question)

`eventual_off_threshold` −0.5→−1.0 (one line, no structural change) is a
poor-man's observer on the OFF side: the sensor's "we've overshot" is
mostly the lead-node exaggeration, so tolerating a bigger apparent
excursion keeps delivering heat the room actually needed and stretches
each run. 5-day sums vs the same baseline/observer arms:

| arm | deg_min_below K·min | starts | osc sum | kWh |
|---|---|---|---|---|
| baseline | 68.3 | 29 | 6.65 | 35.14 |
| eventual_off −1.0 | 61.7 | 20 | 4.70 | 35.12 |
| observer +0.2 K trim | 37.8 | 16 | 4.59 | 39.18 |

- The threshold widening captures **most of the cycling win** (starts
  −31%, osc −29%) at exactly baseline energy — because run-length, not
  restart timing, dominates starts/day. But it captures only ~10% of the
  comfort win: it does nothing in-run (the controller still believes the
  inflated reading while modulating, so warmups still cut early) and
  nothing on the restart side (post-stop sag still fires starts early —
  runs are longer but the gap between them is still fake-triggered).
- −0.75 and a wider immediate_off (−2.5) sit on the same front:
  immediate_off −2.5 trims osc further (0.88 median) but its risk case
  is a real (not fake) overshoot excursion, e.g. solar gain after a long
  run — the fixed threshold can't tell the difference. That is the
  structural limit of the approach: one constant approximating a
  correction that actually varies with compressor power, dampers and
  time-since-stop.
- NOT additive with the observer: once the controller reads de-leaded
  temps, "past target" is real overshoot and the thresholds should stay
  tight. Widening is the cheap interim step; the observer supersedes it.

Ceiling-fan note (Ryan, 2026-07-05): all rooms EXCEPT study run ceiling
fans on low reverse via HA automation once the compressor starts. The
fitted tau_meas/lead constants were calibrated from June data with that
automation active, so fan mixing is baked into them (it is plausibly WHY
the sensor pocket couples as fast as it does). Consequences: (a) the
study's constants are the least trustworthy twice over — no fan AND its
lead was a pooled bedroom prior (no clean fit events); (b) changing the
fan automation (speed, delay, which rooms) invalidates the lead fit —
refit before trusting sim conclusions after any such change.

## Room/damper PID gains — divergent-target sweep (2026-07-05, tasks #4/#5)

The summer-override proxy (`analysis/divergent_targets.py`): winter replay
day, all five zones' targets pinned to (own temp + 0.5 K) from 16:00 —
sub-K contention the winter schedule never produces — then at 19:00 bed_2
steps +1 K (wants more) and bed_3 steps −2 K (satisfied). Gains swept with
the standard texture config (kitchen 2.6, noise 0.012 K/15 s). Three
evenings (06-21/22/27) + a second seed:

- **Current gains (kp 1.0, ki 0.001) handle K-scale override events
  well**: the wants-more zone is served in 0–7 running minutes and the
  satisfied zone sheds −76..−79 damper points, consistently across days
  and seeds. kp alone covers K-scale steps; the years-old detune did not
  break the override use case.
- **Raising gains buys little and hunts**: ki x4 pushes quiet-period
  damper movement to ~2.5x the recorded envelope (0.99–1.06 vs 0.4–0.5
  mv/h — the remembered hand-tuning hunting, reproduced in sim); kp x2 +
  ki x4 gives the best tail tracking (err_all 0.31–0.35 vs 0.35–0.53)
  and strongest sheds but still idles hot on 2 of 3 days (pre_mv 1.65,
  2.40); ki x2 is inconsistent day-to-day (06-21: err_all WORSE, 0.66).
  No robust win anywhere. **Recommendation: leave room_kp/room_ki
  alone.** The sim confirms the hand-tuned point sits at about the real
  hunting margin — with sensor noise modelled, there is no free
  rebalance speed on the table.
- **The actual structural limit is the minimum-airflow inflation loop**
  (docs/calibration.md rebalance probes): sub-K sheds are blocked
  gain-independently because every shed attempt that would violate
  minimum airflow re-inflates ALL integrals (kitchen first, largest
  airflow weight). K-scale sheds punch through via kp; sub-K ones never
  do. If summer sluggishness persists in practice, the lever is that
  loop's design — e.g. inflating only zones with positive demand, or
  excluding deeply-satisfied zones — a control-logic change (Ryan's
  call), not a tuning constant. Sim-testable with this same scenario
  before any deploy, plus the controller CI gate.

Caveats: winter physics (heating), one scenario shape, 10 s replay
cadence; cooling-season dynamics unvalidated until the summer archive
accumulates. The scenario driver is reusable as-is for cooling once
summer data exists.
