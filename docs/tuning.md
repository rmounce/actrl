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
