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
