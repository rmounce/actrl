# Price-aware HVAC scheduling (ideas.md #6)

Study docs. Data plumbing + Phase A findings. Caveman style.

## Data plumbing (analysis/price_cost.py)

- SA1 30-min actual prices: InfluxDB `rp_30m.aemo_dispatch_sa1_30m`
  field `price` ($/MWh), Dec 2021 -> now. Cached to
  `data/prices_sa1.parquet` (gitignored).
- Wholesale -> retail: `~/src/ai-energy-forecast-slop` `tariff_utils.py`
  + `tariff_profile.json` (network loss factor, ToU network tariff --
  peak 17:00-20:59 ~40 c/kWh adder, solar-sponge 10:00-15:59 ~7 c,
  off-peak ~15 c -- GST 1.1 on import leg). pytz added as dev dep.
- Recorded HVAC power = Shelly `power.outdoor_unit + power.indoor_unit`.
- Accounting: all-import (no PV self-consumption credit yet -- absolute
  costs slightly overstated, schedule comparisons fair). Refinement
  needs house net-grid flow.

## Phase A: hindsight value bound (2026-07-06)

June recorded cost: **$90.09 for 118.5 kWh (76 c/kWh volume-weighted vs
33 c time-average import price)**. Decomposition:

- **$51.92 (58%) was ONE day, 2026-06-22**: the 06:00-08:00 cold-morning
  warmup (3.8 kWh at max compressor) collided with a wholesale spike
  ($6.94/kWh avg 06:00-07:00, $16.11 07:00-08:00; day max $20.3k/MWh).
  02:00-05:00 power that morning cost $0.59-0.66/kWh.
- Rest of month: $38.17 / 106.9 kWh (~36 c/kWh). Physics-free
  oracle-shift bound (same daily kWh into cheapest half-hours, capped at
  max draw): $20 -> **ordinary-day ToU ceiling ~= $18/month**, capture
  fraction TBD.
- Peak window (17-21) is only 7% of cost / 8% of energy -- evening
  heating exposure is smaller than expected (night setback starts early);
  the money is in cold-morning spikes, not the daily network peak.

### Spike-day strategy sweep (analysis/preheat_price.py, full sim, real
prices, comfort scored vs ORIGINAL recorded targets)

Strategy: raise all rooms' targets to (day-level + bank_k) from
bank_start until spike start (banking in cheap hours), then recorded
targets minus shave_k through the spike (coast on the banked heat).
Spike window auto-detected: half-hours > 3x day median price.

06-22 (sim baseline $67.60, 11.2 kWh, deg_min_below 22.9):

| arm | cost | kWh | deg_min_below |
|---|---|---|---|
| bank 1.0K @ 02:30, coast | $42.90 (-37%) | 14.0 | 6.0 (better) |
| bank 1.0K @ 02:30, shave 0.5K | **$24.43 (-64%)** | 14.0 | 13.6 (better) |
| bank 1.0K @ 04:00, coast | $70.36 (WORSE) | 13.7 | 5.8 |

- **-$43 on the one day, with comfort BETTER than baseline** (early
  warm-up also fixes the chronically-late cold-morning rise, cf.
  docs/tuning.md preheat study). Extra energy (+2.8 kWh) is already
  priced into the cost figures.
- **Lead time is the critical variable**: the same 1K bank started at
  04:00 ramps INTO the spike and loses money. A static "always heat at
  2am" rule is wrong on spike-free days (wasted kWh) and a late trigger
  is worse than nothing -> this must be forecast-driven (Phase B).
- Spike-free days (06-24, 06-09): auto-detector finds no window ->
  strategy is a construction-level no-op, all arms bit-identical to
  baseline. Safe by default.

### Phase A verdict

Value pool is real and concentrated: ~$40-50/event on spike mornings
(SA cold snaps; June had one, expect several/winter) + ~<=$18/month
ordinary ToU shaping. GO for Phase B: replay 06-22 with VINTAGE
predispatch forecasts (rp_30m.aemo_predispatch_forecast, 239k June rows)
to establish how far ahead the spike was knowable and what lead time a
live planner would actually have had; then the ordinary-day ToU capture
fraction; then distillation into statctrl rules (ideas.md #6 Phase C).

## Phase B: forecast realism (2026-07-06, analysis/spike_forecast.py)

**Magnitude is unforecastable; elevation is actionable.** Predispatch
vintages for the 06-22 05:30-08:00 window never exceeded $500/MWh
forecast (at the 02:00 bank decision: $266-285) against $20,300 actual --
extreme spikes come from real-time events, not day-ahead forecasts. But
$266+ is 2-3x a normal morning, so a LOW threshold works as an
insurance trigger.

Two-winter trigger evaluation (454 mornings, 2025-03-23 ->; decision =
latest predispatch run at/before 02:00 local, window = 05:00-09:00 local
max rrp; "real spike" = actual max > $1000/MWh):

- 5 real spike mornings (4 winter: 2025-06-26, 2025-07-02, 2025-08-10,
  2026-06-22; 1 summer: 2026-01-27 -- cooling-season analog, future).
- **$300/MWh trigger: catches 3/5, fires 37 times (~2.3/month)**. The 2
  misses (fc $130/$170) were forecast fully normal -- unreachable by any
  predispatch threshold.
- False-alarm cost measured in the full sim (June fire-mornings 06-23,
  06-24 with the bank forced): **+$0.22 to +$0.92 per event, with
  discomfort 10.7->0.0 and 18.4->3.1 K.min** -- a false fire is a
  sub-dollar comfort upgrade, not a loss. (Both June "false alarms" were
  actually elevated mornings -- predispatch elevation correlates with
  real elevation even when magnitude is hopeless.)
- Net over 2 winters at $300: ~3 hits x $25-45 saved vs 34 false fires x
  <$1 -- clearly positive, plus better comfort on every fired morning.

**The unforecastable misses are partially recoverable with a no-lead
shave**: Phase A showed the 0.5K shave contributes ~$18 of the 06-22
$43 saving, and shaving needs no lead time -- a real-time price trigger
(Amber current price > ~$1/kWh) can fire mid-spike on mornings the
forecast missed.

### Phase C design (proposed)

Two independent statctrl-level triggers, both sim-tested:

1. **Bank** (forecast, needs lead): at ~02:00, if predispatch max rrp
   over 05:00-09:00 > $300/MWh -> targets to day-level +1K from 02:30,
   revert at 05:00. Expected ~2-3 fires/month in winter, <$1 each,
   comfort-positive; catches ~60-75% of real spikes for $25-45 each.
   Consider a 04:00 re-check for late-appearing forecasts (untested).
2. **Shave** (real-time, no lead): while current import price >
   ~$1/kWh during 05:00-09:00 (or any time), targets -0.5K. Catches the
   unforecastable spikes partially; no-op otherwise.

Value: ~$60-100/winter at current spike frequency + comfort improvement
on trigger mornings. Implementation: statctrl reads the predispatch max
(via a HA template sensor or the published ai_price_forecast series) +
the live Amber price entity. Ordinary-day ToU shaping (<= $18/mo
ceiling) deliberately NOT in scope for the first deploy -- separate,
lower-value lever.

## Phase C progress (2026-07-06, continuous formulations)

Ryan's steer: no hard-coded thresholds. Results of moving both Phase B
triggers to continuous rules:

### Component 1 -- continuous price offset (VALIDATED, deployable candidate)

`analysis/price_offset.py`: offset(t) = k x (retention-discounted max of
the calibrated price forecast over the next 8h - price now), clamped
[-0.75, +1.5] K, added to every room's target. k [K per $/kWh] is the
one preference knob (comfort-vs-money exchange rate); tau ~= 20h is
physics (holding a banked degree leaks at ~UA/C_eff ~= 5%/h).

- Elevated days: -14% cost at comfort parity (06-24 $6.73 -> $5.81).
- Mild days: neutral at k=2 (offset ~0 when prices are flat).
- Spike shave: engages on ACTUAL price with zero lead -- catches even
  unforecast spikes' second half.
- What it CANNOT do: the warmup time-shift. An additive +-1K offset on
  the 16C night setback still leaves rooms 3K below day level when a
  morning spike lands -- the mandatory warmup energy still gets bought
  at spike prices. The spike-day result is poor (-$11 with worse
  comfort vs the discrete experiment's -$43 with better).

### Component 2 -- price-aware warmup start (BLOCKED on an honest fork)

`analysis/warmup_shift.py`: pick the warmup start time by expected cost
(coarse RC model x calibrated price forecast, decided at 00:30 from the
vintage predispatch; full-sim validated). Two findings block it:

1. **The tail is unlearnable as a conditional mean.**
   `analysis/fc_calibration.py` fits E[actual | forecast]: pooled AND
   winter-morning-conditioned fits show ~no uplift (the $0.40-0.55/kWh
   forecast bin fits to ~$0.40-0.46... while the SAME bin realised
   $2.60 mean in the June holdout, because 06-22's $16/kWh sat there).
   2-3 spike events per winter cannot pin E[act|fc]; whichever period
   you fit, the other period's spike is in the holdout. A pure
   expected-cost planner therefore (correctly, per its inputs) declines
   to shift on 06-22.
2. Coarse hold-cost is ~2x low vs the full sim (UA/C_eff maintenance
   underestimates real cycling losses) -- fixable by calibrating one
   constant against sim replays, but moot until (1) is decided.

### The fork (Ryan's call -- this is a risk preference, not a fit)

- **Expected-value planner**: with honest tail estimates it rarely
  banks; you eat a ~$50 morning 1-2x/winter. Optimal iff you only care
  about the mean.
- **Insurance planner**: value forecast prices with an upside-surprise
  term, e.g. g(fc) = fc + lambda x E[(actual - fc)+ | fc] (the expected
  positive surprise -- statistically much stabler than the mean because
  it pools all upside residuals). lambda is the single knob: 0 = pure
  EV, 1 ~= actuarially fair, >1 = risk-averse. Phase B's discrete
  trigger was implicitly lambda>>1 and demonstrably paid: ~$0.5 +
  BETTER comfort per false fire (~2-3/month), ~$40 saved per hit
  (1-2/winter). The asymmetry makes some lambda>0 clearly right; HOW
  MUCH is preference.

Next session: pick lambda form, refit the upside-surprise curve,
calibrate the coarse hold cost against sim, sweep lambda on 06-22 +
2025 winter spike days (coarse decision only -- house archive doesn't
cover 2025) + false-fire days, June-wide neutrality check, then the
statctrl implementation spec.

## Lambda sweep result (2026-07-06): the knob is (nearly) moot

Fixing the coarse model first changed the answer. The hold-cost bug: an
early start was being charged UA x (T_house - T_outdoor) -- the ABSOLUTE
envelope loss -- but the baseline heats the house from the deadline
anyway, so the true marginal cost is holding the ~1.5-2K INCREMENT above
the counterfactual free-floating house (~10x less, the same insight as
the offset's tau=20h). With that fixed (+ upside-surprise knots fitted
on ALL winter data -- insurance-premium mode, leave-one-event-out noted
below), the warmup-shift planner fires from FACE-VALUE forecast
economics alone:

| day | baseline | shifted | comfort | decision (all lam 0-4 identical) |
|---|---|---|---|---|
| 06-22 spike | $67.60 | **$42.99 (-36%)** | 22.9 -> 13.3 K.min (better) | start 02:00 (jit 04:30) |
| 06-23 | $4.33 | $4.39 (+$0.06) | 10.7 -> 0.1 (near-perfect) | 0.5h early |
| 06-24 elevated | $6.73 | $7.55 (+$0.82) | 18.4 -> 11.4 (better) | 2h early, elevated all morning: no spread to harvest |
| 06-09 mild | $0.29 | $0.36 (+$0.07) | 2.1 -> 0.0 (perfect) | 1.5h early, marginal wrong call |

2025 coarse-only decisions (no house archive): 07-02 (the forecastable
spike) shifts 3.5h at lam=0; 08-10 (forecast dead-normal) never fires --
correctly unreachable, that's the realtime shave's job; lam changes only
one borderline morning (07-15 at lam>=2).

**Verdict**: lam 0-4 produces IDENTICAL June decisions -- the honest
marginal hold cost (~$0.05-0.10/h) is so small that face-value forecast
spreads already justify shifting; the upside term only nudges borderline
mornings. Keep lam=1 (actuarially fair) as the default -- it costs
nothing and covers the borderline cases. The wrong-call tax on
non-spike mornings is $0.06-0.82/event, ALWAYS with better comfort
(same insurance profile as Phase B). Net over the 4 sim days: -$23.7.

Caveats: upside knots fitted on all 5 events (insurance-premium mode --
a leave-one-out fit under-prices whichever spike it hasn't seen; with
n=5 events this is climatology, not validated prediction); hold_mult=2
calibrated against a single sim point (06-09); 06-24-style
elevated-all-morning days are systematic ~$1 wrong-calls the coarse
model could avoid by comparing against the elevated warm-window price
rather than assuming cheap pre-dawn (refinement).

## Deployment picture after Phase C sim work

1. actrl: continuous price offset (k=2, tau=20, clamps) -- ToU banking +
   zero-lead spike shave. VALIDATED.
2. statctrl: warmup-start chooser (coarse marginal-cost model, 00:30
   decision from predispatch via g_lam, lam=1) -- the spike-morning
   money. VALIDATED on 06-22 (-36% with better comfort) + 2025 coarse
   decisions sane.
3. Data plumbing: predispatch forecast + live Amber price into HA/
   AppDaemon; fc_calibration knots as constants; refresh monthly-ish.
Implementation = production control-logic changes, Ryan review + CI
gate + staged deploy as usual.

## Production implementation (2026-07-07, pending Ryan review — NOT deployed)

Both validated Phase C components landed in production code, feature-off
by default (all goldens + controller CI bit-exact with the gates off).

**Pure logic — control.py** (stdlib only, unit-tested in
tests/test_price.py): `price_pressure_offset` (k=2, tau=20 h, horizon
8 h, clamps −0.75/+1.5 K, forecasts valued at E[actual|fc]);
`warmup_expected_costs` / `warmup_decide` (line-for-line port of
analysis/warmup_shift.py coarse model, times re-based to "hours from
now"; exact-parity test against the analysis implementation on a frozen
synthetic-morning fixture); fc calibration knots as constants (winter-all
fit to 2026-07-06 — REFRESH monthly-ish via analysis/fc_calibration.py
and re-paste); HVAC/house constants (UA 0.160, C_eff 4.8, Pmax 3.055,
efficiency fit ×0.80) copied from sim/hvac.py + docs/calibration.md.

**actrl.py**: `_get_price_pressure` + `_apply_price_pressure` — see
docs/actrl.md step 5b. Gate: `input_boolean.ac_use_price_pressure`.
Runs after `_calculate_demand` so grid-surplus integral bookkeeping never
sees price demand; bank capped per room at the grid-surplus target bounds
net of applied surplus offset.

**statctrl.py**: `get_price_shift` + effective-start plumbing in
`update_setpoint` — see docs/statctrl.md "Price-aware warmup start".
Gate: `input_boolean.statctrl_price_aware`.

**HA-side setup needed to go live** (Ryan):
1. Create `input_boolean.ac_use_price_pressure` and
   `input_boolean.statctrl_price_aware` helpers (leave off for staged
   rollout — code deploys inert).
2. Verify the price entities exist and update:
   `sensor.amber_5min_current_general_price` (live retail import $/kWh),
   `sensor.dh_unit_load_cost` (EMHASS-published, attr
   `unit_load_cost_forecasts`, records `date` + `dh_unit_load_cost` —
   already tariffed and follows whichever price source EMHASS is
   configured with, the same feed the HWC planner consumes; Ryan's call
   2026-07-07, replacing the shelved PD-direct sensor),
   `sensor.temperature_adelaide`. Knots REFIT against the APF's own
   forecast log 2026-07-07 (fc_calibration.py --apf-log, 4636 pairs
   2025-07-20..2026-07-06 months 5-8, out/fc_calibration_apf_winter_all
   .json → control.py constants): the calibration now matches the
   forecast family production consumes. Behaviour difference vs the
   predispatch fit: the APF p50 never forecasts above ~$0.90 retail, so
   there is no fat top bin (predispatch's ≥$1.20 bin realised 2.93) —
   high APF forecasts mildly OVER-predict (0.80-1.20 bin realises 0.76)
   and spike value lives in the mid-bin upside terms (0.06-0.11 vs
   predispatch's 0.03-0.06). Net: the continuous offset banks less
   aggressively on forecast spikes (top-knot value ≈ +0.8 K vs clamp
   1.5 K), the warmup chooser's insurance term is fatter mid-range, and
   the zero-lead live shave is unaffected. Refit monthly-ish as the APF
   log accumulates (log starts 2025-07-20, so this fit misses May-early-
   Jul 2025 incl. the 2025-07-02 spike; coverage improves each month).
3. Enable one gate at a time; watch `input_number.aircon_price_pressure`
   and the statctrl "Price-aware warmup" log lines.

Caveats carried from the study: knots are climatology (n=5 spike
events); hold_mult=2 single-point; 06-24-style elevated-all-morning days
are ~$1 wrong-calls (refinement identified: compare vs warm-window price
instead of assuming cheap pre-dawn). New in production: warmup deadline
= schedule start (study used end of target ramp — slightly early,
comfort-safe); statctrl decides per room from house-level constants;
grid-surplus offset and price offset coexist (price bank capped net of
surplus) — the price offset likely subsumes grid_surplus long-term
(negative feed-in ⇒ cheap now ⇒ bank), candidate for later removal, NOT
removed unilaterally.

## APF re-validation (2026-07-07)

June arms re-run with APF vintages (price_offset.py / warmup_shift.py
--apf-log) + the APF knots -- the exact forecast family production now
consumes. Logs: analysis/out/{price_offset,warmup_shift}_apf.log.

Warmup chooser: 06-22 spike decision IDENTICAL to predispatch (start
2.0h, $67.60 -> $42.99, -36%, comfort better); 06-23's predispatch
0.5h wrong-call disappears (clean JIT); 06-24 same ~$0.82 comfort-
positive wrong-call (the known warm-window refinement case); 06-09 same
$0.07 insurance at perfect comfort. The spike money survives the
forecast-family change unchanged.

Offset: banking tamer as the knots predicted (off max +0.42/+0.45 K vs
+1.5 clamp under predispatch); savings hold -- 06-22 -$5.45 (-8%,
mostly the live shave during the $16/kWh hour, deliberate degmin cost),
06-24 -$0.72 (-11%, vs -14% predispatch), 06-09 ~flat. If the spike-
hour sag ever feels too deep the knob is shave_max, but the shave IS
the mechanism that recovers unforecastable spikes.

Verdict: no re-tuning needed for the APF feed; constants stand.

## Recorded-trace counterfactual, 06-22 (2026-07-07)

Ryan pushed back that the sim baseline ($67.60) costing more than the
recorded day ($51.91) undersells the strategy. Right lens: the strategy
value needs no simulator on this day -- from the RECORDED trace, $46.48
of the $51.91 was 3.78 kWh bought at avg $12.29/kWh inside 06:00-08:00;
the same energy at that morning's 02:00-04:30 price ($0.60/kWh, +15%
hold overhead) is $2.61 => counterfactual recorded day ~$8. The rec/sim
gap is morning cycling texture repriced across a 20x cliff (same kWh
+-4%); sim claims are arm-vs-arm only. The sim's role is de-risking the
decision rule (fires at 00:30 from real vintages, comfort improves,
wrong-calls cost cents), not predicting the dollar figure.
