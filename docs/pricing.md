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
