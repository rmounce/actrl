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
