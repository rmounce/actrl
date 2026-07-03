# Calibration findings

System-identification results from recorded data. Reproduce with
`analysis/sysid_june.py` (see its docstring). Numbers here are from the
2026-06-01..30 archive (winter, heating season) — re-fit as more months
accumulate, especially summer.

## Method (first pass, 2026-07-02)

- Data: `data/processed/june.parquet` via `calib.py` (1-min grid, see
  docs/data.md). Analysis at 10-min means.
- **Decay fits**: 24 free-running night windows (outdoor unit < 50 W,
  21:00–08:00 local, ≥ 3 h). OLS on dT/dt against driving temperature
  differences; dT/dt over a 30-min forward span.
- **Heating efficiency**: with the house envelope constant known, delivered
  heat per unit thermal mass during heating is
  `q = dT/dt + (T − T_out)/tau_house` [K/h]. The efficiency proxy
  `e = q / P_elec` [K/h per kW] has unknown absolute scale (needs C_house)
  but its *shape* vs compressor power and outdoor temp is directly
  measurable — and that shape is what capacity-control policy depends on.

## House thermal model

- **House-average envelope: tau ≈ 30 h** (r² = 0.73, n = 1033 10-min
  samples; residual gains ≈ 0.02 K/h ≈ nothing). Slow — the house holds
  heat well overnight (~0.2 °C/h at a 6 °C indoor-outdoor delta).
- Per-room two-node fits `dT/dt = a(T_out−T) + c(T_others−T) + g`:

  | room    | tau_out (h) | tau_coupling (h) | gains (K/h) | r²   |
  |---------|------------:|-----------------:|------------:|------|
  | bed_1   |        24.7 |              9.0 |       +0.04 | 0.55 |
  | bed_2   |        30.5 |             45.1 |       +0.00 | 0.71 |
  | bed_3   |        26.4 |              9.8 |       −0.04 | 0.59 |
  | kitchen |       118.6 |              2.9 |       +0.20 | 0.50 |
  | study   |        33.9 |             19.3 |       +0.02 | 0.42 |

- **Kitchen is an open zone**: essentially no direct outdoor path
  (tau_out ≈ 120 h ≈ unidentifiable), equilibrates with the rest of the
  house in ~3 h, plus ~0.2 K/h steady gains (fridge/appliances). A
  single-node kitchen-vs-outdoor model is *wrong* — the naive fit gives
  tau ≈ 3 h with +2.8 K/h phantom gains.
- bed_2 is the most isolated room (weakest coupling, 45 h) — consistent
  with a usually-closed door. bed_1/bed_3 couple to the house at ~9–10 h.
- Adding the coupling term eliminated bed_1's apparent +0.32 K/h night
  gains (occupancy heat is not separately identifiable from coupling at
  this resolution).
- 1R1C per room + inter-room coupling looks sufficient at 10-min
  resolution; no evidence yet that a second (envelope) node is needed for
  control-relevant timescales. Revisit with summer data.

## Heating efficiency shape (the control-relevant result)

Joint fit `e = e0 + b·(P_kW − 0.7) + c·(T_out − 10)`, steady heating
(P > 300 W for 30 min, transients and defrost-ish outliers trimmed):

- All heating samples (n = 590): min→max power efficiency loss **8%**,
  outdoor-temp effect **+5.5%/K**.
- No-sun samples only (n = 421): min→max loss ~14%, outdoor effect
  +5.0%/K (daytime solar gain otherwise gets credited to the AC,
  flattering high-power daytime runs).
- **No-sun + defrost-excised (n = 380, the honest number): min→max loss
  ~17%, outdoor effect +5.0%/K.** Excising defrost barely moves the
  coefficients — the conclusion was already robust to it.

Interpretation, with caveats below:

- The **naive binned curve shows a ~35% efficiency drop** from min to max
  power — but that's mostly confounding: high power happens on cold
  nights. At *fixed* outdoor temp the compressor-speed penalty is a much
  smaller ~17%.
- Outdoor temperature dominates: ~5%/K means a 6 °C outdoor swing moves
  efficiency ~30% — more than the entire compressor-speed range.
- Policy implication (preliminary): "run at minimum power as long as
  possible" is directionally right but worth ~17%, not ~35%. Conversely,
  *when* you heat matters more than *how hard*: heating during the warm
  part of the day (which also tends to be PV-surplus time) beats gentle
  overnight running on both counts. Supports the grid-surplus strategy;
  quantify properly once C_house is pinned down.

## Defrost (detected 2026-07-02)

- Detection: actrl forces `aircon_comp_speed` to max on defrost
  (compressor on + outdoor fan off), so **comp_speed ≥ 15 while the
  outdoor unit draws > 300 W** labels defrost + recovery. Nine June
  episodes, all 03:30–07:00 local at outdoor 3.9–7.1 °C, corroborated by
  `m5atom_inside_coil_inlet_temp` crashing from its normal ~35 °C to
  1–12 °C during each one. (The outside-coil-excursion idea found
  nothing — that sensor never rose > 10 K above ambient; use the inside
  coil / comp-speed signals instead.)
- Overhead: defrost + recovery windows consumed **7.5 kWh of June's
  116.9 kWh heating (6.4%)**. Concentrated where it hurts: cold pre-dawn
  heating (03:00–08:00 local, T_out < 7.5 °C) was 29% of June's heating
  energy and paid **23% of itself into defrost windows** — on top of its
  ~25% cold-outdoor efficiency handicap. The case for shifting heating
  load away from pre-dawn toward warm/PV hours is now quantified from
  three directions at once.

## Time delays (2026-07-03)

How fast does the unit follow a compressor-increment command?
`analysis/lag_fit.py` finds clean steps in the controller's speed estimate
(constant ≥4 min before, ≥8 min after) and fits the Shelly outdoor-power
response (13 s cadence). Superposed-epoch median over 84 clean ±1 running
steps in June:

- 67% of the power step within 10 s, ~90% by 60 s, fully settled by ~3 min
  (a small slow tail after the fast jump).
- Best single first-order fit: **tau ≈ 20 s** — modelled as
  `HvacParams.lag_tau_s` applied to electrical power in the closed loop.
- Shutdowns are instant cuts in the raw traces (kW → ~0 within one or two
  13 s samples), so the lag applies to running changes only; power-off
  resets it.

**Heat-delivery lag, modelled 2026-07-03** (Ryan's observation): a delay on
the order of **minutes** between a compressor-speed change (visible in
power) and the "elbow" in the measured room-temperature trajectory — most
observable near equilibrium during slow increment/decrement stepping.
`analysis/heat_lag_fit.py` takes the clean ±1 steps found by
`analysis/lag_fit.py`, superposes the room-temperature *derivative* (local
OLS slope, ~10 s cadence) around each step sign-flipped by step direction,
and fits `excess(t) = A(1 - exp(-(t-d)/tau))` (dead time `d` + first-order
lag `tau`) by grid search:

| room    |   n | dead time | tau   | plateau     | r²   |
|---------|----:|----------:|------:|------------:|-----:|
| bed_1   | 116 |      120s |   15s | +0.10 K/h   | 0.54 |
| bed_2   | 116 |      465s |  435s | −0.03 K/h   | 0.12 |
| bed_3   | 116 |        0s |  510s | +0.21 K/h   | 0.75 |
| kitchen | 116 |       15s |  180s | +0.65 K/h   | 0.95 |
| study   | 116 |      420s |   60s | +0.11 K/h   | 0.89 |

Kitchen (the open zone, best house-average proxy) gives by far the cleanest
fit (r²=0.95) — dead time ~15 s then tau ~180 s (3 min) to the new
delivered-heat rate. Used that as the lumped Q-delay, applied downstream of
the 20 s electrical power lag: `HvacParams.q_lag_dead_s = 15.0`,
`q_lag_tau_s = 180.0`, via `sim.hvac.DeadTimeLag`. This bundles
refrigerant/coil dynamics, duct transport, and the `*_average_temperature`
sensor averaging — they are not separately identifiable from this data, and
one lumped lag on Q is what the control loop actually experiences. Per-room
fits vary widely (bed_2 essentially flat/uncorrelated, r²=0.12) — that's
consistent with bed_2's already-known weak/uncertain heat-split share
(below), not evidence against the lumped model.

Whole-day replay impact (four June days, vs no-Q-lag baseline): negligible
change to per-room temperature RMSE/bias (the lag is downstream of Q, which
the RC fit already smooths further), and heavy-day energy match improves
slightly (22nd: −19%→−15%, 27th: −23%→−21%) since delayed heat delivery
keeps the compressor running a bit longer for the same demand.

Startup/defrost-recovery traces show a spin-up boost overshoot (~2.5 kW
transient before settling at min power) that the first-order model doesn't
reproduce — acceptable for energy/comfort questions, revisit if cycling
behaviour matters.

## Whole-day closed-loop replay (2026-07-03)

`analysis/replay_day.py` replays a recorded local day end-to-end: real
actrl drives the simulated unit/house while only the exogenous inputs
(BOM outdoor temp, statctrl room targets, unit setpoint) come from the
archive. Room temps and HVAC behaviour are fully simulated — no nudging
toward the record — so this validates the whole stack. Four June days
(08, 15, 22, 27 local):

- **Kitchen tracks within 0.34–0.40 °C RMSE, ~zero bias, on every day.**
  Kitchen is the open zone that dominates the house response, so the
  envelope + HVAC calibration is solid.
- Heavy heating days (22nd: 11.6 kWh, 27th: 9.7 kWh recorded): simulated
  energy −10/−11%, unit-on fraction matches within ~3 points. Missing
  defrost (~6% overhead) accounts for most of the energy deficit.
- Bedrooms/study run cold in the daytime (bias −1 to −2.2 °C, night bias
  −0.3 to −0.9 °C): **solar/internal gains are not modelled** (the RC fit
  is night-only). This is the dominant model gap.
- Mild days (8th, 15th) overpredict energy (+94%, +18% on 2.1/2.8 kWh
  bases): the un-sunlit simulated rooms call for heat the real house
  didn't need — same root cause, amplified by the small denominators.

### Solar-gain term (added 2026-07-03)

`analysis/solar_fit.py` regresses unit-off residuals (night RC params held
fixed) on the Solcast PV proxy `power_pv_5m`: bed_3 is the sunny room
(s = 0.133 K/h per kW, r² = 0.45), bed_1/study modest (0.031/0.042), bed_2
weak (0.029, r² = 0.03), kitchen clamped to 0 (raw fit slightly negative —
coupling-term cross-talk from sunlit bed_3, and kitchen already replays at
~0.4 °C). Now `RoomParams.solar`, driven by `pv_kw` through
`ClosedLoop.step`; zero PV reproduces the night fit exactly.

Replay after: bed_3 daytime RMSE improves ~25–45% (e.g. 2.41 → 1.36 °C on
the 8th), bed_1 and study similarly, mild-day energy error halves
(+94% → +41% on the 8th). Trade-off: heavy-day energy moves from −10/−11%
to −19/−23% — sunlit simulated rooms ask for less heat than the real
system used. Remaining known gaps, in likely order of value:

- **bed_2**: still −1.6 °C daytime bias and NOT PV-correlated (r² = 0.03).
  Its warmth in the record likely arrives via the per-room heat split
  during unit-on periods (AIRFLOW_WEIGHTS equal-split assumption) or
  occupancy gains — next thing to investigate.
- Heavy-day energy deficit beyond defrost (~13–17%): same suspect — if the
  sim misallocates delivered heat, simulated actrl satisfies targets
  sooner than the real house did.

## Caveats / next steps

- r² ≈ 0.28 on the efficiency fit — noisy at 10-min resolution even after
  defrost excision; residual scatter is likely uneven room weighting of
  delivered heat, occupancy/appliance gains, and wind (BOM wind speed is
  exportable if we want it as a regressor).
- tau_house = 29.9 h is baked into q; error in tau shifts the *level* of
  e slightly but barely affects the power/Tout coefficients (heating
  dT/dt >> decay term during active heating).
- **C_house anchor (2026-07-02, weak but physically coherent)**: the unit
  is an ActronAir URC-100AS/LRE-100AS (10 kW nominal); rated capacity/power
  tables transcribed in `analysis/actron_tables.py` (source PDF in
  docs/references/). Matching observed high-power no-sun no-defrost
  samples (only n = 9 in June, P > 1.6 kW) against interpolated table COP
  gives **effective C_house ≈ 4.8 kWh/K (IQR 4.0–5.1) → UA ≈ 160 W/K**,
  implying min-power COP ≈ 5.0 at 10 °C outdoor / ≈ 3.75 at 5 °C —
  sensible part-load inverter numbers vs the ~3.5 rated COP. Caveats:
  duct losses are folded in (this C is referenced to unit-delivered
  heat); rated-COP-at-part-load biases C low; the 4 °C table point is
  defrost-affected (rating-standard artefact), muddying interpolation
  near 5–6 °C. Note temperature data alone can never separate Q from C —
  the table anchor (or a resistive-heater experiment) is structurally
  required, not a shortcut. July's cold mornings will add full-power
  samples and tighten this.
- Winter data only. Cooling-season parameters (and solar gain modelling)
  need summer archives — the monthly export timer is accumulating them.
- Room-level delivered-heat split (damper positions × fan power → per-room
  airflow share) not yet attempted; needed for the full per-room simulator
  (ideas.md §3) rather than house-level.
