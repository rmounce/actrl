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

## Defrost emulation (2026-07-03)

`analysis/defrost_fit.py` characterises the 9 raw June episodes (the
forced-max-speed signal itself, docs/calibration.md "Defrost" above) rather
than the padded overhead window used for energy accounting:

- Duration: median 10 min (IQR 5–17 min).
- Outdoor temp during: median 5.3 °C (IQR 4.3–6.1 °C).
- Outdoor-unit power during: median 1336 W (IQR 1212–1478 W) — well above
  min-power draw (~610 W), and the indoor coil measurably cools (median
  min 14.1 °C vs its normal ~35 °C), consistent with a brief reverse-cycle
  extracting heat from indoors to melt the outdoor coil.
- Prior compressor-on time at outdoor temp < 7.5 °C before triggering:
  median 145 min (IQR 106–353 min, n=8 — small sample, wide spread).
- Recovery (elevated power after the episode, before settling back to the
  pre-episode level): median 11 min, but energy above baseline is mostly
  noise (~0 kWh) except a few clear ~0.2–0.5 kWh cases; not modelled
  separately (see below).

Modelled as a simple accumulator (`sim.hvac.Defrost`): compressor-on time
at outdoor temp < 7.5 °C accumulates; crossing the median trigger (145 min)
starts a defrost lasting the median duration (10 min) at the median power
(1.35 kW) delivering zero heat. No separate recovery-boost model — actrl's
own control loop, running live in the closed loop, reacts to the resulting
temperature dip and drives the compressor back up on its own, which is
what real recovery amounts to. The trigger condition itself isn't
identifiable from this data (it's inside the unit's firmware); the
accumulator is a plausible proxy, not a measured mechanism.

Whole-day replay impact: negligible on room RMSE/bias (episodes are short
relative to a day), heavy-day energy match tightens further (22nd: −0%→
+3%, wobbling slightly high now that some genuinely-wasted energy is
added back in; 27th: −14%→−9%). Mild days unaffected — they rarely sustain
145 min of continuous cold running.

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

### Heat-split mass assumption fixed with real floor areas (2026-07-03)

Ryan supplied the house's NatHERS energy-efficiency report (EOD-3341,
Energy and Outdoor Design, 2020), which gives real floor areas and
construction per room. Two things it settled:

- **Room-size ranking confirmed exactly**: bed_1+WIR 18.9 m² > bed_3
  14.5 m² > bed_2 12.1 m² > study 8.7 m² — matching Ryan's description and,
  independently, the tau_out ranking (envelope loss scales with external
  wall + glazing area, which the report also details per room). All
  external walls are the same R2.5 insulation, so envelope differences
  between rooms are down to area, not insulation quality.
- **bed_2's weak solar-gain fit (r²=0.03 vs bed_3's 0.45) isn't a
  missing-window issue**: it has an NE window (2.0×1.5 m, low-e, SHGC
  0.45) essentially identical to bed_3's NE window (2.0×1.8 m, same
  glazing product/SHGC). Ryan's leading explanation (2026-07-03): the
  neighbouring party wall (bed_2's NW wall, and kitchen's — confirmed a
  true party wall, unlike bed_1's/bed_3's genuinely-external NW walls)
  extends past bed_2's frontage and may shade its NE window, especially
  from the low winter sun the June fit was measured against — a taller/
  forward obstruction blocks a low sun far more than a high one, and an
  intermittently-shaded window regressed against a smooth PV proxy would
  wash out to a weak, noisy fit rather than a moderately smaller one,
  matching what's observed. Testable prediction: bed_2's solar
  correlation should strengthen in summer (higher sun angle clears the
  obstruction) — re-run `analysis/solar_fit.py` once summer data
  accumulates (same export timer as the cooling-calibration work).

`_room_q` (sim/closed_loop.py) conflated two different things under one
weight: the *airflow* split (confirmed correct, duct-count based) and the
*thermal-mass* fraction used only to convert delivered kW into each room's
own K/h forcing (each room's actual temperature *response* to that K/h is
separately, independently fit per room via tau_out/tau_cpl in
sim/house.py — this weight only affects the heat allocation, not the
dynamics). Previously all four single-duct rooms used equal mass weight
(1.0), overstating study's and bed_2's assumed mass by up to ~2x relative
to bed_1. Decoupled into `AIRFLOW_WEIGHTS` (duct count, unchanged) and a
new `MASS_WEIGHTS` set from the real floor areas above (bed_1 1.395, bed_2
0.893, bed_3 1.070, study 0.642, normalised to average 1.0 across the four
rooms; kitchen left at 2.0 — already close to its living-area floor-area
ratio of ~2.3x a bedroom, and separately anchored by its own RC fit).

Whole-day replay impact (four June days): concentrated on the heavy
heating days, as expected (more delivered heat to reallocate). 22nd:
bed_3 daytime bias −1.59→−1.36 °C, study −1.12→−0.79 °C, bed_2
−1.77→−1.60 °C; energy match −15%→−0%. 27th: bed_3 −1.51→−1.46 °C, study
−0.67→−0.57 °C, bed_2 −1.77→−1.68 °C; energy match −21%→−14%. Mild days
(8th, 15th) barely moved on temperature (little heat being delivered to
reallocate) but got somewhat worse on energy over-prediction (8th: +41%
before the heat-delivery lag, +45% after adding it, +57% after this mass
fix) — consistent with the known, still-open "unlit simulated rooms
overheat on mild days" solar/internal gain gap, not a regression from this
change specifically. bed_2 improved the least of the three — still the
top remaining suspect, now with the size-mass confound removed as an
explanation.

### Controller input is feels-like, not average temperature (fixed 2026-07-03)

Found while chasing why the sim started ~80 min late on the cold night of
the 27th (recorded unit: min-power run 02:24–04:32 with the 90-min purge
at 03:54; simulated unit: nothing until 03:44). Production actrl actuates
on per-room **apparent temperature** ("feels like",
`packages/aircon.yaml`): `feels_like = T + 0.33*wvp − 4.0` with
`wvp = RH/100 · 6.105 · e^(17.27T/(237.7+T))` — and the room setpoints in
the frontend/archive are in feels-like units too. At winter indoor
humidity (~55–60% RH) that's a roughly constant **−0.4..−0.5 K** offset
below average_temperature, so a replay feeding raw average temps gives the
simulated controller ~0.45 K of headroom the real one never had: late
starts, lower on-fraction, under-run energy on heavy days. (Recorded
bed_2 on the 27th: average 16.50 °C ≡ feels-like 16.05 °C at the 16.0
setpoint — exactly the observed 02:23 trigger.)

Fix: per-room `average_humidity` was in InfluxDB but not exported — added
to `tools/export_entities.json` (June backfilled) and `calib.py` (same
resampling rule as temperature). `ClosedLoop.step()` takes per-room
`ctrl_offsets` applied only to what the controller reads (the house model
still simulates physical temperature); `analysis/replay_day.py` computes
the offset from *recorded* temp+RH, treating vapour pressure as exogenous
(the sim doesn't model moisture).

Whole-day replay impact (all four days improved or neutral): 27th energy
−9%→−2%, night start now 02:40 vs recorded 02:24 (was 03:44), bed_1 RMSE
0.37→0.20, study 0.70→0.38, bed_3 1.91→1.65; 8th energy +57%→+33%; 15th
+5%; 22nd +3%→+5%. The morning modulation window on the 27th (the
transient-fidelity spot-check that surfaced all this) now tracks the
recorded ramp within ~0.2 K instead of lagging flat for ~90 min and
overshooting; residual: sim peaks at increment 11 vs recorded 9 with a
~0.4 K overshoot at 09:15, shutting off within minutes of the real unit.

Two caveats: the mild-day energy over-prediction (8th +33%) is now the
honest remaining solar/internal-gains gap rather than being partially
masked by the input error; and `wvp` from recorded data means fully
counterfactual scenarios (no recorded day underneath) need a humidity
assumption — a fixed −0.45 K offset is a fine approximation for winter.

### Measured-air lead node (added 2026-07-03, task 009)

Even after the feels-like fix, both spot-check windows showed the sim
lagging then overshooting where the recorded trace rises quickly and
levels off. Cross-correlation showed zero *phase* lag — the mismatch is
magnitude: at matched compressor speed the sim heated the kitchen sensor
2–4x slower than recorded (e.g. 0.67 vs 2.65 K/h at speed 8). The damper
split matched, and the recorded rise is physically impossible for the bulk
mass (3.18 K/h x kitchen's mass share implies more heat than the unit's
entire thermal output). The sensors read a small fast air mass, not the
RC bulk: superposed unit-stop events (12 clean, kitchen damper >50%) sag
at ~2 K/h for ~10 min then relax onto the single-RC decay (~0.5 K/h) by
~40 min.

Model (docs/tasks/009, implemented by a Sonnet subagent, reviewed/merged):
a per-room sensor node layered on the untouched bulk ODE —
`dTm/dt = ((T + lead_h*q) − Tm)/tau_meas_h`. Carries no energy; controller
and all recorded-data comparisons use Tm; `tau_meas_h=0` is an exact
passthrough (regression-gated bit-identical to the pre-task replay).

Fit (`analysis/fast_node_fit.py`, superposed no-sun stop events, slow
trend subtracted, exp fit by grid search): kitchen tau ~13 min, lead
0.23 h, r2=0.94 (n=5; n=12 incl. sunlit gives the same) — its two ducts
dump supply air near the sensors. bed_1 0.07 / bed_2 0.08 h leads (weaker
fits; bed_1's with-sun fit inflates to 0.21, rejected as solar-contaminated).
bed_3's own fit has the wrong sign (n=4, solar) and study has no clean
events — both take the pooled bedroom prior (lead 0.08, tau 15 min).

Closed-loop refit of the kitchen lead (2026-07-03, follow-up): Ryan
observed the sim still cycling at roughly half the recorded frequency at
min power on the 22nd (mean on 52 vs 29 min, off 68 vs 35; same
on-fraction). The controller-eye demand signal (aircon_weighted_error,
now in sim telemetry) swings over the same 0..0.9 band in both but at
half speed in *both* directions — the driving room (kitchen, argmax in
both) has its sensor amplitude ~2x under-modelled. The open-loop stop-fit
is biased low (trend subtraction assumes the node settles inside the
45 min window; q_pre rides the COP model), so the kitchen lead was refit
by closed-loop grid search on the 22nd's replay: lead 0.45 (tau 0.21 h
unchanged) matches recorded cycle counts exactly at midday (4/4 starts,
26%/26% on-fraction) and nearly in the evening (6/7); 0.40 does not (3/4)
— the transition is sharp. Cost: sim energy reads −7/−8% on heavy days
(more time at min power; was +5/−4%) — cycle texture prioritised, energy
gap is the open item. Mild days improved too (8th +28%→+22%, 15th +1%).

Bedroom leads refit the same way (2026-07-03, follow-up 2): the 06-27
early-morning cluster (sim over-revving to increment 11 at 06:40-07:00,
late warmup start, energy under-read) traced via the controller-eye
comparison to bed_1: recorded bed_1 sags fast after the 04:32 stop
(fast node) and retriggers the unit at 05:19, running 79% of 05:00-06:30
and rising 1.1 K before the target ramp; the sim's bed_1 lead (0.07,
stop-fit) barely sags, stays off till 06:00, enters the ramp ~0.5 K
behind, and the deadband integrator winds up to 11. Bedroom leads at 5x
stop-fit (0.35-0.40, consistent with kitchen's 0.45 — same ducts and
sensors) fix it: early-window on-fraction 1.00 (rec 0.79), peak increment
2 (rec 3-5), bed_1 RMSE 0.21. Sharp threshold again (4x behaves like 3x).
Four-day energy after both refits: 8th +21% (solar gap), 15th -0%, 22nd
-1%, 27th -11% (now slightly under-revs the morning ramp: above-min 13%
vs 16%). June-wide scorecard (task 010) is the arbiter from here.

Whole-day replay (pre-refit numbers): improved nearly everywhere. 27th: kitchen RMSE
0.38→0.33, bed_1 0.20 (was 0.37 pre-feels-like), study 0.38→0.48→0.48…
energy −2%→−4%; 22nd: study 0.86→0.70, bed_2 1.85→1.66, energy +5%; 8th
+33%→+28%; 15th +5%→+2%. The 06-27 modulation-window ramp now tracks
recorded within ~0.1–0.3 K at matched increments (8–9 both, was pushing
to 11), and the rate-by-speed gap at speeds 7–8 closed from 3–4x to
~1.3x. Remaining shape gaps: the sim over-increments early in that window
(9–11 vs recorded 3–5 around 06:40–07:00), and it reaches target and cuts
off ~08:30 where the real unit tapered 6→1 until 09:30 (~0.4 K late
overshoot) — candidate next targets, likely controller-state rather than
plant-model effects.

### Orientation-resolved solar gains (fitted 2026-07-03, task 011)

The June-wide scorecard (task 010) showed the dominant remaining error is
mild-day over-heating: sim rooms read cold daytime and the controller
heats a house that was coasting on sun (energy to +86%, sim ~9-10
starts/day vs recorded 3-6 on light days). Hourly replay-bias profiles
localise it: bed_2 runs up to −4.6 K and bed_3 −3.1 K at 09:00–12:00 on
sunny days, decaying through the afternoon — a large *morning* gain the
model misses entirely.

Root cause is the solar basis function, not the coefficients. The
whole-house PV proxy (roughly north array) peaks at noon; a NE window's
direct beam peaks mid-morning and dies by midday. Regressed against the
noon-peaked proxy, bed_2's morning spike washes out to a tiny slope with
r²=0.02 (its "weak solar fit" was never weak sun), and the kitchen's fit
comes out *negative* (its morning gains anti-correlate with the noon
proxy inside off-windows) — which is why it was clamped to zero. A
trajectory-space refit against the same proxy (analysis/
solar_refit_openloop.py — simulate all rooms jointly so the coupling
term sees *simulated*, not recorded, neighbours) converges right back to
the old values, confirming the basis, not the fit method, is at fault.

Fix (analysis/solar_orient_fit.py): two clear-sky direct-beam bases per
room — vertical NE face (morning) and NW face (afternoon); Adelaide's
winter sun tracks sunrise ~61° (ENE) → north → sunset ~299° (WNW), so
SE/SW glazing gets no direct winter beam at all (bed_1's large SW glazing
included — its winter gain must arrive otherwise). Each basis scaled by
a cloudiness index (recorded PV / per-minute-of-day June PV envelope).
Joint trajectory-space fit, head-trimmed unit-off windows, converged in
2 iterations:

| room    | s_ne [K/h @ full sun] | s_nw  |
|---------|----------------------:|------:|
| bed_1   | 0.298 | 0.010 |
| bed_2   | 0.577 | 0.000 |
| bed_3   | 1.354 | 0.000 |
| kitchen | 0.403 | 0.000 |
| study   | 0.271 | 0.082 |

bed_2 at less than half bed_3's gain despite near-identical NE windows
quantifies the neighbour party-wall shading hypothesis as *partial*
shading. All NW terms ≈ 0 in winter (no afternoon beam to fit) — refit
against summer data when it exists. Wired into the sim by task 011
(sim/solar.py + RoomParams.s_ne/s_nw; the old PV-proxy `solar` term is
zeroed and superseded).

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

## Duct/roof-space losses — investigation (2026-07-05, ideas.md #4)

Question: the closed-loop energy refit scaled heat-per-kWh by 0.80 (~20%
unaccounted). Idea #4: split COP-model error from real duct loss using the
m5atom coil sensors + fan state. Result: **the loss pathway is confirmed
but the proposed coil-side energy balance is not viable with current
sensors.**

Sensor semantics (June, ~5000 rows with coil data; the four raw `m5atom_*`
temps have only 11-19% coverage — docs/data.md):

- `m5atom_outside_temp` median 7.0 C during heating ≈ BOM outdoor 7.6 C —
  a real outdoor-ambient sensor. Confirms these are genuine Midea probes.
- `m5atom_outside_coil_temp` 5.3 C during heating (evaporator below
  ambient) — physically sensible.
- `m5atom_inside_temp` 12.7 C during heating: correlates **0.81 with
  outdoor**, 0.57 with room mean, sits ~5 K above outdoor and ~5 K below
  the ~18 C rooms. NOT the follow-me transmitted value (weighted_error is
  ±1 K while this swings 10-20 C). **Ryan confirmed the indoor unit and
  ducting run through unconditioned roof space** — so this is the
  return/roof-space air temperature. The duct-loss pathway is real: return
  air pre-cools ~5 K (room 18 → coil 13) and supply air loses heat to the
  same ~13 C roof on the way back to registers.
- `m5atom_inside_coil_inlet_temp` 31 C during heating. **This is coil-metal
  temperature (T2), not supply-air temperature** — proven: taking it as
  supply air with `inside_temp` as return gives air ΔT 17 K, hence apparent
  COP **8-11** at the fan-curve airflow (440/520/600 l/s min/nom/max,
  `analysis/actron_tables.py`) — physically impossible. At the observed
  ~1.18 kW electrical and a plausible COP 3.3-4.0, the *real* supply-return
  air ΔT is only 5-9 K (Q ≈ 4 kW), i.e. the coil-metal reading overstates
  the air rise ~2x.

Consequences:

- **No usable supply-air sensor exists**, so coil-side output cannot be
  measured directly — the fan curve alone doesn't rescue the balance.
  Duct loss therefore still cannot be cleanly separated from COP-model
  error; the closed-loop 0.80 scale (~20% deficit) stands as an **upper
  bound** on duct loss.
- **Rated-table operating point**: the actron heating table shows capacity
  *rising* as return-air temp falls (outdoor 7 C: 12.2 kW @ 15 C return vs
  10.9 kW @ 20 C, COP ~flat at 3.6). The unit runs on ~13 C roof-space
  return, not 20 C room air, so it sits below even the 15 C row — higher
  capacity, similar COP, but much of that extra heat just re-warms
  pre-cooled return air, dragging COP-*to-room* down. sim/hvac.py's COP
  model could use `inside_temp` (roof return) rather than room temp as the
  indoor operating point — a concrete refinement, but it re-describes the
  0.80 deficit rather than splitting it.
- **To actually quantify duct loss**: add a supply-air temperature probe
  *in the supply duct/register* (and ideally a true return-air probe in the
  airstream at the unit). Cheap hardware; without it the split is
  unobservable. Flagged for the next hardware/data step.

Net: idea #4's premise (duct loss is real and material) is **supported** —
roof-space ducting confirmed, ~20% aggregate deficit, ~5 K return
pre-cooling — but its measurement method is **disproven**; redirect to a
supply-air sensor before spending more analysis effort here.

## Multi-zone damper fidelity — weekday double-ramp mornings (2026-07-05)

Groundwork for room-PID (damper) tuning: does the sim reproduce how zones
share supply air? `analysis/damper_fidelity.py`, 14 scoreable June weekday
mornings where bed_1 AND kitchen ramp together, 05:00–11:00 local,
compared ONLY on minutes where both the recorded and simulated unit run —
dampers freeze at their last position when the unit stops, so a
run-decision timing divergence otherwise reads as hours of fake damper
error (06-08: sim stopped 45 min before the real unit, so its bed_1
froze at 100% through the 08:30 target setback that the real controller
answered by slamming bed_1 shut; ungated stats had bed_1 +22 pts of pure
artifact).

Gated per-room medians (position pts / moves-per-running-hour):

| room | rec mean | sim mean | rec mv/h | sim mv/h | RMSE | dmp bias | temp bias |
|---|---|---|---|---|---|---|---|
| bed_1 | 93.9 | 98.8 | 0.44 | 0.00 | 7.2 | +0.9 | +0.03 |
| bed_2 | 2.6 | 9.3 | 0.00 | 0.32 | 14.4 | +4.3 | −0.55 |
| bed_3 | 10.3 | 13.0 | 0.58 | 0.82 | 16.7 | +1.3 | −0.95 |
| kitchen | 70.5 | 56.0 | 0.52 | 0.52 | 21.0 | −13.1 | −0.02 |
| study | 7.8 | 6.5 | 0.00 | 0.00 | 19.6 | +2.0 | −0.46 |

bed_1→kitchen morning handoff (bed_1 releases, kitchen takes over,
~08:40 like clockwork in the record): sim − rec median +0.08 h
(n=5 mornings where both hand off inside the window; most sim "misses"
are the unit already off by the setback, per above).

Verdict: **usable for relative room-gain A/Bs, with three caveats.**
corr(temp_bias, damper_bias) = −0.51 over 68 room-days — the sim opens
dampers where its rooms read cold, i.e. the controller responds
consistently and the damper gaps are mostly plant-side temperature gaps.

1. **Kitchen −13 pts while running with ~zero temp bias**: the sim holds
   kitchen on target with less damper than reality needed — its heat
   split (AIRFLOW/MASS weights) credits the kitchen too much heat per
   opening point. Plant-side; treat absolute kitchen-share conclusions
   cautiously until the split is refit.
2. **Sim bed_1 never moves off its pin (0.00 vs 0.44 mv/h)**: the
   noise-free sim lacks whatever jitters the real PID off saturation.
   Any gain sweep hunting for the instability threshold MUST inject
   sensor noise (replay ctrl_noise / tune.py --noise-sigma) — hunting is
   noise-excited, and a noise-free sweep will overestimate the stable
   gain margin.
3. **Gate every damper comparison on both-running minutes** (the script
   does); mild-morning run-decision divergence contaminates anything
   ungated.

## Kitchen heat-split refit + damper texture (2026-07-05, follow-up)

Fixes for the fidelity caveats above, in service of room-PID tuning:

- **MASS_WEIGHTS["kitchen"] 2.0 → 2.6** (`analysis/kitchen_split_refit.py`):
  grid against gated damper authority on the double-ramp mornings.
  2.0 (the carried-over equal-mass-era value, never re-derived from the
  NatHERS floor areas like the bedrooms) credited the kitchen ~13 damper
  points too much heat per opening. At 2.6: kitchen gated bias −1.5 pts,
  bed_1 −1.6, kitchen temps still ~0 bias, cycle texture unchanged
  (starts stable across the whole 2.0–3.5 grid). Physically: the open
  kitchen/living/dining zone carries ~30% more effective mass than the
  living-area floor ratio suggested. Side benefits nobody asked for:
  bed_1→kitchen handoff now reproduced on 9/14 mornings (was 5), bed_1's
  noise-free damper move rate 0.34/h (was 0.00), and the June scorecard
  IMPROVED across the board — kit_rmse 0.332 (was 0.340), rms_all 0.649,
  median energy error −0.1% (was −2.5%; the mild-day deficit largely
  closed). New standing baseline in docs/tasks/010 Log; controller-CI
  baseline re-pinned same commit. Watch-for: on_frac now +2 pp high.
- **Measured-noise damper texture** (fidelity pass --noise-sigma 0.012):
  measured sensor noise restores part of the recorded damper jitter
  (bed_1 0.17/h vs recorded 0.40 at the old weight; kitchen exact) but
  not all of it — residual jitter is unmodelled (damper position
  feedback quantisation / humidity wiggle candidates). Gain sweeps must
  inject noise AND still treat margins near pinned zones with ~2x
  caution.
- **Rebalance response, current gains** (recorded, scratch analysis):
  bed_1's ~08:30 setback (−3.3 K demand step) closes its damper in
  11 min median (n=13) WHILE THE UNIT RUNS — kp-driven, adequate. The
  27–56 min outliers are all the unit cycling off mid-setback (dampers
  freeze). So large-step rebalancing is NOT the sluggish regime; the
  felt sluggishness must live in sub-K imbalances where room_ki does
  the work — probed by `analysis/rebalance_step.py` (sim injection of a
  +0.5 K read bias on one zone, latency counted in running minutes).

Probe outcomes (2026-07-05 eve, analysis/rebalance_step.py on 06-22):

- **Evening (19:00) injection: gains DON'T bind.** Latency ~100 running
  min and settle ~76-86% across room_ki x1..x8 — the shed is blocked by
  actrl's minimum-airflow inflation loop, not integral speed: with the
  kitchen the only calling zone, every shed attempt violates min airflow
  and re-inflates ALL integrals (kitchen first — largest airflow
  weight). Structural, gain-independent; also means a summer-override
  zone can stay served whenever it's needed to carry min airflow.
- **Deep-warmup (07:00) injection: ±0.5 K doesn't bind either** —
  everyone is saturated 2-3 K below target, so a sub-K bias doesn't
  reorder authority (and the probe's settle window needs a bounded
  horizon for morning injections; metric artifact noted).
- Net: the sub-K contention regime the room gains actually govern
  BARELY EXISTS in the winter schedule — one dominant zone at a time.
  The right vehicle for the gain sweep is the synthetic
  divergent-target scenario (multiple zones held near-target, then
  biased), which is also the honest summer-override proxy.
- Noise-texture standard config for sweeps: kitchen 2.6 + ctrl_noise
  sigma 0.012 K / tau 15 s -> damper move rates within ~±30% of recorded
  (bed_1 0.33 vs 0.39, kitchen 0.44 vs 0.54, bed_3 0.69 vs 0.49 mv/h).

## Known residual: kitchen cold-morning warmup onset (2026-07-06, Ryan's catch)

Spotted on the texture-comparison artifact: on 06-22 the sim's kitchen
damper lags the recorded ramp-up ~30-45 min (06:10 recorded 80% vs sim
35%) and misses the recorded mid-ramp shed (95→50% across 06:30-07:10).
Diagnosis:

- The REAL kitchen runs ahead of the statctrl ramp (+0.4 K above the
  ramping target by 06:20), so its PID sheds; the sim kitchen runs
  behind and keeps demanding. Damper divergence is downstream of a
  plant-side warmup-speed gap, not the zone controller.
- 06:00-07:00 kitchen temp-rise gap (rec − sim) across 10 mornings:
  ~0 on mild mornings (06-04/05/08: −0.15..−0.01) and large on the
  coldest (06-22 +1.01, 06-25 +0.59, 06-26 +0.60, 06-29 +0.46 K/h;
  median +0.28). The gap scales with how hard the unit ramps →
  high-power warmup physics: the kitchen lead node was fitted on midday
  MIN-POWER cycling (sim/house.py), and the static damper/airflow split
  may underweight the kitchen at full fan. (Breakfast internal gains
  would not correlate with outdoor cold.)
- NOT caused by the mass 2.6 refit — A/B on 06-22 shows 2.0 was worse
  in this window (06:10 damper 20% vs 35%; temps equal-slow). The
  refit's 05:00-11:00 gated means hid the sub-window shape.
- Converges by ~07:50 (temps and dampers) every morning checked.

Impact: modest — kitchen authority during the first warmup hour reads
low in sim on cold mornings; whole-morning gated stats and the June
scorecard are unaffected. Candidate fix if it ever matters: refit the
kitchen lead/flow share against warmup onsets (high-q events) instead of
only midday cycling; treat as a q-dependent lead or a fan-speed-dependent
flow split. Logged as a residual, not chased further now.
