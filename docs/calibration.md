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

## Caveats / next steps

- r² ≈ 0.28 on the efficiency fit — noisy at 10-min resolution even after
  defrost excision; residual scatter is likely uneven room weighting of
  delivered heat, occupancy/appliance gains, and wind (BOM wind speed is
  exportable if we want it as a regressor).
- tau_house = 29.9 h is baked into q; error in tau shifts the *level* of
  e slightly but barely affects the power/Tout coefficients (heating
  dT/dt >> decay term during active heating).
- Absolute COP needs C_house (kWh/K). Candidate methods: (a) step
  experiments — known electrical energy into a closed-up house overnight,
  measure ΔT; (b) manufacturer capacity tables for the Midea unit matched
  to observed compressor increments. Then e converts to real COP and the
  policy trade-offs become kWh-denominated.
- Winter data only. Cooling-season parameters (and solar gain modelling)
  need summer archives — the monthly export timer is accumulating them.
- Room-level delivered-heat split (damper positions × fan power → per-room
  airflow share) not yet attempted; needed for the full per-room simulator
  (ideas.md §3) rather than house-level.
