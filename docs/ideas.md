# Ideas / roadmap

Proposals, roughly ordered. Status markers: `[ ]` proposed, `[~]` in
progress, `[x]` done.

## 1. Near-term fixes (small, low risk)

- `[x]` statctrl `save_adaptive_model`: merge only this room's keys to fix
  the cross-instance overwrite race (docs/statctrl.md).
- `[x]` actrl `_set_static_pressure`: bound the retry loop (5 attempts →
  warn and continue) so a dead ESPHome device can't wedge the app.
- `[x]` actrl: guard `avg_deriv = deriv_sum / weight_sum` against
  `weight_sum == 0`.
- `[x]` actrl: replace stray `print()` with `self.log()`; rename the local
  that shadows module-level `climate_entity` in `_get_current_targets`.
- `[ ]` Reduce/eliminate blocking `time.sleep` in callbacks (AppDaemon
  `run_in` continuations, or at minimum document why each sleep exists).

## 2. Testability refactor

Progress: task 001 (pure-class extraction, done), task 002 (headless
whole-cycle harness + golden scenarios, done), task 003 (capacity controller
extraction, done), task 004 (calibration data export, done — see
`docs/data.md`), task 005 (calibration data loader `calib.py`, done).
See `docs/tasks/`.

Extract the remaining control logic from the `hass.Hass` classes so it can
be unit-tested and later driven by a simulator:
- Done (task 003): `compress`/`midea_reset_quirks`/`midea_runtime_quirks`
  extracted as `control.MideaCapacityController` with 8 golden fixture
  scenarios in `tests/fixtures/capacity/`.
- The PID integral-adjustment passes in `_calculate_pid_outputs` (top-zone
  anti-runaway, normalisation, minimum airflow, negative clamp) — pure given
  (pid states, errors, airflow weights, door states, fan power).
- Keep the AppDaemon class as a thin I/O shell: read entities → call pure
  logic → write entities/services. Config (rooms, entity names, constants)
  becomes a dataclass so tests/simulator can construct variants.
- Tooling: `uv`-managed dev venv, `pytest`. Tests must not import `hassapi`.

Test ideas once extracted:

- Step-sequence invariants: any sequence of demands leaves `prev_step`
  returning to 0; emitted values never set the ramp-up flag unintentionally
  (never ≥ stable+2 outside faithful/hysteresis mode); speed estimate stays
  in [0, increments+margin].
- Deadband integrator: symmetric behaviour, ±1 emission rate vs error size,
  preload on sign change.
- Scenario replays: synthetic error trajectories (approach, overshoot,
  purge-window, defrost) asserting on/off decisions match current behaviour
  (golden tests to protect against regressions during refactoring).
- statctrl: slew step arithmetic, deadline-driven step intervals, adaptive
  EWMA update maths, model merge.

## 3. House thermal simulator (the big one)

Goal: a model of the house + HVAC accurate enough that controller changes can
be evaluated in simulation instead of live trial on the family.

### Components

- **Thermal model**: per-room lumped RC network (room air node, envelope
  node, inter-room coupling, common return-air path). Inputs: outdoor
  temp/humidity, solar gain (can reuse EMHASS PV/irradiance data as a proxy),
  internal gains, window/door states, delivered airflow × supply-air temp per
  room. Low thermal mass (per archive/README.md) suggests a 1R1C or 2R2C per
  room may suffice.
- **HVAC model**: compressor increment → capacity and electrical power as a
  function of indoor/outdoor conditions (heating COP degrades with outdoor
  temp; defrost cycles below ~7 °C outdoor); fan power already measured
  (table in actrl.py `_calculate_pid_outputs`); duct/damper model mapping
  damper positions + static pressure + fan speed → per-room airflow split.
- **Midea controller emulator**: the flip side of
  `MideaCapacityController` — given fake temperature reports, emulate the
  observed stepping/flags/purge/shutdown behaviour so actrl's protocol
  handling is exercised end-to-end.
- **Harness**: runs extracted control logic against the simulator with a
  fake entity layer; produces comfort metrics (time-in-band, overshoot),
  energy use, compressor cycling counts.

Progress (2026-07-02): thermal model (`sim/house.py`, task 008 — 24-night
replay median RMSE 0.14 °C) and unit emulator (`sim/midea_unit.py`, task
007 — closed-loop tracking vs MideaCapacityController within ±2 increments;
11 reviewable assumptions in its docstring) done.

Progress (2026-07-03): HVAC model (`sim/hvac.py` — power linear in
increment 665→3055 W, COP = fitted efficiency shape × C_eff 4.8 kWh/K)
and full closed-loop assembly (`sim/closed_loop.py` — real actrl via the
golden-test harness → follow-me reports → unit emulator → HVAC → house,
10 s cycle) done. Assembling it caught a real protocol subtlety: actrl
transmits `setpoint + mode_sign×demand`, so heat mode reports *below*
setpoint for more compressor (emulator assumption 11). Compressor spin-up
lag identified from June power-step transients (`analysis/lag_fit.py`,
tau ≈ 20 s) and modelled as a first-order lag on electrical power.
Whole-day replay validation done (`analysis/replay_day.py`, four June
days): kitchen 0.34–0.40 °C RMSE with ~zero bias; energy within −11%
on heavy heating days (missing defrost explains most). Dominant gap =
solar/internal gains (night-only RC fit) — bedrooms read 1–2 °C cold in
the day and mild days overpredict heating energy. Per-room solar-gain
term fitted and added (`analysis/solar_fit.py` → `RoomParams.solar`,
driven by `power_pv_5m`): bed_3 daytime RMSE −25–45%, mild-day energy
error halved. Heat-delivery lag (the minutes-scale "elbow" Ryan flagged,
distinct from the 20 s electrical lag) identified from the same clean
±1 steps via room-temperature-derivative superposition
(`analysis/heat_lag_fit.py`): kitchen gives the cleanest fit (dead time
15 s, tau 180 s, r²=0.95), wired in as `sim.hvac.DeadTimeLag` downstream
of the electrical lag — negligible effect on room RMSE, small improvement
to heavy-day energy match (−19%→−15%, −23%→−21%). Heat-split mass
assumption fixed using real floor areas from the house's NatHERS energy
report (bed_1 18.9 m², bed_3 14.5 m², bed_2 12.1 m², study 8.7 m² —
previously all assumed equal): decoupled `_room_q`'s airflow split
(duct-count based, confirmed correct) from its thermal-mass split (now
floor-area based) in sim/closed_loop.py. Concentrated improvement on
heavy heating days — bed_3/study daytime bias notably better, heavy-day
energy match −15%/−21%→−0%/−14% — with bed_2 improving the least (still
the top open question) and mild-day energy over-prediction slightly worse
(pre-existing solar/internal-gain gap, not new). Defrost emulation added
(`analysis/defrost_fit.py`, 9 June episodes → `sim.hvac.Defrost`): a
compressor-on-time-at-cold-outdoor accumulator triggers a ~10 min,
~1.35 kW, zero-heat episode (median observed values); no separate
recovery model, since actrl's own control loop reacts to the temperature
dip live in the closed loop. Heavy-day energy match tightens slightly
further. bed_2's weak solar fit has a leading explanation (neighbour's
party wall shading its NE window at low winter sun angles — testable once
summer data arrives). Remaining: bed_2 daytime bias specifically; cooling
calibration (needs summer data).

Also (2026-07-02): first-pass system identification on June data done —
house envelope tau ≈ 30 h, per-room two-node RC fits, and the heating
efficiency shape vs compressor power / outdoor temp. See
`docs/calibration.md` and `analysis/sysid_june.py`. Key steer: outdoor temp
moves efficiency ~5%/K, dwarfing the ~14% min→max compressor-speed penalty.

### Calibration / cross-check

- Source recorded data: `tools/export_history.py` (task 004, done) archives
  room temps, damper positions, comp speed estimate and other controller
  state, coil temps, outdoor conditions and power from InfluxDB into
  gitignored per-day CSVs — schema notes in `docs/data.md`. Raw history is a
  rolling 30-day window, so run it monthly to accumulate seasonal coverage.
- Fit RC parameters per room via system identification on periods of known
  input (nightly free-running decay is ideal for envelope parameters; step
  changes in damper position for airflow split).
- Validation: replay a held-out day's inputs through the simulator, compare
  simulated vs recorded room temperature trajectories; report per-room RMSE.
  Continuously re-check as seasons change.

### Payoff

- Regression-test controller changes (does the new deadband logic still
  avoid the purge cycle? does it cycle the compressor more?).
- Tune constants (PID gains, thresholds) offline against comfort/energy
  objectives.
- Quantify grid-surplus strategy value in kWh shifted vs comfort cost.

## 4. Smaller ideas

- `[ ]` Config as data: move rooms/entities/constants into `apps.yaml` args
  or a dataclass config so a second house / test rig is possible.
- `[ ]` Structured metrics: publish PID terms and controller state as JSON
  attributes on one entity rather than many `input_number`s (fewer recorder
  rows, easier analysis).
- `[x]` Watchdog: `hvac_watchdog.py` (task 006) alerts on stale/unavailable
  sensors and a wedged control loop — see docs/watchdog.md. Deployed apps
  need a manual apps.yaml block in prod.
- `[x]` Deploy script: `./deploy.sh` copies the deployable files into
  `appdaemon/`.
- Won't do (for now): anything building on statctrl adaptive optimum start
  (e.g. exposing learned minutes-per-degree as sensors) — feature trialed
  and deliberately left inert, not a focus area (docs/statctrl.md).

## 5. Hypotheses from the calibration/tuning sessions (2026-07-05, Fable brainstorm)

Ranked roughly by expected payoff. Each is testable with tools that now
exist (closed-loop sim, analysis/tune.py, analysis/preheat.py,
analysis/comfort.py, the native-cadence raw archive).

- `[~]` **Bulk-state estimator in the controller** (structural, high confidence).
  The room sensors read a fast air node, not the bulk mass (task 009): they
  over-read progress while running (lead_h*q) and sag ~2 K/h for ~10 min
  after every stop — sensor relaxation, not real heat loss. The controller
  reacts to both artifacts: post-stop sags can retrigger runs early
  (cycling), and in-run leads cause premature satisfaction. Inverting the
  (already-fitted) lead model inside actrl — control on estimated bulk
  temperature instead of raw sensor — should cut cycling and overshoot in
  one move. The sim can A/B this exactly since it models both nodes.
  Oracle A/B done 2026-07-05 (`analysis/oracle_bulk_estimator.py`, findings
  in docs/tuning.md): directionally confirmed but reframed — the real win
  is cold-time (deg_min_below −58%) and cycling (−31% osc, −1–2 starts/day),
  NOT overshoot (above-band discomfort is already 0 in heating; the sensor
  lead was masking chronic under-heating). Costs +9% energy = the
  previously-missing heat, so it's a comfort/energy rebalance not a free
  lunch. Open: (a) a *realizable* observer — literal `Tm − lead·q` fixes
  only the in-run lead, the post-stop sag needs `Tm + tau_meas·dṪm − lead·q`
  with a q proxy; (b) an equal-comfort comparison to isolate the pure
  cycling/energy benefit.

- **Thermal-mass arbitrage / heat banking** (energy, medium-high).
  COP moves ~5%/K of outdoor temp; June days swing 5-10 K; house tau ~30 h,
  so heat stored at 15:00 is substantially still there at 21:00. Hypothesis:
  shifting the evening comfort load into the warm/solar afternoon (pre-heat
  1-2 K above target on a decaying schedule) is cheaper per delivered
  degree-hour than heating at 18:00-21:00, even before PV self-consumption
  credit. preheat.py generalises directly (evening deadline instead of
  morning). Connects to the existing grid-surplus target widening.

- **Mild-day timing freedom is a feature** (energy/PV, medium-high).
  The mild-day "run-decision degeneracy" (sim -41% energy, same comfort)
  isn't a model bug to fix — it says on mild days run *timing* is nearly
  free. That's exactly when PV surplus exists. A statctrl rule "on
  forecast-mild days, bias all discretionary running into the PV window"
  costs ~zero comfort by construction. The sim can bound the comfort cost.

- **Duct losses may be the biggest efficiency lever, and they're
  measurable** (physical, medium). The closed-loop energy refit scaled
  heat-per-kWh by 0.80; some of that is fit error, but if even half is real
  duct/roof-space loss, ~10-20% of every heating kWh is warming the roof.
  The m5atom coil temperature entities (inside_coil_inlet, inside_temp) +
  fan state could estimate coil-side output vs the fitted room-side
  delivery on the same runs, splitting COP error from duct loss. If ducts
  are leaking heat, insulation beats any control change ever tested here.

- **Schedule-aware gain scheduling** (tuning, medium). The deadband_ki x4
  comfort win concentrates in transients (warmups/tapers); its noise
  sensitivity concentrates in steady state (oscillation tax appears >~0.03 K
  noise). Hypothesis: high ki only while |weighted_error| or the schedule
  ramp is active, standard ki in steady state — transient benefits without
  the steady-state risk margin. Trivial to express in control.py, easy to
  sweep in tune.py.

- **Stagger the morning room deadlines** (energy/comfort, medium-low).
  All rooms currently warm up together into the coldest hour (worst COP +
  defrost accumulation + peak load forces high compressor speeds). One
  shared compressor means simultaneous deadlines fight each other.
  Staggering (e.g. bedrooms first, kitchen 30-45 min later, or by room
  time-constant) flattens demand into the min-power regime the unit is
  demonstrably most efficient in. preheat.py + per-room synth schedules can
  test this now.

- **Defrost-aware run shaping** (energy, low-medium). Defrost triggers on
  accumulated cold-running time (~145 min below 7.5 C) and purges every
  ~90 min at low speed. On marginal cold mornings, deliberately splitting
  the warmup into blocks that reset the accumulators (or timing the ramp so
  defrost lands before the comfort deadline, not at it) might avoid paying
  defrost during the critical window. The sim models both mechanisms;
  wholly untested.

- **Controller CI via the sim** (workflow, low effort/high leverage).
  Every future actrl/control change gets replayed over 3 canonical days
  (cold 06-22, overcast 06-30, mild 06-09) with comfort metrics diffed
  against the standing baseline before deploy — the tuning harness already
  does everything except run in CI. Catches "the new logic cycles the
  compressor 2x" regressions for ~10 min of compute.

- **Summer flips the solar geometry** (calibration, seasonal note). The NE
  morning beam that *helps* winter heating becomes a cooling load on summer
  mornings (bed_2/bed_3 first), and the currently-zero NW terms will
  activate on summer evenings (sun sets further south, lower, into the
  glazing). Cooling calibration should expect the bias pattern to invert by
  orientation and time of day; bed_2's neighbour-wall shading prediction
  (solar r2 should rise in summer) doubles as a model check.
