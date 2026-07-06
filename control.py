# Pure (HA-independent) control logic shared by AppDaemon apps and tests.

from collections import deque
from datetime import datetime

# per interval
# 0.2C per minute
# double the rate of statctrl.py, otherwise they cancel each other until the target is reached
grid_surplus_open_window_rate = 0.2 * (10.0 / 60.0)

# in minutes
interval = 10.0 / 60.0  # 10 seconds

# per second
global_deadband_ki = 0.0125
# a 0.1 deg error will accumulate 1 in ~13.33 minutes

# soft start to avoid overshoot
# start at min power and gradually report the actual rval
# 5 min delay at min power
# 7.5 min to be safe
# covers a defrost cycle
soft_delay = int(7.5 / interval)
# then gradually report the actual rval over 2.5 mins
soft_ramp = int(2.5 / interval)

# wait for 5 minutes of soft start before dropping to absolute minimum power
minimum_temp_intervals = int(5 / interval)

# Saturate after 14 power increments (slight over-estimate, appears to be closer to 13)
compressor_power_increments = 14

# Additional safety margin when switching between stepping up / down
compressor_power_safety_margin = 2

# over 45 mins immediate_off_threshold will ramp to eventual_off_threshold, and reset after 90 min purge delay
min_power_delay = int(45 / interval)

# Every 90 mins at low power it runs at full speed for about a minute.
# Will be out of sync if we ramp down to min power after running at higher power
# for a while, not a big deal. Mainly focused on marginal operation where
# demanded power is slightly below minimum output.
purge_delay = int(90 / interval)

# setpoint - it takes about 7.5 (sometimes longer) to bring a/c to min power
# don't hold it there forever as it'll shut down after 1hr at this temp
# 45 mins should be safe
min_power_time = int(45 / interval)

# when the error is greater than 2.0C
# let the Midea controller do its thing
faithful_threshold = 2.0
desired_on_threshold = 0.0

# After reducing to minimum power and holding for a while, turn off at a tighter threshold
eventual_off_threshold = -0.5

# Give up on incremental control and cut to minimum power to avoid overshooting
# This includes the derivative
# min_power_threshold = -1.25
min_power_threshold = -2.0

# Worst case, turn off if we have overshot massively.
immediate_off_threshold = -1.5

# Offsets for celsius
# 1 is usually sufficient
ac_on_threshold = 1

ac_stable_threshold = 1
ac_off_threshold = -2


class MyWMA:
    def __init__(self, window):
        self.window = window
        self.clear()

    def clear(self):
        self.history = deque([])
        self.pad()

    def pad(self):
        while len(self.history) < self.window:
            self.history.appendleft(0.0)
        while len(self.history) > self.window:
            self.history.popleft()

    def set(self, value):
        self.history.append(value)
        self.pad()

    def get(self):
        i = 0
        i_sum = 0
        val_sum = 0.0

        for val in self.history:
            i += 1
            i_sum += i
            val_sum += val * i

        return val_sum / i_sum


# ignores steps due to changed target
class MyDeriv:
    def __init__(self, window, factor):
        self.wma = MyWMA(window=window)
        self.factor = factor
        self.clear()

    def clear(self):
        self.wma.clear()
        self.prev_error = None
        self.prev_target = None

    def set(self, error, target):
        if self.prev_error is None:
            self.prev_error = error
        if self.prev_target is None:
            self.prev_target = target

        error_delta = error - self.prev_error
        target_delta = target - self.prev_target

        # compensate for changes due to target
        actual_delta = error_delta + target_delta

        self.wma.set(actual_delta)
        self.prev_error = error
        self.prev_target = target

    def get(self):
        return self.factor * self.wma.get()


class MyPID:
    def __init__(self, kp, ki, kd, window):
        self.kp = kp
        self.ki = ki
        self.deriv = MyDeriv(window=window, factor=kd)
        self.clear()

    def clear(self):
        """Reset controller state."""
        self.deriv.clear()
        self.p_term = 0.0
        self.i_term = 0.0

    def update(self, error, setpoint):
        """Update PID computation with new error and setpoint values."""
        self.p_term = error * self.kp
        self.i_term += error * self.ki
        self.deriv.set(error, setpoint)

    def get_output(self):
        """Get raw PID output."""
        return self.p_term + self.i_term + self.deriv.get()

    def set_integral(self, value):
        """Directly set the integral term."""
        self.i_term = value

    def adjust_integral(self, adjustment):
        """Apply an adjustment to the integral term."""
        self.i_term += adjustment


class DeadbandIntegrator:
    def __init__(self, ki):
        self.ki = ki
        self.clear()

    def clear(self):
        self.integral = 0.0
        self.increment_count = 0

    def set(self, error):
        if error > 0 and self.increment_count <= 0:
            self.integral = max(0.75, self.integral)
            self.increment_count = 1

        if error < 0 and self.increment_count >= 0:
            self.integral = min(-0.75, self.integral)
            self.increment_count = -1

        self.integral += error * self.ki

        rval = 0

        if self.integral > 1:
            self.integral = min(1, self.integral - 2)
            rval = 1
        elif self.integral < -1:
            self.integral = max(-1, self.integral + 2)
            rval = -1

        # print(
        #    f"input: {error}, integral: {self.integral}, increment_count: {self.increment_count} rval: {rval}"
        # )
        return rval

    def get(self):
        return self.integral


class WindowStateHandler:
    def __init__(self, accumulation_rate=grid_surplus_open_window_rate):
        self.window_offsets = {}
        self.accumulation_rate = accumulation_rate

    def update(self, room, window_open):
        if room not in self.window_offsets:
            self.window_offsets[room] = 0

        if window_open:
            self.window_offsets[room] += self.accumulation_rate
        else:
            self.window_offsets[room] = 0

    def get_offset(self, room):
        return self.window_offsets.get(room, 0)


class MideaCapacityController:
    def __init__(self, log=lambda msg: None):
        self.log = log
        self.on_counter = 0
        self.min_power_counter = 0
        self.max_power_counter = 0
        self.prev_step = 0
        self.guesstimated_comp_speed = 0
        self.compressor_totally_off = True
        self.prev_unsigned_compressed_error = 0
        self.deadband_integrator = DeadbandIntegrator(
            ki=(global_deadband_ki * 60.0 * interval)
        )

    def compress(self, error, deriv):
        # Tighten the deadband with runtime, with the goal of turning off
        # before the high power 'purge' that occurs after 90 mins of continuous
        # operation at low speed. This 'purge' often pushes us out of the
        # deadband anyway, so it's more efficient to just turn off prior.
        # Reset in sync with the purge period. Desync isn't a big deal.
        wrapped_on_counter = self.min_power_counter % purge_delay
        min_power_progress = min(1.0, wrapped_on_counter / min_power_delay)

        if self.guesstimated_comp_speed <= 0 and error <= (
            immediate_off_threshold * (1 - min_power_progress)
            + eventual_off_threshold * min_power_progress
        ):
            self.compressor_totally_off = True

        if self.guesstimated_comp_speed > 0 and error <= immediate_off_threshold:
            self.compressor_totally_off = True

        if self.compressor_totally_off:
            self.on_counter = 0
            self.min_power_counter = 0
            self.max_power_counter = 0

            if error < desired_on_threshold:
                # sometimes -2 isn't the true off_threshold?!
                # shut things down more decisively
                return self.midea_reset_quirks(ac_off_threshold - 1)
            else:
                self.compressor_totally_off = False
                self.deadband_integrator.clear()
                self.log(f"starting compressor {ac_on_threshold}")

        # "blip" the power to get AC to start
        if self.on_counter < 1:
            return self.midea_reset_quirks(ac_on_threshold)

        # conditions in which to consider the derivative
        # - aircon is currently running
        # - the current temp (ignoring RoC) has not yet reached off threshold
        # goal of the derivative is to proactively reduce/increase compressor power, but not to influence on/off state
        error = error + deriv

        if error > faithful_threshold:
            # Bypass soft start for big errors
            self.on_counter = max(self.on_counter, soft_delay)

            self.deadband_integrator.clear()
            return self.midea_runtime_quirks(
                ac_stable_threshold + 2 + error - faithful_threshold
            )

        if error <= min_power_threshold:
            self.deadband_integrator.clear()
            return self.midea_runtime_quirks(ac_off_threshold + 1)

        if self.on_counter < soft_delay:
            self.log("soft start, on_counter: " + str(self.on_counter))
            self.deadband_integrator.clear()
            return self.midea_runtime_quirks(ac_stable_threshold - 1)

        return self.midea_runtime_quirks(
            ac_stable_threshold + self.deadband_integrator.set(error)
        )

    def midea_reset_quirks(self, rval):
        self.guesstimated_comp_speed = 0
        self.prev_step = 0
        return rval

    def midea_runtime_quirks(self, rval):
        rval = round(rval)

        # Process these early so that prev_step is restored to 0 and doesn't give rise
        # to weird edge cases.

        # Midea controller seems to have a "NEAR_TARGET_RAMP_DOWN" flag that is set when
        # the sensed temperature is equal or overshooting the setpoint. and un-set when
        # the sensed temperature is 1C or greater from satisfying the setpoint.
        # It also seems to have a "FAR_FROM_TARGET_RAMP_UP" flag that is set when sensed
        # is more than 3C from the setpoint (2C from stable), and un-set (when???)
        # Both sequences of step-up and step-down return values are crafted to avoid leaving
        # this flag set by returning a positive value before returning to the "stable" value,
        # otherwise it will not be possible to maintain equilibrium with stable compressor
        # speed.

        # Sequence to increment speed by +1 with final value = 0 and no flags set.
        # Rules:
        # - Each value change increases/decreases speed by 1
        # - value >= 2 sets "Ramp up" flag
        # - don't know how to clear the "Ramp up" flag!
        # - value <= -1 sets "Ramp down" flag
        # - value >= 1 clears "Ramp down" flag
        step_up_sequence = [1, 2, 0]
        if self.prev_step > 0:
            rval = ac_stable_threshold + step_up_sequence[self.prev_step]
            self.prev_step += 1
            if self.prev_step >= len(step_up_sequence):
                self.prev_step = 0
            return rval

        # Sequence to decrement speed by -1, jumping up to +1 to un-set the internal ramp down flag?
        # For cooling mode, perhaps a simpler sequence of [-1, 1, 0] will be needed to get a single decrement
        step_down_sequence = [-1, -2, 1, 0]
        if self.prev_step < 0:
            rval = ac_stable_threshold + step_down_sequence[-self.prev_step]
            self.prev_step -= 1
            if -self.prev_step >= len(step_down_sequence):
                self.prev_step = 0
            return rval

        # Begin step up sequence, unless already at max power
        # FIX: Added 'and self.max_power_counter == 0'
        # This prevents the stepping logic from intercepting execution when we
        # should be locked in the high-priority "Max Power" hysteresis loop.
        if (
            rval == ac_stable_threshold + 1
            and self.max_power_counter == 0
            and self.guesstimated_comp_speed
            < (compressor_power_increments + compressor_power_safety_margin)
        ):
            self.guesstimated_comp_speed = max(
                compressor_power_safety_margin, self.guesstimated_comp_speed + 1
            )
            # Don't perform the increment sequence if jumping to max power
            if self.guesstimated_comp_speed < (
                compressor_power_increments + compressor_power_safety_margin
            ):
                self.prev_step = 1
                return ac_stable_threshold + step_up_sequence[0]

        # Begin step down sequence, unless already at min power
        if rval == ac_stable_threshold - 1 and self.guesstimated_comp_speed > 0:
            self.max_power_counter = 0
            self.guesstimated_comp_speed = min(
                compressor_power_increments,
                self.guesstimated_comp_speed - 1,
            )
            # Don't perform the decrement sequence if dropping to min power
            if self.guesstimated_comp_speed > 0:
                self.prev_step = -1
                return ac_stable_threshold + step_down_sequence[0]

        # Bypass the stepping behaviour for extreme errors above faithful_threshold
        # in favor of simple hysteresis
        # Entry: rval >= 3 (Stable + 2)
        # Stay:  rval >= 1 (Stable + 0) -> Catches rval=1 (Integrator Reset) and rval=2
        threshold_offset = 0 if self.max_power_counter > 0 else 2

        if rval >= ac_stable_threshold + threshold_offset:

            self.log(
                f"Hysteresis Active. rval: {rval}, counter: {self.max_power_counter}"
            )

            # Assume that there is demand for max power (plus lower and upper safety margin)
            self.guesstimated_comp_speed = (
                compressor_power_increments + compressor_power_safety_margin
            )
            self.max_power_counter += 1

            # FIX: Stronger Output Latching
            # Instead of just adding 1 (which turns rval=1 into 2),
            # we latch to the previous value to keep the output stable at 3.
            # We only allow the value to rise, not fall, while in this mode.
            return max(rval, self.prev_unsigned_compressed_error)

        # Saturated, just keep demanding a compressor speed increase
        if rval >= ac_stable_threshold and (
            self.guesstimated_comp_speed
            >= compressor_power_increments + compressor_power_safety_margin
        ):
            return ac_stable_threshold + 1
        else:
            self.max_power_counter = 0

        # Any larger offset should jump to minimum power
        if rval < ac_stable_threshold - 1:
            self.guesstimated_comp_speed = 0

        # Saturated, demand absolute minimum power
        if self.guesstimated_comp_speed <= 0:
            # Be more conservative for the first 5 minutes after startup to avoid stopping the compressor
            if self.on_counter < minimum_temp_intervals:
                rval = min(ac_stable_threshold - 1, rval)
            else:
                rval = min(ac_off_threshold + 1, rval)

        if rval < ac_stable_threshold:
            self.min_power_counter += 1
            if self.min_power_counter % min_power_time == (min_power_time - 1):
                # 'blip' the feels like temp to reset the AC's internal timer
                # and prevent the system from shutting down completely
                self.prev_unsigned_compressed_error = ac_stable_threshold + 1

            # aircon seems to react to edges
            # so provide as many as possible to quickly reduce power?
            rval = max(rval, self.prev_unsigned_compressed_error - 1)

        else:
            self.min_power_counter = 0

        return rval


# ---------------------------------------------------------------------------
# Price-aware control (docs/pricing.md). Pure logic; the AppDaemon apps feed
# in prices read from HA entities and act on the returned numbers.
# ---------------------------------------------------------------------------

# Comfort-vs-money exchange rate [K per $/kWh]: the ONLY preference knob of
# the continuous offset. Swept in docs/pricing.md "Continuous price offset":
# k=2 was comfort-parity cheaper on elevated days, k=4 traded comfort.
price_offset_k = 2.0

# Banked-heat retention [h]: holding a banked degree leaks at the house's
# free-running loss rate UA/C_eff ~ 5%/h, so heat banked h hours early is
# worth exp(-h/tau) of face value. Physics, not preference; NOT the ~2.5 h
# room-temperature decay constant (that mistake made banking look worthless
# -- docs/pricing.md "retention semantics").
price_retention_tau_h = 20.0

# How far ahead the offset looks [h]: matches the useful predispatch skill
# horizon studied in Phase B.
price_horizon_h = 8.0

# Offset clamps [K]: bank = pre-heat above target, shave = sag below.
# Asymmetric because comfort risk is asymmetric (winter).
price_bank_max_k = 1.5
price_shave_max_k = 0.75

# Forecast-price calibration knots (analysis/fc_calibration.py, fitted on
# ALL winter data to 2026-07-06, months 5-8, ~3 h lead -- the
# "insurance-premium" fit, docs/pricing.md "Phase C fork"). All in retail
# import $/kWh. e_actual = E[actual | forecast] (restores the fat-tail
# value that predispatch magnitude understates); upside = E[(actual-fc)+]
# (the insurance term for the warmup chooser's g_lambda). Refresh
# monthly-ish as data accumulates by re-running fc_calibration.py.
fc_knots_retail = [
    0.0967, 0.1699, 0.2261, 0.2752, 0.3423, 0.4790, 0.6393, 0.8964, 5.9483,
]
fc_knots_e_actual = [
    0.1138, 0.1821, 0.2377, 0.2851, 0.3499, 0.4945, 0.6364, 0.8485, 2.9321,
]
fc_knots_upside = [
    0.0236, 0.0190, 0.0237, 0.0241, 0.0286, 0.0631, 0.0415, 0.1041, 0.3369,
]

# Warmup-start chooser constants (docs/pricing.md "Warmup-start chooser").
# House/HVAC numbers come from docs/calibration.md; they only steer WHEN the
# morning warmup buys its energy, not any temperature the house is driven to.
warmup_ua_kw_per_k = 0.160  # envelope loss
warmup_c_eff_kwh_per_k = 4.8  # effective thermal capacity
warmup_p_max_kw = 3.055  # electrical draw at max compressor increment
# Fitted heating efficiency proxy e(P_kW, Tout) [K/h per kW] from
# sim/hvac.py (open-loop fit x 0.80 closed-loop energy refit).
warmup_eff_e0 = 1.045 * 0.80
warmup_eff_per_kw = -0.101 * 0.80
warmup_eff_per_k = 0.0522 * 0.80
warmup_eff_floor = 0.2
# Maintenance-cost fudge vs the full sim (single-point calibration on the
# mild 06-09 morning): absorbs cycling overhead the increment model misses.
warmup_hold_mult = 2.0
# Free-floating house sag overnight [K/h]: the hold counterfactual.
warmup_sag_k_per_h = 0.25
# Insurance knob lambda for g_lambda(fc) = fc + lambda * upside(fc).
# Moot 0-4 on June decisions once hold cost was fixed; 1 = face value plus
# one expected upside surprise.
warmup_lambda = 1.0


def forecast_hours_ahead(records, value_key, now_utc, date_key="date"):
    """[(hours_from_now, price)] from an EMHASS-style forecast attribute.

    records: the `unit_load_cost_forecasts` attribute of an EMHASS-published
    price entity (sensor.dh_unit_load_cost etc.) -- a list of dicts with an
    ISO `date` and the price under the entity's own suffix (the same
    convention Ryan's HWC planner consumes, so this follows whatever price
    source EMHASS is currently configured with, already tariffed).
    """
    out = []
    for item in records:
        ts = datetime.fromisoformat(str(item[date_key]).replace("Z", "+00:00"))
        h = (ts - now_utc).total_seconds() / 3600.0
        out.append((h, float(item[value_key])))
    return out


def interp_knots(x, xs, ys):
    """Piecewise-linear interpolation, clamped at both ends (np.interp)."""
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            frac = (x - xs[i - 1]) / (xs[i] - xs[i - 1])
            return ys[i - 1] + frac * (ys[i] - ys[i - 1])
    return ys[-1]


def price_pressure_offset(
    now_price,
    future_prices,
    k=price_offset_k,
    tau_h=price_retention_tau_h,
    horizon_h=price_horizon_h,
    bank_max=price_bank_max_k,
    shave_max=price_shave_max_k,
):
    """Continuous price-pressure target offset [K] (docs/pricing.md).

        offset = k * (max_h E[actual|fc](t+h) * exp(-h/tau) - p_now)

    clamped to [-shave_max, +bank_max]. Positive = bank (pre-heat/pre-cool:
    the future is dearer than now, discounted by retention losses);
    negative = shave (now is the expensive hour). Flat prices give ~0 (the
    retention discount and the mild E[actual|fc] uplift nearly cancel).

    now_price: retail import price now [$ / kWh].
    future_prices: iterable of (hours_ahead, forecast_retail_price); entries
    outside (0, horizon_h] are ignored. Forecasts are valued at their
    conditional mean actual (fc_knots) -- the fat-tail restoration that made
    the offset act on spike mornings in the study.

    Returns 0.0 when there is no usable future data (graceful: no forecast,
    no action).
    """
    best = None
    e = 2.718281828459045
    for h, fc in future_prices:
        if h <= 0 or h > horizon_h:
            continue
        value = interp_knots(fc, fc_knots_retail, fc_knots_e_actual) * (
            e ** (-h / tau_h)
        )
        if best is None or value > best:
            best = value
    if best is None:
        return 0.0
    return min(bank_max, max(-shave_max, k * (best - now_price)))


def _warmup_efficiency(p_kw, t_out):
    e = (
        warmup_eff_e0
        + warmup_eff_per_kw * (p_kw - 0.7)
        + warmup_eff_per_k * (t_out - 10.0)
    )
    return max(warmup_eff_floor, e)


def warmup_expected_costs(
    candidates,
    price_at,
    t_out,
    deadline_h,
    t_bulk0,
    t_day,
    hold_mult=warmup_hold_mult,
):
    """Expected cost [$] of each candidate warmup start (docs/pricing.md).

    All times are hours from now (the decision instant). For each candidate
    start the house warms at full capacity until it reaches t_day, then pays
    MARGINAL maintenance until deadline_h + 2 -- marginal meaning the
    increment above the counterfactual free-floating house (which sags at
    warmup_sag_k_per_h and would be heated from the deadline anyway); the
    baseline warmup energy is bought regardless, only WHEN differs. A
    candidate that misses t_day by deadline_h is infeasible (inf).

    price_at(h) -> forecast retail price [$ / kWh] at h hours from now,
    already g_lambda-adjusted via warmup_glambda. t_out: outdoor temp [C],
    held constant over the horizon (pre-dawn winter is flat enough for a
    WHEN decision).

    Ported from the validated analysis/warmup_shift.py coarse_costs.
    """
    p_max = warmup_p_max_kw
    c_eff = warmup_c_eff_kwh_per_k

    def warm_rate(temp):
        # efficiency() * P is already the delivered house rate [K/h]
        return (
            _warmup_efficiency(p_max, t_out) * p_max
            - warmup_ua_kw_per_k * (temp - t_out) / c_eff
        )

    def hold_power(temp, t_h):
        t_float = t_bulk0 - warmup_sag_k_per_h * max(t_h, 0.0)
        q_rate = warmup_ua_kw_per_k * max(temp - t_float, 0.0) / c_eff
        p = q_rate / max(_warmup_efficiency(1.0, t_out), 0.1)
        return hold_mult * q_rate / max(_warmup_efficiency(p, t_out), 0.1)

    costs = {}
    for t_s in candidates:
        cost, t, temp = 0.0, float(t_s), t_bulk0
        while t < deadline_h + 2.0:  # hold a little past deadline
            if temp < t_day:  # warm phase at capacity
                p_kw = p_max
                temp += max(warm_rate(temp), 0.05) * 0.5
            else:  # hold phase: maintenance
                p_kw = hold_power(temp, t)
            cost += p_kw * 0.5 * price_at(t)
            t += 0.5
            if t >= deadline_h and temp < t_day:
                # did not reach the day target by the deadline: infeasible
                cost = float("inf")
                break
        costs[float(t_s)] = cost
    return costs


def warmup_glambda(fc_retail, lam=warmup_lambda):
    """Insurance-priced forecast value: fc + lam * E[(actual - fc)+ | fc]."""
    return fc_retail + lam * interp_knots(
        fc_retail, fc_knots_retail, fc_knots_upside
    )


def warmup_decide(
    future_prices,
    t_out,
    deadline_h,
    t_bulk0,
    t_day,
    lam=warmup_lambda,
    hold_mult=warmup_hold_mult,
):
    """(start_h, jit_h, shift_h) for the upcoming warmup, hours from now.

    future_prices: iterable of (hours_ahead, forecast_retail_price) on a
    ~30-min grid covering [now, deadline_h + 2]. deadline_h: hours until
    the schedule needs the day target t_day reached, from t_bulk0 now.
    Candidates run every 30 min from max(0.5, deadline_h - 8) to
    deadline_h - 0.5 (just-in-time). Returns None when no candidate is
    feasible or there is no usable price data (caller falls back to
    just-in-time behaviour).

    shift_h = jit_h - start_h is how much earlier than just-in-time to
    begin; 0 on flat mornings (the leak penalty makes just-in-time
    cheapest), > 0 only when the forecast spread pays for the leak.
    """
    grid = sorted(
        (float(h), warmup_glambda(float(p), lam)) for h, p in future_prices
    )
    if not grid or deadline_h < 1.0:
        return None

    def price_at(h):
        # step (ffill) lookup, clamped to the grid ends
        best = grid[0][1]
        for gh, gp in grid:
            if gh <= h:
                best = gp
            else:
                break
        return best

    candidates = []
    t_s = max(0.5, deadline_h - 8.0)
    while t_s < deadline_h - 0.49:
        candidates.append(round(t_s, 6))
        t_s += 0.5
    if not candidates:
        return None
    costs = warmup_expected_costs(
        candidates, price_at, t_out, deadline_h, t_bulk0, t_day,
        hold_mult=hold_mult,
    )
    feasible = [ts for ts, c in costs.items() if c != float("inf")]
    if not feasible:
        return None
    t_star = min(feasible, key=lambda ts: costs[ts])
    t_jit = max(feasible)
    return t_star, t_jit, max(0.0, t_jit - t_star)
