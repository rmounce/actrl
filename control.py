# Pure (HA-independent) control logic shared by AppDaemon apps and tests.

from collections import deque

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
