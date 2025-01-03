import hassapi as hass
import datetime
import math

# from simple_pid import PID
from collections import deque
import time

rooms = ["bed_1", "bed_2", "bed_3", "kitchen", "study"]

# in minutes
interval = 10.0 / 60.0  # 10 seconds

# WMA over the last 5 minutes
global_temp_deriv_window = 5
# to predict 10 minutes into the future
global_temp_deriv_factor = 10.0

# per second
global_ki = 0.00025
# a 0.1 deg error will accumulate 0.1 in ~60 minutes

# per second
global_deadband_ki = 0.0125
# a 0.1 deg error will accumulate 1 in ~13.33 minutes

# in seconds, time for aircon to ramp up power 1 increment & stay there
step_up_time = 200
step_up_intervals = step_up_time / (60.0 * interval)
# shorter as it latches instantly
# 120s sufficient in cooling mode to step down?
# previously an extra 30s was added to make soft start more reliable
step_down_time = 120
step_down_intervals = step_down_time / (60.0 * interval)

# seems to be 6 minutes / 360 seconds in heating mode!?!?
# round up to 390 seconds to be really sure
heat_step_down_time = 390
heat_step_down_intervals = heat_step_down_time / (60.0 * interval)

# If the top 100% zone has its door closed, open at least 50% in a second zone
# sqrt stuff going on due to damper value scaling
# approx 1.29
min_airflow_with_closed_door = 2 - math.sqrt(2) / 2

# swing full scale across 2.0C of error
normalised_damper_range = 2.0

# leave this as 1 so that all PID values are in degrees celsius
room_kp = 1.0

# per second
room_ki = 0.001
# a 0.1 deg error will accumulate 0.1 in ~15 minutes

# Allow a small negative integral to accumulate to keep an over-satisfied room's
# PID output below 0 to avoid noise pushing it back up into "control authority".
# In the old PID implementation this value was effectively -0.2 (clamp_high=1.2)
# This won't prevent a PID from going lower than this value by P & D terms, only I.
room_pid_minimum = -0.1

# kd considers the last 10 minutes
room_deriv_window = 10.0
# looking 2 minutes into the future
room_deriv_factor = 2.0

# percent
damper_deadband = 7.5
damper_round = 5

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

# Saturate after 15 power increments (note: a guess / slight over-estimate, the number of increments hasn't been measured)
compressor_power_increments = 15

# over 45 mins desired_off_threshold will ramp to min_power_threshold, and reset after 90 min purge delay
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

# in cooling mode, how long to keep blowing the fan
off_fan_running_time = int(2.5 / interval)

# step to within 0.1C of target on large adjustments
target_ramp_step_threshold = 0.1
# 100% per minute above threshold
target_ramp_proportional = 1.0 * interval
# linear below 0.1C
target_ramp_linear_threshold = 0.1
target_ramp_linear_increment = target_ramp_proportional * target_ramp_linear_threshold


# when the error is greater than 2.0C
# let the Midea controller do its thing
faithful_threshold = 2.0
desired_on_threshold = 0.0

# Worst case, turn off if we have overshot massively.
desired_off_threshold = -1.5
# After reducing to minimum power and holding for a while, turn off at a tighter threshold
min_power_threshold = -0.5

# try using FREEDOM UNITS
ac_celsius = True

# Midea constants depending on temp units
if ac_celsius:
    ac_unit_from_celsius = 1.0
    ac_on_threshold = 1
    ac_stable_threshold = 1
    ac_off_threshold = -2
else:
    # Fahrenheit
    ac_unit_from_celsius = 1.8
    # https://github.com/esphome/feature-requests/issues/1627#issuecomment-1365639966
    ac_on_threshold = 3
    ac_stable_threshold = 2
    ac_off_threshold = -3

null_state = "unknown"


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
            # self.increment_count = max(1, self.increment_count)
            rval = 1
        elif self.integral < -1:
            self.integral = max(-1, self.integral + 2)
            # self.increment_count = min(-1, self.increment_count)
            rval = -1

        # self.increment_count += rval
        # lazy heuristic to avoid overshoot due to time delay
        # if self.increment_count == 3:
        #    rval = 0

        print(
            f"input: {error}, integral: {self.integral}, increment_count: {self.increment_count} rval: {rval}"
        )
        return rval

    def get(self):
        return self.integral


class Actrl(hass.Hass):
    def initialize(self):
        self.log("INITIALISING")
        self.pids = {}
        self.temp_derivs = {}
        self.targets = {"heat": {}, "cool": {}}
        self.rooms_enabled = {}
        self.damper_pos = {}
        self.prev_unsigned_compressed_error = 0
        self.min_power_counter = 0
        self.off_fan_running_counter = 0
        self.outer_ramp_count = 0
        self.outer_ramp_rval = 1
        self.guesstimated_comp_speed = 0
        if self.get_state("climate.aircon") in ["heat", "cool"]:
            self.mode = self.get_state("climate.aircon")
        else:
            self.mode = "off"

        if self.get_state("input_boolean.ac_already_on_bypass") == "on":
            self.log("ASSUMING THAT THE AIRCON IS ALREADY RUNNING")
            self.compressor_totally_off = False
            self.on_counter = soft_delay + soft_ramp
        else:
            self.compressor_totally_off = True
            self.on_counter = 0

        self.deadband_integrator = DeadbandIntegrator(
            ki=(global_deadband_ki * 60.0 * interval),
        )

        for room in rooms:
            self.pids[room] = MyPID(
                kp=room_kp,
                ki=(room_ki * 60.0 * interval),
                kd=room_deriv_factor / interval,
                window=int(room_deriv_window / interval),
            )
            self.temp_derivs[room] = MyDeriv(
                window=int(global_temp_deriv_window / interval),
                factor=global_temp_deriv_factor / interval,
            )
            self.rooms_enabled[room] = False
            self.damper_pos[room] = float(
                self.get_entity("cover." + room).get_state("current_position")
            )
        # run every interval (in minutes)
        self.run_every(self.main, "now", 60.0 * interval)

    def main(self, kwargs):
        self.log("")
        self.log("#### BEGIN CYCLE ####")
        prev_temps = {}
        temps = {}
        errors = {"heat": {}, "cool": {}}
        damper_vals = {}
        pid_vals = {}
        heat_cool_sign = 1.0
        cur_targets = {"heat": {}, "cool": {}}

        celsius_setpoint = float(
            self.get_entity("climate.aircon").get_state("temperature")
        )

        for room in rooms:
            if self.get_state("input_boolean.ac_use_feels_like") == "on":
                temps[room] = float(self.get_state("sensor." + room + "_feels_like"))
            else:
                temps[room] = float(
                    self.get_state("sensor." + room + "_average_temperature")
                )

            self.temp_derivs[room].set(temps[room], 0)

            if self.get_state("climate." + room + "_aircon") == "heat_cool":
                cur_targets["heat"][room] = self.get_entity(
                    "climate." + room + "_aircon"
                ).get_state("target_temp_low")
                cur_targets["cool"][room] = self.get_entity(
                    "climate." + room + "_aircon"
                ).get_state("target_temp_high")
            elif self.get_state("climate." + room + "_aircon") == "heat":
                cur_targets["heat"][room] = self.get_entity(
                    "climate." + room + "_aircon"
                ).get_state("temperature")
            elif self.get_state("climate." + room + "_aircon") == "cool":
                cur_targets["cool"][room] = self.get_entity(
                    "climate." + room + "_aircon"
                ).get_state("temperature")

            for mode in ["heat", "cool"]:
                if room in cur_targets[mode]:
                    if room in self.targets[mode]:
                        target_delta = (
                            cur_targets[mode][room] - self.targets[mode][room]
                        )

                        # self.log(f"about to ramp target room: {room}, target_delta: {target_delta}, smooth_target: {self.targets[mode][room]}, ultimate target: {cur_targets[mode][room]}")

                        if abs(target_delta) <= target_ramp_linear_increment:
                            self.targets[mode][room] = cur_targets[mode][room]
                        # Floating point fun!
                        elif abs(target_delta) <= (target_ramp_linear_threshold + 1e-9):
                            self.targets[mode][room] += math.copysign(
                                target_ramp_linear_increment, target_delta
                            )
                            self.log(
                                f"linearly ramping target room: {room}, smooth target: {str(self.targets[mode][room])}, ultimate target: {str(cur_targets[mode][room])}"
                            )
                        elif abs(target_delta) <= (target_ramp_step_threshold + 1e-9):
                            self.targets[mode][room] += (
                                target_delta * target_ramp_proportional
                            )
                            self.log(
                                f"proportionally ramping target room: {room}, smooth target:{str(self.targets[mode][room])}, ultimate target: {str(cur_targets[mode][room])}"
                            )
                        else:
                            self.targets[mode][room] = cur_targets[mode][
                                room
                            ] - math.copysign(target_ramp_step_threshold, target_delta)
                            self.log(
                                f"stepping target room: {room}, smooth target:{str(self.targets[mode][room])}, ultimate target: {str(cur_targets[mode][room])}"
                            )
                    else:
                        self.log(
                            f"setting {mode} target for previously disabled room {room}"
                        )
                        self.targets[mode][room] = cur_targets[mode][room]
                    errors[mode][room] = temps[room] - self.targets[mode][room]
                else:
                    if room in self.targets[mode]:
                        self.targets[mode].pop(room)

        # First run, no previous temps
        if not prev_temps:
            prev_temps = temps

        cooling_demand = max(errors["cool"].values(), default=float("-inf"))
        heating_demand = -min(errors["heat"].values(), default=float("inf"))

        self.log(f"heating_demand: {heating_demand}, cooling_demand: {cooling_demand}")

        new_mode = None

        if self.get_state("climate.aircon") == "cool" and cooling_demand > (
            heating_demand + desired_off_threshold
        ):
            new_mode = "cool"
        elif self.get_state("climate.aircon") == "heat" and heating_demand > (
            cooling_demand + desired_off_threshold
        ):
            new_mode = "heat"
        elif cooling_demand > heating_demand:
            new_mode = "cool"
        elif heating_demand > cooling_demand:
            new_mode = "heat"

        self.log(f"new_mode {new_mode} (old mode {self.mode})")

        if new_mode is None or (self.mode is not None and (new_mode != self.mode)):
            self.mode = new_mode
            # all zones disabled
            for room, pid in self.pids.items():
                pid.clear()
            self.compressor_totally_off = True
            self.on_counter = 0
            self.deadband_integrator.clear()
            if self.get_state("climate.aircon") != "fan_only":
                self.try_set_mode("off")
            self.set_fake_temp(celsius_setpoint, ac_stable_threshold, False)
            self.get_entity("input_number.aircon_weighted_error").set_state(
                state=null_state
            )
            self.get_entity("input_number.aircon_avg_deriv").set_state(state=null_state)
            self.get_entity("input_number.aircon_meta_integral").set_state(
                state=null_state
            )
            return

        self.mode = new_mode

        demand = None

        if self.mode == "heat":
            heat_cool_sign = -1.0
            error = heat_cool_sign * heating_demand
        elif self.mode == "cool":
            heat_cool_sign = 1.0
            demand = heat_cool_sign * cooling_demand
        else:
            self.log(f"SOMETHING BAD HAPPENED, invalid mode: {self.mode}")
            return

        # Calculate raw PID outputs
        pid_outputs = {}
        for room, error in errors[self.mode].items():
            if not self.rooms_enabled[room]:
                self.pids[room].clear()
                self.rooms_enabled[room] = True

            self.pids[room].update(
                heat_cool_sign * error, heat_cool_sign * self.targets[mode][room]
            )
            pid_outputs[room] = self.pids[room].get_output()
            # self.log(f"{room} raw PID output: {pid_outputs[room]} (P: {self.pids[room].p_term:.3f}, I: {self.pids[room].i_term:.3f}, D: {self.pids[room].deriv.get():.3f})")

        if len(pid_outputs) > 1:
            # Prevent the highest zone from "running away" by adjusting its
            # integral term such it is within 2.1C (difference_between_top_two - allowable_difference)
            # of the next highest zone.

            # 2D array, sorted high to low
            sorted_pid_outputs = sorted(
                pid_outputs.items(), key=lambda item: item[1], reverse=True
            )
            difference_between_top_two = (
                sorted_pid_outputs[0][1] - sorted_pid_outputs[1][1]
            )
            allowable_difference = normalised_damper_range - room_pid_minimum
            difference_beyond_allowable = (
                difference_between_top_two - allowable_difference
            )
            if difference_beyond_allowable > 0:
                top_zone = sorted_pid_outputs[0][0]

                # If the integral term is greater than the margin by which the next highest zone is below 0
                # then the integral term is keeping the next highest zone on the cusp of being closed.
                if self.pids[top_zone].i_term > difference_beyond_allowable:
                    self.log("Adjusting top integral")
                    self.pids[top_zone].adjust_integral(-difference_beyond_allowable)
                    pid_outputs[top_zone] = self.pids[top_zone].get_output()

        # Adjust all PIDs' integral terms relative to normalised_damper_range
        max_output = max(pid_outputs.values())
        offset = normalised_damper_range - max_output
        for room in pid_outputs:
            self.pids[room].adjust_integral(offset)
            pid_outputs[room] = self.pids[room].get_output()

        # Handle closed door case for top zone
        if len(pid_outputs) > 1:
            top_zone = sorted_pid_outputs[0][0]
            if not self.get_door_state(top_zone):
                # self.log(f"Door closed for {top_zone}, ensuring minimum airflow")
                min_sum = min_airflow_with_closed_door * normalised_damper_range

                while True:
                    positive_outputs = {
                        room: max(0, output) for room, output in pid_outputs.items()
                    }
                    if sum(positive_outputs.values()) >= min_sum:
                        break

                    for room in pid_outputs:
                        if room != top_zone:
                            self.pids[room].adjust_integral(0.001)
                            pid_outputs[room] = self.pids[room].get_output()

        for room in pid_outputs:
            allowable_difference = room_pid_minimum
            difference_beyond_allowable = pid_outputs[room] - allowable_difference
            # Only adjust if integral term is negative (the goal here is to avoid wind-down accumulating)
            if difference_beyond_allowable < 0 and self.pids[room].i_term < 0:
                # If the integral is more negative than output then the
                # integral is keeping the zone on the cusp of being closed.
                if self.pids[room].i_term < difference_beyond_allowable:
                    # self.log("Adjusting very negative integral for room: " + room)
                    self.pids[room].adjust_integral(-difference_beyond_allowable)
                # Otherwise, the zone would be closed anyway. Reset any negative wind-down.
                else:
                    # self.log("Resetting negative integral for room: " + room)
                    self.pids[room].set_integral(0)

                pid_outputs[room] = self.pids[room].get_output()

            self.get_entity(f"input_number.{room}_pid").set_state(
                state=pid_outputs[room]
            )
            self.log(
                f"{room} adjusted PID output: {pid_outputs[room]} (P: {self.pids[room].p_term:.3f}, I: {self.pids[room].i_term:.3f}, D: {self.pids[room].deriv.get():.3f})"
            )

        # Disabled rooms
        for room in rooms - cur_targets[self.mode].keys():
            if self.get_entity("cover." + room).get_state("current_position") != "0":
                self.log("Closing damper for disabled room: " + room)
                self.set_damper_pos(room, 0, False)

        deriv_sum = 0
        error_sum = 0.0
        weight_sum = 0.0

        for room, output in pid_outputs.items():
            clamped_output = max(0, output)
            deriv_sum += clamped_output * self.temp_derivs[room].get()
            error_sum += clamped_output * errors[self.mode][room]
            weight_sum += clamped_output

            damper_vals[room] = 100.0 * (clamped_output / normalised_damper_range)

        avg_deriv = deriv_sum / weight_sum

        # Use the state of the zone with the highest demand rather than weighted demand 
        #weighted_error = error_sum / weight_sum
        weighted_error = demand

        self.get_entity("input_number.aircon_weighted_error").set_state(
            state=weighted_error
        )
        self.get_entity("input_number.aircon_avg_deriv").set_state(state=avg_deriv)

        self.log(
            "compressor_totally_off: "
            + str(self.compressor_totally_off)
            + ", guesstimated_comp_speed: "
            + str(self.guesstimated_comp_speed)
            + ", min_power_counter: "
            + str(self.min_power_counter)
            + ", on_counter: "
            + str(self.on_counter)
            + ", weighted_error: "
            + str(weighted_error)
            + ", avg_deriv: "
            + str(avg_deriv)
        )

        unsigned_compressed_error = self.compress(
            weighted_error * heat_cool_sign, avg_deriv * heat_cool_sign
        )
        self.prev_unsigned_compressed_error = unsigned_compressed_error

        compressed_error = heat_cool_sign * unsigned_compressed_error
        self.log("compressed_error: " + str(compressed_error))

        self.on_counter += 1
        if self.get_state("input_boolean.ac_min_power") == "on":
            self.on_counter = min(self.on_counter, soft_delay - 1)

        if (
            self.get_state("climate.aircon") in ["cool", "heat"]
            and unsigned_compressed_error <= ac_off_threshold
            or (
                self.off_fan_running_counter > 0
                and unsigned_compressed_error < ac_stable_threshold
            )
        ):
            self.off_fan_running_counter += 1
        else:
            self.off_fan_running_counter = 0

        if (self.off_fan_running_counter >= off_fan_running_time) or (
            self.get_state("climate.aircon") == "off"
            and unsigned_compressed_error < ac_stable_threshold
        ):
            self.log("temp beyond target, turning off altogether")
            self.try_set_mode("off")
            self.off_fan_running_counter = 0
            self.on_counter = 0
            for room in sorted(damper_vals, key=damper_vals.get, reverse=True):
                self.set_damper_pos(room, damper_vals[room], True)
            return
        else:
            for room in sorted(damper_vals, key=damper_vals.get, reverse=True):
                self.set_damper_pos(room, damper_vals[room], False)

        # Unsure if it does anything, send the current feels like immediately before powering on
        if self.get_state("climate.aircon") == "off":
            self.set_fake_temp(celsius_setpoint, compressed_error, True)

        self.try_set_mode(self.mode)
        self.get_entity("input_number.aircon_meta_integral").set_state(
            state=self.deadband_integrator.get()
        )
        self.set_fake_temp(celsius_setpoint, compressed_error, True)

    def set_fake_temp(self, celsius_setpoint, compressed_error, transmit=True):
        self.get_entity("input_number.fake_temperature").set_state(
            state=(celsius_setpoint + compressed_error)
        )
        if not transmit:
            return
        if ac_celsius:
            # Power on, FM update, mode auto, Fan auto, setpoint 25C?, room temp
            self.call_service(
                "esphome/infrared_send_raw_command",
                command=[
                    0xA4,
                    0x82,
                    0x48,
                    0x7F,
                    (int)(celsius_setpoint + compressed_error + 1),
                ],
            )
        else:
            # Power On, FM Update, Mode Auto, Fan Auto, Setpoint 68F, Room temp
            self.call_service(
                "esphome/infrared_send_raw_command",
                command=[
                    0xA4,
                    0x82,
                    0x66,
                    0x7F,
                    (int)((1.8 * celsius_setpoint + 32) + compressed_error - 31),
                ],
            )
        time.sleep(0.1)

    def set_damper_pos(self, room, damper_val, open_only=False):
        # i don't know fluid dynamics but it seems nonlinear, lets try squaring!
        damper_val = 100.0 * (1.0 - pow(1.0 - (damper_val / 100.0), 2))

        actual_cur_pos = float(
            self.get_entity("cover." + room).get_state("current_position")
        )
        if abs(self.damper_pos[room] - actual_cur_pos) > 5:
            self.damper_pos[room] = actual_cur_pos
        cur_pos = self.damper_pos[room]

        damper_log = f"{room} damper scaled: {damper_val}, cur_pos: {cur_pos}, actual_cur_pos: {actual_cur_pos}"
        self.get_entity("input_number." + room + "_damper_target").set_state(
            state=damper_val
        )

        cur_deadband = damper_deadband
        # damper won't do much if the fan isn't running
        # cur_deadband = (
        #    (2.0 * damper_deadband)
        #    if (self.compressor_totally_off and self.mode == 'heat')
        #    else damper_deadband
        # )

        if (damper_val > 99.9 and actual_cur_pos < 100.0) or (
            (not open_only)
            and (
                (damper_val < 0.1 and actual_cur_pos > 0.0)
                or (damper_val > (cur_pos + cur_deadband))
                or (damper_val < (cur_pos - cur_deadband))
            )
        ):
            self.log(damper_log + " adjusting")

            if cur_pos < damper_val < (cur_pos + damper_round) + cur_deadband:
                rounded_damper_val = cur_pos + damper_round
            elif cur_pos > damper_val > (cur_pos - damper_round) - cur_deadband:
                rounded_damper_val = cur_pos - damper_round
            else:
                rounded_damper_val = damper_round * round(damper_val / damper_round)
            self.call_service(
                "cover/set_cover_position",
                entity_id=("cover." + room),
                position=rounded_damper_val,
            )
            self.damper_pos[room] = rounded_damper_val
            time.sleep(0.1)
        else:
            self.log(damper_log + " within deadband")

    def try_set_mode(self, mode):
        if self.get_state("climate.aircon") != mode:
            self.call_service(
                "climate/set_hvac_mode", entity_id="climate.aircon", hvac_mode=mode
            )
            # workaround to retransmit IR code
            time.sleep(0.1)
            self.call_service(
                "climate/set_hvac_mode", entity_id="climate.aircon", hvac_mode=mode
            )
            time.sleep(0.1)

    def compress(self, error, deriv):
        # Tighten the deadband with runtime, with the goal of turning off
        # before the high power 'purge' that occurs after 90 mins of continuous
        # operation at low speed. This 'purge' often pushes us out of the
        # deadband anyway, so it's more efficient to just turn off prior.
        # Reset in sync with the purge period. Desync isn't a big deal.
        wrapped_on_counter = self.min_power_counter % purge_delay
        min_power_progress = min(1.0, wrapped_on_counter / min_power_delay)

        if self.guesstimated_comp_speed <= 0 and error <= (
            desired_off_threshold * (1 - min_power_progress)
            + min_power_threshold * min_power_progress
        ):
            self.compressor_totally_off = True

        if self.guesstimated_comp_speed > 0 and error <= desired_off_threshold:
            self.compressor_totally_off = True

        if self.compressor_totally_off:
            self.on_counter = 0
            self.min_power_counter = 0

            if error < desired_on_threshold:
                # sometimes -2 isn't the true off_threshold?!
                # shut things down more decisively
                return self.midea_reset_quirks(ac_off_threshold - 1)
            else:
                self.compressor_totally_off = False
                self.deadband_integrator.clear()
                print(f"starting compressor {ac_on_threshold}")

        # "blip" the power to get AC to start
        if self.on_counter < 1:
            return self.midea_reset_quirks(ac_on_threshold)

        # conditions in which to consider the derivative
        # - aircon is currently running
        # - the current temp (ignoring RoC) has not yet reached off threshold
        # goal of the derivative is to proactively reduce/increase compressor power, but not to influence on/off state
        error = error + deriv

        if error > min_power_threshold and (self.on_counter < soft_delay):
            if self.on_counter < soft_delay:
                print("soft start, on_counter: " + str(self.on_counter))
            return self.midea_runtime_quirks(ac_stable_threshold - 1)

        if error > faithful_threshold:
            if self.on_counter < (soft_delay + soft_ramp):
                ramp_progress = (self.on_counter - soft_delay) / soft_ramp
                print(
                    "ramping "
                    + str(ramp_progress)
                    + ", on_counter: "
                    + str(self.on_counter)
                )
            else:
                ramp_progress = 1

            return self.midea_runtime_quirks(
                ac_stable_threshold
                + ramp_progress
                * (2 + ac_unit_from_celsius * (error - faithful_threshold))
            )

        if error <= min_power_threshold:
            return self.midea_runtime_quirks(ac_off_threshold + 1)

        if self.prev_unsigned_compressed_error > ac_stable_threshold + 1:
            self.deadband_integrator.clear()
            return self.midea_runtime_quirks(ac_stable_threshold + 1)
        elif self.prev_unsigned_compressed_error < ac_stable_threshold - 2:
            self.deadband_integrator.clear()
            return self.midea_runtime_quirks(ac_stable_threshold - 1)

        return self.midea_runtime_quirks(
            ac_stable_threshold + self.deadband_integrator.set(error)
        )

    def midea_reset_quirks(self, rval):
        self.outer_ramp_count = 0
        self.outer_ramp_rval = rval
        self.guesstimated_comp_speed = 0
        return rval

    def midea_runtime_quirks(self, rval):
        rval = round(rval)

        if (
            (self.outer_ramp_count > 0 and rval < ac_stable_threshold)
            or (self.outer_ramp_count < 0 and rval > ac_stable_threshold)
            or (self.outer_ramp_count > step_up_intervals)
            or (
                self.outer_ramp_count
                < -(
                    heat_step_down_intervals
                    if self.mode == "heat"
                    else step_down_intervals
                )
            )
        ):
            self.outer_ramp_count = 0

        # restart if the temp goes even further in the direction we're holding
        if (rval > ac_stable_threshold and rval > self.outer_ramp_rval) or (
            rval == self.outer_ramp_rval == ac_stable_threshold + 1
        ):
            self.outer_ramp_rval = rval
            self.outer_ramp_count = 1
            # Saturate after 15 power increments (note: a guess / slight over-estimate, the number of increments hasn't been measured)
            self.guesstimated_comp_speed = min(
                compressor_power_increments + 2,
                max(2, self.guesstimated_comp_speed + 1),
            )
        elif (rval < ac_stable_threshold and rval < self.outer_ramp_rval) or (
            rval == self.outer_ramp_rval == ac_stable_threshold - 1
        ):
            self.outer_ramp_rval = rval
            self.outer_ramp_count = -1
            if rval < (ac_stable_threshold - 2):
                self.guesstimated_comp_speed = -minimum_temp_intervals
            else:
                self.guesstimated_comp_speed = max(
                    -minimum_temp_intervals, self.guesstimated_comp_speed - 1
                )

        if self.outer_ramp_count > 0:
            self.outer_ramp_count += 1
            rval = self.outer_ramp_rval
        elif self.outer_ramp_count < 0:
            self.outer_ramp_count -= 1
            rval = self.outer_ramp_rval
        else:
            self.outer_ramp_rval = rval

        # Saturated, just keep demanding a compressor speed increase
        if self.guesstimated_comp_speed >= compressor_power_increments + 2:
            rval = max(ac_stable_threshold + 1, rval)

        if self.guesstimated_comp_speed <= 0:
            rval = min(ac_stable_threshold - 1, rval)

        # After 5 minutes worth of consecutive decrements assume that we want the absolute minimum power
        if self.guesstimated_comp_speed <= -minimum_temp_intervals:
            rval = min(ac_off_threshold + 1, rval)

        if rval < ac_stable_threshold:
            self.min_power_counter += 1
            if self.min_power_counter % min_power_time == (min_power_time - 1):
                # 'blip' the feels like temp to reset the AC's internal timer
                self.prev_unsigned_compressed_error = ac_stable_threshold + 1

            # aircon seems to react to edges
            # so provide as many as possible to quickly reduce power?
            # may be extra helpful in fahrenheit mode? (seemingly not)
            return max(rval, self.prev_unsigned_compressed_error - 1)
        else:
            self.min_power_counter = 0

        # cooling mode
        # at the setpoint ac will start ramping back power
        # and will continue ramping back at setpoint+1
        # a brief jump up to setpoint+2 is needed to arrest the fall and stabilise output power
        # in heating mode there is no gradual rampdown at all, it drops straight to min power
        # heating mode
        # sometimes the same in reverse?
        if (
            (self.mode == "cool")
            and self.prev_unsigned_compressed_error < ac_stable_threshold
            and rval >= ac_stable_threshold
        ):
            return max(rval, ac_stable_threshold + 1)
        if (
            self.mode == "heat"
            and self.prev_unsigned_compressed_error > ac_stable_threshold
            and rval <= ac_stable_threshold
        ):
            return min(rval, ac_stable_threshold - 1)

        return rval

    def get_door_state(self, room):
        """Check if a room's door is open."""
        entity_id = f"binary_sensor.{room}_door"
        state = self.get_state(entity_id)
        return True if state is None else state == "on"
