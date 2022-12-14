import hassapi as hass
import datetime
import math

# from simple_pid import PID
from collections import deque
import time

rooms = ["bed_1", "bed_2", "bed_3", "kitchen", "study"]

# in minutes
interval = 0.20  # 12 seconds

# WMA over the last 5 minutes
global_temp_deriv_window = 5
# to predict 10 minutes into the future
global_temp_deriv_factor = 10.0

# report 0.25 degree offset to aircon as 1 degree, 4x amplification
global_compress_factor = 0.25

# per second
global_ki = 0.00025
# a 0.1 deg error will accumulate 0.1 in ~60 minutes

# per second
global_deadband_ki = 0.025
# a 0.1 deg error will accumulate 1 in ~3.33 minutes

# in seconds, time for aircon to ramp up power 1 increment & stay there
step_up_time = 200
# shorter as it latches instantly
# 120s sufficient to step down, but an extra 30 seconds added to make soft start more reliable
step_down_time = 120

# swing full scale across 2.0C of error
room_kp = 1.0

# per second
room_ki = 0.001
# a 0.1 deg error will accumulate 0.1 in ~15 minutes

# kd considers the last 10 minutes
room_deriv_window = 10.0

# for a constant 1.0 deg / min over the last 10 mins, swing full scale from -1 to 1
room_deriv_factor = 2.0

# percent
damper_deadband = 5.0
damper_round = 5
# 10% of extra virtual closed-ness
damper_fully_closed_buffer = 10.0

# soft start to avoid overshoot
# start at min power and gradually report the actual rval
# 5 min delay at min power
# covers a defrost cycle
soft_delay = int(7.5 / interval)
# then gradually report the actual rval over 5 mins
soft_ramp = int(7.5 / interval)

# setpoint - 1 takes about this long to bring a/c to min power
# don't hold it there forever as it'll shut down after 1hr at this temp
min_power_time = int(7.5 / interval)

# in cooling mode, how long to keep blowing the fan
off_fan_running_time = int(2.5 / interval)

# 10% per minute above threshold
target_ramp_proportional = 0.1 * interval
# linear below 0.1C
target_ramp_linear_threshold = 0.1
target_ramp_linear_increment = target_ramp_proportional * target_ramp_linear_threshold


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

        self.wma.set((error - self.prev_error) + (target - self.prev_target))
        self.prev_error = error
        self.prev_target = target

    def get(self):
        return self.factor * self.wma.get()


class MyPID:
    def __init__(self, kp, ki, kd, window, clamp_low, clamp_high):
        self.kp = kp
        self.ki = ki
        self.deriv = MyDeriv(window, kd)
        self.clamp_low = clamp_low
        self.clamp_high = clamp_high
        self.clear()

    def clear(self):
        self.deriv.clear()
        self.last_val = 0.0
        self.integral = 0.0

    def set(self, error, target):
        # print(" error " + str(error) + " target " + str(target))
        val = error - target
        self.last_val = val
        self.integral += val
        self.deriv.set(error, target)
        # clamp between +-1
        # if not self.clamp_low <= self.get() <= self.clamp_high:
        if self.get() > self.clamp_high:
            clamp_to = self.clamp_high
            # else:
            #    clamp_to = self.clamp_low
            self.integral = (
                -(clamp_to + self.deriv.get() + (self.last_val * self.kp)) / self.ki
            )

    def scale(self, factor):
        current = self.get()
        scaledcurrent = 1.0 - current
        target = 1.0 - (scaledcurrent * factor)
        delta = current - target
        self.integral += delta / self.ki

    def get(self):
        return (
            (-self.last_val * self.kp)
            + (-self.integral * self.ki)
            + (-self.deriv.get())
        )


class MySimplerIntegral:
    def __init__(self, ki, clamp_low, clamp_high):
        self.ki = ki
        self.clamp_low = clamp_low
        self.clamp_high = clamp_high
        self.clear()

    def clear(self):
        self.integral = 0.0

    def set(self, error):
        self.integral += error * self.ki
        if self.integral < self.clamp_low:
            self.integral = self.clamp_low
        if self.integral > self.clamp_high:
            self.integral = self.clamp_high

    def get(self):
        return self.integral


class DeadbandIntegrator:
    def __init__(self, ki, step_up_intervals, step_down_intervals):
        self.ki = ki
        self.step_up_intervals = step_up_intervals
        self.step_down_intervals = step_down_intervals
        self.clear()

    def clear(self):
        self.integral = 0.0
        self.ramp_count = 0

    def clear_set(self, integral):
        self.integral = integral
        self.ramp_count = 0

    def set(self, error):
        self.integral += error * self.ki
        print("input " + str(error) + " integral " + str(self.integral))

        if abs(self.ramp_count) > 0 and (
            (
                self.ramp_count > self.step_up_intervals
                or self.ramp_count < -self.step_down_intervals
            )
            or abs(self.integral - math.copysign(1.0, self.ramp_count)) > 2.0
        ):
            print("over thresh, resetting")
            # print("step_intervals " + str(self.step_intervals))
            print(
                "other thingy "
                + str(abs(self.integral - math.copysign(1.0, self.ramp_count)))
            )
            self.ramp_count = 0

        if self.ramp_count == 0 and abs(self.integral) > 1.0:
            print("over thresh, starting")
            self.ramp_count = math.copysign(1, self.integral)
            self.integral -= math.copysign(2.0, self.ramp_count)

        print("ramp_count " + str(self.ramp_count))

        if self.integral < -1:
            self.integral = -1
        if self.integral > 1:
            self.integral = 1

        if abs(self.ramp_count) > 0:
            print("aircon is ramping")
            self.ramp_count += math.copysign(1, self.ramp_count)
            return math.copysign(1, self.ramp_count)
        else:
            return 0

    def get(self):
        return self.integral


class Actrl(hass.Hass):
    def initialize(self):
        self.log("INITIALISING")
        self.pids = {}
        self.targets = {}
        self.rooms_enabled = {}
        self.temp_deriv = MyDeriv(
            window=int(global_temp_deriv_window / interval),
            factor=int(global_temp_deriv_window / interval),
        )
        self.temp_integral = MySimplerIntegral(
            ki=(global_ki * 60.0 * interval),
            clamp_low=0,
            clamp_high=global_compress_factor,
        )
        self.ramping_down = True
        self.min_power_counter = 0
        self.off_fan_running_counter = 0
        self.heat_mode = False

        if self.get_state("input_boolean.ac_already_on_bypass") == "on":
            self.log("ASSUMING THAT THE AIRCON IS ALREADY RUNNING")
            self.compressor_totally_off = False
            self.on_counter = soft_delay + soft_ramp
        else:
            self.compressor_totally_off = True
            self.on_counter = 0

        self.deadband_integrator = DeadbandIntegrator(
            ki=(global_deadband_ki * 60.0 * interval),
            step_up_intervals=step_up_time / (60.0 * interval),
            step_down_intervals=step_down_time / (60.0 * interval),
        )

        for room in rooms:
            self.pids[room] = MyPID(
                kp=room_kp,
                ki=(room_ki * 60.0 * interval),
                kd=int(room_deriv_factor / interval),
                window=int(room_deriv_window / interval),
                clamp_low=-1.0,
                clamp_high=1.0,
            )
            self.rooms_enabled[room] = True
        # run every interval (in minutes)
        self.run_every(self.main, "now", 60.0 * interval)
        self.main(None)

    def main(self, kwargs):
        self.log("#### BEGIN CYCLE ####")
        temps = {}
        errors = {}
        damper_vals = {}
        pid_vals = {}
        disabled_rooms = []
        heat_cool_sign = 1.0
        all_disabled = True
        heat_room_count = 0
        cool_room_count = 0

        main_setpoint = float(
            self.get_entity("climate.aircon").get_state("temperature")
        )

        for room in rooms:
            if self.get_state("climate." + room + "_aircon") != "off":
                all_disabled = False

                if self.get_state("input_boolean.ac_use_feels_like") == "on":
                    temps[room] = float(
                        self.get_state("sensor." + room + "_feels_like")
                    )
                else:
                    temps[room] = float(
                        self.get_state("sensor." + room + "_average_temperature")
                    )

                cur_target = float(
                    self.get_entity("climate." + room + "_aircon").get_state(
                        "temperature"
                    )
                )
                if room in self.targets:
                    target_delta = cur_target - self.targets[room]

                    if abs(target_delta) <= target_ramp_linear_increment:
                        self.targets[room] = cur_target
                    elif abs(target_delta) <= target_ramp_linear_threshold:
                        self.targets[room] += math.copysign(
                            target_ramp_linear_increment, target_delta
                        )
                        self.log(
                            "linearly ramping target room: "
                            + room
                            + ", smooth target: "
                            + str(self.targets[room])
                        )
                    else:
                        self.targets[room] += target_delta * target_ramp_proportional
                        self.log(
                            "proportionally ramping target room: "
                            + room
                            + ", smooth target: "
                            + str(self.targets[room])
                        )
                else:
                    self.log("setting target for previously disabled room " + room)
                    self.targets[room] = cur_target

                errors[room] = temps[room] - self.targets[room]
                if self.get_state("climate." + room + "_aircon") == "heat":
                    heat_room_count += 1
                if self.get_state("climate." + room + "_aircon") == "cool":
                    cool_room_count += 1
            else:
                disabled_rooms.append(room)
                if room in self.targets:
                    self.targets.pop(room)

        if all_disabled:
            self.get_entity("input_number.fake_temperature").set_state(
                state=main_setpoint
            )
            self.get_entity("input_number.aircon_weighted_error").set_state(
                state=float("nan")
            )
            self.get_entity("input_number.aircon_avg_deriv").set_state(
                state=float("nan")
            )
            self.get_entity("input_number.aircon_integrated_error").set_state(
                state=float("nan")
            )
            self.get_entity("input_number.aircon_meta_integral").set_state(
                state=float("nan")
            )
            for room, pid in self.pids.items():
                pid.clear()
            self.temp_deriv.clear()
            self.temp_integral.clear()
            self.compressor_totally_off = True
            self.on_counter = 0
            self.deadband_integrator.clear()
            if self.get_state("climate.aircon") != "fan_only":
                self.try_set_mode("off")
            return

        if heat_room_count > 0 and cool_room_count == 0:
            self.turn_on("input_boolean.heat_mode")
        elif cool_room_count > 0 and heat_room_count == 0:
            self.turn_off("input_boolean.heat_mode")

        if self.get_state("input_boolean.heat_mode") == "on":
            heat_cool_sign = -1.0
            self.heat_mode = True
        else:
            self.heat_mode = False

        unweighted_avg_error = sum(errors.values()) / len(errors.values())

        pre_avg_weight_sum = 0
        pre_avg_value_sum = 0
        for room, error in errors.items():
            weight = 1.0 - heat_cool_sign * self.pids[room].get()
            self.log(
                "room: " + room + ", error: " + str(error) + ", weight: " + str(weight)
            )
            pre_avg_weight_sum += weight
            pre_avg_value_sum += weight * error

        avg_error = pre_avg_value_sum / pre_avg_weight_sum
        self.log(
            "naive average: "
            + str(unweighted_avg_error)
            + ", weighted average: "
            + str(avg_error)
        )

        for room, error in errors.items():
            if not self.rooms_enabled[room]:
                self.pids[room].clear()
                self.rooms_enabled[room] = True
                # a new room has been enabled, reset the deriv history
                self.temp_deriv.clear()
                self.deadband_integrator.clear()
                # and return half-way through soft-start
                self.on_counter = min(self.on_counter, soft_delay)

            self.pids[room].set(error, avg_error)
            pid_vals[room] = heat_cool_sign * self.pids[room].get()
            # self.log(room + " PID outcome was " + str(pid_vals[room]))

        unscaled_min_pid = min(pid_vals.values())
        unscaled_max_pid = max(pid_vals.values())

        scalefactor = 2.0 / ((1.0 - unscaled_min_pid) + (1.0 - unscaled_max_pid))
        self.log("scaling all PIDs by: " + str(scalefactor))
        for room, pid_val in pid_vals.items():
            self.pids[room].scale(scalefactor)
            pid_vals[room] = heat_cool_sign * self.pids[room].get()
            self.get_entity("input_number." + room + "_pid").set_state(
                state=pid_vals[room]
            )

        for room in disabled_rooms:
            if self.rooms_enabled[room]:
                self.pids[room].clear()
                self.rooms_enabled[room] = False
                self.get_entity("input_number." + room + "_pid").set_state(
                    state=float("nan")
                )
                # a new room has been disabled, reset the deriv history
                self.temp_deriv.clear()

            if self.get_entity("cover." + room).get_state("current_position") != "0":
                self.log("Closing damper for disabled room: " + room)
                self.set_damper_pos(room, 0)

        min_pid = min(pid_vals.values())

        error_sum = 0.0
        target_sum = 0.0
        weight_sum = 0.0

        for room, pid_val in pid_vals.items():
            # keep low values way down in the hole
            scaled = max(
                0,
                -damper_fully_closed_buffer
                + (100.0 + damper_fully_closed_buffer)
                * ((1.001 - pid_val) / (1.001 - min_pid)),
            )

            damper_vals[room] = scaled
            target_sum += self.targets[room] * scaled
            error_sum += errors[room] * scaled
            weight_sum += scaled

        weighted_error = error_sum / weight_sum
        weighted_target = target_sum / weight_sum

        self.temp_deriv.set(weighted_error, weighted_target)
        avg_deriv = self.temp_deriv.get()

        self.temp_integral.set(weighted_error)

        self.get_entity("input_number.aircon_weighted_error").set_state(
            state=weighted_error
        )
        self.get_entity("input_number.aircon_avg_deriv").set_state(state=avg_deriv)
        self.get_entity("input_number.aircon_integrated_error").set_state(
            state=self.temp_integral.get()
        )

        self.log(
            "compressor_totally_off: "
            + str(self.compressor_totally_off)
            + ", on_counter: "
            + str(self.on_counter)
            + ", weighted_error pre-integral: "
            + str(weighted_error)
        )
        self.log(
            "avg_deriv: "
            + str(avg_deriv)
            + ", temp_integral: "
            + str(self.temp_integral.get())
        )

        weighted_error += self.temp_integral.get()

        unsigned_compressed_error = self.compress(
            weighted_error * heat_cool_sign, avg_deriv * heat_cool_sign
        )

        if self.get_state("climate.aircon") == "off":
            compressed_error = heat_cool_sign * min(unsigned_compressed_error, 0)
        else:
            compressed_error = heat_cool_sign * unsigned_compressed_error
        self.log(
            "weighted_error post-integral: "
            + str(weighted_error)
            + ", compressed_error: "
            + str(compressed_error)
        )
        self.get_entity("input_number.fake_temperature").set_state(
            state=(main_setpoint + compressed_error)
        )
        self.get_entity("input_number.aircon_meta_integral").set_state(
            state=self.deadband_integrator.get()
        )

        self.on_counter += 1

        if (
            self.get_state("climate.aircon") == "cool"
            and unsigned_compressed_error <= -2
            or (self.off_fan_running_counter > 0 and unsigned_compressed_error < 1)
        ):
            self.off_fan_running_counter += 1
        else:
            self.off_fan_running_counter = 0

        if not self.heat_mode and (
            self.off_fan_running_counter >= off_fan_running_time
            or (
                self.get_state("climate.aircon") == "off"
                and unsigned_compressed_error < 1
            )
        ):
            self.log("cool mode and temp too low, turning off altogether")
            self.try_set_mode("off")
            self.off_fan_running_counter = 0
            self.on_counter = 0
            return

        if False:
            self.log("heat mode and indoor fan is not running, not adjusting dampers")
        else:
            for room in sorted(damper_vals, key=damper_vals.get, reverse=True):
                # damper_val = damper_vals[room]
                self.set_damper_pos(room, damper_vals[room])

        if self.heat_mode:
            self.try_set_mode("heat")
        elif self.get_state("climate.aircon") == "dry":
            self.try_set_mode("dry")
        else:
            self.try_set_mode("cool")

    def set_damper_pos(self, room, damper_val):
        cur_pos = float(self.get_entity("cover." + room).get_state("current_position"))
        damper_log = (
            room + " damper scaled: " + str(damper_val) + ", cur_pos: " + str(cur_pos)
        )
        self.get_entity("input_number." + room + "_damper_target").set_state(
            state=damper_val
        )

        # damper won't do much if the fan isn't running
        cur_deadband = (
            (2.0 * damper_deadband)
            if (self.compressor_totally_off and self.heat_mode)
            else damper_deadband
        )

        if (
            (damper_val > 99.9 and cur_pos < 100.0)
            or (damper_val < 0.1 and cur_pos > 0.0)
            or (damper_val > (cur_pos + cur_deadband))
            or (damper_val < (cur_pos - cur_deadband))
        ):
            self.log(damper_log + " adjusting")
            rounded_damper_val = damper_round * round(damper_val / damper_round)
            self.call_service(
                "cover/set_cover_position",
                entity_id=("cover." + room),
                position=rounded_damper_val,
            )
            time.sleep(0.1)
        else:
            self.log(damper_log + " within deadband")

    def try_set_mode(self, mode):
        if self.get_state("climate.aircon") != mode:
            self.call_service(
                "climate/set_hvac_mode", entity_id="climate.aircon", hvac_mode=mode
            )

    def actually_compress(self, error):
        # tell the aircon that temp variations are worse than reality
        # static 0.5 degree offset
        return 0.5 + error / global_compress_factor

    def compress(self, error, deriv):
        on_threshold = 1
        off_threshold = -2

        # higher threshold in cool mode while already running
        if self.get_state("climate.aircon") == "cool":
            initial_on_threshold = 2
        else:
            initial_on_threshold = 1

        rval = self.actually_compress(error)

        # conditions in which to consider the derivative
        # - aircon is currently running
        # - the current temp (ignoring RoC) has not yet reached off threshold
        # goal of the derivative is to proactively reduce/increase compressor power, but not to influence on/off state
        if not self.compressor_totally_off and (round(rval) > off_threshold):
            rval = max((off_threshold + 1), self.actually_compress(error + deriv))

        rval = min(rval, 15.0)
        rval = max(rval, -15.0)
        unrounded_rval = rval
        rval = round(rval)

        if rval <= off_threshold:
            self.compressor_totally_off = True
            # sometimes -2 isn't the true off_threshold?!
            # shut things down more decisively
            rval = min(rval, -3)

        if self.compressor_totally_off:
            self.on_counter = 0
            self.min_power_counter = 0

            if rval < on_threshold:
                return rval
            else:
                self.compressor_totally_off = False
                self.temp_deriv.clear()
                self.deadband_integrator.clear()
                return initial_on_threshold

        if self.on_counter < soft_delay and rval >= 0:
            print("soft start, on_counter: " + str(self.on_counter))
            self.ramping_down = True
            return 0

        if self.on_counter < (soft_delay + soft_ramp) and rval > 2:
            ramp_progress = (self.on_counter - soft_delay) / soft_ramp
            print(
                "ramping "
                + str(ramp_progress)
                + ", on_counter: "
                + str(self.on_counter)
            )
            return round((ramp_progress * unrounded_rval) + ((1.0 - ramp_progress) * 1))

        if 0 <= rval and rval <= 2:
            unrounded_rval = (
                self.deadband_integrator.set(
                    (unrounded_rval - 1.0) * global_compress_factor
                )
                + 1.0
            )
            rval = round(unrounded_rval)
        else:
            self.deadband_integrator.clear()

        if rval <= 0:
            self.ramping_down = True

        if rval == -1:
            self.min_power_counter += 1
            if self.min_power_counter <= min_power_time:
                return -1
            else:
                return 0
        else:
            self.min_power_counter = 0

        # behaviour only observed in cooling mode
        # at the setpoint ac will start ramping back power
        # and will continue ramping back at setpoint+1
        # a brief jump up to setpoint+2 is needed to arrest the fall and stabilise output power
        # in heating mode there is no gradual rampdown at all, it drops straight to min power

        if self.ramping_down and rval > 0:
            self.ramping_down = False
            return max(rval, initial_on_threshold)

        return rval
