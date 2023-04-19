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
# 120s sufficient to step down, but an extra 30 seconds added to make soft start more reliable
step_down_time = 120
step_down_intervals = step_down_time / (60.0 * interval)

# swing full scale across 2.0C of error
room_kp = 1.0

# per second
room_ki = 0.001
# a 0.1 deg error will accumulate 0.1 in ~15 minutes

# kd considers the last 10 minutes
room_deriv_window = 10.0
# looking 2 minutes into the future
room_deriv_factor = 2.0

# percent
damper_deadband = 7.5
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

# setpoint - it takes about 7.5 (sometimes longer) to bring a/c to min power
# don't hold it there forever as it'll shut down after 1hr at this temp
# 30 mins should be safe
min_power_time = int(30 / interval)

# in cooling mode, how long to keep blowing the fan
off_fan_running_time = int(2.5 / interval)

# 20% per minute above threshold
target_ramp_proportional = 0.2 * interval
# linear below 0.1C
target_ramp_linear_threshold = 0.1
target_ramp_linear_increment = target_ramp_proportional * target_ramp_linear_threshold


# when the error is greater than 2.0C
# let the Midea controller do its thing
faithful_threshold = 2.0
desired_on_threshold = 0.0
min_power_threshold = -0.75
desired_off_threshold = -1.0

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
    def __init__(self, window, factor, clamp_reverse_deriv=False):
        self.wma = MyWMA(window=window)
        self.factor = factor
        self.clamp_reverse_deriv = clamp_reverse_deriv
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

        # avoid conditions where the change in target temp outpaces the change in actual temp
        # wherein the derivative term ends up working against us
        # limit the derivative to zero if it's going in the opposite direction to the temp trend
        if self.clamp_reverse_deriv and (
            (target_delta > 0 and target_delta > -error_delta)
            or (target_delta < 0 and target_delta < -error_delta)
        ):
            self.wma.set(0)
        else:
            self.wma.set(error_delta + target_delta)
        self.prev_error = error
        self.prev_target = target

    def get(self):
        return self.factor * self.wma.get()


class MyPID:
    def __init__(self, kp, ki, kd, window, clamp_low, clamp_high):
        self.kp = kp
        self.ki = ki
        self.deriv = MyDeriv(window=window, factor=kd, clamp_reverse_deriv=False)
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
        self.targets = {}
        self.rooms_enabled = {}
        self.damper_pos = {}
        self.temp_deriv = MyDeriv(
            window=int(global_temp_deriv_window / interval),
            factor=global_temp_deriv_factor / interval,
            clamp_reverse_deriv=True,
        )
        self.prev_unsigned_compressed_error = 0
        self.min_power_counter = 0
        self.off_fan_running_counter = 0
        self.outer_ramp_count = 0
        self.outer_ramp_rval = 1
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
        )

        for room in rooms:
            self.pids[room] = MyPID(
                kp=room_kp,
                ki=(room_ki * 60.0 * interval),
                kd=room_deriv_factor / interval,
                window=int(room_deriv_window / interval),
                clamp_low=-1.0,
                clamp_high=1.0,
            )
            self.rooms_enabled[room] = True
            self.damper_pos[room] = float(
                self.get_entity("cover." + room).get_state("current_position")
            )
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

        celsius_setpoint = float(
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
            for room, pid in self.pids.items():
                pid.clear()
            self.temp_deriv.clear()
            self.compressor_totally_off = True
            self.on_counter = 0
            self.deadband_integrator.clear()
            if self.get_state("climate.aircon") != "fan_only":
                self.try_set_mode("off")
            self.set_fake_temp(celsius_setpoint, ac_stable_threshold, False)
            self.get_entity("input_number.aircon_weighted_error").set_state(
                state=float("nan")
            )
            self.get_entity("input_number.aircon_avg_deriv").set_state(
                state=float("nan")
            )
            self.get_entity("input_number.aircon_meta_integral").set_state(
                state=float("nan")
            )
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

        self.get_entity("input_number.aircon_weighted_error").set_state(
            state=weighted_error
        )
        self.get_entity("input_number.aircon_avg_deriv").set_state(state=avg_deriv)

        self.log(
            "compressor_totally_off: "
            + str(self.compressor_totally_off)
            + ", on_counter: "
            + str(self.on_counter)
            + ", weighted_error: "
            + str(weighted_error)
            + ", weighted_target: "
            + str(weighted_target)
        )
        self.log("avg_deriv: " + str(avg_deriv))

        unsigned_compressed_error = self.compress(
            weighted_error * heat_cool_sign, avg_deriv * heat_cool_sign
        )
        self.prev_unsigned_compressed_error = unsigned_compressed_error

        compressed_error = heat_cool_sign * unsigned_compressed_error
        self.log("compressed_error: " + str(compressed_error))

        self.on_counter += 1

        if (
            self.get_state("climate.aircon") == "cool"
            and unsigned_compressed_error <= ac_off_threshold
            or (
                self.off_fan_running_counter > 0
                and unsigned_compressed_error < ac_stable_threshold
            )
        ):
            self.off_fan_running_counter += 1
        else:
            self.off_fan_running_counter = 0

        if not self.heat_mode and (
            self.off_fan_running_counter >= off_fan_running_time
            or (
                self.get_state("climate.aircon") == "off"
                and unsigned_compressed_error < ac_stable_threshold
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


        # Unsure if it does anything, send the current feels like immediately before powering on
        if self.get_state("climate.aircon") == "off":
            self.set_fake_temp(celsius_setpoint, compressed_error, True)

        if self.heat_mode:
            self.try_set_mode("heat")
        elif self.get_state("climate.aircon") == "dry":
            self.try_set_mode("dry")
        else:
            self.try_set_mode("cool")
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

    def set_damper_pos(self, room, damper_val):
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

        if error <= desired_off_threshold:
            self.compressor_totally_off = True

        if self.compressor_totally_off:
            self.on_counter = 0
            self.min_power_counter = 0

            if error < desired_on_threshold:
                # sometimes -2 isn't the true off_threshold?!
                # shut things down more decisively
                return ac_off_threshold - 1
            else:
                self.compressor_totally_off = False
                self.temp_deriv.clear()
                self.deadband_integrator.clear()
                print(f"starting compressor {ac_on_threshold}")

        # "blip" the power to get AC to start
        if self.on_counter < 1:
            return ac_on_threshold

        # conditions in which to consider the derivative
        # - aircon is currently running
        # - the current temp (ignoring RoC) has not yet reached off threshold
        # goal of the derivative is to proactively reduce/increase compressor power, but not to influence on/off state
        error = error + deriv

        if self.on_counter < soft_delay and error > min_power_threshold:
            print("soft start, on_counter: " + str(self.on_counter))
            return ac_stable_threshold - 1

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

        if error < min_power_threshold:
            self.min_power_counter += 1
            if self.min_power_counter <= min_power_time:
                # aircon seems to react to edges
                # so provide as many as possible to quickly reduce power?
                # may be extra helpful in fahrenheit mode? (seemingly not)
                return max(
                    ac_off_threshold + 1, self.prev_unsigned_compressed_error - 1
                )
            else:
                return ac_off_threshold + 2
        else:
            self.min_power_counter = 0

        if self.prev_unsigned_compressed_error > ac_stable_threshold + 1:
            self.deadband_integrator.clear()
            return self.midea_runtime_quirks(ac_stable_threshold + 1)
        elif self.prev_unsigned_compressed_error < ac_stable_threshold - 1:
            self.deadband_integrator.clear()
            return self.midea_runtime_quirks(ac_stable_threshold - 1)

        return self.midea_runtime_quirks(
            ac_stable_threshold + self.deadband_integrator.set(error)
        )

    def midea_runtime_quirks(self, rval):
        rval = round(rval)

        if (
            (self.outer_ramp_count > 0 and rval < ac_stable_threshold)
            or (self.outer_ramp_count < 0 and rval > ac_stable_threshold)
            or (self.outer_ramp_count > step_up_intervals)
            or (self.outer_ramp_count < -step_down_intervals)
        ):
            self.outer_ramp_count = 0

        # restart if the temp goes even further in the direction we're holding
        if (rval > ac_stable_threshold and rval > self.outer_ramp_rval) or (
            rval == self.outer_ramp_rval == ac_stable_threshold + 1
        ):
            self.outer_ramp_rval = rval
            self.outer_ramp_count = 1
        elif (rval < ac_stable_threshold and rval < self.outer_ramp_rval) or (
            rval == self.outer_ramp_rval == ac_stable_threshold - 1
        ):
            self.outer_ramp_rval = rval
            self.outer_ramp_count = -1

        if self.outer_ramp_count > 0:
            self.outer_ramp_count += 1
            rval = self.outer_ramp_rval
        elif self.outer_ramp_count < 0:
            self.outer_ramp_count -= 1
            rval = self.outer_ramp_rval
        else:
            self.outer_ramp_rval = rval

        # behaviour only observed in cooling mode
        # at the setpoint ac will start ramping back power
        # and will continue ramping back at setpoint+1
        # a brief jump up to setpoint+2 is needed to arrest the fall and stabilise output power
        # in heating mode there is no gradual rampdown at all, it drops straight to min power
        if (
            self.prev_unsigned_compressed_error < ac_stable_threshold
            and rval >= ac_stable_threshold
        ):
            return max(rval, ac_stable_threshold + 1)

        return rval
