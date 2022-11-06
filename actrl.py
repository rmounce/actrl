import hassapi as hass
import datetime
import math

# from simple_pid import PID
from collections import deque
import time

rooms = ["bed_1", "bed_2", "bed_3", "kitchen", "study"]

# in minutes
interval = 0.25  # 15 seconds

# WMA over the last 7.5 minutes
global_temp_deriv_window = 7.5
# to predict 10 minutes into the future
global_temp_deriv_factor = 10.0

# report 0.2 degree offset to aircon as 1 degree, 5x amplification
global_compress_factor = 0.2

# per second
global_ki = 0.00025
# a 0.1 deg error will accumulate 0.1 in ~60 minutes


room_kp = 0.5

# per second
room_ki = 0.0005
# a 0.1 deg error will accumulate 0.1 in ~30 minutes

# kd considers the last 15 minutes
room_deriv_window = 15.0
# for a constant 1.0 deg / min over the last 15 mins, swing full scale
# or for 1 degree every 5 mins, contribute 0.66
room_deriv_factor = 2.0

# percent
damper_deadband = 5.0
damper_round = 5

# soft start to avoid overshoot
# start at min power and gradually report the actual rval
# 5 min delay at min power
# covers a defrost cycle
soft_delay = int(7.5 / interval)
# then gradually report the actual rval over 5 mins
soft_ramp = int(7.5 / interval)


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
        print(" error " + str(error) + " target " + str(target))
        val = error - target
        self.last_val = val
        self.integral += val
        self.deriv.set(error, target)
        # clamp between +-1
        if not self.clamp_low <= self.get() <= self.clamp_high:
            if self.get() > self.clamp_high:
                clamp_to = self.clamp_high
            else:
                clamp_to = self.clamp_low
            self.integral = (
                -(clamp_to + self.deriv.get() + (self.last_val * self.kp)) / self.ki
            )

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


class Actrl(hass.Hass):
    def initialize(self):
        self.pids = {}
        self.rooms_enabled = {}
        self.temp_deriv = MyDeriv(
            window=int(global_temp_deriv_window / interval),
            factor=int(global_temp_deriv_window / interval),
        )
        self.temp_integral = MySimplerIntegral(
            ki=(global_ki * 60.0 * interval),
            clamp_low=-global_compress_factor,
            clamp_high=global_compress_factor,
        )
        self.ramping_down = True
        self.totally_off = True
        self.heat_mode = False
        self.on_counter = 0

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
        temps = {}
        targets = {}
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
                temps[room] = float(
                    self.get_state("sensor." + room + "_average_temperature")
                )
                targets[room] = float(
                    self.get_entity("climate." + room + "_aircon").get_state(
                        "temperature"
                    )
                )
                errors[room] = temps[room] - targets[room]
                if self.get_state("climate." + room + "_aircon") == "heat":
                    heat_room_count += 1
                if self.get_state("climate." + room + "_aircon") == "cool":
                    cool_room_count += 1
            else:
                disabled_rooms.append(room)

        if all_disabled:
            self.get_entity("input_number.fake_temperature").set_state(
                state=main_setpoint
            )
            self.get_entity("input_number.aircon_weighted_error").set_state(
                state=float("nan")
            )
            self.get_entity("input_number.aircon_integrated_error").set_state(
                state=float("nan")
            )
            for room, pid in self.pids.items():
                pid.clear()
            self.temp_deriv.clear()
            self.temp_integral.clear()
            self.totally_off = True
            self.on_counter = 0
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
            self.try_set_mode("heat")
        else:
            self.heat_mode = False
            self.try_set_mode("cool")

        avg_error = sum(errors.values()) / len(errors.values())

        for room, error in errors.items():
            if not self.rooms_enabled[room]:
                self.pids[room].clear()
                self.rooms_enabled[room] = True
                # a new room has been enabled, reset the deriv history
                self.temp_deriv.clear()
                # and return half-way through soft-start
                self.on_counter = min(self.on_counter, soft_delay)

            print("setting " + room)
            self.pids[room].set(error, avg_error)
            pid_vals[room] = heat_cool_sign * self.pids[room].get()
            print("outcome was " + str(pid_vals[room]))
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
                self.log("closing damper for disabled room " + room)
                self.set_damper_pos(room, 0)

        min_pid = min(pid_vals.values())

        error_sum = 0.0
        target_sum = 0.0
        weight_sum = 0.0

        for room, pid_val in pid_vals.items():
            scaled = 100.0 * ((1.01 - pid_val) / (1.01 - min_pid))

            damper_vals[room] = scaled
            target_sum += targets[room] * scaled
            error_sum += errors[room] * scaled
            weight_sum += scaled

        weighted_error = error_sum / weight_sum
        weighted_target = target_sum / weight_sum

        self.log(
            "oc " + str(self.on_counter) + " error weighted " + str(weighted_error)
        )

        self.temp_deriv.set(weighted_error, weighted_target)
        avg_deriv = self.temp_deriv.get()

        self.get_entity("input_number.aircon_weighted_error").set_state(
            state=weighted_error
        )
        self.temp_integral.set(weighted_error)
        self.get_entity("input_number.aircon_integrated_error").set_state(
            state=self.temp_integral.get()
        )
        weighted_error += self.temp_integral.get()

        weighted_error *= heat_cool_sign
        avg_deriv *= heat_cool_sign

        compressed_error = heat_cool_sign * self.compress(weighted_error, avg_deriv)
        self.log(
            "deriv "
            + str(avg_deriv)
            + " integral "
            + str(self.temp_integral.get())
            + " compressed "
            + str(compressed_error)
        )
        self.get_entity("input_number.fake_temperature").set_state(
            state=(main_setpoint + compressed_error)
        )

        self.on_counter += 1

        if False:
            self.log("indoor fan is not running, not adjusting dampers")
        else:
            for room in sorted(damper_vals, key=damper_vals.get, reverse=True):
                # damper_val = damper_vals[room]
                self.set_damper_pos(room, damper_vals[room])

    def set_damper_pos(self, room, damper_val):
        cur_pos = float(self.get_entity("cover." + room).get_state("current_position"))
        self.get_entity("input_number." + room + "_damper").set_state(state=cur_pos)
        self.log(room + " scaled: " + str(damper_val) + " cur_pos: " + str(cur_pos))
        self.get_entity("input_number." + room + "_damper_target").set_state(
            state=damper_val
        )

        # damper won't do much if the fan isn't running
        cur_deadband = (
            (2.0 * damper_deadband)
            if (self.totally_off and self.heat_mode)
            else damper_deadband
        )

        if (
            (damper_val > 99.9 and cur_pos < 100.0)
            or (damper_val > (cur_pos + cur_deadband))
            or (damper_val < (cur_pos - cur_deadband))
        ):
            self.log("setting " + room)
            rounded_damper_val = damper_round * round(damper_val / damper_round)
            self.call_service(
                "cover/set_cover_position",
                entity_id=("cover." + room),
                position=rounded_damper_val,
            )
            time.sleep(0.1)
        else:
            self.log("within deadband, not setting " + room)

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
        if self.heat_mode:
            on_threshold = 1
        else:
            on_threshold = 2

        off_threshold = -2

        rval = self.actually_compress(error)

        # conditions in which to consider the derivative
        # - aircon is currently running
        # - the current temp (ignoring RoC) has not yet reached off threshold
        # goal of the derivative is to proactively reduce/increase compressor power, but not to influence on/off state
        if not self.totally_off and (round(rval) > off_threshold):
            rval = max((off_threshold + 1), self.actually_compress(error + deriv))

        rval = min(rval, 15.0)
        rval = max(rval, -15.0)
        unrounded_rval = rval
        rval = round(rval)

        if rval <= off_threshold:
            self.totally_off = True

        if self.totally_off:
            self.on_counter = 0
            if rval < on_threshold:
                return rval

        self.totally_off = False

        if rval > on_threshold:
            if self.on_counter < soft_delay:
                print("soft start, on_counter: " + str(self.on_counter))
                return on_threshold
            if self.on_counter < (soft_delay + soft_ramp):
                ramp_progress = (self.on_counter - soft_delay) / soft_ramp
                print(
                    "ramping "
                    + str(ramp_progress)
                    + ", on_counter: "
                    + str(self.on_counter)
                )
                return round(
                    (ramp_progress * unrounded_rval)
                    + ((1.0 - ramp_progress) * on_threshold)
                )

        # behaviour only observed in cooling mode
        # at the setpoint ac will start ramping back power
        # and will continue ramping back at setpoint+1
        # a brief jump up to setpoint+2 is needed to arrest the fall and stabilise output power
        # in heating mode there is no gradual rampdown at all, it drops straight to min power
        if rval <= 0:
            self.ramping_down = True
            return rval

        if self.ramping_down:
            self.ramping_down = False
            return max(rval, on_threshold)

        return rval
