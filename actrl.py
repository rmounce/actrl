import hassapi as hass  # type: ignore
import math

# from simple_pid import PID
from collections import deque
import time

device_name = "m5atom"
climate_entity = f"climate.{device_name}_climate"
static_pressure_entity = f"number.{device_name}_static_pressure"
follow_me_service = f"esphome/{device_name}_send_follow_me"
compressor_entity = f"binary_sensor.{device_name}_compressor"
outdoor_fan_entity = f"binary_sensor.{device_name}_outdoor_fan"

# Kitchen has 2 ducts, min airflow isn't an issue there
# The rest of the rooms are comparable in size
room_airflow = {
    "bed_1": 1.0,
    "bed_2": 1.0,
    "bed_3": 1.0,
    "kitchen": 2.0,
    "study": 1.0,
}

rooms = list(room_airflow.keys())

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
# match the zone10e step size
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

# Saturate after 14 power increments (slight over-estimate, appears to be closer to 13)
compressor_power_increments = 14

# Additional safety margin when switching between stepping up / down
compressor_power_safety_margin = 3

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

# After 30 mins of struggling and running the compressor at max speed, briefly shut down the system to increase static pressure
max_power_static_pressure_increment_time = int(30 / interval)

# Not much point in complicating matters with a middle ground between efficiency and max output
initial_static_pressure = {"cool": 2, "heat": 2}
max_static_pressure = 4

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

# After reducing to minimum power and holding for a while, turn off at a tighter threshold
eventual_off_threshold = -0.5

# Give up on incremental control and cut to minimum power to avoid overshooting
# This includes the derivative
#min_power_threshold = -1.25
min_power_threshold = -2.0

# Worst case, turn off if we have overshot massively.
immediate_off_threshold = -1.5

# Target 750W surplus power before ramping down
grid_surplus_lower_threshold = 750
# extra 750W buffer when the system is fully off
grid_surplus_off_buffer = 750
# smaller 250W buffer when the system is running
grid_surplus_on_buffer = 250

# per interval
# 1.0C = 1000W for 5 minutes
grid_surplus_ki = interval / (1000 * 5)

# per interval
# 0.2C per minute
# double the rate of statctrl.py, otherwise they cancel each other until the target is reached
grid_surplus_open_window_rate = 0.2 * interval

# Don't wind-up more than 1.0C
grid_surplus_max_offset = 1.0

# set some boundaries before things get too weird
# e.g. during night mode the midpoint of (16+24)/2 = 20, a bit too chilly
grid_surplus_min_cooling = 21
grid_surplus_max_heating = 21

# Offsets for celsius
# 1 is usually sufficient
ac_on_threshold = 1

ac_stable_threshold = 1
ac_off_threshold = -2

null_state = "unknown"

mode_sign = {"cool": 1.0, "heat": -1.0}


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
        self.max_power_counter = 0
        self.off_fan_running_counter = 0
        self.prev_step = 0
        self.guesstimated_comp_speed = int(
            float(self.get_state("input_number.aircon_comp_speed"))
        )
        self.grid_surplus_integral = float(
            self.get_state("input_number.grid_surplus_integral")
        )
        if self.get_state(climate_entity) in ["heat", "cool"]:
            self.mode = self.get_state(climate_entity)
            self.log("ASSUMING THAT THE AIRCON IS ALREADY RUNNING")
            self.compressor_totally_off = False
            self.on_counter = soft_delay + soft_ramp
        else:
            self.mode = "off"
            self.compressor_totally_off = True
            self.on_counter = 0

        self.deadband_integrator = DeadbandIntegrator(
            ki=(global_deadband_ki * 60.0 * interval),
        )
        self.window_handler = WindowStateHandler()

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
        if self.get_state("input_boolean.ac_manual_mode") == "on":
            self.log("Manual mode, skipping")
            return
        self.log("")
        self.log("#### BEGIN CYCLE ####")
        temps = self._get_current_temperatures()
        cur_targets = self._get_current_targets()
        self._update_room_targets(temps, cur_targets)

        self._add_grid_surplus()

        errors, cooling_demand, heating_demand = self._calculate_demand(temps)

        self.get_entity("input_number.grid_surplus_integral").set_state(
            state=self.grid_surplus_integral
        )
        self.log(
            f"heating_demand: {heating_demand:.3f}, cooling_demand: {cooling_demand:.3f}"
        )

        new_mode, demand = self._determine_new_mode(cooling_demand, heating_demand)
        self.log(f"new_mode {new_mode} (old mode {self.mode})")

        celsius_setpoint = float(
            self.get_entity(climate_entity).get_state("temperature")
        )

        if self._handle_mode_change(new_mode, celsius_setpoint):
            return

        self.mode = new_mode

        pid_outputs = self._calculate_pid_outputs(errors)

        # Disabled rooms
        for room in rooms - cur_targets[self.mode].keys():
            if self.get_entity("cover." + room).get_state("current_position") != "0":
                self.log("Closing damper for disabled room: " + room)
                self.set_damper_pos(room, 0, False)

        deriv_sum = 0
        error_sum = 0.0
        weight_sum = 0.0

        damper_vals = {}

        for room, output in pid_outputs.items():
            clamped_output = max(0, output)
            deriv_sum += clamped_output * self.temp_derivs[room].get()
            error_sum += clamped_output * errors[self.mode][room]
            weight_sum += clamped_output

            damper_vals[room] = 100.0 * (clamped_output / normalised_damper_range)

        avg_deriv = deriv_sum / weight_sum

        # Use the state of the zone with the highest demand rather than weighted demand
        # weighted_error = error_sum / weight_sum
        weighted_error = mode_sign[self.mode] * demand

        self.get_entity("input_number.aircon_weighted_error").set_state(
            state=weighted_error
        )
        self.get_entity("input_number.aircon_avg_deriv").set_state(state=avg_deriv)

        unsigned_compressed_error = self.compress(
            weighted_error * mode_sign[self.mode], avg_deriv * mode_sign[self.mode]
        )
        self.prev_unsigned_compressed_error = unsigned_compressed_error

        compressed_error = mode_sign[self.mode] * unsigned_compressed_error
        self.log(
            f"weighted_error: {weighted_error:.3f}, avg_deriv: {avg_deriv:.3f}, compressed_error: {compressed_error}"
        )

        self.on_counter += 1
        if self.get_state("input_boolean.ac_min_power") == "on":
            self.on_counter = min(self.on_counter, soft_delay - 1)

        if (
            self.get_state(climate_entity) == "heat"
            and self.get_state(compressor_entity) == "on"
            and self.get_state(outdoor_fan_entity) == "off"
            and self.guesstimated_comp_speed
            < (compressor_power_increments + compressor_power_safety_margin)
        ):
            self.log(f"Defrost cycle detected, request max speed on restart")
            self.guesstimated_comp_speed = (
                compressor_power_increments + compressor_power_safety_margin
            )

        self.get_entity("input_number.aircon_comp_speed").set_state(
            state=self.guesstimated_comp_speed
        )
        self.log(
            f"compressor_totally_off: {self.compressor_totally_off}, guesstimated_comp_speed: {self.guesstimated_comp_speed}, prev_step: {self.prev_step}"
        )
        self.log(
            f"min_power_counter: {self.min_power_counter}, max_power_counter: {self.max_power_counter}, on_counter: {self.on_counter}"
        )

        if (
            self.get_state(climate_entity) in ["cool", "heat"]
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
            self.get_state(climate_entity) == "off"
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

        was_off = self.get_state(climate_entity) == "off"

        # System is struggling for more than 15 mins, 'change gear' to max airflow
        if self.max_power_counter > max_power_static_pressure_increment_time:
            self._set_static_pressure(max_static_pressure)
        elif was_off:
            self._set_static_pressure(initial_static_pressure[self.mode])

        self.try_set_mode(self.mode)
        self.try_set_fan_mode(self._determine_fan_mode())
        self.get_entity("input_number.aircon_meta_integral").set_state(
            state=self.deadband_integrator.get()
        )
        if was_off:
            # Ensure that an extra follow me update packet is sent
            # And that they are sent AFTER the C3 command to power on the unit
            # Otherwise the initial 'blip' to ac_on_threshold to power on the compressor may not be processed?
            # sometimes 0.5 isn't enough to ensure ordering!
            # bumped up to 1.0s with a break between each message
            time.sleep(1.0)
            self.set_fake_temp(celsius_setpoint, compressed_error, True)
            time.sleep(1.0)
        self.set_fake_temp(celsius_setpoint, compressed_error, True)

    def _set_static_pressure(self, new_static_pressure):
        while int(float(self.get_state(static_pressure_entity))) != new_static_pressure:
            self.log(
                f"CHANGING STATIC PRESSURE FROM {float(self.get_state(static_pressure_entity))} TO {new_static_pressure}"
            )
            self.try_set_mode("off")
            self.call_service(
                "number/set_value",
                entity_id=static_pressure_entity,
                value=new_static_pressure,
            )
            # Typically reported in ~2 seconds
            time.sleep(1)

    def _get_current_temperatures(self):
        temps = {}
        for room in rooms:
            if self.get_state("input_boolean.ac_use_feels_like") == "on":
                feels_like_value = self.get_state("sensor." + room + "_feels_like")
                if feels_like_value is not None:
                    temps[room] = float(feels_like_value)
                else:
                    self.log(
                        f"'sensor.{room}_feels_like' is None. Falling back to 'sensor.{room}_average_temperature'."
                    )
                    temps[room] = float(
                        self.get_state("sensor." + room + "_average_temperature")
                    )
            else:
                temps[room] = float(
                    self.get_state("sensor." + room + "_average_temperature")
                )
            self.window_handler.update(
                room, self.get_state(f"binary_sensor.{room}_window") == "on"
            )
        return temps

    def _get_current_targets(self):
        cur_targets = {"heat": {}, "cool": {}}
        for room in rooms:
            climate_state = self.get_state("climate." + room + "_aircon")
            climate_entity = self.get_entity("climate." + room + "_aircon")

            if climate_state == "heat_cool":
                cur_targets["heat"][room] = climate_entity.get_state("target_temp_low")
                cur_targets["cool"][room] = climate_entity.get_state("target_temp_high")
            elif climate_state == "heat":
                cur_targets["heat"][room] = climate_entity.get_state("temperature")
            elif climate_state == "cool":
                cur_targets["cool"][room] = climate_entity.get_state("temperature")
        return cur_targets

    def _update_room_target(self, room, mode, cur_targets):
        target_delta = cur_targets[mode][room] - self.targets[mode][room]

        if abs(target_delta) <= target_ramp_linear_increment:
            self.targets[mode][room] = cur_targets[mode][room]
        elif abs(target_delta) <= (target_ramp_linear_threshold + 1e-9):
            self.targets[mode][room] += math.copysign(
                target_ramp_linear_increment, target_delta
            )
            self.log(
                f"linearly ramping target room: {room}, smooth target: {str(self.targets[mode][room])}, ultimate target: {str(cur_targets[mode][room])}"
            )
        elif abs(target_delta) <= (target_ramp_step_threshold + 1e-9):
            self.targets[mode][room] += target_delta * target_ramp_proportional
            self.log(
                f"proportionally ramping target room: {room}, smooth target:{str(self.targets[mode][room])}, ultimate target: {str(cur_targets[mode][room])}"
            )
        else:
            self.targets[mode][room] = cur_targets[mode][room] - math.copysign(
                target_ramp_step_threshold, target_delta
            )
            self.log(
                f"stepping target room: {room}, smooth target:{str(self.targets[mode][room])}, ultimate target: {str(cur_targets[mode][room])}"
            )

    def _update_room_targets(self, temps, cur_targets):
        for room in rooms:
            self.temp_derivs[room].set(temps[room], 0)

            if room in cur_targets["heat"]:
                if room in self.targets["heat"]:
                    self._update_room_target(room, "heat", cur_targets)
                else:
                    self.log(f"setting heat target for previously disabled room {room}")
                    self.targets["heat"][room] = cur_targets["heat"][room]
            elif room in self.targets["heat"]:
                self.targets["heat"].pop(room)

            if room in cur_targets["cool"]:
                if room in self.targets["cool"]:
                    self._update_room_target(room, "cool", cur_targets)
                else:
                    self.log(f"setting cool target for previously disabled room {room}")
                    self.targets["cool"][room] = cur_targets["cool"][room]
            elif room in self.targets["cool"]:
                self.targets["cool"].pop(room)

    def _calculate_room_errors(self, temps):
        errors = {"heat": {}, "cool": {}}
        # if every zone and mode overshoots, the integral should saturate to prevent wind-up
        min_grid_surplus_overshoot = float("inf")

        for room in rooms:
            # default
            for mode in errors.keys():
                if room in self.targets[mode]:
                    errors[mode][room] = mode_sign[mode] * (
                        temps[room] - self.targets[mode][room]
                    )

            # room in auto mode with both heat/cool targets; handle grid surplus
            if room in self.targets["heat"] and room in self.targets["cool"]:
                # examples for a room with setpoints of 19 and 25 C
                # (25 - 19 + (-1.5)) / 2 = 2.75
                midpoint_offset = (
                    self.targets["cool"][room]
                    - self.targets["heat"][room]
                    + immediate_off_threshold
                ) / 2
                # 25 - 21 = 4
                cool_offset = self.targets["cool"][room] - grid_surplus_min_cooling
                # 21 - 19 = 2
                heat_offset = grid_surplus_max_heating - self.targets["heat"][room]

                max_mode_offset = {}
                max_mode_offset["cool"] = min(midpoint_offset, cool_offset)
                max_mode_offset["heat"] = min(midpoint_offset, heat_offset)
                open_window_offset = self.window_handler.get_offset(room)

                # self.log(
                #    f"Adjusting {room} offset within limits of heat: {heat_offset:.3f}, midpoint: {midpoint_offset:.3f}, cool: {cool_offset:.3f}, window: {open_window_offset:.3f}"
                # )

                if midpoint_offset <= 0:
                    self.log(
                        f"WARNING: heat/cool targets for room {room} are within {immediate_off_threshold} C of each other"
                    )
                else:
                    for mode in errors.keys():
                        window_limited_offset = max(
                            0,
                            min(
                                max_mode_offset[mode],
                                self.grid_surplus_integral,
                            )
                            - open_window_offset,
                        )
                        grid_surplus_overshoot = max(
                            0, self.grid_surplus_integral - window_limited_offset
                        )
                        min_grid_surplus_overshoot = min(
                            min_grid_surplus_overshoot, grid_surplus_overshoot
                        )
                        if (
                            self.get_state(f"input_boolean.ac_use_grid_surplus_{mode}")
                            == "on"
                        ):
                            errors[mode][room] += window_limited_offset

        if min_grid_surplus_overshoot < float("inf"):
            self.grid_surplus_integral -= min_grid_surplus_overshoot

        return errors

    def _calculate_demand(self, temps):
        errors = self._calculate_room_errors(temps)
        cooling_demand = max(errors["cool"].values(), default=float("-inf"))
        heating_demand = max(errors["heat"].values(), default=float("-inf"))

        demand_beyond_grid_surplus_max_offset = (
            max(cooling_demand, heating_demand) - grid_surplus_max_offset
        )
        if demand_beyond_grid_surplus_max_offset > 0:
            self.grid_surplus_integral -= demand_beyond_grid_surplus_max_offset
            self.grid_surplus_integral = max(0, self.grid_surplus_integral)
            errors = self._calculate_room_errors(temps)
            cooling_demand = max(errors["cool"].values(), default=float("-inf"))
            heating_demand = max(errors["heat"].values(), default=float("-inf"))
        return errors, cooling_demand, heating_demand

    def _determine_new_mode(self, cooling_demand, heating_demand):
        if self.get_state(climate_entity) == "cool" and cooling_demand > (
            heating_demand + immediate_off_threshold
        ):
            return "cool", cooling_demand
        elif self.get_state(climate_entity) == "heat" and heating_demand > (
            cooling_demand + immediate_off_threshold
        ):
            return "heat", heating_demand
        elif cooling_demand > heating_demand:
            return "cool", cooling_demand
        elif heating_demand > cooling_demand:
            return "heat", heating_demand
        return None

    def _handle_mode_change(self, new_mode, celsius_setpoint):
        if new_mode is None or (self.mode is not None and (new_mode != self.mode)):
            self.mode = new_mode
            for room, pid in self.pids.items():
                pid.clear()
            self.compressor_totally_off = True
            self.on_counter = 0
            self.deadband_integrator.clear()
            if self.get_state(climate_entity) != "fan_only":
                self.try_set_mode("off")
            self.set_fake_temp(celsius_setpoint, ac_stable_threshold, False)
            self._reset_metrics()
            return True
        return False

    def _reset_metrics(self):
        self.get_entity("input_number.aircon_weighted_error").set_state(
            state=null_state
        )
        self.get_entity("input_number.aircon_avg_deriv").set_state(state=null_state)
        self.get_entity("input_number.aircon_meta_integral").set_state(state=null_state)

    def _add_grid_surplus(self):
        grid_surplus = -float(
            self.get_state("sensor.power_grid_fronius_power_flow_0_fronius_lan")
        )
        # 10kW solar 24/7, yeah that'd be nice
        # grid_surplus = 10000.0

        grid_surplus_upper_threshold = grid_surplus_lower_threshold + (
            grid_surplus_off_buffer
            if self.compressor_totally_off
            else grid_surplus_on_buffer
        )

        if grid_surplus > grid_surplus_upper_threshold:
            self.grid_surplus_integral += grid_surplus_ki * (
                grid_surplus - grid_surplus_upper_threshold
            )
        elif grid_surplus < grid_surplus_lower_threshold:
            self.grid_surplus_integral += grid_surplus_ki * (
                grid_surplus - grid_surplus_lower_threshold
            )

        self.grid_surplus_integral = max(0.0, self.grid_surplus_integral)

    def _calculate_pid_outputs(self, errors):
        # Calculate raw PID outputs
        pid_outputs = {}
        for room, error in errors[self.mode].items():
            if not self.rooms_enabled[room]:
                self.pids[room].clear()
                self.rooms_enabled[room] = True

            self.pids[room].update(
                error,
                mode_sign[self.mode] * self.targets[self.mode][room],
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

        # Measured fan power usage at different static pressure settings
        sp_power = {}
        sp_power[1] = {"low": 44, "medium": 60.5, "high": 80.5}
        sp_power[2] = {"low": 97, "medium": 115, "high": 144}
        sp_power[3] = {"low": 169, "medium": 187, "high": 210}
        sp_power[4] = {"low": 236, "medium": 261.5, "high": 293}

        # SP2 low speed is used as a baseline for a single open duct
        baseline_airflow = 1.0 - 1e-9
        baseline_power = sp_power[2]["low"]

        cur_static_pressure = int(float(self.get_state(static_pressure_entity)))
        cur_fan_speed = self.get_entity(climate_entity).get_state("fan_mode")

        # Default to the safest values
        if cur_static_pressure not in sp_power:
            cur_static_pressure = 4

        if cur_fan_speed not in sp_power[cur_static_pressure]:
            cur_fan_speed = "high"

        min_airflow = baseline_airflow * (
            sp_power[cur_static_pressure][cur_fan_speed] / baseline_power
        )

        # self.log(f"Door closed for {top_zone}, ensuring minimum airflow")
        min_sum = min_airflow * normalised_damper_range

        adjusted_room_airflow = {
            room: airflow * (0.25 if not self.get_door_state(room) else 1.0)
            for room, airflow in room_airflow.items()
        }

        while True:
            positive_outputs = {
                room: max(0, output * adjusted_room_airflow[room])
                for room, output in pid_outputs.items()
            }
            if sum(positive_outputs.values()) >= min_sum:
                break
            no_integral_adjusted = True
            for room in pid_outputs:
                if pid_outputs[room] < normalised_damper_range:
                    no_integral_adjusted = False
                    self.pids[room].adjust_integral(0.0001)
                    pid_outputs[room] = self.pids[room].get_output()
            if no_integral_adjusted:
                # Too few zones enabled to satisfy minimum airflow.. not much we can do
                break

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
                f"{room} adjusted PID output: {pid_outputs[room]:.3f} (P: {self.pids[room].p_term:.3f}, I: {self.pids[room].i_term:.3f}, D: {self.pids[room].deriv.get():.3f})"
            )
        return pid_outputs

    def get_door_state(self, room):
        entity_id = f"binary_sensor.{room}_door"
        state = self.get_state(entity_id)
        return True if state is None else state == "on"

    def set_fake_temp(self, celsius_setpoint, compressed_error, transmit=True):
        self.get_entity("input_number.fake_temperature").set_state(
            state=(celsius_setpoint + compressed_error)
        )
        if not transmit:
            return
        # Power on, FM update, mode auto, Fan auto, setpoint 25C?, room temp
        # self.call_service(
        #    "esphome/infrared_send_raw_command",
        #    command=[
        #        0xA4,
        #        0x82,
        #        0x48,
        #        0x7F,
        #        (int)(celsius_setpoint + compressed_error + 1),
        #    ],
        # )
        self.call_service(
            follow_me_service,
            temperature=celsius_setpoint + compressed_error,
        )
        time.sleep(0.1)

    def set_damper_pos(self, room, damper_val, open_only=False):
        actual_cur_pos = float(
            self.get_entity("cover." + room).get_state("current_position")
        )
        if abs(self.damper_pos[room] - actual_cur_pos) > 5:
            self.damper_pos[room] = actual_cur_pos
        cur_pos = self.damper_pos[room]

        damper_log = f"{room} damper scaled: {damper_val:.3f}, cur_pos: {cur_pos}, actual_cur_pos: {actual_cur_pos}"
        self.get_entity("input_number." + room + "_damper_target").set_state(
            state=damper_val
        )

        cur_deadband = damper_deadband

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

    def try_set_mode(
        self,
        mode,
    ):
        if self.get_state(climate_entity) != mode:
            self.call_service(
                "climate/set_hvac_mode", entity_id=climate_entity, hvac_mode=mode
            )
            # workaround to retransmit IR code
            time.sleep(0.1)
            self.call_service(
                "climate/set_hvac_mode", entity_id=climate_entity, hvac_mode=mode
            )
            time.sleep(0.1)

    def _determine_fan_mode(self):
        current_fan_mode = self.get_entity(climate_entity).get_state("fan_mode")

        low_to_medium = compressor_power_safety_margin
        medium_to_low = 0

        medium_to_high = compressor_power_increments
        high_to_medium = compressor_power_increments - compressor_power_safety_margin

        # Determine fan speed with hysteresis
        if self.guesstimated_comp_speed >= medium_to_high:
            return "high"
        elif self.guesstimated_comp_speed <= medium_to_low:
            return "low"
        elif (
            current_fan_mode == "high"
            and self.guesstimated_comp_speed >= high_to_medium
        ):
            return "high"
        elif (
            current_fan_mode == "low" and self.guesstimated_comp_speed <= low_to_medium
        ):
            return "low"
        else:
            return "medium"

    def try_set_fan_mode(self, fan_mode):
        if self.get_entity(climate_entity).get_state("fan_mode") != fan_mode:
            self.call_service(
                "climate/set_fan_mode", entity_id=climate_entity, fan_mode=fan_mode
            )
            # workaround to retransmit IR code
            time.sleep(0.1)
            self.call_service(
                "climate/set_fan_mode", entity_id=climate_entity, fan_mode=fan_mode
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
                print(f"starting compressor {ac_on_threshold}")

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
            print("soft start, on_counter: " + str(self.on_counter))
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

        # Reset any step that may have been in progress
        # Failsafe, should never be reachable
        if self.prev_step != 0:
            self.prev_step = 0
            self.log("Resetting self.prev_step which shouldn't have been set anyway")


        # Begin step up sequence, unless already at max power
        if rval == ac_stable_threshold + 1 and self.guesstimated_comp_speed < (
            compressor_power_increments + compressor_power_safety_margin
        ):
            self.guesstimated_comp_speed = max(
                compressor_power_safety_margin, self.guesstimated_comp_speed + 1
            )
            self.prev_step = 1
            return ac_stable_threshold + step_up_sequence[0]

        # Begin step down sequence, unless already at min power
        if rval == ac_stable_threshold - 1 and self.guesstimated_comp_speed > 0:
            self.guesstimated_comp_speed = min(
                compressor_power_increments,
                self.guesstimated_comp_speed - 1,
            )
            self.prev_step = -1
            return ac_stable_threshold + step_down_sequence[0]


        # Bypass the stepping behaviour for extreme errors above faithful_threshold
        # in favor of simple hysteresis
        if rval > ac_stable_threshold + 1:
            # Assume that there is demand for max power (plus lower and upper safety margin)
            self.guesstimated_comp_speed = (
                compressor_power_increments + compressor_power_safety_margin
            )
            self.max_power_counter += 1

            # Simple hysteresis, provide a stable value and keep commanding max power when
            # we drop from a huge error to a smaller (but still substantial) error.
            # Otherwise a "downward step" ends up briefly reducing power output while we're
            # still struggling to reach the target!
            if self.prev_unsigned_compressed_error > rval:
                return rval + 1
            else:
                return rval

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
            self.guesstimated_comp_speed = -minimum_temp_intervals

        # Saturated, just keep demanding a compressor speed decrease
        if self.guesstimated_comp_speed <= 0:
            rval = min(ac_stable_threshold - 1, rval)

        # After 5 minutes worth of consecutive decrements assume that we want the absolute minimum power
        if self.guesstimated_comp_speed <= -minimum_temp_intervals:
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
