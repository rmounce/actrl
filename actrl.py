import hassapi as hass  # type: ignore
import math

# from simple_pid import PID
from collections import deque
from datetime import datetime, timezone
import time

from control import (
    forecast_hours_ahead,
    price_pressure_offset,
    MyWMA,
    MyDeriv,
    MyPID,
    DeadbandIntegrator,
    WindowStateHandler,
    MideaCapacityController,
    interval,
    soft_delay,
    soft_ramp,
    compressor_power_increments,
    compressor_power_safety_margin,
    immediate_off_threshold,
    ac_stable_threshold,
    ac_off_threshold,
)

device_name = "m5atom"
climate_entity = f"climate.{device_name}_climate"
static_pressure_entity = f"number.{device_name}_static_pressure"
# ESPHome user services are prefixed with the node name, not the retained
# friendly/entity-name prefix used by the migrated Home Assistant entities.
follow_me_service = "esphome/hvac_xye_send_follow_me"
compressor_entity = f"binary_sensor.{device_name}_compressor"
outdoor_fan_entity = f"binary_sensor.{device_name}_outdoor_fan"
actrl_status_entity = "sensor.actrl_status"
debug_logging_entity = "input_boolean.actrl_debug_logging"
local_inhibit_entity = "switch.hvac_xye_m5atom_local_inhibit"

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

# WMA over the last 5 minutes
global_temp_deriv_window = 5
# to predict 10 minutes into the future
global_temp_deriv_factor = 10.0

# per second
global_ki = 0.00025
# a 0.1 deg error will accumulate 0.1 in ~60 minutes

# swing full scale across 2.0C of error
normalised_damper_range = 2.0

# leave this as 1 so that all PID values are in degrees celsius
room_kp = 1.0

# per second
room_ki = 0.001
# a 0.1 deg error will accumulate 0.1 in ~15 minutes

# One-shot catch-up for deliberate target steps; small adjustments retain
# normal relative integration. Thresholds are in degrees Celsius.
room_activation_step = 1.0
room_activation_error = 0.5

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

# Target 750W surplus power before ramping down
grid_surplus_lower_threshold = 300
# extra buffer when the system is fully off
grid_surplus_off_buffer = 200
# smaller buffer when the system is running
grid_surplus_on_buffer = 100

# per interval
# 1.0C = 2000W for 30 seconds
# or, 2000W of a single 5 minute interval over the next 60 minutes for 6 minutes
grid_surplus_ki = interval / (2000 * 0.5)

# Don't wind-up more than 1.0C
grid_surplus_max_offset = 1.0

# set some boundaries before things get too weird
# e.g. during night mode the midpoint of (16+24)/2 = 20, a bit too chilly
grid_surplus_min_cooling = 21
grid_surplus_max_heating = 21

# Price-aware target pressure (docs/pricing.md "Continuous price offset"):
# a continuous K offset from live vs forecast retail prices -- pre-heat/
# pre-cool when the coming hours are dearer than now, back off when now is
# the expensive hour. Gated by input_boolean.ac_use_price_pressure;
# missing/off (and any missing price entity) means offset 0 and behaviour
# identical to before. The forecast comes from the EMHASS-published
# unit-load-cost series (already tariffed, follows whichever price source
# EMHASS is configured with) -- same feed the HWC planner consumes.
price_pressure_boolean = "input_boolean.ac_use_price_pressure"
price_forecast_entity = "sensor.dh_unit_load_cost"
price_forecast_attr = "unit_load_cost_forecasts"
price_now_entity = "sensor.amber_5min_current_general_price"

null_state = "unknown"

mode_sign = {"cool": 1.0, "heat": -1.0}

# Output->damper convexity; 1.0 = linear (historical behaviour). 1.5
# adopted 2026-07-06: widens sub-K zone contrast with winter texture
# unchanged; 2.0 = +50% kitchen damper duty, 3.0 hunts. Position->flow
# calibration (docs/calibration.md) found the real register curve linear
# to mildly convex, so sim effect sizes hold. See damper_share() docstring
# and docs/tuning.md "Damper contrast shaping".
damper_share_gamma = 1.5


def damper_share(output):
    """Fraction of fully-open damper commanded for a PID output.

    Linear today: max(0, output) / normalised_damper_range. Single source of
    truth for the output->damper mapping -- used for the damper command, the
    minimum-airflow check and the demand/deriv weighting, so a shaped
    (convex) variant swapped in via analysis/ctrl_overrides.py stays
    self-consistent (the min-airflow constraint is physical: it must be met
    by actual damper openings, not by output-space bookkeeping).

    Deliberately unclipped above 1.0: renorm float epsilon can leave the top
    output a hair over range and the historical damper command passed that
    through (goldens encode it).

    damper_share_gamma > 1 makes the mapping convex: more damper contrast
    for the same output contrast, tightening sub-K zone rebalancing that
    the renorm otherwise floors. Swept in docs/tuning.md "Damper contrast
    shaping": 1.5 = winter texture provably unchanged; 2.0 = best tracking
    /-5% scenario energy but ~+50% kitchen damper duty; 3.0 HUNTS (never).
    """
    base = max(0.0, output) / normalised_damper_range
    if damper_share_gamma != 1.0:
        base **= damper_share_gamma
    return base


def min_airflow_inflation(pids, pid_outputs, adjusted_room_airflow, min_sum):
    """Ensure enough weighted positive PID output to satisfy minimum airflow.

    Stateless: tops up this cycle's pid_outputs by an equal increment across
    all rooms below full range until the airflow-weighted sum of positive
    outputs reaches min_sum. Integrals are never written, so the top-up
    carries no memory -- the moment other zones' demand covers min_sum, a
    satisfied zone's damper falls straight back to its true PID value.
    (The stateful predecessor wrote the top-up into i_term, leaving
    counterfeit demand that took ~tens of minutes of room_ki to unwind and
    ratcheted satisfied zones up relative to the range-capped top zone --
    study in docs/tuning.md "Min-airflow inflation policy".)

    Requires the negative-integral clamp to run BEFORE this pass: the top-up
    no longer props raw outputs up, so the clamp must see raw outputs or a
    satisfied zone's integral winds down unboundedly behind a healthy-looking
    topped-up output.

    The equal increment is solved by bisection on the airflow delivered as a
    function of the increment (monotone; the continuous limit of the old
    0.0001-step loop). Airflow is measured through damper_share() so the
    constraint holds for the dampers physically commanded, whatever the
    output->damper mapping. `pids` is unused but kept so analysis policies
    remain drop-in substitutes via analysis/ctrl_overrides.py.
    """

    def delivered(delta):
        return normalised_damper_range * sum(
            damper_share(min(o + delta, normalised_damper_range))
            * adjusted_room_airflow[room]
            for room, o in pid_outputs.items()
        )

    below_range = [r for r, o in pid_outputs.items() if o < normalised_damper_range]
    if delivered(0.0) >= min_sum or not below_range:
        # Nothing to do, or too few zones enabled to satisfy minimum airflow
        return
    hi = max(normalised_damper_range - pid_outputs[r] for r in below_range)
    if delivered(hi) > min_sum:
        lo = 0.0
        for _ in range(80):
            mid = 0.5 * (lo + hi)
            if delivered(mid) < min_sum:
                lo = mid
            else:
                hi = mid
    for room in below_range:
        pid_outputs[room] = min(
            pid_outputs[room] + hi, normalised_damper_range
        )


class Actrl(hass.Hass):
    def initialize(self):
        self._set_debug_logging(
            debug_logging_entity,
            "state",
            None,
            self.get_state(debug_logging_entity),
            {},
        )
        self.listen_state(self._set_debug_logging, debug_logging_entity)
        self.log("INITIALISING")
        self.pids = {}
        self.temp_derivs = {}
        self.targets = {"heat": {}, "cool": {}}
        self.previous_requested_targets = {"heat": {}, "cool": {}}
        self.activation_steps = {"heat": set(), "cool": set()}
        self.rooms_enabled = {}
        self.damper_pos = {}
        self.pause_reason = None
        self.off_fan_running_counter = 0
        self.capacity = MideaCapacityController(
            log=self.log,
            debug=lambda message: self.log(message, level="DEBUG"),
        )
        self.capacity.guesstimated_comp_speed = int(
            float(self.get_state("input_number.aircon_comp_speed"))
        )
        self.grid_surplus_integral = float(
            self.get_state("input_number.grid_surplus_integral")
        )
        if self.get_state(climate_entity) in ["heat", "cool"]:
            self.mode = self.get_state(climate_entity)
            self.log("ASSUMING THAT THE AIRCON IS ALREADY RUNNING")
            self.capacity.compressor_totally_off = False
            self.capacity.on_counter = soft_delay + soft_ramp
        else:
            self.mode = "off"
            self.capacity.compressor_totally_off = True
            self.capacity.on_counter = 0

        self.window_handler = WindowStateHandler()
        self.missing_room_climate_entities = set()

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
        self._publish_status("initializing")

    def _set_debug_logging(self, entity, attribute, old, new, kwargs):
        """Apply the HA debug toggle immediately, without reloading this app."""
        enabled = new == "on"
        if old is not None and enabled:
            self.log("Debug logging enabled from Home Assistant")
        elif old is not None:
            self.log("Debug logging disabled from Home Assistant")
        self.set_log_level("DEBUG" if enabled else "INFO")

    def _publish_status(
        self,
        status,
        *,
        mode=None,
        plant_mode=None,
        lead_room=None,
        lead_temperature=None,
        lead_target=None,
        demand=None,
        active_rooms=0,
    ):
        """Publish the stable display/monitoring contract for this controller."""
        attributes = {
            "friendly_name": "actrl HVAC Status",
            "mode": mode or self.mode or "off",
            "plant_mode": plant_mode or self.get_state(climate_entity) or "unknown",
            "active_rooms": int(active_rooms),
            # AppDaemon's HA REST adapter prunes False recursively, and numeric
            # zero compares equal to False. Preserve a stopped compressor by
            # using the same string workaround as aircon_comp_speed.
            "capacity_step": str(int(self.capacity.guesstimated_comp_speed)),
            "capacity_max": compressor_power_increments,
            # Always changes, even when the status does not, and gives consumers
            # an explicit liveness signal for the ten-second control loop.
            "heartbeat": int(time.time()),
        }
        optional_attributes = {
            "lead_room": lead_room,
            "lead_temperature": lead_temperature,
            "lead_target": lead_target,
            "demand": demand,
            "grid_surplus_offset": self.grid_surplus_integral,
        }
        attributes.update(
            {
                key: value
                for key, value in optional_attributes.items()
                if value is not None
                and (not isinstance(value, float) or math.isfinite(value))
            }
        )
        self.get_entity(actrl_status_entity).set_state(
            state=status, attributes=attributes
        )

    def _get_pause_reason(self):
        if self.get_state(local_inhibit_entity) == "on":
            return "inhibited"
        if self.get_state("input_boolean.ac_manual_mode") == "on":
            return "manual"
        return None

    def _pause_control(self, reason):
        if reason != self.pause_reason:
            self.log(f"{reason.capitalize()} mode active, resetting internal state")
            self.pause_reason = reason
        self._reset_internal_state()
        self._publish_status(reason, mode="off", plant_mode="off")

    @staticmethod
    def _lead_context(mode, errors, temps, targets):
        if mode is None or not errors.get(mode):
            return {}
        lead_room = max(errors[mode], key=errors[mode].get)
        return {
            "lead_room": lead_room,
            "lead_temperature": temps[lead_room],
            "lead_target": targets[mode][lead_room],
            "active_rooms": len(errors[mode]),
        }

    def main(self, kwargs):
        pause_reason = self._get_pause_reason()
        if pause_reason is not None:
            self._pause_control(pause_reason)
            return
        if self.pause_reason is not None:
            self.log(f"{self.pause_reason.capitalize()} mode released, resuming control")
            self.pause_reason = None
        self.log("#### BEGIN CYCLE ####", level="DEBUG")
        temps = self._get_current_temperatures()
        cur_targets = self._get_current_targets()
        self._update_room_targets(temps, cur_targets)

        self._add_grid_surplus()
        self.price_pressure = self._get_price_pressure()

        errors, cooling_demand, heating_demand = self._calculate_demand(temps)
        cooling_demand, heating_demand = self._apply_price_pressure(
            errors, cooling_demand, heating_demand
        )

        self.get_entity("input_number.grid_surplus_integral").set_state(
            state=str(float(self.grid_surplus_integral))
        )
        self.log(
            f"heating_demand: {heating_demand:.3f}, cooling_demand: {cooling_demand:.3f}",
            level="DEBUG",
        )

        new_mode, demand = self._determine_new_mode(cooling_demand, heating_demand)
        if new_mode != self.mode:
            self.log(f"new_mode {new_mode} (old mode {self.mode})")
        else:
            self.log(f"new_mode {new_mode} (old mode {self.mode})", level="DEBUG")
        status_context = self._lead_context(new_mode, errors, temps, self.targets)
        status_context.update(
            {
                "mode": new_mode or "off",
                "demand": mode_sign[new_mode] * demand if new_mode is not None else None,
            }
        )

        celsius_setpoint = float(
            self.get_entity(climate_entity).get_state("temperature")
        )

        if self._handle_mode_change(new_mode, celsius_setpoint):
            self._publish_status(
                "idle" if new_mode is None else "switching",
                plant_mode="off",
                **status_context,
            )
            return

        self.mode = new_mode

        pid_outputs = self._calculate_pid_outputs(errors)

        # Disabled rooms
        for room in set(rooms) - set(cur_targets[self.mode]):
            if self.get_entity("cover." + room).get_state("current_position") != "0":
                self.log("Closing damper for disabled room: " + room)
                self.set_damper_pos(room, 0, False)

        deriv_sum = 0
        error_sum = 0.0
        weight_sum = 0.0

        damper_vals = {}

        for room, output in pid_outputs.items():
            # Weight by commanded damper share (== clamped output while the
            # mapping is linear) so demand/deriv stay consistent with the
            # air actually delivered if the mapping is reshaped.
            share_weight = damper_share(output) * normalised_damper_range
            deriv_sum += share_weight * self.temp_derivs[room].get()
            error_sum += share_weight * errors[self.mode][room]
            weight_sum += share_weight

            damper_vals[room] = 100.0 * damper_share(output)

        avg_deriv = deriv_sum / weight_sum if weight_sum > 0 else 0.0

        # Use the state of the zone with the highest demand rather than weighted demand
        # weighted_error = error_sum / weight_sum
        weighted_error = mode_sign[self.mode] * demand

        self.get_entity("input_number.aircon_weighted_error").set_state(
            state=str(float(weighted_error))
        )
        self.get_entity("input_number.aircon_avg_deriv").set_state(state=str(avg_deriv))

        unsigned_compressed_error = self.capacity.compress(
            weighted_error * mode_sign[self.mode], avg_deriv * mode_sign[self.mode]
        )
        self.capacity.prev_unsigned_compressed_error = unsigned_compressed_error

        compressed_error = mode_sign[self.mode] * unsigned_compressed_error
        self.log(
            f"weighted_error: {weighted_error:.3f}, avg_deriv: {avg_deriv:.3f}, compressed_error: {compressed_error}",
            level="DEBUG",
        )

        self.capacity.on_counter += 1
        if self.get_state("input_boolean.ac_min_power") == "on":
            self.capacity.on_counter = min(self.capacity.on_counter, soft_delay - 1)

        if (
            self.get_state(climate_entity) == "heat"
            and self.get_state(compressor_entity) == "on"
            and self.get_state(outdoor_fan_entity) == "off"
            and self.capacity.guesstimated_comp_speed
            < (compressor_power_increments + compressor_power_safety_margin)
        ):
            self.log(f"Defrost cycle detected, request max speed on restart")
            self.capacity.guesstimated_comp_speed = (
                compressor_power_increments + compressor_power_safety_margin
            )

        self.get_entity("input_number.aircon_comp_speed").set_state(
            state=str(float(self.capacity.guesstimated_comp_speed))
        )
        self.log(
            f"compressor_totally_off: {self.capacity.compressor_totally_off}, guesstimated_comp_speed: {self.capacity.guesstimated_comp_speed}, prev_step: {self.capacity.prev_step}",
            level="DEBUG",
        )
        self.log(
            f"min_power_counter: {self.capacity.min_power_counter}, max_power_counter: {self.capacity.max_power_counter}, on_counter: {self.capacity.on_counter}",
            level="DEBUG",
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
            self.capacity.on_counter = 0
            for room in sorted(damper_vals, key=damper_vals.get, reverse=True):
                self.set_damper_pos(room, damper_vals[room], True)
            self._publish_status(
                "idle",
                plant_mode="off",
                **status_context,
            )
            return
        else:
            for room in sorted(damper_vals, key=damper_vals.get, reverse=True):
                self.set_damper_pos(room, damper_vals[room], False)

        was_off = self.get_state(climate_entity) == "off"

        # System is struggling for more than 30 mins, 'change gear' to max airflow
        if self.capacity.max_power_counter > max_power_static_pressure_increment_time:
            self._set_static_pressure(max_static_pressure)
        elif was_off:
            self._set_static_pressure(initial_static_pressure[self.mode])

        self.try_set_mode(self.mode)
        self.try_set_fan_mode(self._determine_fan_mode())
        self.get_entity("input_number.aircon_meta_integral").set_state(
            state=str(float(self.capacity.deadband_integrator.get()))
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
        self._publish_status(
            "heating" if self.mode == "heat" else "cooling",
            **status_context,
        )

    def _reset_internal_state(self):
        """Resets the script state counters and flags, but preserves PID objects."""
        # Force mode to None so _handle_mode_change won't wipe PIDs on resume
        self.mode = None
        self.previous_requested_targets = {"heat": {}, "cool": {}}
        self.activation_steps = {"heat": set(), "cool": set()}
        self.capacity.compressor_totally_off = True
        self.capacity.on_counter = 0
        self.capacity.min_power_counter = 0
        self.capacity.max_power_counter = 0
        self.off_fan_running_counter = 0
        self.capacity.prev_step = 0

        # Reset Integrators (except Room PIDs)
        self.capacity.deadband_integrator.clear()

        # Reset Derivatives (Historic data is likely stale)
        for room in self.temp_derivs:
            self.temp_derivs[room].clear()

        # Reset Metrics
        self._reset_metrics()

        # Also reset state that would normally be persisted on a hot reload
        self.capacity.guesstimated_comp_speed = 0
        self.grid_surplus_integral = 0

    def _set_static_pressure(self, new_static_pressure):
        # Bounded retries so an unresponsive device can't wedge the app;
        # the next cycle will try again anyway.
        for _ in range(5):
            if (
                int(float(self.get_state(static_pressure_entity)))
                == new_static_pressure
            ):
                return
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
        self.log(
            f"Static pressure change to {new_static_pressure} not confirmed, giving up until next cycle",
            level="WARNING",
        )

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
        climate_states = self.get_state("climate")
        available_climates = (
            set(climate_states.keys()) if isinstance(climate_states, dict) else set()
        )
        missing_room_climate_entities = set()

        for room in rooms:
            room_climate_entity = "climate." + room + "_aircon"
            if room_climate_entity not in available_climates:
                missing_room_climate_entities.add(room_climate_entity)
                continue

            climate_state = self.get_state(room_climate_entity)
            room_climate = self.get_entity(room_climate_entity)

            if climate_state == "heat_cool":
                cur_targets["heat"][room] = room_climate.get_state("target_temp_low")
                cur_targets["cool"][room] = room_climate.get_state("target_temp_high")
            elif climate_state == "heat":
                cur_targets["heat"][room] = room_climate.get_state("temperature")
            elif climate_state == "cool":
                cur_targets["cool"][room] = room_climate.get_state("temperature")

        if missing_room_climate_entities != self.missing_room_climate_entities:
            if missing_room_climate_entities:
                self.log(
                    "Missing room climate entities: "
                    + ", ".join(sorted(missing_room_climate_entities)),
                    level="WARNING",
                )
            self.missing_room_climate_entities = missing_room_climate_entities

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
                f"linearly ramping target room: {room}, smooth target: {str(self.targets[mode][room])}, ultimate target: {str(cur_targets[mode][room])}",
                level="DEBUG",
            )
        elif abs(target_delta) <= (target_ramp_step_threshold + 1e-9):
            self.targets[mode][room] += target_delta * target_ramp_proportional
            self.log(
                f"proportionally ramping target room: {room}, smooth target:{str(self.targets[mode][room])}, ultimate target: {str(cur_targets[mode][room])}",
                level="DEBUG",
            )
        else:
            self.targets[mode][room] = cur_targets[mode][room] - math.copysign(
                target_ramp_step_threshold, target_delta
            )
            self.log(
                f"stepping target room: {room}, smooth target:{str(self.targets[mode][room])}, ultimate target: {str(cur_targets[mode][room])}",
                level="DEBUG",
            )

    def _update_room_targets(self, temps, cur_targets):
        # Compare consecutive requested targets, not smoothed targets or
        # effective errors. Never accumulate small scheduled ramp steps.
        self.activation_steps = {"heat": set(), "cool": set()}
        for mode in cur_targets:
            for room, target in cur_targets[mode].items():
                previous = self.previous_requested_targets[mode].get(room)
                if previous is not None and (
                    -mode_sign[mode] * (target - previous)
                    >= room_activation_step - 1e-9
                ):
                    self.activation_steps[mode].add(room)
        self.previous_requested_targets = {
            mode: dict(targets) for mode, targets in cur_targets.items()
        }
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
        # what the grid-surplus pass actually applied per room/mode, so the
        # price-pressure pass can cap the COMBINED banking offset at the
        # same target bounds (bookkeeping only, no behaviour change)
        self.grid_surplus_applied = {"heat": {}, "cool": {}}
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
                            self.grid_surplus_applied[mode][room] = (
                                window_limited_offset
                            )

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
        if not math.isfinite(max(cooling_demand, heating_demand)):
            return None, float("-inf")

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
        return None, max(cooling_demand, heating_demand)

    def _handle_mode_change(self, new_mode, celsius_setpoint):
        if new_mode != self.mode or new_mode is None:
            self.activation_steps = {"heat": set(), "cool": set()}
        if new_mode is None or (self.mode is not None and (new_mode != self.mode)):
            self.mode = new_mode
            for room, pid in self.pids.items():
                pid.clear()
            self.capacity.compressor_totally_off = True
            self.capacity.on_counter = 0
            self.capacity.deadband_integrator.clear()
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
        # Define the number of 5-minute intervals to look ahead (12 * 5 = 60 minutes)
        lookahead_intervals = 12
        grid_surplus = 0  # Default to 0 surplus

        try:
            # Fetch the forecast data from the EMHASS sensor attribute
            forecast_list = self.get_state(
                "sensor.mpc_p_pv_curtailment", attribute="forecasts"
            )

            if forecast_list and isinstance(forecast_list, list):
                # Get the curtailment values for the next hour (first 12 elements)
                curtailment_forecasts = [
                    float(item.get("mpc_p_pv_curtailment", 0))
                    for item in forecast_list[:lookahead_intervals]
                ]

                # Calculate the average curtailment over the lookahead window
                if curtailment_forecasts:
                    grid_surplus = sum(curtailment_forecasts) / len(
                        curtailment_forecasts
                    )
                    self.log(
                        f"EMHASS Forecast: Avg curtailment of {grid_surplus:.2f}W over the next hour.",
                        level="DEBUG",
                    )
                else:
                    self.log("EMHASS forecast list was empty, assuming 0 surplus.")

            else:
                self.log(
                    "Warning: Could not retrieve valid EMHASS forecast data for curtailment."
                )

        except Exception as e:
            self.log(f"Error processing EMHASS forecast: {e}", level="ERROR")
            grid_surplus = 0  # Ensure we fail safe

        # The rest of the integral logic remains the same, but now fed by the forecast
        grid_surplus_upper_threshold = grid_surplus_lower_threshold + (
            grid_surplus_off_buffer
            if self.capacity.compressor_totally_off
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

    def _get_price_pressure(self):
        """Continuous price-pressure offset [K] for this cycle.

        Reads the live retail price and the tariffed price-forecast sensor
        and hands them to control.price_pressure_offset (where all the
        actual logic and constants live -- docs/pricing.md). Any missing or
        malformed entity means 0.0: no price data, no price behaviour.
        """
        if self.get_state(price_pressure_boolean) != "on":
            return 0.0
        try:
            now_price = float(self.get_state(price_now_entity))
            future = forecast_hours_ahead(
                self.get_state(price_forecast_entity, attribute=price_forecast_attr),
                price_forecast_entity.split(".", 1)[1],
                datetime.now(timezone.utc),
            )
            offset = price_pressure_offset(now_price, future)
        except (TypeError, ValueError, KeyError) as e:
            self.log(f"Price pressure unavailable: {e}", level="WARNING")
            offset = 0.0
        self.get_entity("input_number.aircon_price_pressure").set_state(
            state=str(float(offset))
        )
        if offset != 0.0:
            self.log(f"price_pressure: {offset:+.3f}", level="DEBUG")
        return offset

    def _apply_price_pressure(self, errors, cooling_demand, heating_demand):
        """Add the price offset to room errors; returns updated demands.

        Positive (bank) offsets are capped per room at the same target
        bounds the grid-surplus offset respects (21C, midpoint, open
        windows), net of any surplus offset already applied this cycle.
        Negative (shave) offsets apply as-is -- they only reduce runtime
        and are already clamped in control.py. Runs AFTER
        _calculate_demand so the grid-surplus integral bookkeeping (which
        winds down on demand beyond its own cap) never sees price demand.
        """
        offset = self.price_pressure
        if offset == 0.0:
            return cooling_demand, heating_demand
        for mode in errors:
            for room in errors[mode]:
                if offset > 0.0:
                    if mode == "cool":
                        bound = self.targets["cool"][room] - grid_surplus_min_cooling
                    else:
                        bound = grid_surplus_max_heating - self.targets["heat"][room]
                    if room in self.targets["heat"] and room in self.targets["cool"]:
                        midpoint_offset = (
                            self.targets["cool"][room]
                            - self.targets["heat"][room]
                            + immediate_off_threshold
                        ) / 2
                        bound = min(bound, midpoint_offset)
                    bound -= self.grid_surplus_applied[mode].get(room, 0.0)
                    room_offset = max(
                        0.0,
                        min(offset, bound) - self.window_handler.get_offset(room),
                    )
                else:
                    room_offset = offset
                errors[mode][room] += room_offset
        return (
            max(errors["cool"].values(), default=float("-inf")),
            max(errors["heat"].values(), default=float("-inf")),
        )

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

        # Seed only the qualifying zone up to the existing raw leader. Do
        # this before normalisation/airflow inflation: top-up is not demand.
        # Consume once even if effective demand is too small this cycle.
        activation_steps = self.activation_steps[self.mode]
        self.activation_steps[self.mode] = set()
        leader = max(pid_outputs.values())
        for room in sorted(activation_steps & pid_outputs.keys()):
            if errors[self.mode][room] < room_activation_error - 1e-9:
                continue
            boost = leader - pid_outputs[room]
            if boost > 0:
                self.pids[room].adjust_integral(boost)
                pid_outputs[room] = self.pids[room].get_output()
                self.log(f"Target-step catch-up {room}: integral +{boost:.3f}")

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
                    self.log("Adjusting top integral", level="DEBUG")
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

        # Negative wind-down clamp must run BEFORE the min-airflow top-up:
        # the top-up is stateless and no longer props raw outputs up, so the
        # clamp has to see raw outputs (see min_airflow_inflation docstring).
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

        min_airflow_inflation(self.pids, pid_outputs, adjusted_room_airflow, min_sum)

        for room in pid_outputs:
            self.get_entity(f"input_number.{room}_pid").set_state(
                state=str(float(pid_outputs[room]))
            )
            self.log(
                f"{room} adjusted PID output: {pid_outputs[room]:.3f} (P: {self.pids[room].p_term:.3f}, I: {self.pids[room].i_term:.3f}, D: {self.pids[room].deriv.get():.3f})",
                level="DEBUG",
            )
        return pid_outputs

    def get_door_state(self, room):
        entity_id = f"binary_sensor.{room}_door"
        state = self.get_state(entity_id)
        return True if state is None else state == "on"

    def set_fake_temp(self, celsius_setpoint, compressed_error, transmit=True):
        self.get_entity("input_number.fake_temperature").set_state(
            state=str(float(celsius_setpoint + compressed_error))
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
            state=str(float(damper_val))
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
            self.log(damper_log + " within deadband", level="DEBUG")

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
        if self.capacity.guesstimated_comp_speed >= medium_to_high:
            return "high"
        elif self.capacity.guesstimated_comp_speed <= medium_to_low:
            return "low"
        elif (
            current_fan_mode == "high"
            and self.capacity.guesstimated_comp_speed >= high_to_medium
        ):
            return "high"
        elif (
            current_fan_mode == "low" and self.capacity.guesstimated_comp_speed <= low_to_medium
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
