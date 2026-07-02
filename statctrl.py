import appdaemon.plugins.hass.hassapi as hass
import datetime
import json
import os


class Statctrl(hass.Hass):
    def initialize(self):
        self.room = self.args.get("room")
        self.modes = (
            ["heat", "cool"] if "type" not in self.args else [self.args["type"]]
        )
        self.states = ["turbo", "active", "inactive"]
        self.default_step_size = 0.1
        self.active_timers = {}
        self.learning_sessions = {}
        self.adaptive_model_path = self.args.get(
            "adaptive_model_path",
            os.path.join(os.path.dirname(__file__), "statctrl_adaptive.json"),
        )
        self.adaptive_model = self.load_adaptive_model()
        self.adaptive_enabled_default = self.get_bool_arg(
            "adaptive_optimum_start", False
        )
        self.adaptive_default_minutes_per_degree = float(
            self.args.get("adaptive_default_minutes_per_degree", 60.0)
        )
        self.adaptive_safety_factor = float(
            self.args.get("adaptive_safety_factor", 1.2)
        )
        self.adaptive_tolerance = float(self.args.get("adaptive_tolerance", 0.2))
        self.adaptive_min_error = float(self.args.get("adaptive_min_error", 0.4))
        self.adaptive_min_sample_minutes = float(
            self.args.get("adaptive_min_sample_minutes", 5.0)
        )
        self.adaptive_min_minutes_per_degree = float(
            self.args.get("adaptive_min_minutes_per_degree", 10.0)
        )
        self.adaptive_max_minutes_per_degree = float(
            self.args.get("adaptive_max_minutes_per_degree", 180.0)
        )
        self.adaptive_max_lead_minutes = float(
            self.args.get("adaptive_max_lead_minutes", 180.0)
        )
        self.adaptive_alpha = float(self.args.get("adaptive_alpha", 0.25))

        self.run_every(self.periodic_check, "now", 60)

        entities_to_monitor = {f"input_boolean.{self.room}_manual_ac"}
        entities_to_monitor.add(f"climate.{self.room}_aircon")
        entities_to_monitor.add(f"timer.{self.room}_timed_turbo")
        entities_to_monitor.add(f"timer.{self.room}_timed_active")
        entities_to_monitor.add(f"input_boolean.{self.room}_scheduled_heat")
        entities_to_monitor.add(f"input_boolean.{self.room}_scheduled_cool")
        entities_to_monitor.add(f"binary_sensor.{self.room}_window")
        entities_to_monitor.add("input_boolean.statctrl_adaptive_optimum_start")
        entities_to_monitor.add(f"input_boolean.{self.room}_adaptive_optimum_start")

        for state in self.states:
            entities_to_monitor.add(f"input_number.{self.room}_setpoint_{state}_high")
            entities_to_monitor.add(f"input_number.{self.room}_setpoint_{state}_low")

        for entity in entities_to_monitor:
            self.listen_state(self.handle_change, entity)

    def periodic_check(self, kwargs):
        for mode in self.modes:
            if mode not in self.active_timers:  # Only check if no active slew timer
                self.update_setpoint(mode)

    def schedule_next_update(self, mode, delay):
        # Cancel any existing timer for this mode
        if mode in self.active_timers:
            self.cancel_timer(self.active_timers[mode])

        # Schedule new timer and store handle
        self.active_timers[mode] = self.run_in(self.handle_timer, delay, mode=mode)

    def handle_timer(self, kwargs):
        mode = kwargs["mode"]
        if mode in self.active_timers:
            del self.active_timers[mode]
        self.update_setpoint(mode)

    def get_current_state(self, mode):
        if self.get_state(f"binary_sensor.{self.room}_window") == "on":
            return "window_open"
        if self.timer_active(f"timer.{self.room}_timed_turbo"):
            return "turbo"
        elif (
            self.timer_active(f"timer.{self.room}_timed_active")
            or self.get_state(f"input_boolean.{self.room}_scheduled_{mode}") == "on"
        ):
            return "active"
        return "inactive"

    def timer_active(self, timer_entity):
        return self.get_state(timer_entity) == "active"

    def get_bool_arg(self, name, default):
        value = self.args.get(name, default)
        if isinstance(value, bool):
            return value
        return str(value).lower() in ["1", "true", "yes", "on"]

    def adaptive_enabled(self):
        room_switch = self.get_state(f"input_boolean.{self.room}_adaptive_optimum_start")
        if room_switch in ["on", "off"]:
            return room_switch == "on"

        global_switch = self.get_state("input_boolean.statctrl_adaptive_optimum_start")
        if global_switch in ["on", "off"]:
            return global_switch == "on"

        return self.adaptive_enabled_default

    def load_adaptive_model(self):
        try:
            with open(self.adaptive_model_path, "r", encoding="utf-8") as model_file:
                model = json.load(model_file)
        except FileNotFoundError:
            return {"version": 1, "models": {}}
        except (OSError, json.JSONDecodeError) as error:
            self.warning(f"Could not load adaptive optimum start model: {error}")
            return {"version": 1, "models": {}}

        if not isinstance(model, dict):
            return {"version": 1, "models": {}}
        model.setdefault("version", 1)
        model.setdefault("models", {})
        return model

    def save_adaptive_model(self):
        latest_model = self.load_adaptive_model()
        latest_model["models"].update(self.adaptive_model["models"])
        self.adaptive_model = latest_model

        path = self.adaptive_model_path
        tmp_path = f"{path}.{self.room}.tmp"
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(tmp_path, "w", encoding="utf-8") as model_file:
                json.dump(self.adaptive_model, model_file, indent=2, sort_keys=True)
            os.replace(tmp_path, path)
        except OSError as error:
            self.warning(f"Could not save adaptive optimum start model: {error}")

    def model_key(self, mode):
        return f"{self.room}:{mode}"

    def get_current_temperature(self):
        if self.get_state("input_boolean.ac_use_feels_like") == "on":
            feels_like = self.get_state(f"sensor.{self.room}_feels_like")
            if feels_like not in [None, "unknown", "unavailable"]:
                return float(feels_like)

        return float(self.get_state(f"sensor.{self.room}_average_temperature"))

    def comfort_error(self, mode, target):
        temp = self.get_current_temperature()
        if mode == "heat":
            return max(0.0, target - temp)
        return max(0.0, temp - target)

    def get_slew_on_rate(self):
        return float(self.get_state("input_number.statctrl_slew_on"))

    def get_model_minutes_per_degree(self, mode):
        model = self.adaptive_model["models"].get(self.model_key(mode), {})
        learned = model.get("minutes_per_degree")
        if learned is not None:
            return float(learned)

        try:
            return max(
                self.adaptive_min_minutes_per_degree,
                min(
                    self.adaptive_max_minutes_per_degree,
                    60.0 / self.get_slew_on_rate(),
                ),
            )
        except (TypeError, ValueError, ZeroDivisionError):
            return self.adaptive_default_minutes_per_degree

    def start_learning_session(self, mode, target, source):
        error = self.comfort_error(mode, target)
        if error < self.adaptive_min_error:
            return

        session = self.learning_sessions.get(mode)
        if session and abs(session["target"] - target) <= self.adaptive_tolerance:
            return

        self.learning_sessions[mode] = {
            "start": datetime.datetime.now().astimezone(),
            "initial_error": error,
            "target": target,
            "source": source,
        }
        self.log(
            f"Adaptive optimum start learning {self.room} {mode}: "
            f"{error:.2f}C from target via {source}"
        )

    def update_learning_session(self, mode, current_state, active_target):
        session = self.learning_sessions.get(mode)
        scheduled_active = (
            self.get_state(f"input_boolean.{self.room}_scheduled_{mode}") == "on"
        )

        if current_state == "inactive" and not session:
            return

        if scheduled_active and not session:
            self.start_learning_session(mode, active_target, "scheduled_active")
            return

        if not session:
            return

        if abs(session["target"] - active_target) > self.adaptive_tolerance:
            self.learning_sessions.pop(mode, None)
            self.start_learning_session(mode, active_target, "target_changed")
            return

        if self.comfort_error(mode, active_target) > self.adaptive_tolerance:
            return

        elapsed_minutes = (
            datetime.datetime.now().astimezone() - session["start"]
        ).total_seconds() / 60.0
        initial_error = session["initial_error"]
        self.learning_sessions.pop(mode, None)

        if (
            elapsed_minutes < self.adaptive_min_sample_minutes
            or initial_error < self.adaptive_min_error
        ):
            return

        sample = elapsed_minutes / initial_error
        sample = max(
            self.adaptive_min_minutes_per_degree,
            min(self.adaptive_max_minutes_per_degree, sample),
        )

        key = self.model_key(mode)
        model = self.adaptive_model["models"].setdefault(key, {})
        previous = model.get("minutes_per_degree")
        if previous is None:
            updated = sample
            samples = 1
        else:
            updated = ((1.0 - self.adaptive_alpha) * float(previous)) + (
                self.adaptive_alpha * sample
            )
            samples = int(model.get("samples", 0)) + 1

        model["minutes_per_degree"] = updated
        model["samples"] = samples
        model["last_sample_minutes_per_degree"] = sample
        model["updated"] = datetime.datetime.now().astimezone().isoformat()
        self.save_adaptive_model()
        self.log(
            f"Adaptive optimum start updated {self.room} {mode}: "
            f"{updated:.1f} min/C from {samples} samples"
        )

    def handle_adaptive_start(self, mode, current_setpoint, active_target, next_start):
        # Return True when adaptive mode has handled the scheduled pre-start decision.
        if not self.should_step_to_target(current_setpoint, active_target, mode):
            return True

        error = self.comfort_error(mode, active_target)
        if error <= self.adaptive_tolerance:
            return True

        time_until_next = (
            next_start - datetime.datetime.now().astimezone()
        ).total_seconds() / 60.0
        if time_until_next <= 0:
            return False

        minutes_per_degree = self.get_model_minutes_per_degree(mode)
        lead_minutes = min(
            self.adaptive_max_lead_minutes,
            error * minutes_per_degree * self.adaptive_safety_factor,
        )

        if time_until_next > lead_minutes:
            return True

        self.start_learning_session(mode, active_target, "prestart")
        self.log(
            f"Adaptive optimum start for {self.room} {mode}: "
            f"{error:.2f}C error, {time_until_next:.1f} min until schedule, "
            f"{lead_minutes:.1f} min lead"
        )
        self.slew_on_step(mode, current_setpoint, active_target, next_start)
        return True

    def slew_on_step(self, mode, current_setpoint, target, deadline=None):
        new_temp = self.step_towards(current_setpoint, target, mode)
        self.set_climate(mode, new_temp)

        if new_temp == target:
            return

        steps_remaining = abs(target - new_temp) / self.get_step_size()
        if steps_remaining <= 0:
            return

        if deadline is not None:
            seconds_remaining = (
                deadline - datetime.datetime.now().astimezone()
            ).total_seconds()
            if seconds_remaining > 0:
                self.schedule_next_update(mode, seconds_remaining / steps_remaining)
                return

        slew_on_rate = self.get_slew_on_rate()
        time_to_next = (self.get_step_size() * 3600) / slew_on_rate
        self.schedule_next_update(mode, time_to_next)

    def get_setpoints(self, mode):
        # Too lazy to create helpers for "window_open" setpoints
        if mode == "heat":
            suffix = "low"
            window_open_offset = -2.0
        else:
            suffix = "high"
            window_open_offset = 2.0
        setpoints = {
            state: float(
                self.get_state(f"input_number.{self.room}_setpoint_{state}_{suffix}")
            )
            for state in self.states
        }
        setpoints["window_open"] = setpoints["inactive"] + window_open_offset
        return setpoints

    def get_current_setpoint(self, mode):
        climate_entity = f"climate.{self.room}_aircon"
        attr = "target_temp_low" if mode == "heat" else "target_temp_high"
        # self.log(f"Getting current setpoint attribute {attr} for {mode} from {climate_entity}...")
        setpoint = self.get_state(climate_entity, attribute=attr)
        # self.log(f"Current setpoint for {mode} is {setpoint}")
        return float(self.get_state(climate_entity, attribute=attr))

    def get_step_size(self):
        climate_entity = f"climate.{self.room}_aircon"
        step_size = self.get_state(climate_entity, attribute="target_temp_step")
        if step_size is None:
            return self.default_step_size
        return float(step_size)

    def step_towards(self, current, target, mode):
        step_size = self.get_step_size()
        direction = -1 if mode == "cool" else 1
        if not self.should_step_to_target(current, target, mode):
            direction *= -1
        new_temp = current + (step_size * direction)
        if direction > 0:
            return min(new_temp, target)
        return max(new_temp, target)

    def should_step_to_target(self, current, target, mode):
        if mode == "heat":
            return current < target
        return current > target

    def update_setpoint(self, mode):
        if (
            self.get_state(f"input_boolean.{self.room}_manual_ac") == "on"
            or self.get_state(f"climate.{self.room}_aircon") != "heat_cool"
        ):
            if mode in self.active_timers:
                self.cancel_timer(self.active_timers[mode])
                del self.active_timers[mode]
            return

        current_state = self.get_current_state(mode)
        current_setpoint = self.get_current_setpoint(mode)
        setpoints = self.get_setpoints(mode)
        target = setpoints[current_state]
        self.update_learning_session(mode, current_state, setpoints["active"])

        # Handle instant updates if we are outside the current mode's bounds
        if self.should_step_to_target(current_setpoint, target, mode):
            if self.adaptive_enabled() and current_state == "active":
                self.slew_on_step(mode, current_setpoint, target)
                return

            # self.log("Immediate update")
            self.set_climate(mode, target)
            return

        # Handle inactive state with upcoming schedule
        if current_state == "inactive":
            next_start = self.get_next_scheduled_start(mode)
            if next_start and self.should_step_to_target(
                current_setpoint, setpoints["active"], mode
            ):
                if self.adaptive_enabled() and self.handle_adaptive_start(
                    mode, current_setpoint, setpoints["active"], next_start
                ):
                    return

                time_until_next = (
                    next_start - datetime.datetime.now().astimezone()
                ).total_seconds() / 3600
                setpoint_delta = abs(setpoints["active"] - current_setpoint)
                slew_on_rate = self.get_slew_on_rate()

                if (setpoint_delta / time_until_next) > slew_on_rate:
                    self.slew_on_step(mode, current_setpoint, setpoints["active"], next_start)
                    return
        # self.log("No immediate action required")

        # Handle slew-off
        if not (
            current_setpoint == target
            or self.should_step_to_target(current_setpoint, target, mode)
        ):
            # self.log("Slew-off")
            new_temp = self.step_towards(current_setpoint, target, mode)
            self.set_climate(mode, new_temp)

            if new_temp != target:
                slew_off_rate = float(self.get_state("input_number.statctrl_slew_off"))
                time_to_next = (self.get_step_size() * 3600) / slew_off_rate
                self.schedule_next_update(mode, time_to_next)

    def set_climate(self, mode, temp):
        climate_entity = f"climate.{self.room}_aircon"
        # self.log(f"Setting {mode} setpoint to {temp}...")
        if mode == "heat":
            self.call_service(
                "climate/set_temperature",
                entity_id=climate_entity,
                target_temp_low=temp,
                target_temp_high=self.get_current_setpoint("cool"),
            )
        else:
            self.call_service(
                "climate/set_temperature",
                entity_id=climate_entity,
                target_temp_low=self.get_current_setpoint("heat"),
                target_temp_high=temp,
            )

    def handle_change(self, entity, attribute, old, new, kwargs):
        for mode in self.modes:
            self.update_setpoint(mode)

    def get_next_scheduled_start(self, mode):
        entity_id = f"input_boolean.{self.room}_scheduled_{mode}"

        # Filter schedule switches
        schedule_switches = [
            entity
            for entity in self.get_state("switch")
            if entity.startswith("switch.schedule_")
            and self.get_state(entity) == "on"  # Only include enabled switches
        ]

        matching_schedules = {}

        # Log the state of each schedule switch
        for switch in schedule_switches:
            switch_entity = self.get_entity(switch)
            full_state = switch_entity.get_state(attribute="all")
            if entity_id in full_state["attributes"].get("entities", []):
                matching_schedules[switch] = full_state["attributes"]
                # self.log(f"Found schedule '{switch}' that affects '{entity_id}'.")

        if len(matching_schedules) == 0:
            # self.error("Could not find a schedule that affects 'entity_id'.")
            return None

        # Get all schedule entities
        next_time = None

        # Iterate through the matching schedule entity
        for entity, attributes in matching_schedules.items():
            # self.log(f"Checking schedule '{entity}'...")
            actions = attributes.get("actions", [])
            timeslots = attributes.get("timeslots", [])

            # Check if the next timeslot has an 'on' action
            if (
                attributes["actions"][attributes["next_slot"]]["service"]
                == "input_boolean.turn_on"
            ):
                schedule_time = datetime.datetime.fromisoformat(
                    attributes["next_trigger"]
                )

                # Calculate next time
                if next_time is None or schedule_time < next_time:
                    next_time = schedule_time

        # self.log(f"Next 'on' time for {entity_id}: {next_time}")

        return next_time
