import appdaemon.plugins.hass.hassapi as hass
import datetime


class Statctrl(hass.Hass):
    def initialize(self):
        self.room = self.args.get("room")
        self.modes = (
            ["heat", "cool"] if "type" not in self.args else [self.args["type"]]
        )
        self.states = ["turbo", "active", "inactive"]
        self.step_size = 0.1
        self.active_timers = {}

        self.run_every(self.periodic_check, "now", 60)

        entities_to_monitor = {f"input_boolean.{self.room}_manual_ac"}
        entities_to_monitor.add(f"climate.{self.room}_aircon")
        entities_to_monitor.add(f"timer.{self.room}_timed_turbo")
        entities_to_monitor.add(f"timer.{self.room}_timed_active")
        entities_to_monitor.add(f"input_boolean.{self.room}_scheduled_heat")
        entities_to_monitor.add(f"input_boolean.{self.room}_scheduled_cool")
        entities_to_monitor.add(f"binary_sensor.{self.room}_window")

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

        # Handle instant updates if we are outside the current mode's bounds
        if self.should_step_to_target(current_setpoint, target, mode):
            # self.log("Immediate update")
            self.set_climate(mode, target)
            return

        # Handle inactive state with upcoming schedule
        if current_state == "inactive":
            next_start = self.get_next_scheduled_start(mode)
            if next_start and self.should_step_to_target(
                current_setpoint, setpoints["active"], mode
            ):
                time_until_next = (
                    next_start - datetime.datetime.now().astimezone()
                ).total_seconds() / 3600
                setpoint_delta = abs(setpoints["active"] - current_setpoint)
                slew_on_rate = float(self.get_state("input_number.statctrl_slew_on"))

                if (setpoint_delta / time_until_next) > slew_on_rate:
                    new_temp = current_setpoint + (
                        self.step_size * (-1 if mode == "cool" else 1)
                    )
                    self.set_climate(mode, new_temp)

                    steps_remaining = (
                        abs(setpoints["active"] - new_temp) / self.step_size
                    )
                    time_to_next = time_until_next / steps_remaining
                    self.schedule_next_update(mode, time_to_next * 3600)
                    return
        # self.log("No immediate action required")

        # Handle slew-off
        if not (
            current_setpoint == target
            or self.should_step_to_target(current_setpoint, target, mode)
        ):
            # self.log("Slew-off")
            new_temp = current_setpoint + (
                self.step_size * (-1 if mode == "heat" else 1)
            )
            self.set_climate(mode, new_temp)

            slew_off_rate = float(self.get_state("input_number.statctrl_slew_off"))
            time_to_next = (self.step_size * 3600) / slew_off_rate
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
