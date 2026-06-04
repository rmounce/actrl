import appdaemon.plugins.hass.hassapi as hass
from dateutil.parser import parse
from datetime import timedelta

class Statoptimctrl(hass.Hass):
    """
    Final version with intelligent, "slew-off" only logic. The setpoint jumps
    instantly to meet a new target but slews gracefully to a less demanding
    temperature for comfort.
    """

    def initialize(self):
        self.log("Starting Statoptimctrl.")
        self.rooms = self.args.get("rooms", [])
        if not self.rooms:
            self.error("No rooms configured. Please add 'rooms: [...]' to your apps.yaml.")
            return

        try:
            slew_rate_per_hour = float(self.get_state("input_number.statctrl_slew_off"))
            self.slew_amount = slew_rate_per_hour / 60.0
            self.log(f"Slew rate configured to {slew_rate_per_hour}°/hr ({self.slew_amount:.2f}°/min).")
        except (ValueError, TypeError):
            self.slew_amount = 0.1
            self.warning(f"Could not read 'input_number.statctrl_slew_off'. Defaulting to {self.slew_amount}°/min.")
        
        self.last_power_forecasts = {room: {'heat': 0, 'cool': 0} for room in self.rooms}

        for room in self.rooms:
            self.log(f"Initializing control for room: {room}")
            entities_to_monitor = {
                f"sensor.statoptim_p_{room}_heat", f"sensor.statoptim_p_{room}_cool",
                f"sensor.statoptim_temp_predicted_{room}_heat", f"sensor.statoptim_temp_predicted_{room}_cool",
                f"input_boolean.{room}_manual_ac", f"binary_sensor.{room}_window",
                f"timer.{room}_timed_turbo", f"timer.{room}_timed_active",
                f"input_boolean.{room}_scheduled_heat", f"input_boolean.{room}_scheduled_cool",
            }
            for state in ["active", "inactive", "turbo"]:
                entities_to_monitor.add(f"input_number.{room}_setpoint_{state}_low")
                entities_to_monitor.add(f"input_number.{room}_setpoint_{state}_high")

            for entity in entities_to_monitor:
                self.listen_state(self.handle_change, entity, room=room)

        self.run_minutely(self.periodic_check)

    def periodic_check(self, kwargs):
        self.log("Performing periodic, wall-clock aligned check for all rooms.")
        for room in self.rooms:
            self.update_thermostat(room)

    def handle_change(self, entity, attribute, old, new, kwargs):
        room = kwargs.get("room")
        if room:
            self.log(f"Change detected on {entity} for room {room}. Triggering immediate update.")
            self.update_thermostat(room)

    def _get_current_state(self, room):
        if self.get_state(f"binary_sensor.{room}_window") == "on": return "window_open"
        if self.get_state(f"timer.{room}_timed_turbo") == "active": return "turbo"
        if (self.get_state(f"timer.{room}_timed_active") == "active" or
                self.get_state(f"input_boolean.{room}_scheduled_heat") == "on" or
                self.get_state(f"input_boolean.{room}_scheduled_cool") == "on"):
            return "active"
        return "inactive"
    
    def _get_base_setpoints(self, room, state):
        if state == "window_open":
            inactive_low = float(self.get_state(f"input_number.{room}_setpoint_inactive_low"))
            inactive_high = float(self.get_state(f"input_number.{room}_setpoint_inactive_high"))
            return (inactive_low - 2.0, inactive_high + 2.0)
        else:
            low = float(self.get_state(f"input_number.{room}_setpoint_{state}_low"))
            high = float(self.get_state(f"input_number.{room}_setpoint_{state}_high"))
            return (low, high)

    def get_forecast_value(self, entity_id, attribute, value_key, interpolate: bool, mode: str = None):
        now = self.datetime(aware=True) + timedelta(minutes=1)
        forecast_list = self.get_state(entity_id, attribute=attribute)
        if not forecast_list or not isinstance(forecast_list, list) or len(forecast_list) < 1:
            self.warning(f"Forecast list for {entity_id} is empty or invalid."); return None
        try:
            parsed = sorted([{"date": parse(i["date"]), "value": float(i[value_key])} for i in forecast_list], key=lambda x: x["date"])
        except (KeyError, TypeError, ValueError) as e:
            self.error(f"Error parsing forecast data for {entity_id}. Error: {e}"); return None
        if now < parsed[0]["date"]: return parsed[0]["value"]
        if now >= parsed[-1]["date"]: return parsed[-1]["value"]
        p1, p2 = next(((parsed[i], parsed[i+1]) for i in range(len(parsed)-1) if parsed[i]["date"] <= now < parsed[i+1]["date"]), (None, None))
        if p1 is None: return parsed[-1]["value"]
        if not interpolate: return p1["value"]
        if mode == 'cool' and p2['value'] > p1['value']: return p2['value']
        if mode == 'heat' and p2['value'] < p1['value']: return p2['value']
        total_diff = (p2["date"] - p1["date"]).total_seconds()
        if total_diff == 0: return p1["value"]
        now_diff = (now - p1["date"]).total_seconds()
        return p1["value"] + ((p2["value"] - p1["value"]) * (now_diff / total_diff))

    def update_thermostat(self, room):
        if self.get_state(f"input_boolean.{room}_manual_ac") == "on":
            self.log(f"Manual override for {room} is on. Skipping."); return
            
        climate_entity = f"climate.{room}_aircon"
        try:
            current_low = float(self.get_state(climate_entity, attribute="target_temp_low"))
            current_high = float(self.get_state(climate_entity, attribute="target_temp_high"))
        except (ValueError, TypeError):
            self.warning(f"Could not get current setpoints for {climate_entity}. Skipping update."); return

        current_state = self._get_current_state(room)
        try:
            base_low, base_high = self._get_base_setpoints(room, current_state)
            overshoot_heat = float(self.get_state(f"input_number.{room}_setpoint_turbo_low"))
            overshoot_cool = float(self.get_state(f"input_number.{room}_setpoint_turbo_high"))
        except (ValueError, TypeError) as e:
            self.error(f"Could not read helpers for {room} (state: {current_state}). Error: {e}"); return

        heat_power = self.get_forecast_value(f"sensor.statoptim_p_{room}_heat", "deferrables_schedule", f"statoptim_p_{room}_heat", False)
        cool_power = self.get_forecast_value(f"sensor.statoptim_p_{room}_cool", "deferrables_schedule", f"statoptim_p_{room}_cool", False)
        heat_temp = self.get_forecast_value(f"sensor.statoptim_temp_predicted_{room}_heat", "predicted_temperatures", f"statoptim_temp_predicted_{room}_heat", True, mode='heat')
        cool_temp = self.get_forecast_value(f"sensor.statoptim_temp_predicted_{room}_cool", "predicted_temperatures", f"statoptim_temp_predicted_{room}_cool", True, mode='cool')

        if any(v is None for v in [heat_power, heat_temp, cool_power, cool_temp]):
            self.warning(f"Forecasts for {room} are unavailable. Skipping."); return

        target_low = min(max(base_low, heat_temp), overshoot_heat) if heat_power > 0 else min(base_low, overshoot_heat)
        target_high = max(min(base_high, cool_temp), overshoot_cool) if cool_power > 0 else max(base_high, overshoot_cool)

        previous_power = self.last_power_forecasts[room]
        jump_heat = previous_power['heat'] > 0 and heat_power == 0
        jump_cool = previous_power['cool'] > 0 and cool_power == 0
        
        # --- "Slew Off" Only Logic ---
        if jump_heat:
            new_low = target_low
            self.log(f"[{room}] Heat Jump: EMHASS pre-heat cycle ended. Jumping to target {target_low:.2f}°")
        elif current_low > target_low: # Only slew if moving "off" (decreasing heat setpoint)
            new_low = max(target_low, current_low - self.slew_amount)
        else: # Otherwise, jump instantly to the new "on" target
            new_low = target_low

        if jump_cool:
            new_high = target_high
            self.log(f"[{room}] Cool Jump: EMHASS pre-cool cycle ended. Jumping to target {target_high:.2f}°")
        elif current_high < target_high: # Only slew if moving "off" (increasing cool setpoint)
            new_high = min(target_high, current_high + self.slew_amount)
        else: # Otherwise, jump instantly to the new "on" target
            new_high = target_high
            
        self.last_power_forecasts[room] = {'heat': heat_power, 'cool': cool_power}
        
        self.log(f"[{room}] State: {current_state.upper()}, Target -> L:{target_low:.2f} H:{target_high:.2f}. Applying -> L:{new_low:.2f}, H:{new_high:.2f}")
        self.call_service("climate/set_temperature", entity_id=climate_entity,
            target_temp_low=round(new_low, 1), target_temp_high=round(new_high, 1))
