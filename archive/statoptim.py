import appdaemon.plugins.hass.hassapi as hass
import datetime
import requests
import json
import pytz

class Statoptim(hass.Hass):
    def initialize(self):
        self.listen_event(self.run_planner_callback, "STATOPTIM.RUN_PLANNER")
        self.log("Statoptim Planner: Ready and listening for 'STATOPTIM.RUN_PLANNER' event.")

    def _log_exception(self, message):
        self.error(message)

    def run_planner_callback(self, event_name, data, kwargs):
        self.log(f"Received {event_name}. Starting planner execution.")
        
        config = self.args
        base_url = config.get('emhass_base_url', '').rstrip('/')
        if not base_url: self.error("Config error: 'emhass_base_url' not defined."); return
        
        raw_prefix = config.get("publish_prefix", "")
        opt_prefix = f"{raw_prefix}_" if raw_prefix else ""
        
        horizon_hours = int(config['prediction_horizon_hours'])
        num_timesteps = horizon_hours * 2

        forecasts = self._get_core_forecasts(config, num_timesteps)
        if forecasts is None: return
        soc_values = self._get_soc_values(config, horizon_hours)
        if soc_values is None: return
        zones_data = self._process_zones(config, num_timesteps)
        if zones_data is None: return

        payload = {
            "publish_prefix": opt_prefix,
            "entity_save": True,
            "prediction_horizon": num_timesteps, "optimization_time_step": 30,
            "pv_power_forecast": forecasts['pv'], "load_power_forecast": forecasts['load'],
            "load_cost_forecast": forecasts['load_cost'], "prod_price_forecast": forecasts['prod_price'],
            "outdoor_temperature_forecast": forecasts['outdoor_temp'],
            "soc_init": soc_values['soc_init'], "soc_final": soc_values['soc_final'],
            "number_of_deferrable_loads": zones_data['num_loads'],
            "nominal_power_of_deferrable_loads": zones_data['nominal_powers'],
            "def_load_config": zones_data['def_loads'], 
            "custom_deferrable_forecast_id": zones_data['custom_ids'],
            "custom_predicted_temperature_id": zones_data['custom_temp_ids'],
            "battery_minimum_state_of_charge": config.get('battery_minimum_state_of_charge', 0.15),
            "battery_maximum_state_of_charge": config.get('battery_maximum_state_of_charge', 0.95),
            "treat_deferrable_load_as_semi_cont": [False] * zones_data['num_loads'],
            "set_deferrable_load_single_constant": [False] * zones_data['num_loads'],
            "set_deferrable_startup_penalty": [0] * zones_data['num_loads'],
        }
        
        optim_success = self._call_emhass_optim_api(base_url, payload)
        if optim_success:
            self._call_emhass_publish_api(base_url, raw_prefix, zones_data)
            
        self.log("Planner execution finished.")

    def _call_emhass_optim_api(self, base_url, payload):
        url = f"{base_url}/action/naive-mpc-optim"
        self.log(f"Sending optimization payload to {url}...")
        try:
            response = requests.post(url, json=payload, timeout=300)
            response.raise_for_status()
            self.log(f"Successfully called optimization API. Status: {response.status_code}")
            return True
        except requests.exceptions.RequestException as e:
            self.error(f"Error calling optimization API: {e}")
            if hasattr(e, 'response') and e.response is not None: self.error(f"Response body: {e.response.text}")
            self._log_exception("Optimization API call failed")
            return False

    def _call_emhass_publish_api(self, base_url, publish_prefix, zones_data):
        if not publish_prefix:
            self.warning("'publish_prefix' not defined, skipping publish call.")
            return

        url = f"{base_url}/action/publish-data"

        simplified_def_loads = []
        for load in zones_data['def_loads']:
            if "thermal_config" in load:
                simplified_def_loads.append({"thermal_config": {}})
            else:
                simplified_def_loads.append({})

        payload = {
            "publish_prefix": publish_prefix,
            "def_load_config": simplified_def_loads,
            "custom_deferrable_forecast_id": zones_data['custom_ids'],
            "custom_predicted_temperature_id": zones_data['custom_temp_ids']
        }
        
        self.log(f"Calling publish endpoint at {url} for prefix '{publish_prefix}'...")
        try:
            response = requests.post(url, json=payload, timeout=60)
            response.raise_for_status()
            self.log(f"Successfully called publish API. Status: {response.status_code}")
        except requests.exceptions.RequestException as e:
            self.error(f"Error calling publish API: {e}")
            if hasattr(e, 'response') and e.response is not None:
                self.error(f"Response body: {e.response.text}")
            self._log_exception("Publish API call failed")


    def _get_interpolated_weather_forecast_manual(self, config, num_timesteps):
        self.log("Retrieving and interpolating weather forecast (using explicit service call)...")
        weather_entity = config['outdoor_temp_entity']
        try:
            tz_str = self.args.get("time_zone")
            if not tz_str:
                self.error("Required 'time_zone' parameter not found in app configuration (apps.yaml). The script cannot continue.")
                return None
            local_tz = pytz.timezone(tz_str)
            
            response_data = self.call_service(
                "weather/get_forecasts",
                entity_id=weather_entity,
                type="hourly"
            )

            if not response_data:
                self.error("Weather service call returned no data."); return None

            result_data = response_data.get("result", {})
            service_response = result_data.get("response", {})
            hourly_forecasts = service_response.get(weather_entity, {}).get("forecast", [])
            
            if not hourly_forecasts:
                self.error("Weather service call returned data, but it contained no 'forecast' list."); return None

            processed_forecasts = []
            for f in hourly_forecasts:
                naive_dt = datetime.datetime.fromisoformat(f['datetime'])
                local_dt = local_tz.localize(naive_dt)
                processed_forecasts.append({'ts': local_dt.timestamp(), 'temp': f['temperature']})

            interpolated_temps = []
            
            now_utc = self.get_now()
            snapped_minute = (now_utc.minute // 30) * 30
            now_snapped_utc = now_utc.replace(minute=snapped_minute, second=0, microsecond=0)
            
            for i in range(num_timesteps):
                target_dt = now_snapped_utc + datetime.timedelta(minutes=i * 30)
                target_ts = target_dt.timestamp()

                p1, p2 = None, None
                for point in processed_forecasts:
                    if point['ts'] <= target_ts: p1 = point
                    if point['ts'] >= target_ts: p2 = point; break
                
                if p1 and p2:
                    time_diff, temp_diff = p2['ts'] - p1['ts'], p2['temp'] - p1['temp']
                    if time_diff == 0:
                        interpolated_temp = p1['temp']
                    else:
                        ratio = (target_ts - p1['ts']) / time_diff
                        interpolated_temp = p1['temp'] + (temp_diff * ratio)
                    interpolated_temps.append(round(interpolated_temp, 2))
                elif p1:
                    interpolated_temps.append(p1['temp'])
                elif p2:
                    interpolated_temps.append(p2['temp'])
                else:
                    self.warning(f"Could not find weather data for timestep {i}, defaulting to 15°C")
                    interpolated_temps.append(15)

            self.log(f"Successfully retrieved and manually interpolated {len(interpolated_temps)} weather records.")
            return interpolated_temps[:num_timesteps]
        except Exception as e:
            error_type = type(e).__name__
            self._log_exception(f"Failed to get/interpolate weather forecast: An exception of type '{error_type}' occurred. Details: {repr(e)}")
            return None

    def _get_core_forecasts(self, config, num_timesteps):
        self.log("Fetching core forecasts...")
        forecasts = {}
        forecast_configs = config.get('forecast_config', {}) 
        if not forecast_configs: self.error("Config error: 'forecast_config' block not found."); return None

        weather_temps = self._get_interpolated_weather_forecast_manual(config, num_timesteps)
        if weather_temps is None: return None
        forecasts['outdoor_temp'] = weather_temps

        for key, entity_config in forecast_configs.items():
            try:
                entity_id, attribute, value_key = entity_config['entity_id'], entity_config['attribute'], entity_config.get('value_key')
                forecast_data = self.get_state(entity_id, attribute=attribute)
                if forecast_data is None: self.error(f"Error fetching '{key}': Attribute '{attribute}' not found on '{entity_id}'."); return None
                processed_list = [float(item[value_key]) for item in forecast_data] if value_key else [float(item) for item in forecast_data]
                
                if len(processed_list) < num_timesteps: 
                    self.error(f"Error for '{key}': Data is too short. Expected {num_timesteps}, got {len(processed_list)}."); return None
                
                forecasts[key] = processed_list[:num_timesteps]
                self.log(f"Successfully fetched {len(forecasts[key])} points for '{key}'.")
            except Exception as e:
                self._log_exception(f"Error fetching forecast for '{key}' from '{entity_id}': {e}"); return None
        return forecasts
    
    def _get_soc_values(self, config, horizon_hours):
        self.log("Fetching SOC values...")
        try:
            soc_init = round(float(self.get_state(config['battery_soc_entity'])) / 100.0, 4)
            soc_schedule = self.get_state(config['soc_forecast_entity'], attribute='battery_scheduled_soc')
            if not soc_schedule: self.error("SOC forecast schedule attribute 'battery_scheduled_soc' not available."); return None
            target_time = self.get_now() + datetime.timedelta(hours=horizon_hours)
            soc_final_percent = next((float(e['dh_soc_batt_forecast']) for e in soc_schedule if self.convert_utc(e['date']) >= target_time), None)
            if soc_final_percent is None:
                self.warning(f"Could not find future SOC forecast point. Using last known value.")
                soc_final_percent = float(soc_schedule[-1]['dh_soc_batt_forecast'])
            soc_final = round(soc_final_percent / 100.0, 4)
            self.log(f"SOC Init: {soc_init*100:.1f}%, SOC Final Target: {soc_final*100:.1f}%")
            return {'soc_init': soc_init, 'soc_final': soc_final}
        except Exception as e:
            self._log_exception(f"Failed to get SOC values: {e}"); return None
            
    def _process_zones(self, config, num_timesteps):
        self.log("Processing thermal zones...")
        all_def_loads, all_custom_ids, all_nominal_powers, all_custom_temp_ids = [], [], [], []
        
        nominal_power = float(config.get('nominal_power', 2500.0))

        for zone_name, zone_config in config['zones'].items():
            self.log(f" > Processing zone: {zone_name}")
            try:
                start_temp = float(self.get_state(f"sensor.{zone_name}_feels_like"))

                for sense in ['heat', 'cool']:
                    desired_temps = self._generate_desired_temps_for_zone(zone_name, sense, num_timesteps)
                    
                    if sense == 'heat':
                        thermal_config = {
                            "sense": "heat",
                            "outdoor_temperature_offset": 1*float(zone_config['outdoor_temperature_offset']),
                            "heating_rate": float(zone_config['heating_rate']),
                            "cooling_constant": float(zone_config['cooling_constant_heat']),
                            "start_temperature": start_temp,
                            "desired_temperatures": desired_temps,
                            "overshoot_temperature": float(zone_config['overshoot_heat_temp'])
                        }
                    else:
                        thermal_config = {
                            "sense": "cool",
                            "outdoor_temperature_offset": 1*float(zone_config['outdoor_temperature_offset']),
                            "heating_rate": -1.0 * float(zone_config['cooling_rate']),
                            "cooling_constant": float(zone_config['cooling_constant_cool']),
                            "start_temperature": start_temp,
                            "desired_temperatures": desired_temps,
                            "overshoot_temperature": float(zone_config['overshoot_cool_temp'])
                        }
                        
                    all_def_loads.append({"thermal_config": thermal_config})
                    all_nominal_powers.append(nominal_power)
                    power_entity_id = f"sensor.p_{zone_name}_{sense}"
                    power_friendly_name = f"{zone_name.replace('_', ' ').title()} {sense.title()}"
                    all_custom_ids.append({ "entity_id": power_entity_id, "unit_of_measurement": "W", "friendly_name": power_friendly_name })
                    temp_entity_id = f"sensor.temp_predicted_{zone_name}_{sense}"
                    temp_friendly_name = f"{zone_name.replace('_', ' ').title()} {sense.title()} Predicted Temperature"
                    all_custom_temp_ids.append({ "entity_id": temp_entity_id, "unit_of_measurement": "°C", "friendly_name": temp_friendly_name })

            except Exception as e:
                self._log_exception(f"Failed to process zone '{zone_name}': {e}"); return None
        
        return {
            "def_loads": all_def_loads, 
            "custom_ids": all_custom_ids, 
            "num_loads": len(all_def_loads), 
            "nominal_powers": all_nominal_powers,
            "custom_temp_ids": all_custom_temp_ids
        }


    def _generate_desired_temps_for_zone(self, zone, sense, num_timesteps):
        self.log(f"    - Generating desired temps for zone='{zone}', sense='{sense}'")
        inactive_suffix, active_suffix = ("low", "low") if sense == "heat" else ("high", "high")
        inactive_setpoint = float(self.get_state(f"input_number.{zone}_setpoint_inactive_{inactive_suffix}"))
        active_setpoint = float(self.get_state(f"input_number.{zone}_setpoint_active_{active_suffix}"))
        desired_temps = [inactive_setpoint] * num_timesteps
        entity_to_match = f"input_boolean.{zone}_scheduled_{sense}"
        
        tz_str = self.args.get("time_zone")
        if not tz_str:
            self.error("Required 'time_zone' parameter not found in app configuration (apps.yaml). The script cannot continue.")
            return desired_temps
        local_tz = pytz.timezone(tz_str)

        now_utc_raw = self.get_now()
        snapped_minute = (now_utc_raw.minute // 30) * 30
        now_utc = now_utc_raw.replace(minute=snapped_minute, second=0, microsecond=0)
        now_local = now_utc.astimezone(local_tz)
        
        weekday_map = ["mon", "tue", "wed", "thu", "fri", "sat", "sun"]
        active_windows = []
        found_schedule = False

        for switch_id in self.get_state("switch"):
            if switch_id.startswith("switch.schedule_"):
                # >> START FIX: Check if the schedule switch entity is enabled ('on') before processing it <<
                if self.get_state(switch_id) == 'on':
                    try:
                        attributes = self.get_state(switch_id, attribute="all")["attributes"]
                        if entity_to_match in attributes.get("entities", []):
                            found_schedule = True
                            self.log(f"      > Found matching schedule '{switch_id}' for '{entity_to_match}'")
                            
                            weekdays_attr = attributes.get("weekdays")
                            actions = attributes.get("actions", [])
                            timeslots_attr = attributes.get("timeslots")

                            if not all([weekdays_attr, actions, timeslots_attr]):
                                self.warning(f"        - Schedule '{switch_id}' is missing required attributes. Skipping.")
                                continue
                            
                            if isinstance(timeslots_attr, str): timeslots = [t.strip() for t in timeslots_attr.split(',')]
                            elif isinstance(timeslots_attr, list): timeslots = timeslots_attr
                            else: continue
                            
                            raw_days = []
                            if isinstance(weekdays_attr, str):
                                raw_days = [day.strip() for day in weekdays_attr.split(',')]
                            elif isinstance(weekdays_attr, list):
                                raw_days = weekdays_attr

                            active_days = []
                            for day_keyword in raw_days:
                                if day_keyword == 'daily': active_days.extend(weekday_map)
                                elif day_keyword == 'workday': active_days.extend(weekday_map[:5])
                                elif day_keyword == 'weekend': active_days.extend(weekday_map[5:])
                                else: active_days.append(day_keyword)
                            active_days = list(set(active_days)) 

                            if not active_days: continue
                            
                            if len(actions) != len(timeslots): continue

                            for i, action in enumerate(actions):
                                if isinstance(action, dict) and action.get('service') == 'input_boolean.turn_on':
                                    timeslot_item = timeslots[i]
                                    if isinstance(timeslot_item, str) and " - " in timeslot_item:
                                        start_str, end_str = timeslot_item.split(" - ")
                                        start_h, start_m, _ = map(int, start_str.split(':'))
                                        end_h, end_m, _ = map(int, end_str.split(':'))

                                        for day_offset in range(-1, 3):
                                            check_day = now_local + datetime.timedelta(days=day_offset)
                                            if weekday_map[check_day.weekday()] in active_days:
                                                start_time = check_day.replace(hour=start_h, minute=start_m, second=0, microsecond=0)
                                                end_time = check_day.replace(hour=end_h, minute=end_m, second=0, microsecond=0)
                                                if end_time <= start_time: end_time += datetime.timedelta(days=1)
                                                
                                                forecast_end_time = now_local + datetime.timedelta(hours=num_timesteps/2)
                                                if start_time < forecast_end_time and end_time > now_local:
                                                    active_windows.append((start_time, end_time))
                    except Exception as e:
                        self.error(f"Error processing schedule switch '{switch_id}': {type(e).__name__}: {e}")
                        continue
                # >> END FIX <<

        if not found_schedule:
            self.log(f"      > WARNING: No active schedules found for '{entity_to_match}'. Using inactive setpoints.")
            return desired_temps

        unique_windows = sorted(list(set(active_windows)))

        for step in range(num_timesteps):
            timestep_time_utc = now_utc + datetime.timedelta(minutes=step * 30)
            timestep_time_local = timestep_time_utc.astimezone(local_tz)
            
            for start_window, end_window in unique_windows:
                if start_window <= timestep_time_local < end_window:
                    desired_temps[step] = active_setpoint
                    break
                    
        self.log(f"      > Successfully generated temperature forecast. Active setpoint applied to {desired_temps.count(active_setpoint)} of {num_timesteps} timesteps.")
        return desired_temps
