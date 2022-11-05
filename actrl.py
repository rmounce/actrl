import hassapi as hass
import datetime
import math
from simple_pid import PID
from collections import deque
import time

rooms = [
         "bed_1",
         "bed_2",
         "bed_3",
         "kitchen",
         "study"
         ]

# in minutes
interval = 0.25 # 15 seconds

# WMA over the last 7.5 minutes
temp_deriv_window = int(7.5 / interval)
# to predict 10 minutes into the future
temp_deriv_factor = int(10.0 / interval)

class MyWMA:
    def __init__(self, window_size):
        self.window_size = window_size
        self.clear()

    def clear(self):
        self.history = deque([])
        self.pad()

    def pad(self):
        while len(self.history) < self.window_size:
            self.history.appendleft(0.0)
        while len(self.history) > self.window_size:
            self.history.popleft()

    def update(self, value):
        self.history.append(value)
        self.pad()

    def average(self):
        i = 0
        i_sum = 0
        val_sum = 0.0

        for val in self.history:
            i = i + 1
            i_sum = i_sum + i
            val_sum = val_sum + val * i

        return val_sum / i_sum


class Actrl(hass.Hass):
    def initialize(self):
        self.pids = {}
        self.roll_deriv = deque([])
        self.prev_error = None
        self.prev_target = None
        self.ramping_down = True
        self.totally_off = True
        self.heat_mode = False
        self.on_counter = 0

        for room in rooms:
            self.pids[room]=PID(Kp=1.0, Ki=0.001, Kd=0, setpoint=0, sample_time=None, output_limits=(-1.0, 1.0))
        #self.run_minutely(self.main, None)
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

        main_setpoint=float(self.get_entity("climate.aircon").get_state("temperature"))

        for room in rooms:
            if self.get_state("climate."+room+"_aircon") != "off":
                all_disabled = False
                temps[room] = float(self.get_entity("climate."+room+"_aircon").get_state("current_temperature"))
                targets[room] = float(self.get_entity("climate."+room+"_aircon").get_state("temperature"))
                errors[room] = temps[room] - targets[room]
                if self.get_state("climate."+room+"_aircon") == "heat":
                    heat_room_count+=1
                if self.get_state("climate."+room+"_aircon") == "cool":
                    cool_room_count+=1
            else:
                disabled_rooms.append(room)

        if all_disabled:
            #adjusted_temp = main_setpoint + avg_error
            #self.get_entity("input_number.fake_temperature").set_state(state=adjusted_temp)
            self.get_entity("input_number.fake_temperature").set_state(state=main_setpoint)
            for room, pid in self.pids.items():
                pid.auto_mode = False
            self.roll_deriv = deque([])
            self.prev_error = None
            self.prev_avg_target = None
            self.totally_off = True
            self.on_counter = 0
            if self.get_state("climate.aircon") != "fan_only":
                self.try_set_mode("off")
                #self.call_service('climate/turn_off', entity_id="climate.aircon")
            return
        #all_disabled = (self.get_state("input_boolean.manual_aircon") == "on" or self.get_state("climate.aircon") not in ["cool","heat"])

        if heat_room_count > 0 and cool_room_count == 0:
            self.turn_on("input_boolean.heat_mode")
        elif cool_room_count > 0 and heat_room_count == 0:
            self.turn_off("input_boolean.heat_mode")

        if self.get_state("input_boolean.heat_mode") == "on":
            heat_cool_sign = -1.0
            self.heat_mode = True
            self.try_set_mode("heat")
            #self.call_service('climate/set_hvac_mode', entity_id="climate.aircon", hvac_mode="heat")
        else:
            self.heat_mode = False
            self.try_set_mode("cool")
            #self.call_service('climate/set_hvac_mode', entity_id="climate.aircon", hvac_mode="cool")

        avg_temp   = sum(temps.values())   / len(temps.values())
        avg_target = sum(targets.values()) / len(targets.values())
        avg_error  = sum(errors.values())  / len(errors.values())


        for room, error in errors.items():
            if not self.pids[room].auto_mode:
                self.pids[room].set_auto_mode(True, last_output=0.0)
                # a new room has been enabled, reset the deriv history
                self.roll_deriv = deque([])
                self.prev_error = None
                self.prev_avg_target = None

            pid_vals[room] = self.pids[room](heat_cool_sign * (error - avg_error))
            self.get_entity("input_number."+room+"_pid").set_state(state=pid_vals[room])

        for room in disabled_rooms:
            if self.pids[room].auto_mode:
                self.pids[room].auto_mode = False
                # a new room has been disabled, reset the deriv history
                self.roll_deriv = deque([])
                self.prev_error = None
                self.prev_avg_target = None

            if self.get_entity("cover."+room).get_state("current_position") != "0":
                self.log("closing damper for disabled room " + room)
                self.call_service('cover/set_cover_position', entity_id=("cover."+room), position=0)

        min_pid = min(pid_vals.values())
        max_pid = max(pid_vals.values())

        error_sum = 0.0
        target_sum = 0.0
        weight_sum = 0.0


        for room, pid_val in pid_vals.items():
            scaled = 100.0*((1.01 - pid_val) / (1.01 - min_pid))

            damper_vals[room] = scaled
            target_sum += targets[room] * scaled
            error_sum += errors[room] * scaled
            weight_sum += scaled

        weighted_error = error_sum / weight_sum
        weighted_target = target_sum / weight_sum

        if self.prev_error is None:
            self.prev_error = weighted_error
        if self.prev_target is None:
            self.prev_target = weighted_target

        raw_deriv = (weighted_error - self.prev_error) + (weighted_target - self.prev_target)
        self.prev_error = weighted_error
        self.prev_target = weighted_target

        # average the last 5 minutes to predict the next 10
        deriv_window = 20
        deriv_weight = 40

        self.roll_deriv.append(raw_deriv)
        while len(self.roll_deriv) < deriv_window:
            self.roll_deriv.appendleft(0.0)
        while len(self.roll_deriv) > deriv_window:
            self.roll_deriv.popleft()
        avg_deriv = deriv_weight * sum(self.roll_deriv) / len(self.roll_deriv)

        # don't shut off the compressor when the RoC is too fast, cap at min power
        # compress(-1)
        #deriv_floor = -(1.0 + 0.5) * 0.3

        weighted_error *= heat_cool_sign
        avg_deriv *= heat_cool_sign

        #compressed_error = heat_cool_sign * self.compress(weighted_error if weighted_error < deriv_floor else max(deriv_floor, weighted_error + avg_deriv))
        compressed_error = heat_cool_sign * self.compress(weighted_error, avg_deriv)
        self.log("error weighted "+str(weighted_error)+" deriv "+str(avg_deriv)+" compressed "+str(compressed_error))
        self.get_entity("input_number.fake_temperature").set_state(state=(main_setpoint+compressed_error))

        damper_deadband = 4.0
        #if self.totally_off and self.get_state("input_boolean.heat_mode") == "on":
        if False:
            self.log("indoor fan is not running, not adjusting dampers")
        else:
#            for room, damper_val in damper_vals.items():
            for room in sorted(damper_vals, key=damper_vals.get, reverse=True):
                damper_val = damper_vals[room]

                cur_pos = float(self.get_entity("cover."+room).get_state("current_position"))
                self.log(room + " scaled: " + str(damper_val) + " cur_pos: " + str(cur_pos))
                if (damper_val > 99.9 and cur_pos < 100.0) or (damper_val > (cur_pos+damper_deadband)) or (damper_val < (cur_pos-damper_deadband)):
                    self.log("setting " + room)
                    self.call_service('cover/set_cover_position', entity_id=("cover."+room), position=(damper_val))
                    time.sleep(1)
                else:
                    self.log("within deadband, not setting " + room)


    def try_set_mode(self, mode):
        if self.get_state("climate.aircon") != mode:
            self.call_service('climate/set_hvac_mode', entity_id="climate.aircon", hvac_mode=mode)

    def actually_compress(self, error):
        # tell the aircon that temp variations are 5x worse than reality
        return 0.5 + error / 0.2

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

        rval = min(rval,  15.0)
        rval = max(rval, -15.0)
        unrounded_rval = rval
        rval = round(rval)

        if rval <= off_threshold:
            self.totally_off = True
            self.on_counter = 0

        if self.totally_off and rval < on_threshold:
            return rval
        
        self.totally_off = False
        self.on_counter += 1

        # soft start to avoid overshoot
        # start at min power and gradually report the actual rval
        # 7.5 min delay at min power
        soft_delay = 30
        # then
        # 7.5 min ramp to real temp
        soft_ramp = 30

        if rval > on_threshold:
            if self.on_counter < soft_delay:
                return on_threshold
            if self.on_counter < (soft_delay + soft_ramp):
                ramp_progress = (self.on_counter - soft_delay) / soft_ramp
                print("ramping " + str(ramp_progress))
                return round((ramp_progress * unrounded_rval) + ((1.0-ramp_progress) * on_threshold))


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

