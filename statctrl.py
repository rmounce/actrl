import hassapi as hass


class Statctrl(hass.Hass):
    def initialize(self):
        self.target_high = None
        self.target_low = None
        self.step_time = None

        self.loop_handle = None

        for entity in [
            "manual",
            "contact",
            "occupancy",
            "hard_on",
            "soft_on",
            "sleep_mode",
        ]:
            if entity in self.args:
                self.listen_state(self.main_state, self.args[entity])
        self.main()

    def main_state(self, entity, attribute, old, new, cb_args):
        self.main()

    def main_timer(self, cb_args):
        self.main()

    def main(self):
        self.cancel_timer(handle=self.loop_handle, silent=True)

        open = self.get_state(self.args["contact"]) == "on"
        manual = "manual" in self.args and self.get_state(self.args["manual"]) == "on"
        sleep_mode = (
            "sleep_mode" in self.args
            and self.get_state(self.args["sleep_mode"]) == "on"
        )
        hard_on = (
            "hard_on" in self.args and self.get_state(self.args["hard_on"]) == "on"
        )
        soft_on = (
            not hard_on
            and "soft_on" in self.args
            and self.get_state(self.args["soft_on"]) == "on"
        )
        occupied = (
            hard_on
            or soft_on
            or (
                "occupancy" in self.args
                and self.get_state(self.args["occupancy"]) == "on"
            )
        )

        if manual and not open:
            self.log("MANUAL")
            return
        if open:
            self.log("OPEN")
            self.target_high = self.args["open_high"]
            self.target_low = self.args["open_low"]
            self.step_time = self.args["open_step_time"]
        elif not occupied:
            self.log("UNOCCUPIED")
            self.target_high = self.args["vacant_high"]
            self.target_low = self.args["vacant_low"]
            self.step_time = self.args["vacant_step_time"]
        else:
            if sleep_mode:
                self.log("BEDTIME")
                self.target_high = self.args["occupied_high"]
                self.target_low = self.args["vacant_low"]
            else:
                self.log("OCCUPIED & CLOSED")
                self.target_high = self.args["occupied_high"]
                self.target_low = self.args["occupied_low"]
            if soft_on:
                self.step_time = self.args["vacant_step_time"]
            else:
                self.step_time = self.args["occupied_step_time"]

        self.set_temp()

    def set_temp(self):
        self.log("Setting temp")

        cur_climate = self.get_entity(self.args["climate"])
        cur_high = cur_climate.get_state("target_temp_high")
        cur_low = cur_climate.get_state("target_temp_low")

        if (cur_high != self.target_high) or (cur_low != self.target_low):
            if self.step_time == 0:
                cur_high = self.target_high
                cur_low = self.target_low
            else:
                if cur_high < self.target_high:
                    cur_high = min(self.target_high, cur_high + self.args["step_size"])
                elif cur_high > self.target_high:
                    cur_high = max(self.target_high, cur_high - self.args["step_size"])

                if cur_low < self.target_low:
                    cur_low = min(self.target_low, cur_low + self.args["step_size"])
                elif cur_low > self.target_low:
                    cur_low = max(self.target_low, cur_low - self.args["step_size"])

            self.call_service(
                "climate/set_temperature",
                entity_id=self.args["climate"],
                target_temp_high=cur_high,
                target_temp_low=cur_low,
            )

        if (cur_high != self.target_high) or (cur_low != self.target_low):
            self.loop_handle = self.run_in(self.main_timer, self.step_time)
