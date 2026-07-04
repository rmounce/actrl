"""Deterministic emulator of the Midea ducted unit's observed follow-me
behaviour -- the flip side of `control.MideaCapacityController`.

The real Actrl app tricks the Midea unit's own "follow me" thermostat logic
into stepping its compressor speed by transmitting a fake ambient
temperature once per 10 s control interval. This module emulates the unit's
side of that exchange: given a stream of reported temperatures against a
fixed setpoint, it dead-reckons the unit's compressor speed and internal
quirks (ramp flags, purge cycle, setpoint-reached shutdown) the same way
`MideaCapacityController.guesstimated_comp_speed` dead-reckons them from the
controller side.

ONLY SOURCE OF BEHAVIOUR: the comments/docstrings in control.py
(`MideaCapacityController`, `compress`, `midea_runtime_quirks`) and the
"Capacity control" section of docs/actrl.md. Nothing here invents behaviour
beyond those two sources -- see the Assumptions table below for every place
they were silent and a choice had to be made.

No defrost emulation (needs outdoor conditions, future task). No power
on/off protocol emulation (the follow-me channel only controls compressor
*speed* while the unit is already running; on/off is a separate HA climate
entity in the real system, out of scope here -- see Assumption 8).

## Assumptions

| # | Behaviour                          | Sources say                                                                 | Choice made                                                                                                          | Parameter                |
|---|-------------------------------------|-------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------|---------------------------|
| 1 | Magnitude of ordinary stepping      | "Each +-1C *change* of reported error steps speed by +-1 increment"          | Each interval's report change contributes at most +-1 to comp_speed regardless of magnitude (sign of the delta only), not a magnitude-proportional step. Verified against the recorded fixtures: the crafted step-up sequence (`[+1, +2, 0]` offsets, 3 transitions) nets exactly +1 with this rule, matching `guesstimated_comp_speed`'s observed +1-per-event growth in cold_start.json/on_counter_clamp.json. The crafted step-down sequence (`[-1, -2, +1, 0]` offsets, 4 transitions -- one longer) nets -2 instead of the true -1 under this rule; that known one-increment overshoot on decrement events is inside the +-2 tracking tolerance and is documented rather than chased with sequence-specific pattern matching (which would require the emulator to know control.py's internals, defeating the point of an independent emulator). | n/a                       |
| 2 | Rate while a ramp flag is latched   | "ramps speed toward max/0 ... regardless of subsequent reports" -- no rate given | 1 increment per 10 s interval, same cadence as ordinary stepping, for internal consistency.                          | `ramp_flag_rate`          |
| 3 | Ramp-up latch debounce, clear, and climb rate | "reported_error >= +2 sets a latching ramp-up flag ... no known way to clear" -- but control.py's own step-up sequence (`[+1, +2, 0]` offsets from stable=1, i.e. raw values 2, 3) transiently crosses +2 on *every single* ordinary +1 speed increment, which is inconsistent with the documented goal of sustaining "equilibrium with stable compressor speed". Recorded closed-loop replay data (2026-06-22 06:20-09:20, task 010/012 investigation) shows the real outdoor unit peaking at ~2.5 kW (mid-speed) then tapering smoothly in lockstep with actrl's step-down sequence (`[-1, -2, +1, 0]` offsets) from 07:15-09:20 -- the leading elements reach reported_error -1/-2, and the real unit demonstrably follows them back down, disproving "never clears". The same recording shows the real climb rate is ~1 increment per 3-6 min, not 1 per 10 s cycle -- an unclearable, fast-climbing latch pins the emulator at max speed for hours in closed-loop replay, producing a hard cutoff instead of the observed taper. | Require reported_error >= threshold for `ramp_up_debounce_cycles` (default 3) *consecutive* intervals before latching (unchanged). Once latched, a report at or below `ramp_down_set_threshold` (the same threshold that sets the ramp-down flag, default -1 -- i.e. a decrement-demand report such as the step-down sequence's leading elements) clears the ramp-up flag and the debounce streak. While latched, speed climbs by `ramp_flag_rate` only once every `ramp_up_period_cycles` intervals (default 18, ~3 min at 10 s cycles); on the other latched intervals ordinary delta-stepping still applies rather than being suppressed (observed deep-demand reports hold roughly constant, so delta is usually 0 and speed holds between climbs). | `ramp_up_threshold`, `ramp_up_debounce_cycles`, `ramp_up_period_cycles` |
| 4 | Ramp-down set/clear                 | "<= -1 sets it, >= +1 clears it" -- fully specified, and control.py's step-down sequence (raw values 0, -1, 2, 1) is self-clearing by design. | Implemented literally, instantaneous, every interval, no debounce needed.                                            | `ramp_down_set_threshold`, `ramp_down_clear_threshold` |
| 5 | Purge low-speed threshold           | "continuous running below a low-speed threshold" -- threshold not named       | comp_speed <= 2, echoing `compressor_power_safety_margin` (the controller's own notion of a "near minimum" band).     | `purge_speed_threshold`   |
| 6 | Purge duration                      | "~1 min full-speed purge"                                                     | Exactly 6 intervals (60 s / 10 s).                                                                                    | `purge_duration_cycles`   |
| 7 | Purge effect on speed               | "full-speed purge, then return to prior speed"                               | Purge forces comp_speed to `max_speed` and freezes it there (ordinary stepping/ramp logic suspended) for the purge duration, then restores the pre-purge speed; the 90 min clock restarts from 0 regardless of speed at restore time. | n/a |
| 8 | No restart after shutdown           | "Holding at setpoint-reached ... shuts the unit down entirely"; no source describes a follow-me-driven power-on rule (power on/off is a separate HA climate-entity protocol in the real system). | Once shut down by rule 6, `running` stays False and comp_speed 0 for the rest of the instance's life; `reset()` is provided for test convenience but never called automatically. | n/a |
| 9 | "Setpoint-reached report"           | "Holding at the setpoint-reached report for ~1h shuts the unit down ... any report change resets that timer" | Interpreted literally: reported_error == 0 (reported temp exactly equals setpoint). Any interval with reported_error != 0 resets the counter to 0. This is exactly the condition the `min_power_time` "blip" in `compress()` exists to avoid (it forces a nonzero report periodically), so the two designs interlock as intended by the source comment. | `shutdown_after_cycles`  |
| 10| Initial state                       | n/a                                                                            | Constructed already running, comp_speed 0, no flags set, `prev_reported_error` 0 -- mirrors `MideaCapacityController.__init__`'s own cold-start assumptions (guesstimated_comp_speed 0, no flags). | n/a |
| 11| `reported_temp` -> `reported_error` | "one reported temperature per control interval ... work in offsets ... integer degrees" | `reported_error = round(mode_sign * (reported_temp - setpoint))`, mode_sign +1 cool / -1 heat mirroring actrl.py's `mode_sign`: the rule thresholds are written in demand space (cooling convention); in heat mode the real system reports BELOW setpoint to demand more compressor (found via closed-loop sim 2026-07-03). Rules evaluate the rounded integer offset, matching `compress()`'s own `round(rval)`. | `mode` |

These are all reviewable/tunable constructor parameters precisely so they
can be corrected against recorded data without touching the rule structure.
"""

from __future__ import annotations

# Mirrors control.py's real-world scale (10 s control interval); duplicated
# here rather than imported so this module has zero dependency on control.py
# (see task 007: the emulator must not couple to the thing it's exercising).
DEFAULT_INTERVAL_SECONDS = 10.0
DEFAULT_MAX_SPEED = 14  # control.py's compressor_power_increments

# ~90 minutes at 10 s/interval.
DEFAULT_PURGE_DELAY_CYCLES = int(90 * 60 / DEFAULT_INTERVAL_SECONDS)
# ~1 minute.
DEFAULT_PURGE_DURATION_CYCLES = int(60 / DEFAULT_INTERVAL_SECONDS)
# ~1 hour holding at the setpoint-reached report.
DEFAULT_SHUTDOWN_AFTER_CYCLES = int(60 * 60 / DEFAULT_INTERVAL_SECONDS)

DEFAULT_RAMP_UP_THRESHOLD = 2
DEFAULT_RAMP_UP_DEBOUNCE_CYCLES = 3
# ~3 min at 10 s/interval -- one latched climb increment per period, per the
# 2026-06-22 recorded taper (see Assumption 3).
DEFAULT_RAMP_UP_PERIOD_CYCLES = 18
DEFAULT_RAMP_DOWN_SET_THRESHOLD = -1
DEFAULT_RAMP_DOWN_CLEAR_THRESHOLD = 1
DEFAULT_RAMP_FLAG_RATE = 1
DEFAULT_PURGE_SPEED_THRESHOLD = 2


class MideaUnit:
    """Emulates the Midea ducted unit's compressor-speed response to a
    stream of follow-me temperature reports against a fixed setpoint.

    Call `step(reported_temp)` once per 10 s control interval (matching the
    real cadence). Read-only telemetry attributes for assertions:
    `comp_speed`, `ramp_up_flag`, `ramp_down_flag`, `running`,
    `low_speed_minutes`, `purging`.
    """

    def __init__(
        self,
        setpoint,
        mode="cool",
        max_speed=DEFAULT_MAX_SPEED,
        interval_seconds=DEFAULT_INTERVAL_SECONDS,
        purge_delay_cycles=DEFAULT_PURGE_DELAY_CYCLES,
        purge_duration_cycles=DEFAULT_PURGE_DURATION_CYCLES,
        purge_speed_threshold=DEFAULT_PURGE_SPEED_THRESHOLD,
        shutdown_after_cycles=DEFAULT_SHUTDOWN_AFTER_CYCLES,
        ramp_up_threshold=DEFAULT_RAMP_UP_THRESHOLD,
        ramp_up_debounce_cycles=DEFAULT_RAMP_UP_DEBOUNCE_CYCLES,
        ramp_up_period_cycles=DEFAULT_RAMP_UP_PERIOD_CYCLES,
        ramp_down_set_threshold=DEFAULT_RAMP_DOWN_SET_THRESHOLD,
        ramp_down_clear_threshold=DEFAULT_RAMP_DOWN_CLEAR_THRESHOLD,
        ramp_flag_rate=DEFAULT_RAMP_FLAG_RATE,
    ):
        if mode not in ("cool", "heat"):
            raise ValueError(f"mode must be 'cool' or 'heat', got {mode!r}")
        self.setpoint = setpoint
        # Demand-space convention: all the rules in this module (and the
        # documented flag thresholds) are written for cooling, where a
        # report ABOVE setpoint asks for more compressor. actrl transmits
        # setpoint + mode_sign*demand (mode_sign: cool +1, heat -1 — see
        # actrl.py `mode_sign` and `compressed_error`), so in heat mode a
        # report BELOW setpoint asks for more compressor and we flip the
        # sign back into demand space here.
        self.mode_sign = 1.0 if mode == "cool" else -1.0
        self.mode = mode
        self.max_speed = max_speed
        self.interval_seconds = interval_seconds
        self.purge_delay_cycles = purge_delay_cycles
        self.purge_duration_cycles = purge_duration_cycles
        self.purge_speed_threshold = purge_speed_threshold
        self.shutdown_after_cycles = shutdown_after_cycles
        self.ramp_up_threshold = ramp_up_threshold
        self.ramp_up_debounce_cycles = ramp_up_debounce_cycles
        self.ramp_up_period_cycles = ramp_up_period_cycles
        self.ramp_down_set_threshold = ramp_down_set_threshold
        self.ramp_down_clear_threshold = ramp_down_clear_threshold
        self.ramp_flag_rate = ramp_flag_rate

        self.reset()

    def reset(self):
        """Return to the assumed cold-start state (see Assumption 10)."""
        self.comp_speed = 0
        self.ramp_up_flag = False
        self.ramp_down_flag = False
        self.running = True

        self._prev_reported_error = 0
        self._ramp_up_streak = 0
        self._ramp_up_period_counter = 0

        # Purge / low-speed clock.
        self.purging = False
        self._purge_cycles_remaining = 0
        self._pre_purge_speed = 0
        self._low_speed_counter = 0

        # Setpoint-reached shutdown clock.
        self._at_setpoint_counter = 0

        # Last computed reported_error, exposed for debugging/telemetry.
        self.reported_error = 0

    @property
    def low_speed_minutes(self):
        """Minutes of continuous low-speed running counted toward the next
        purge (Assumption 5/6)."""
        return self._low_speed_counter * self.interval_seconds / 60.0

    @property
    def minutes_at_setpoint(self):
        """Minutes of continuous reported_error == 0 counted toward the
        setpoint-reached shutdown (Assumption 9)."""
        return self._at_setpoint_counter * self.interval_seconds / 60.0

    def step(self, reported_temp) -> None:
        """Advance one control interval given the newly reported (fake)
        ambient temperature."""
        reported_error = round(self.mode_sign * (reported_temp - self.setpoint))
        self.reported_error = reported_error

        if not self.running:
            # No follow-me-driven power-on rule in the sources (Assumption 8).
            self._prev_reported_error = reported_error
            return

        delta = reported_error - self._prev_reported_error

        # --- Ramp-up flag clear rule (Assumption 3, task 012): a decrement-
        # --- demand report (the same threshold that sets the ramp-down
        # --- flag) clears the latch and the debounce streak. Recorded data
        # --- shows the real unit follows a step-down sequence back down, so
        # --- this must run before the (unchanged) set logic below.
        if reported_error <= self.ramp_down_set_threshold:
            self.ramp_up_flag = False
            self._ramp_up_streak = 0
            self._ramp_up_period_counter = 0

        # --- Ramp-down flag: rule 4, fully specified, no debounce needed. ---
        if reported_error <= self.ramp_down_set_threshold:
            self.ramp_down_flag = True
        elif reported_error >= self.ramp_down_clear_threshold:
            self.ramp_down_flag = False

        # --- Ramp-up flag: rule 3, latching, debounced (Assumption 3). ---
        if not self.ramp_up_flag:
            if reported_error >= self.ramp_up_threshold:
                self._ramp_up_streak += 1
            else:
                self._ramp_up_streak = 0
            if self._ramp_up_streak >= self.ramp_up_debounce_cycles:
                self.ramp_up_flag = True

        # --- Purge handling (rule 5, Assumptions 5-7). ---
        if self.purging:
            self._purge_cycles_remaining -= 1
            if self._purge_cycles_remaining <= 0:
                self.purging = False
                self.comp_speed = self._pre_purge_speed
                self._low_speed_counter = 0
        else:
            if self.comp_speed <= self.purge_speed_threshold:
                self._low_speed_counter += 1
            else:
                self._low_speed_counter = 0

            if self._low_speed_counter >= self.purge_delay_cycles:
                self._pre_purge_speed = self.comp_speed
                self.purging = True
                self._purge_cycles_remaining = self.purge_duration_cycles
                self.comp_speed = self.max_speed

            if not self.purging:
                # --- Ordinary speed update: flags take priority over the
                # --- plain delta-stepping rule (rule 3 explicitly overrides
                # --- "subsequent reports" while ramping).
                if self.ramp_up_flag:
                    # Task 012: the latched climb is slow (~1 increment per
                    # 3-6 min observed, Assumption 3), not every cycle like
                    # the ramp-down path. On the cycles between climbs,
                    # ordinary delta-stepping still applies rather than
                    # being suppressed (in the observed regime actrl holds
                    # its report roughly constant during deep demand, so
                    # delta is usually 0 and speed holds).
                    self._ramp_up_period_counter += 1
                    if self._ramp_up_period_counter >= self.ramp_up_period_cycles:
                        self._ramp_up_period_counter = 0
                        self.comp_speed = min(
                            self.max_speed, self.comp_speed + self.ramp_flag_rate
                        )
                    else:
                        step = (delta > 0) - (delta < 0)
                        self.comp_speed = max(
                            0, min(self.max_speed, self.comp_speed + step)
                        )
                elif self.ramp_down_flag:
                    self.comp_speed = max(
                        0, self.comp_speed - self.ramp_flag_rate
                    )
                else:
                    step = (delta > 0) - (delta < 0)  # sign(delta): -1, 0, +1
                    self.comp_speed = max(
                        0, min(self.max_speed, self.comp_speed + step)
                    )

        # --- Setpoint-reached shutdown clock (rule 6, Assumption 9). ---
        if reported_error == 0:
            self._at_setpoint_counter += 1
            if self._at_setpoint_counter >= self.shutdown_after_cycles:
                self.running = False
                self.comp_speed = 0
                self.purging = False
        else:
            self._at_setpoint_counter = 0

        self._prev_reported_error = reported_error
