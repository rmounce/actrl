"""Tests for sim.midea_unit.MideaUnit.

Part 1: unit tests for each behavioural rule in the module docstring.
Part 2: closed-loop tracking test driving control.MideaCapacityController
against MideaUnit over the recorded compress()-input trajectories in
tests/fixtures/capacity/*.json (read-only; only the `compress` op inputs
are used -- external-mutation fixtures are skipped, see below).
"""

import json
from pathlib import Path

import pytest

from control import MideaCapacityController, compressor_power_increments, faithful_threshold, soft_delay
from sim.midea_unit import MideaUnit

SETPOINT = 24


# ---------------------------------------------------------------------------
# Part 1: rule-by-rule unit tests
# ---------------------------------------------------------------------------


def test_speed_bounds_0_and_max():
    u = MideaUnit(setpoint=SETPOINT, max_speed=5)
    # Task 012: the latched ramp-up climb is slow (1 increment per
    # ramp_up_period_cycles), so give it enough cycles to actually reach
    # max_speed via the latch rather than the old fast-climb cadence.
    for _ in range(u.ramp_up_debounce_cycles + u.ramp_up_period_cycles * u.max_speed + 5):
        u.step(SETPOINT + 5)  # push far above max
    assert u.comp_speed == 5

    u2 = MideaUnit(setpoint=SETPOINT, max_speed=5)
    for _ in range(20):
        u2.step(SETPOINT - 5)  # push far below min
    assert u2.comp_speed == 0


def test_ordinary_step_changes_speed_by_delta():
    u = MideaUnit(setpoint=SETPOINT)
    u.step(SETPOINT + 1)
    assert u.comp_speed == 1
    u.step(SETPOINT + 1)  # no change -> no step
    assert u.comp_speed == 1
    # Task 013: a report exactly at setpoint (SETPOINT + 0) now holds
    # regardless of delta, so use a nonzero error to keep testing ordinary
    # delta-stepping (still sign-capped: -2 change -> -1 speed).
    u.step(SETPOINT - 1)  # -2 change -> -1 speed
    assert u.comp_speed == 0
    u.step(SETPOINT + 1)  # +2 change -> +1 speed
    assert u.comp_speed == 1


def test_ordinary_step_caps_at_one_regardless_of_jump_magnitude():
    """A single-cycle multi-degree jump only steps speed by 1, not by the
    full magnitude (Assumption 1: sign of the delta only)."""
    u = MideaUnit(setpoint=SETPOINT)
    u.step(SETPOINT + 5)  # a big single-cycle jump
    assert u.comp_speed == 1
    assert u.ramp_up_flag is False


def test_step_down_sequence_nets_minus_one():
    """Task 013: actrl's crafted step-down sequence (`[-1, -2, +1, 0]`
    offsets from stable, i.e. reported errors `[0, -1, 2, 1]`) nets -1 under
    sign-of-delta stepping plus the at-setpoint-holds rule, matching
    `guesstimated_comp_speed`'s dead-reckoning of the same sequence
    (previously netted -2 before this task, doubling the emulated taper
    rate in closed-loop replay)."""
    u = MideaUnit(setpoint=SETPOINT)
    # Prime directly to "stable" (comp_speed 8, prev_reported_error 1)
    # rather than via a warm-up step, to make the starting state explicit.
    u.comp_speed = 8
    u._prev_reported_error = 1
    for rval in (0, -1, 2, 1):
        u.step(SETPOINT + rval)
    assert u.comp_speed == 7


def test_step_up_sequence_nets_plus_one():
    """Mirror-image of test_step_down_sequence_nets_minus_one: the step-up
    sequence (`[+1, +2, 0]` offsets from stable, i.e. reported errors
    `[2, 3, 1]`) nets +1, unchanged by the at-setpoint-holds rule (none of
    its reports are exactly at setpoint)."""
    u = MideaUnit(setpoint=SETPOINT)
    u.comp_speed = 8
    u._prev_reported_error = 1
    for rval in (2, 3, 1):
        u.step(SETPOINT + rval)
    assert u.comp_speed == 9


def test_at_setpoint_report_holds_speed():
    """Task 013: a report exactly at setpoint holds compressor speed (no
    ordinary step), regardless of the preceding delta -- the old rule would
    have stepped -1 here (delta 0 - 1 = -1)."""
    u = MideaUnit(setpoint=SETPOINT)
    u.comp_speed = 5
    u._prev_reported_error = 1
    u.step(SETPOINT)  # reported_error == 0 -> hold
    assert u.comp_speed == 5


def test_ramp_up_flag_latches_and_ignores_holds():
    """Task 012: once latched, reports that hold/step within actrl's
    observed deep-demand band (0..+1) do not clear the flag, and speed never
    decreases while latched -- it only climbs, at the slow latched rate."""
    u = MideaUnit(setpoint=SETPOINT, max_speed=14)
    # Sustain reported_error >= threshold for the full debounce window.
    for _ in range(u.ramp_up_debounce_cycles):
        u.step(SETPOINT + 2)
    assert u.ramp_up_flag is True

    u.step(SETPOINT + 1)  # transient step into the hold band
    assert u.ramp_up_flag is True
    speed_at_hold_start = u.comp_speed
    speed_before = speed_at_hold_start

    for _ in range(3 * u.ramp_up_period_cycles):
        u.step(SETPOINT + 1)  # constant report -> ordinary delta is 0
        assert u.ramp_up_flag is True
        assert u.comp_speed >= speed_before
        speed_before = u.comp_speed

    # The slow latched climb still advances speed over this window even
    # though ordinary delta-stepping contributed nothing (delta stayed 0).
    assert u.comp_speed > speed_at_hold_start


def test_ramp_up_flag_clears_on_decrement_demand():
    """Task 012: recorded data (2026-06-22 taper) shows the real unit
    follows a step-down sequence back down, so a single decrement-demand
    report (<= ramp_down_set_threshold) clears the ramp-up latch."""
    u = MideaUnit(setpoint=SETPOINT, max_speed=14)
    # Build up some speed first via brief "burst" excursions (2, 3, 1 -- the
    # same shape as control.py's crafted step-up sequence) that don't
    # sustain 3 consecutive cycles above the ramp-up threshold, so the flag
    # doesn't latch here -- mirrors test_ramp_down_flag_ramps_speed_toward_zero.
    for _ in range(5):
        u.step(SETPOINT + 2)
        u.step(SETPOINT + 3)
        u.step(SETPOINT + 1)
    assert u.ramp_up_flag is False
    assert u.comp_speed == 5

    for _ in range(u.ramp_up_debounce_cycles):
        u.step(SETPOINT + 2)
    assert u.ramp_up_flag is True
    speed_before = u.comp_speed

    u.step(SETPOINT - 1)  # decrement-demand report clears the latch
    assert u.ramp_up_flag is False
    assert u.comp_speed <= speed_before

    speed_before2 = u.comp_speed
    for _ in range(5):
        u.step(SETPOINT - 5)
    assert u.comp_speed < speed_before2  # ordinary stepping can reduce speed now


def test_ramp_up_does_not_latch_on_transient_step_sequence_spike():
    """The ordinary step-up sequence (offsets 2, 3, 1 from stable) transiently
    crosses the +2 threshold for 2 cycles; per the debounce assumption this
    must not permanently latch the ramp-up flag, or routine operation would
    always redline to max speed (see Assumption 3 in sim/midea_unit.py)."""
    u = MideaUnit(setpoint=SETPOINT)
    for _ in range(10):
        u.step(SETPOINT + 2)
        u.step(SETPOINT + 3)
        u.step(SETPOINT + 1)
    assert u.ramp_up_flag is False


def test_latched_climb_rate_is_slow():
    """Task 012: the latched ramp-up climb is ~1 increment per 3-6 min
    observed (2026-06-22 taper), i.e. once per `ramp_up_period_cycles`
    intervals, not once per 10 s interval."""
    u = MideaUnit(setpoint=SETPOINT, max_speed=14)
    for _ in range(u.ramp_up_debounce_cycles):
        u.step(SETPOINT + 2)
    assert u.ramp_up_flag is True
    speed_after_latch = u.comp_speed  # near 0: debounce window only nets +1

    for _ in range(2 * u.ramp_up_period_cycles):
        u.step(SETPOINT + 2)  # constant report while latched -> delta 0

    assert u.comp_speed == speed_after_latch + 2


def test_ramp_down_flag_sets_and_clears():
    u = MideaUnit(setpoint=SETPOINT)
    u.step(SETPOINT + 1)  # get speed off the floor first
    u.step(SETPOINT - 1)
    assert u.ramp_down_flag is True

    u.step(SETPOINT + 1)
    assert u.ramp_down_flag is False


def test_ramp_down_flag_ramps_speed_toward_zero_regardless_of_reports():
    u = MideaUnit(setpoint=SETPOINT)
    # Build up speed via repeated brief "burst" excursions (2, 3, 1 -- the
    # same shape as control.py's crafted step-up sequence), each netting +1
    # under sign-capped stepping without sustaining 3 consecutive cycles
    # above the ramp-up threshold, so the flag never latches.
    for _ in range(5):
        u.step(SETPOINT + 2)
        u.step(SETPOINT + 3)
        u.step(SETPOINT + 1)
    assert u.comp_speed == 5
    assert u.ramp_up_flag is False

    u.step(SETPOINT - 1)  # sets ramp-down flag
    assert u.ramp_down_flag is True

    # Even a report suggesting "hold" (no delta) should keep ramping down
    # while the flag is set.
    speed_before = u.comp_speed
    u.step(SETPOINT - 1)
    assert u.comp_speed <= speed_before


def test_purge_after_sustained_low_speed_then_restores():
    u = MideaUnit(
        setpoint=SETPOINT,
        purge_delay_cycles=10,
        purge_duration_cycles=3,
        purge_speed_threshold=2,
    )
    # Hold at min speed (reported_error == prev, delta 0) for the purge delay.
    u.step(SETPOINT)  # comp_speed stays 0 <= threshold
    for _ in range(9):
        u.step(SETPOINT)
    assert u.purging is True
    assert u.comp_speed == u.max_speed

    for _ in range(2):
        u.step(SETPOINT)
        assert u.purging is True

    u.step(SETPOINT)
    assert u.purging is False
    assert u.comp_speed == 0  # restored to pre-purge speed
    assert u._low_speed_counter == 0  # clock restarted


def test_shutdown_after_holding_setpoint_reached_report():
    u = MideaUnit(setpoint=SETPOINT, shutdown_after_cycles=5)
    for _ in range(4):
        u.step(SETPOINT)
        assert u.running is True
    u.step(SETPOINT)
    assert u.running is False
    assert u.comp_speed == 0


def test_shutdown_timer_resets_on_any_report_change_the_blip():
    """Mirrors compress()'s min_power_time 'blip': a periodic nonzero report
    must reset the setpoint-reached shutdown clock so it never fires."""
    u = MideaUnit(setpoint=SETPOINT, shutdown_after_cycles=5)
    for _ in range(100):
        for _ in range(3):
            u.step(SETPOINT)
        u.step(SETPOINT + 1)  # the "blip"
    assert u.running is True


def test_unit_does_not_restart_after_shutdown():
    u = MideaUnit(setpoint=SETPOINT, shutdown_after_cycles=2)
    u.step(SETPOINT)
    u.step(SETPOINT)
    assert u.running is False
    u.step(SETPOINT + 5)  # a big demand jump should not turn it back on
    assert u.running is False
    assert u.comp_speed == 0


def test_reset_returns_to_cold_start_state():
    u = MideaUnit(setpoint=SETPOINT)
    u.step(SETPOINT + 3)
    u.step(SETPOINT + 3)
    u.step(SETPOINT + 3)
    assert u.comp_speed != 0 or u.ramp_up_flag
    u.reset()
    assert u.comp_speed == 0
    assert u.ramp_up_flag is False
    assert u.ramp_down_flag is False
    assert u.running is True


def test_import_is_stdlib_only():
    # A real regression here would only show up in a subprocess without
    # hassapi stubbed; the acceptance criterion covers that. This is just a
    # smoke check that nothing at module scope reaches for hassapi/actrl.
    import sim.midea_unit as m

    assert not hasattr(m, "hassapi")


# ---------------------------------------------------------------------------
# Part 2: closed-loop tracking test
# ---------------------------------------------------------------------------

FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures" / "capacity"

# These fixtures apply external state mutations to the controller (simulated
# defrost detection forcing guesstimated_comp_speed to max; the
# input_boolean.ac_min_power on_counter clamp) that have no follow-me
# equivalent for the emulator to observe -- per the task spec these may be
# (and are) skipped for the tracking assertion.
SKIP_TRACKING_PATTERNS = {"external_defrost_override", "on_counter_clamp"}

TRACKING_FIXTURE_FILES = sorted(p.name for p in FIXTURES_DIR.glob("*.json"))


def load(name):
    with open(FIXTURES_DIR / name) as f:
        return json.load(f)


@pytest.mark.parametrize("filename", TRACKING_FIXTURE_FILES)
def test_closed_loop_tracking(filename):
    cases = load(filename)
    for case in cases:
        if case["pattern"] in SKIP_TRACKING_PATTERNS:
            continue

        controller = MideaCapacityController(log=lambda msg: None)
        unit = MideaUnit(setpoint=SETPOINT)
        checked = 0

        for step in case["steps"]:
            if step["op"] == "compress":
                error = step["error"]
                deriv = step["deriv"]
                on_counter_before = controller.on_counter

                rval = controller.compress(error, deriv)
                unit.step(SETPOINT + rval)

                # Same bookkeeping the harness/main() performs between calls.
                controller.prev_unsigned_compressed_error = rval
                controller.on_counter += 1

                # Skip: soft-start (rule spec explicitly excludes it),
                # faithful mode (error+deriv beyond faithful_threshold hands
                # control to the Midea's own hysteresis, which this emulator
                # doesn't model as a distinct regime), and saturation by the
                # +-2 safety margin (guesstimated_comp_speed > 14).
                in_soft_start = on_counter_before < soft_delay
                in_faithful = (error + deriv) > faithful_threshold
                saturated = controller.guesstimated_comp_speed > compressor_power_increments

                # Skip: controller's "totally off" reset (midea_reset_quirks
                # instantly zeroes guesstimated_comp_speed as a heuristic
                # response to a large negative error; the emulator instead
                # ramps its speed down over several intervals like a real
                # compressor would, so the two intentionally diverge for a
                # few cycles right after the reset -- see immediate_off.json).
                controller_off_reset = controller.compressor_totally_off

                # Skip: the unit's own purge cycle (rule 5) forces comp_speed
                # to max_speed for ~1 min independent of demand; control.py's
                # own comment says this desync against guesstimated_comp_speed
                # is expected ("not a big deal") -- see min_power.json, which
                # is specifically long enough to trigger a purge.
                purging = unit.purging

                if not (
                    in_soft_start
                    or in_faithful
                    or saturated
                    or controller_off_reset
                    or purging
                ):
                    checked += 1
                    assert (
                        abs(unit.comp_speed - controller.guesstimated_comp_speed)
                        <= 2
                    ), (
                        f"{filename}/{case['pattern']}: emulator speed "
                        f"{unit.comp_speed} vs controller estimate "
                        f"{controller.guesstimated_comp_speed} (error={error}, "
                        f"deriv={deriv})"
                    )

            elif step["op"] == "set_attr":
                setattr(controller, step["attr"], step["value"])
            else:
                raise AssertionError(f"unknown op {step['op']!r}")

        # Sanity: every non-skipped fixture should actually exercise the
        # tracking assertion at least once (otherwise the skip conditions
        # above would be silently hiding the whole scenario).
        assert checked > 0, f"{filename}/{case['pattern']}: nothing tracked"
