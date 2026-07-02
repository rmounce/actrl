"""Deterministic whole-cycle scenarios for the headless harness.

Temperature trajectories are simple hardcoded arithmetic, not a thermal
model: goldens only need determinism, not realism.
"""

ROOMS = ["bed_1", "bed_2", "bed_3", "kitchen", "study"]


def base_world(
    climate_state="off",
    setpoint=24.0,
    fan_mode="low",
    static_pressure="2",
    comp_speed="0.0",
    grid_surplus_integral="0.0",
    forecasts=None,
):
    world = {
        "climate.m5atom_climate": {
            "state": climate_state,
            "attributes": {"temperature": setpoint, "fan_mode": fan_mode},
        },
        "number.m5atom_static_pressure": {"state": static_pressure},
        "binary_sensor.m5atom_compressor": {"state": "off"},
        "binary_sensor.m5atom_outdoor_fan": {"state": "off"},
        "input_boolean.ac_manual_mode": {"state": "off"},
        "input_boolean.ac_min_power": {"state": "off"},
        "input_boolean.ac_use_feels_like": {"state": "off"},
        "input_boolean.ac_use_grid_surplus_heat": {"state": "off"},
        "input_boolean.ac_use_grid_surplus_cool": {"state": "off"},
        "input_number.aircon_comp_speed": {"state": comp_speed},
        "input_number.grid_surplus_integral": {"state": grid_surplus_integral},
        "sensor.mpc_p_pv_curtailment": {
            "state": "0",
            "attributes": {"forecasts": forecasts or []},
        },
    }
    for room in ROOMS:
        world[f"cover.{room}"] = {"state": "closed", "attributes": {"current_position": 0.0}}
        world[f"binary_sensor.{room}_window"] = {"state": "off"}
        world[f"binary_sensor.{room}_door"] = {"state": "on"}
    return world


def room_climates(world, mode, targets, temps):
    for room in ROOMS:
        attrs = {}
        if mode == "heat_cool":
            attrs["target_temp_low"] = targets[room][0]
            attrs["target_temp_high"] = targets[room][1]
        else:
            attrs["temperature"] = targets[room]
        world[f"climate.{room}_aircon"] = {"state": mode, "attributes": attrs}
        world[f"sensor.{room}_average_temperature"] = {"state": str(temps[room])}
    return world


def temp_updates(cycles, start_temps, rate_per_cycle):
    """Linear temperature trajectories: {cycle: {sensor: {"state": ...}}}."""
    updates = {}
    for cycle in range(cycles):
        updates[cycle] = {
            f"sensor.{room}_average_temperature": {
                "state": str(round(start + rate_per_cycle * cycle, 4))
            }
            for room, start in start_temps.items()
        }
    return updates


def heat_approach():
    """Cold rooms, heat mode: on-blip, soft start, stepping, approach, overshoot."""
    targets = {"bed_1": 21.0, "bed_2": 20.5, "bed_3": 21.0, "kitchen": 21.5, "study": 20.0}
    start = {room: t - 1.0 for room, t in targets.items()}
    cycles = 120
    world = base_world()
    room_climates(world, "heat", targets, start)
    return {
        "name": "heat_approach",
        "initial": world,
        "cycles": cycles,
        "updates": temp_updates(cycles, start, 0.015),
    }


def stays_off():
    """Rooms already warmer than heat targets: system must stay off."""
    targets = {"bed_1": 21.0, "bed_2": 20.5, "bed_3": 21.0, "kitchen": 21.5, "study": 20.0}
    start = {room: t + 0.5 for room, t in targets.items()}
    world = base_world()
    room_climates(world, "heat", targets, start)
    return {"name": "stays_off", "initial": world, "cycles": 15, "updates": {}}


def cool_grid_surplus():
    """heat_cool rooms, hot house, PV curtailment feeding the surplus integral."""
    targets = {room: (19.0, 25.0) for room in ROOMS}
    start = {room: 25.8 for room in ROOMS}
    cycles = 90
    forecasts = [{"mpc_p_pv_curtailment": 2500.0} for _ in range(24)]
    world = base_world(forecasts=forecasts)
    world["input_boolean.ac_use_grid_surplus_cool"]["state"] = "on"
    room_climates(world, "heat_cool", targets, start)
    return {
        "name": "cool_grid_surplus",
        "initial": world,
        "cycles": cycles,
        "updates": temp_updates(cycles, start, -0.01),
    }


def window_open_heat():
    """Heating with one window opening mid-run and a closed door elsewhere."""
    targets = {"bed_1": 21.0, "bed_2": 20.5, "bed_3": 21.0, "kitchen": 21.5, "study": 20.0}
    start = {room: t - 0.8 for room, t in targets.items()}
    cycles = 60
    world = base_world()
    world["binary_sensor.bed_2_door"]["state"] = "off"
    room_climates(world, "heat", targets, start)
    updates = temp_updates(cycles, start, 0.01)
    updates[20]["binary_sensor.bed_1_window"] = {"state": "on"}
    updates[45]["binary_sensor.bed_1_window"] = {"state": "off"}
    return {
        "name": "window_open_heat",
        "initial": world,
        "cycles": cycles,
        "updates": updates,
    }


def defrost_heat():
    """Already-running heat with a defrost signature appearing mid-run."""
    targets = {"bed_1": 21.0, "bed_2": 20.5, "bed_3": 21.0, "kitchen": 21.5, "study": 20.0}
    start = {room: t - 1.5 for room, t in targets.items()}
    cycles = 45
    world = base_world(climate_state="heat", comp_speed="5.0")
    world["binary_sensor.m5atom_compressor"]["state"] = "on"
    world["binary_sensor.m5atom_outdoor_fan"]["state"] = "on"
    room_climates(world, "heat", targets, start)
    updates = temp_updates(cycles, start, 0.005)
    # outdoor fan stops while compressor runs -> defrost detected
    updates[15]["binary_sensor.m5atom_outdoor_fan"] = {"state": "off"}
    updates[15]["binary_sensor.m5atom_compressor"] = {"state": "on"}
    updates[25]["binary_sensor.m5atom_outdoor_fan"] = {"state": "on"}
    return {
        "name": "defrost_heat",
        "initial": world,
        "cycles": cycles,
        "updates": updates,
    }


def manual_mode():
    """Manual mode on: main() must reset and do nothing each cycle."""
    targets = {"bed_1": 21.0, "bed_2": 20.5, "bed_3": 21.0, "kitchen": 21.5, "study": 20.0}
    start = {room: t - 1.0 for room, t in targets.items()}
    world = base_world()
    world["input_boolean.ac_manual_mode"]["state"] = "on"
    room_climates(world, "heat", targets, start)
    return {"name": "manual_mode", "initial": world, "cycles": 5, "updates": {}}


def cool_overshoot_off():
    """Cooling that overshoots hard: ramp down, min power, off with fan run-on."""
    targets = {room: 24.5 for room in ROOMS}
    start = {room: 25.5 for room in ROOMS}
    cycles = 130
    world = base_world()
    room_climates(world, "cool", targets, start)
    return {
        "name": "cool_overshoot_off",
        "initial": world,
        "cycles": cycles,
        "updates": temp_updates(cycles, start, -0.03),
    }


def mode_switch_heat_to_cool():
    """heat_cool rooms swinging from too-cold to too-hot: heat -> off -> cool."""
    targets = {room: (19.0, 25.0) for room in ROOMS}
    start = {room: 18.5 for room in ROOMS}
    cycles = 170
    world = base_world()
    room_climates(world, "heat_cool", targets, start)
    return {
        "name": "mode_switch_heat_to_cool",
        "initial": world,
        "cycles": cycles,
        "updates": temp_updates(cycles, start, 0.05),
    }


def feels_like_fallback():
    """ac_use_feels_like on; some rooms lack the sensor and must fall back."""
    targets = {"bed_1": 21.0, "bed_2": 20.5, "bed_3": 21.0, "kitchen": 21.5, "study": 20.0}
    start = {room: t - 1.0 for room, t in targets.items()}
    cycles = 40
    world = base_world()
    world["input_boolean.ac_use_feels_like"]["state"] = "on"
    room_climates(world, "heat", targets, start)
    # feels-like present (and warmer) for two rooms only; others fall back
    world["sensor.bed_1_feels_like"] = {"state": str(start["bed_1"] + 0.3)}
    world["sensor.kitchen_feels_like"] = {"state": str(start["kitchen"] + 0.3)}
    updates = temp_updates(cycles, start, 0.01)
    for cycle in range(cycles):
        updates[cycle]["sensor.bed_1_feels_like"] = {
            "state": str(round(start["bed_1"] + 0.3 + 0.01 * cycle, 4))
        }
        updates[cycle]["sensor.kitchen_feels_like"] = {
            "state": str(round(start["kitchen"] + 0.3 + 0.01 * cycle, 4))
        }
    return {
        "name": "feels_like_fallback",
        "initial": world,
        "cycles": cycles,
        "updates": updates,
    }


def min_power_hold():
    """ac_min_power switched on mid-run: on_counter clamped below soft_delay."""
    targets = {"bed_1": 21.0, "bed_2": 20.5, "bed_3": 21.0, "kitchen": 21.5, "study": 20.0}
    start = {room: t - 1.2 for room, t in targets.items()}
    cycles = 80
    world = base_world()
    room_climates(world, "heat", targets, start)
    updates = temp_updates(cycles, start, 0.008)
    updates[30]["input_boolean.ac_min_power"] = {"state": "on"}
    updates[60]["input_boolean.ac_min_power"] = {"state": "off"}
    return {
        "name": "min_power_hold",
        "initial": world,
        "cycles": cycles,
        "updates": updates,
    }


ALL_SCENARIOS = [
    heat_approach,
    stays_off,
    cool_grid_surplus,
    window_open_heat,
    defrost_heat,
    manual_mode,
    cool_overshoot_off,
    mode_switch_heat_to_cool,
    feels_like_fallback,
    min_power_hold,
]
