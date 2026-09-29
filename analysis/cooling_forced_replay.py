"""First cooling calibration: replay recorded HVAC power and dampers.

The controller and Midea emulator are bypassed. Each window starts from
measured room temperatures; the existing house model receives the recorded
outdoor/indoor electrical power, actual damper positions, outdoor weather,
and the existing orientation solar terms. A single scalar maps electrical
kW to house-average cooling K/h. Reported "COP proxy" is scalar * the
winter effective house capacity (4.8 kWh/K), not a measured equipment COP.

This is a diagnostic for September 2026's first seven cooling windows. It does
not overwrite sim parameters. Its initial cooling defaults are documented
in docs/calibration.md; room solar gains, sensor lead, and airflow
split are not independently identified from these few windows.

Run from the repo root after exporting/loading September history::

    ./.venv/bin/python analysis/cooling_forced_replay.py \
        --parquet data/processed/sep23_28.parquet \
        --parquet data/processed/sep28_29.parquet

Add future windows with `--window NAME START END RUN_START` and select
fitting windows with repeated `--train NAME`; omitted options use the
September starter cohort and its 28th/29th fitting split.

Dev-only: pandas/numpy. No deployed app imports this file.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
for path in (ROOT, ROOT / "tests"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from sim.closed_loop import AIRFLOW_WEIGHTS, MASS_WEIGHTS  # noqa: E402
from sim.house import House, HouseParams, ROOMS  # noqa: E402
from sim.hvac import DeadTimeLag, Hvac  # noqa: E402
from sim.solar import AZ_NE, AZ_NW, vertical_irradiance  # noqa: E402

LOCAL_TZ = "Australia/Adelaide"
WINDOWS = {
    "24-evening": ("2026-09-24 21:45", "2026-09-24 23:15", "2026-09-24 22:09"),
    "27-afternoon": ("2026-09-27 13:15", "2026-09-27 17:20", "2026-09-27 13:44"),
    "28-afternoon": ("2026-09-28 13:15", "2026-09-28 16:20", "2026-09-28 14:00"),
    "29-afternoon": ("2026-09-29 10:45", "2026-09-29 16:40", "2026-09-29 11:25"),
    "29-evening": ("2026-09-29 20:00", "2026-09-29 23:15", "2026-09-29 20:40"),
    "29-late": ("2026-09-29 23:15", "2026-09-30 00:40", "2026-09-29 23:40"),
    "30-night": ("2026-09-30 00:55", "2026-09-30 02:25", "2026-09-30 01:27"),
}
TRAIN = ("28-afternoon", "29-afternoon")
GRID = np.arange(0.1, 1.51, 0.05)  # house-average K/h per electrical kW


def load_archives(paths: list[Path]) -> pd.DataFrame:
    data = pd.concat([pd.read_parquet(path) for path in paths]).sort_index()
    data = data[~data.index.duplicated(keep="last")]
    data.index = data.index.tz_convert(LOCAL_TZ)
    # HA recorder queried 2026-09-30 confirms these covers were closed at
    # the archive's head (2026-09-23 UTC). Numeric exports omit unchanged
    # states and the September seed archive has a gap. Seed only this
    # confirmed cohort; never interpret arbitrary missing dampers as shut.
    seed = pd.Timestamp("2026-09-23", tz="UTC").tz_convert(LOCAL_TZ)
    if seed in data.index:
        for room in ("bed_2", "bed_3", "study"):
            column = f"damper.{room}"
            if pd.isna(data.loc[seed, column]):
                data.loc[seed, column] = 0.0
            data[column] = data[column].ffill()
    kitchen_seed = pd.Timestamp("2026-09-24 21:45", tz=LOCAL_TZ)
    if kitchen_seed in data.index and pd.isna(data.loc[kitchen_seed, "damper.kitchen"]):
        # HA recorder: kitchen still 100% at 21:45, closed 22:09:23.
        data.loc[kitchen_seed, "damper.kitchen"] = 100.0
        data["damper.kitchen"] = data["damper.kitchen"].ffill()
    # Same cloudiness proxy as analysis/replay_day.py: daily PV power
    # divided by the archive's minute-of-day envelope. PV is forecast, not
    # measured irradiance; solar sensitivity is reported separately.
    pv = data["power_pv_5m"].clip(lower=0).fillna(0)
    minute = data.index.hour * 60 + data.index.minute
    envelope = pv.groupby(minute).transform("max").replace(0, np.nan)
    data["_cloudiness"] = (pv / envelope).clip(0, 1).fillna(0)
    return data


def window_arrays(data: pd.DataFrame, name: str, spec: tuple[str, str, str]) -> tuple:
    start, end, run_start = spec
    window = data.loc[start:end]
    if len(window) < 2 or not window.index.to_series().diff().iloc[1:].eq(pd.Timedelta("1min")).all():
        raise ValueError(f"{name}: missing one-minute archive data")
    observed = np.array(
        [window[f"{room}_average_temperature"].to_numpy(float) for room in ROOMS]
    ).T
    if not np.isfinite(observed).all():
        raise ValueError(f"{name}: room temperature gap")
    dampers = np.array(
        [window[f"damper.{room}"].to_numpy(float) / 100 for room in ROOMS]
    ).T
    outdoor_w = window["power.outdoor_unit"].clip(lower=0).to_numpy(float)
    indoor_w = window["power.indoor_unit"].clip(lower=0).to_numpy(float)
    if not all(np.isfinite(values).all() for values in (dampers, outdoor_w, indoor_w)):
        raise ValueError(f"{name}: missing recorded damper or power input")
    power_kw = np.where(outdoor_w > 100, (outdoor_w + indoor_w) / 1000, 0)
    t_out = window["temperature_adelaide"].ffill().bfill().to_numpy(float)
    cloudiness = window["_cloudiness"].to_numpy(float)
    sun_ne = np.array(
        [vertical_irradiance(ts, AZ_NE) * c for ts, c in zip(window.index, cloudiness)]
    )
    sun_nw = np.array(
        [vertical_irradiance(ts, AZ_NW) * c for ts, c in zip(window.index, cloudiness)]
    )
    score_mask = window.index >= pd.Timestamp(run_start, tz=LOCAL_TZ)
    # Fed rooms get more weight; closed rooms retain a small weight to
    # expose envelope/solar drift in the reported score.
    active_weight = np.maximum(0.1, dampers[power_kw > 0.2].mean(axis=0))
    return window, observed, dampers, power_kw, t_out, sun_ne, sun_nw, score_mask, active_weight


def replay(
    inputs: tuple, cooling_scale: float | None, solar_scale: float = 1.0,
    *, legacy_sensor: bool = False,
) -> np.ndarray:
    window, observed, dampers, power_kw, t_out, sun_ne, sun_nw, _, _ = inputs
    params = HouseParams()
    if legacy_sensor:
        # For negative forcing, reversing the magnitude coefficient exactly
        # reproduces the former signed-q kitchen sensor adjustment.
        params = params.replace_room("kitchen", lead_q_h=-params.rooms["kitchen"].lead_q_h)
    house = House(params, dict(zip(ROOMS, observed[0])), dt_s=10)
    hvac = Hvac()
    delivered_lag = DeadTimeLag(15, 180, cycle_s=10)
    predicted = np.empty_like(observed)
    predicted[0] = observed[0]
    masses = np.array([MASS_WEIGHTS[room] for room in ROOMS])
    airflow = np.array([AIRFLOW_WEIGHTS[room] for room in ROOMS])
    mass_fraction = masses / masses.sum()
    for i in range(1, len(window)):
        flows = dampers[i] * airflow
        shares = flows / flows.sum() if flows.sum() > 0 else np.zeros(len(ROOMS))
        room_scale = shares / mass_fraction
        efficiency = (
            hvac.efficiency(power_kw[i], t_out[i])
            if cooling_scale is None else cooling_scale
        )
        q_target = -efficiency * power_kw[i]
        for _ in range(6):  # 10-second house/control cadence
            if power_kw[i] == 0:
                delivered_lag.reset()  # same shutdown handling as ClosedLoop
            q_house = delivered_lag.step(q_target)
            house.step(
                t_out[i],
                dict(zip(ROOMS, q_house * room_scale)),
                dt_s=10,
                sun_ne=solar_scale * sun_ne[i],
                sun_nw=solar_scale * sun_nw[i],
            )
        predicted[i] = [house.temps_measured[room] for room in ROOMS]
    return predicted


def score(inputs: tuple, predicted: np.ndarray) -> tuple[float, np.ndarray]:
    _, observed, _, _, _, _, _, mask, weights = inputs
    error = predicted[mask] - observed[mask]
    rmse = np.sqrt(np.mean(error**2, axis=0))
    weighted_rmse = np.sqrt(np.mean(error**2 * weights) / np.mean(weights))
    return float(weighted_rmse), rmse


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--parquet", type=Path, action="append", required=True)
    parser.add_argument("--window", nargs=4, action="append", metavar=("NAME", "START", "END", "RUN_START"))
    parser.add_argument("--train", action="append", help="window name used to fit the shared coefficient")
    args = parser.parse_args()
    data = load_archives(args.parquet)
    specs = dict(WINDOWS)
    for name, start, end, run_start in args.window or []:
        specs[name] = (start, end, run_start)
    train = tuple(args.train or TRAIN)
    if any(name not in specs for name in train):
        parser.error("every --train name must match a window")
    windows = {name: window_arrays(data, name, spec) for name, spec in specs.items()}
    for solar_scale in (0.0, 1.0):
        scores = {
            (name, float(scale)): score(inputs, replay(inputs, scale, solar_scale))[0]
            for name, inputs in windows.items()
            for scale in GRID
        }
        print(f"solar scale {solar_scale:.0f}")
        for name in windows:
            best = min(GRID, key=lambda s: scores[name, float(s)])
            print(f"  {name:15s} best e={best:.2f} score={scores[name, float(best)]:.2f}°C")
        shared = min(
            GRID, key=lambda s: sum(scores[name, float(s)] ** 2 for name in train)
        )
        print(f"  shared train e={shared:.2f} (effective COP proxy {shared * 4.8:.2f})")
        for name in windows:
            legacy = score(windows[name], replay(windows[name], None, solar_scale, legacy_sensor=True))[0]
            print(f"    {name:15s} legacy={legacy:.2f}°C candidate={scores[name, float(shared)]:.2f}°C")
    electrical_fit(data, specs)


def electrical_fit(data: pd.DataFrame, specs: dict) -> None:
    """Minimum-power fit on old runs; independent overnight validation."""
    samples = {}
    for name, (start, end, _) in specs.items():
        window = data.loc[start:end]
        stable = window["aircon_comp_speed"].rolling(7).max() < 0.2
        power = window["power.outdoor_unit"]
        samples[name] = window[stable & power.between(150, 600)]
    fit_names = ("24-evening", "27-afternoon", "28-afternoon", "29-afternoon")
    train = pd.concat([samples[name] for name in fit_names])
    features = np.column_stack([np.ones(len(train)), train["temperature_adelaide"] - 20])
    target = train["power.outdoor_unit"].to_numpy()
    coefficient = np.linalg.lstsq(features, target, rcond=None)[0]
    print(f"minimum electrical fit n={len(train)}: {coefficient[0]:.1f} W at 20°C, {coefficient[1]:.1f} W/K")
    hvac = Hvac()
    for name in ("29-evening", "29-late", "30-night"):
        window = samples[name]
        predicted = np.array([
            hvac.power_kw(0, "cool", temperature) * 1000
            - hvac.params.cool_p_indoor_min_w
            for temperature in window["temperature_adelaide"]
        ])
        actual = window["power.outdoor_unit"].to_numpy()
        candidate_error = np.sqrt(np.mean((actual - predicted) ** 2))
        legacy_error = np.sqrt(np.mean((actual - hvac.params.p_outdoor_min_w) ** 2))
        print(f"  {name:15s} n={len(window)} outdoor RMSE legacy={legacy_error:.0f} W candidate={candidate_error:.0f} W")


if __name__ == "__main__":
    main()
