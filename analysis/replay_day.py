"""Whole-day closed-loop replay validation (docs/ideas.md #3 "Validation").

Replays a recorded local day through sim.closed_loop.ClosedLoop: the real
actrl logic runs against the simulated house/unit while the *exogenous*
inputs are driven from the archive — outdoor temperature (BOM, the sysid
input), per-room statctrl targets, and the unit setpoint. Room temperatures
and HVAC behaviour are then fully simulated (nothing is nudged back toward
the record), so divergence measures the whole model stack end to end.

Known structural gaps when reading the results (docs/calibration.md):
solar/internal gains are not modelled (the RC fit is night-only), so
daytime room temps should read low; defrost is not modelled, so simulated
energy should read low on cold mornings by roughly the defrost overhead.

Usage:
    uv run python analysis/replay_day.py --date 2026-06-22 [--out sim.csv]

Prints per-room temperature RMSE/bias (whole day and night-only) and
simulated vs recorded HVAC energy.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from zoneinfo import ZoneInfo

import numpy as np
import pandas as pd

_ROOT = Path(__file__).resolve().parent.parent
for p in (str(_ROOT), str(_ROOT / "tests")):
    if p not in sys.path:
        sys.path.insert(0, p)

from scenarios import base_world, room_climates  # noqa: E402
from sim.closed_loop import ClosedLoop, ROOMS  # noqa: E402
from sim.solar import AZ_NE, AZ_NW, vertical_irradiance  # noqa: E402

LOCAL_TZ = ZoneInfo("Australia/Adelaide")
CYCLES_PER_MIN = 6  # 10 s control cycles


def load_day(parquet: Path, date: str) -> pd.DataFrame:
    df = pd.read_parquet(parquet)
    if df.index.tz is None:
        df.index = df.index.tz_localize("UTC")
    # Cloudiness (recorded PV / per-minute-of-day PV clear-sky envelope,
    # analysis/solar_orient_fit.cloudiness) must be computed from the FULL
    # parquet before the day is sliced out below -- the envelope is a max
    # over the whole archive's minute-of-day buckets, so slicing first would
    # make a single day's own (possibly cloudy) minute its own ceiling.
    pv = df["power_pv_5m"].fillna(0).clip(lower=0)
    mod = df.index.tz_convert("UTC").hour * 60 + df.index.tz_convert("UTC").minute
    env = pv.groupby(mod).transform("max").replace(0, np.nan)
    df = df.copy()
    df["_cloudiness"] = (pv / env).clip(0, 1).fillna(0)

    start = pd.Timestamp(date, tz=LOCAL_TZ)
    end = start + pd.Timedelta("1D")
    day = df[(df.index >= start) & (df.index < end)]
    if len(day) < 24 * 60:
        raise SystemExit(f"incomplete day: {len(day)} minutes in archive")
    return day


def build_loop(day: pd.DataFrame) -> ClosedLoop:
    first = day.iloc[0]
    setpoint = float(first["climate.m5atom_climate.temperature"])
    targets = {
        r: (
            float(first[f"climate.{r}_aircon.target_temp_low"]),
            float(first[f"climate.{r}_aircon.target_temp_high"]),
        )
        for r in ROOMS
    }
    temps = {r: float(first[f"{r}_average_temperature"]) for r in ROOMS}
    world = base_world(setpoint=setpoint)
    room_climates(world, "heat_cool", targets, temps)
    return ClosedLoop(world, temps, setpoint=setpoint)


def feels_like_offsets(day: pd.DataFrame) -> dict[str, np.ndarray]:
    """Per-room (feels_like - average_temperature) offset per minute.

    Production actrl actuates on the apparent temperature
    (packages/aircon.yaml): feels_like = T + 0.33*wvp - 4.0, with
    wvp = RH/100 * 6.105 * exp(17.27*T/(237.7+T)). The offset is computed
    from the *recorded* temp+humidity — vapour pressure is treated as
    exogenous (the sim doesn't model moisture), so the sim's controller
    sees T_sim + offset. Typically ~ -0.4..-0.5 K at winter indoor RH."""
    out = {}
    for r in ROOMS:
        t = day[f"{r}_average_temperature"].astype(float).ffill()
        rh = day[f"{r}_average_humidity"].astype(float).ffill()
        wvp = rh / 100.0 * 6.105 * np.exp(17.27 * t / (237.7 + t))
        out[r] = (0.33 * wvp - 4.0).fillna(0.0).values
    return out


def replay(day: pd.DataFrame, ctrl_noise: dict | None = None) -> pd.DataFrame:
    """Replay one loaded day. `ctrl_noise` optionally adds synthetic sensor
    noise to what the controller reads (and ONLY the controller -- the
    plant and the recorded comparisons are untouched, same channel as the
    feels-like offsets): dict(sigma=K, tau_s=seconds, seed=int). Per-room
    independent AR(1) at the 10 s cycle cadence; tau_s <= 0 gives white
    noise per cycle. Measured floor (docs/tuning.md 2026-07-05): ~0.006 K
    white at 1-min cadence; the sub-minute band is unobserved in the
    archive, so stress-test above the floor rather than trusting it."""
    loop = build_loop(day)
    noise_state = {r: 0.0 for r in ROOMS}
    if ctrl_noise:
        rng = np.random.default_rng(ctrl_noise.get("seed", 0))
        n_sigma = float(ctrl_noise["sigma"])
        tau_s = float(ctrl_noise.get("tau_s", 0.0))
        n_phi = np.exp(-10.0 / tau_s) if tau_s > 0 else 0.0
        n_scale = n_sigma * np.sqrt(1.0 - n_phi**2)
    tout = day["temperature_adelaide"].ffill().values
    pv = (day["power_pv_5m"].fillna(0).clip(lower=0) / 1000.0).values
    cloudiness = day["_cloudiness"].values
    sun_ne = np.array(
        [vertical_irradiance(ts, AZ_NE) * c for ts, c in zip(day.index, cloudiness)]
    )
    sun_nw = np.array(
        [vertical_irradiance(ts, AZ_NW) * c for ts, c in zip(day.index, cloudiness)]
    )
    offsets = feels_like_offsets(day)
    sp = day["climate.m5atom_climate.temperature"].ffill().values
    lows = {r: day[f"climate.{r}_aircon.target_temp_low"].ffill().values for r in ROOMS}
    highs = {r: day[f"climate.{r}_aircon.target_temp_high"].ffill().values for r in ROOMS}

    rows = []
    for i in range(len(day) * CYCLES_PER_MIN):
        m = i // CYCLES_PER_MIN
        updates = {
            "climate.m5atom_climate": {"attributes": {"temperature": float(sp[m])}},
            **{
                f"climate.{r}_aircon": {
                    "attributes": {
                        "target_temp_low": float(lows[r][m]),
                        "target_temp_high": float(highs[r][m]),
                    }
                }
                for r in ROOMS
            },
        }
        if ctrl_noise:
            for r in ROOMS:
                noise_state[r] = n_phi * noise_state[r] + n_scale * rng.standard_normal()
        rows.append(
            loop.step(
                t_out=float(tout[m]),
                updates=updates,
                pv_kw=float(pv[m]),
                sun_ne=float(sun_ne[m]),
                sun_nw=float(sun_nw[m]),
                ctrl_offsets={r: float(offsets[r][m]) + noise_state[r] for r in ROOMS},
            )
        )
    loop.close()

    sim = pd.DataFrame(rows[CYCLES_PER_MIN - 1 :: CYCLES_PER_MIN], index=day.index)
    return sim


def report(day: pd.DataFrame, sim: pd.DataFrame) -> None:
    local_hour = day.index.tz_convert(LOCAL_TZ).hour
    night = (local_hour >= 21) | (local_hour < 8)

    # Recorded data *is* the measured signal (the room sensors are the
    # measured-air lead node, sim/house.py docs/tasks/009), so compare
    # against Tm_{room} rather than the bulk T_{room} -- equal when the
    # node is disabled (the current default params).
    print(f"{'room':<10} {'RMSE':>6} {'bias':>6}   {'night RMSE':>10} {'night bias':>10}")
    for r in ROOMS:
        rec = day[f"{r}_average_temperature"].astype(float)
        err = sim[f"Tm_{r}"] - rec
        print(
            f"{r:<10} {np.sqrt((err**2).mean()):6.2f} {err.mean():+6.2f}   "
            f"{np.sqrt((err[night]**2).mean()):10.2f} {err[night].mean():+10.2f}"
        )

    sim_kwh = sim["p_kw"].sum() / 60.0  # 1-min rows of kW
    rec_w = day["power.outdoor_unit"].clip(lower=0) + day["power.indoor_unit"].clip(lower=0)
    rec_kwh = rec_w.sum() / 60.0 / 1000.0
    print(f"\nHVAC energy: sim {sim_kwh:.2f} kWh vs recorded {rec_kwh:.2f} kWh "
          f"({(sim_kwh - rec_kwh) / rec_kwh:+.0%})")

    # Min-power outdoor draw is ~610 W; >300 W means "unit running" and
    # >800 W means "stepped above minimum".
    sim_on = (sim["p_kw"] > 0).mean()
    rec_on = (day["power.outdoor_unit"] > 300).mean()
    sim_hi = (sim["increment"] > 0).mean()
    rec_hi = (day["power.outdoor_unit"] > 800).mean()
    print(f"Unit-on fraction:    sim {sim_on:.0%} vs recorded {rec_on:.0%}")
    print(f"Above-min fraction:  sim {sim_hi:.0%} vs recorded {rec_hi:.0%}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--date", required=True, help="local (Adelaide) date, e.g. 2026-06-22")
    ap.add_argument("--parquet", default="data/processed/june.parquet", type=Path)
    ap.add_argument("--out", type=Path, help="write simulated 1-min telemetry CSV")
    args = ap.parse_args()

    day = load_day(args.parquet, args.date)
    sim = replay(day)
    report(day, sim)
    if args.out:
        sim.to_csv(args.out)
        print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
