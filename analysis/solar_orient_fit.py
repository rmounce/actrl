"""Orientation-resolved per-room solar fit (trajectory space).

Why: the single all-day PV proxy cannot represent window-orientation
timing. Hourly replay-bias profiles (2026-07-03) show bed_2/bed_3 running
3-4.6 K cold at 09:00-12:00 on sunny days — direct morning sun through
their NE windows — while the whole-house PV proxy (roughly north array)
peaks at noon: regressed against it, bed_2's morning spike washed out to
a tiny slope with r2=0.02 instead of showing up as the dominant gain it
is.

Model: two clear-sky irradiance basis functions per room, direct sun on a
vertical NE face (morning) and a NW face (afternoon) — the house grid per
the NatHERS report is NW/NE/SW/SE, and Adelaide's winter sun tracks
sunrise ~61 deg (ENE) -> north -> sunset ~299 deg (WNW), so SE/SW glazing
receives no direct winter beam (bed_1's large SW glazing included). Each
basis is scaled by a cloudiness index (recorded PV / clear-sky envelope):

    solar_forcing_r(t) [K/h] = s_ne_r * I_NE(t) * c(t) + s_nw_r * I_NW(t) * c(t)

I_az(t) = max(0, cos(elevation) * cos(sun_azimuth - az)) for a vertical
surface, 0 when the sun is below ~3 deg elevation. c(t) is recorded PV
divided by the June per-minute-of-day PV envelope (clear-sky proxy),
clipped to [0, 1].

Fit: same open-loop trajectory method as analysis/solar_refit_openloop.py
— simulate all five rooms jointly over head-trimmed unit-off windows,
re-anchored every 30 min; regress each room's recorded-minus-simulated
delta on the two basis functions (through origin, jointly); iterate to a
fixed point. Coefficients are clipped at 0 (a window cannot un-shine).

Usage:
    uv run python analysis/solar_orient_fit.py [--parquet ...] [--iters 4]
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pandas as pd

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from sim.house import House, HouseParams, ROOMS  # noqa: E402

LAT = -34.93  # Adelaide
LON = 138.60
OFF_W = 50
MIN_WINDOW_MIN = 120
TRIM_HEAD_MIN = 45
STEP_MIN = 30
DT_S = 60.0
AZ_NE = 45.0
AZ_NW = 315.0  # winter sun sets ~299 deg (WNW); SW gets no direct winter beam
MIN_ELEV_DEG = 3.0


def sun_position(index: pd.DatetimeIndex) -> tuple[np.ndarray, np.ndarray]:
    """(elevation_rad, azimuth_rad from north, clockwise) — NOAA-style
    approximation, adequate for irradiance weighting."""
    utc = index.tz_convert("UTC")
    doy = utc.dayofyear.to_numpy()
    frac_h = (utc.hour + utc.minute / 60.0).to_numpy()
    gamma = 2 * np.pi / 365.0 * (doy - 1 + (frac_h - 12) / 24.0)
    decl = (
        0.006918 - 0.399912 * np.cos(gamma) + 0.070257 * np.sin(gamma)
        - 0.006758 * np.cos(2 * gamma) + 0.000907 * np.sin(2 * gamma)
        - 0.002697 * np.cos(3 * gamma) + 0.00148 * np.sin(3 * gamma)
    )
    eqtime = 229.18 * (
        0.000075 + 0.001868 * np.cos(gamma) - 0.032077 * np.sin(gamma)
        - 0.014615 * np.cos(2 * gamma) - 0.040849 * np.sin(2 * gamma)
    )
    tst = frac_h * 60.0 + eqtime + 4.0 * LON  # true solar time [min]
    ha = np.deg2rad(tst / 4.0 - 180.0)  # hour angle
    lat = np.deg2rad(LAT)
    sin_elev = np.sin(lat) * np.sin(decl) + np.cos(lat) * np.cos(decl) * np.cos(ha)
    elev = np.arcsin(np.clip(sin_elev, -1, 1))
    cos_az = (np.sin(decl) - np.sin(lat) * sin_elev) / (np.cos(lat) * np.cos(elev))
    az = np.arccos(np.clip(cos_az, -1, 1))
    az = np.where(ha > 0, 2 * np.pi - az, az)  # afternoon: west of north
    return elev, az


def vertical_irradiance(index: pd.DatetimeIndex, az_deg: float) -> pd.Series:
    """Clear-sky direct-beam factor on a vertical surface facing az_deg."""
    elev, az = sun_position(index)
    face = np.deg2rad(az_deg)
    fac = np.cos(elev) * np.cos(az - face)
    fac = np.where(np.rad2deg(elev) < MIN_ELEV_DEG, 0.0, np.clip(fac, 0.0, None))
    return pd.Series(fac, index=index)


def cloudiness(df: pd.DataFrame) -> pd.Series:
    """Recorded PV / June per-minute-of-day PV envelope, clipped [0,1]."""
    pv = df["power_pv_5m"].fillna(0).clip(lower=0)
    mod = df.index.tz_convert("UTC").hour * 60 + df.index.tz_convert("UTC").minute
    env = pv.groupby(mod).transform("max").replace(0, np.nan)
    return (pv / env).clip(0, 1).fillna(0)


def off_windows(df: pd.DataFrame) -> list[tuple[pd.Timestamp, pd.Timestamp]]:
    off = df["power.outdoor_unit"].fillna(np.inf) < OFF_W
    grp = (off != off.shift()).cumsum()
    head = pd.Timedelta(minutes=TRIM_HEAD_MIN)
    out = []
    for _, seg in df.index.to_series().groupby(grp):
        if not off.loc[seg.iloc[0]]:
            continue
        w0, w1 = seg.iloc[0] + head, seg.iloc[-1]
        if (w1 - w0) >= pd.Timedelta(minutes=MIN_WINDOW_MIN):
            out.append((w0, w1))
    return out


def simulate_window(params: HouseParams, day: pd.DataFrame, forcing: dict,
                    w0: pd.Timestamp, w1: pd.Timestamp) -> pd.DataFrame:
    """Open-loop q=0 sim, re-anchored every STEP_MIN; solar applied as an
    external per-room q so the fit can iterate without HouseParams.solar
    (which is held at 0 here)."""
    win = day.loc[w0:w1]
    tout = win["temperature_adelaide"].ffill()
    rows = []
    for s0 in win.index[::STEP_MIN]:
        s1 = s0 + pd.Timedelta(minutes=STEP_MIN)
        if s1 > w1:
            break
        try:
            temps0 = {r: float(win.loc[s0, f"{r}_average_temperature"]) for r in ROOMS}
            temps1 = {r: float(win.loc[s1, f"{r}_average_temperature"]) for r in ROOMS}
        except KeyError:
            continue
        if any(np.isnan(v) for v in list(temps0.values()) + list(temps1.values())):
            continue
        house = House(params, temps0)
        seg = win.loc[s0:s1]
        for ts in seg.index[1:]:
            q = {r: forcing[r].loc[ts] for r in ROOMS}
            house.step(float(tout.loc[ts]), q, dt_s=DT_S, pv_kw=0.0)
        rows.append({
            "b_ne": float(forcing["_b_ne"].loc[s0:s1].mean()),
            "b_nw": float(forcing["_b_nw"].loc[s0:s1].mean()),
            **{f"err_{r}": temps1[r] - house.temps[r] for r in ROOMS},
        })
    return pd.DataFrame(rows)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--parquet", default="data/processed/june.parquet", type=Path)
    ap.add_argument("--iters", default=4, type=int)
    args = ap.parse_args()
    df = pd.read_parquet(args.parquet)
    if df.index.tz is None:
        df.index = df.index.tz_localize("UTC")

    c = cloudiness(df)
    b_ne = vertical_irradiance(df.index, AZ_NE) * c
    b_nw = vertical_irradiance(df.index, AZ_NW) * c
    windows = off_windows(df)
    print(f"{len(windows)} unit-off windows; basis peaks: "
          f"NE {b_ne.groupby(b_ne.index.tz_convert('Australia/Adelaide').hour).mean().idxmax()}h, "
          f"NW {b_nw.groupby(b_nw.index.tz_convert('Australia/Adelaide').hour).mean().idxmax()}h (local)")

    # hold HouseParams.solar at 0; solar enters as external q
    rooms0 = {r: replace(HouseParams().rooms[r], solar=0.0) for r in ROOMS}
    params = HouseParams(rooms=rooms0)

    s_ne = {r: 0.0 for r in ROOMS}
    s_nw = {r: 0.0 for r in ROOMS}
    for it in range(args.iters):
        forcing = {r: s_ne[r] * b_ne + s_nw[r] * b_nw for r in ROOMS}
        forcing["_b_ne"] = b_ne
        forcing["_b_nw"] = b_nw
        samples = pd.concat(
            [simulate_window(params, df, forcing, w0, w1) for w0, w1 in windows],
            ignore_index=True,
        )
        X = samples[["b_ne", "b_nw"]].to_numpy()
        parts = [f"iter {it}: n={len(samples)}"]
        for r in ROOMS:
            y = samples[f"err_{r}"].to_numpy() / (STEP_MIN / 60.0)
            coef, *_ = np.linalg.lstsq(X, y, rcond=None)
            s_ne[r] = max(0.0, s_ne[r] + float(coef[0]))
            s_nw[r] = max(0.0, s_nw[r] + float(coef[1]))
            parts.append(f"{r}: ne={s_ne[r]:.2f} nw={s_nw[r]:.2f}")
        print("  ".join(parts))

    print("\nfitted orientation solar terms [K/h at clear-sky full sun]:")
    print(f"{'room':8s} {'s_ne':>7s} {'s_nw':>7s}")
    for r in ROOMS:
        print(f"{r:8s} {s_ne[r]:7.3f} {s_nw[r]:7.3f}")


if __name__ == "__main__":
    main()
