#!/usr/bin/env python3
"""First-pass system identification on the June 2026 archive.
Reproduces the numbers in docs/calibration.md. Run from the repo root:

    uv run python calib.py --start 2026-06-01 --end 2026-06-30 \
        --out data/processed/june.parquet
    uv run python analysis/sysid_june.py [data/processed/june.parquet]

Three stages:
  1. Free-running night decay -> per-room two-node RC fit
     dT/dt = a*(T_out - T) + c*(T_others - T) + g
  2. Pooled house-average decay -> envelope time constant tau_house
  3. Heating efficiency shape: q = dT/dt + (T - T_out)/tau_house is
     delivered heat per unit thermal mass [K/h]; regress e = q/P_kW on
     power and outdoor temp, with and without solar-gain hours, and with
     defrost episodes excised.
  4. Defrost detection + overhead: actrl forces aircon_comp_speed to max
     on defrost (compressor on + outdoor fan off), so comp_speed >= 15
     while the outdoor unit draws power labels defrost + recovery.
     Corroborated by m5atom_inside_coil_inlet_temp crashing from ~35 C to
     1-12 C during each episode. (The outside-coil-temp excursion idea
     from docs/calibration.md v1 found nothing - that sensor never rose
     more than ~10 K above ambient in June.)

Dev-only analysis tooling (pandas); never imported by the deployed apps.
"""
from __future__ import annotations

import sys

import numpy as np
import pandas as pd

ROOMS = ["bed_1", "bed_2", "bed_3", "kitchen", "study"]
OFF_W = 50  # outdoor unit below this = free-running
NIGHT = (21, 8)  # local hours, no solar
MIN_WINDOW_MIN = 180
STEP = "10min"  # analysis resolution


def night_windows(df: pd.DataFrame) -> list[tuple[pd.Timestamp, pd.Timestamp]]:
    local = df.index.tz_convert("Australia/Adelaide")
    off = df["power.outdoor_unit"].fillna(np.inf) < OFF_W
    hr = pd.Series(local.hour + local.minute / 60, index=df.index)
    night = (hr >= NIGHT[0]) | (hr < NIGHT[1])
    eligible = off & night
    grp = (eligible != eligible.shift()).cumsum()
    return [
        (seg.iloc[0], seg.iloc[-1])
        for _, seg in df.index.to_series().groupby(grp)
        if eligible.loc[seg.iloc[0]] and len(seg) >= MIN_WINDOW_MIN
    ]


def ols(X: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, float]:
    coef, res, *_ = np.linalg.lstsq(X, y, rcond=None)
    r2 = 1 - res[0] / np.sum((y - y.mean()) ** 2) if len(res) and np.var(y) > 0 else np.nan
    return coef, r2


def decay_fits(df: pd.DataFrame, windows) -> None:
    temps = {r: df[f"{r}_average_temperature"] for r in ROOMS}
    tout = df["temperature_adelaide"]

    def prep(s, w0, w1):
        return s.loc[w0:w1].resample(STEP).mean()

    # pooled house average, single-node
    X, Y = [], []
    for w0, w1 in windows:
        t = prep(sum(temps.values()) / len(ROOMS), w0, w1)
        o = prep(tout, w0, w1)
        dTdt = (t.shift(-3) - t) / 0.5  # K/h over the next 30 min
        m = dTdt.notna() & (o - t).notna()
        X.append((o - t)[m].to_numpy())
        Y.append(dTdt[m].to_numpy())
    x, y = np.concatenate(X), np.concatenate(Y)
    (slope, icept), r2 = ols(np.vstack([x, np.ones_like(x)]).T, y)
    print(f"house-average envelope: tau={1 / slope:.1f} h, gains={icept:+.3f} K/h, "
          f"r2={r2:.3f}, n={len(y)}")

    # per-room two-node
    print("\nper-room: dT/dt = a*(T_out-T) + c*(T_others-T) + g  [pooled windows]")
    print(f"{'room':8s} {'tau_out(h)':>10s} {'tau_cpl(h)':>10s} {'gains':>7s} {'r2':>6s}")
    for room in ROOMS:
        others = sum(v for k, v in temps.items() if k != room) / (len(ROOMS) - 1)
        Xo, Xc, Y = [], [], []
        for w0, w1 in windows:
            t = prep(temps[room], w0, w1)
            o = prep(tout, w0, w1)
            oth = prep(others, w0, w1)
            dTdt = (t.shift(-3) - t) / 0.5
            m = dTdt.notna() & (o - t).notna() & (oth - t).notna()
            Xo.append((o - t)[m].to_numpy())
            Xc.append((oth - t)[m].to_numpy())
            Y.append(dTdt[m].to_numpy())
        xo, xc, y = map(np.concatenate, (Xo, Xc, Y))
        (a, c, g), r2 = ols(np.vstack([xo, xc, np.ones_like(xo)]).T, y)
        tau_o = 1 / a if a > 1e-4 else float("inf")
        tau_c = 1 / c if c > 1e-4 else float("inf")
        print(f"{room:8s} {tau_o:10.1f} {tau_c:10.1f} {g:+7.3f} {r2:6.3f}")


def defrost_mask(df: pd.DataFrame, pad: str = "15min") -> pd.Series:
    """True for defrost + recovery windows: actrl's forced comp_speed max
    while the outdoor unit draws power, padded by `pad` on both sides."""
    sig = ((df["aircon_comp_speed"] >= 15) & (df["power.outdoor_unit"] > 300)).fillna(False)
    grp = (sig != sig.shift()).cumsum()
    mask = pd.Series(False, index=df.index)
    padding = pd.Timedelta(pad)
    for _, seg in df.index.to_series().groupby(grp):
        if sig.loc[seg.iloc[0]]:
            mask.loc[seg.iloc[0] - padding : seg.iloc[-1] + padding] = True
    return mask


def defrost_overhead(df: pd.DataFrame) -> None:
    mask = defrost_mask(df)
    P = df["power.outdoor_unit"] + df["power.indoor_unit"]
    heat_kwh = P[P > 300].sum() / 1000 / 60
    dfr_kwh = P[mask & (P > 300)].sum() / 1000 / 60
    local = df.index.tz_convert("Australia/Adelaide")
    cold_am = (
        pd.Series((local.hour >= 3) & (local.hour < 8), index=df.index)
        & (df["temperature_adelaide"] < 7.5)
    )
    cm_kwh = P[cold_am & (P > 300)].sum() / 1000 / 60
    print(f"heating energy {heat_kwh:.1f} kWh; defrost+recovery {dfr_kwh:.1f} kWh "
          f"({100 * dfr_kwh / heat_kwh:.1f}%); cold mornings (03-08 local, "
          f"Tout<7.5C) {cm_kwh:.1f} kWh ({100 * cm_kwh / heat_kwh:.0f}%), "
          f"{100 * dfr_kwh / cm_kwh:.0f}% of it in defrost windows")


def efficiency_shape(df: pd.DataFrame, tau_house: float) -> None:
    T = sum(df[f"{r}_average_temperature"] for r in ROOMS) / len(ROOMS)
    t10 = T.resample(STEP).mean()
    o10 = df["temperature_adelaide"].resample(STEP).mean()
    p10 = (df["power.outdoor_unit"] + df["power.indoor_unit"]).resample(STEP).mean()
    pv10 = df["power_pv_5m"].resample(STEP).mean()
    dfr10 = defrost_mask(df).resample(STEP).max().astype(bool)
    dTdt = (t10.shift(-1) - t10.shift(1)) / (2 / 6)  # central diff, K/h
    q = dTdt + (t10 - o10) / tau_house
    heating = (p10 > 300) & (p10.shift(1) > 300) & (p10.shift(-1) > 300)
    nosun = pv10.fillna(0) < 100

    for label, extra in [
        ("all heating", heating),
        ("no-sun only", heating & nosun),
        ("no-sun, no-defrost", heating & nosun & ~dfr10),
    ]:
        d = pd.DataFrame({"e": q / (p10 / 1000), "P": p10, "Tout": o10})[extra].dropna()
        d = d[d.e.between(-2, 15)]  # trim defrost/transient nonsense
        X = np.vstack([np.ones(len(d)), d.P / 1000 - 0.7, d.Tout - 10]).T
        (a, b, c), r2 = ols(X, d.e.to_numpy())
        loss = 100 * (1 - (a + b * 1.8) / a)
        print(f"{label:18s} n={len(d):4d}: e = {a:.3f} {b:+.3f}*(P_kW-0.7) "
              f"{c:+.4f}*(Tout-10)  r2={r2:.3f}  "
              f"min->max loss {loss:.0f}%, Tout effect {100 * c / a:.1f}%/K")


def main() -> int:
    path = sys.argv[1] if len(sys.argv) > 1 else "data/processed/june.parquet"
    df = pd.read_parquet(path)
    windows = night_windows(df)
    print(f"{len(windows)} free-running night windows >= {MIN_WINDOW_MIN} min\n")
    decay_fits(df, windows)
    print("\nheating efficiency proxy e = q/P [K/h per kW] "
          "(q referenced to tau_house=29.9):")
    efficiency_shape(df, tau_house=29.9)
    print("\ndefrost overhead:")
    defrost_overhead(df)
    return 0


if __name__ == "__main__":
    sys.exit(main())
