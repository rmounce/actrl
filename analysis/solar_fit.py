"""Per-room solar/daytime gain fit (the dominant gap found by
analysis/replay_day.py — see docs/calibration.md "Whole-day replay").

Holds the night-calibrated RC parameters (sim/house.py defaults) FIXED and
regresses the model residual during unit-off periods on the Solcast PV
"power now" estimate (power_pv_5m, W) as an irradiance proxy:

    resid = dT/dt - [a*(T_out-T) + c*(T_others-T) + g]  ~=  s * PV_filt_kW

PV is optionally passed through a first-order filter (tau grid-searched
per room) so west-facing rooms that warm late and release slowly can be
captured. Fitting residuals (rather than refitting everything jointly)
keeps the validated night calibration untouched; the solar term is purely
additive and zero at night by construction.

Usage:
    uv run python analysis/solar_fit.py [--parquet data/processed/june.parquet]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from sim.house import HouseParams  # noqa: E402

ROOMS = ["bed_1", "bed_2", "bed_3", "kitchen", "study"]
OFF_W = 50
MIN_WINDOW_MIN = 120
STEP = "10min"


def off_windows(df: pd.DataFrame) -> list[tuple[pd.Timestamp, pd.Timestamp]]:
    off = df["power.outdoor_unit"].fillna(np.inf) < OFF_W
    grp = (off != off.shift()).cumsum()
    return [
        (seg.iloc[0], seg.iloc[-1])
        for _, seg in df.index.to_series().groupby(grp)
        if off.loc[seg.iloc[0]] and len(seg) >= MIN_WINDOW_MIN
    ]


def fit(df: pd.DataFrame) -> None:
    params = HouseParams().rooms
    temps = {r: df[f"{r}_average_temperature"] for r in ROOMS}
    tout = df["temperature_adelaide"]
    pv_kw = (df["power_pv_5m"].fillna(0) / 1000.0).clip(lower=0)
    windows = off_windows(df)
    print(f"{len(windows)} unit-off windows >= {MIN_WINDOW_MIN} min (day + night)")

    print(f"{'room':8s} {'s (K/h per kW)':>14s} {'filt tau(h)':>11s} "
          f"{'resid r2':>8s} {'n':>6s}")
    for room in ROOMS:
        p = params[room]
        others = sum(v for k, v in temps.items() if k != room) / (len(ROOMS) - 1)
        best = None
        for tau_h in [0.0, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0]:
            if tau_h > 0:
                # EWM on the 1-min grid ~ first-order filter.
                pv_f = pv_kw.ewm(halflife=pd.Timedelta(hours=tau_h * np.log(2)),
                                 times=pv_kw.index).mean()
            else:
                pv_f = pv_kw
            X, Y = [], []
            for w0, w1 in windows:
                t = temps[room].loc[w0:w1].resample(STEP).mean()
                o = tout.loc[w0:w1].resample(STEP).mean()
                oth = others.loc[w0:w1].resample(STEP).mean()
                pvw = pv_f.loc[w0:w1].resample(STEP).mean()
                dTdt = (t.shift(-3) - t) / 0.5
                pred = (
                    (o - t) / p.tau_out + (oth - t) / p.tau_cpl + p.gain
                )
                m = dTdt.notna() & pred.notna() & pvw.notna()
                X.append(pvw[m].to_numpy())
                Y.append((dTdt - pred)[m].to_numpy())
            x, y = np.concatenate(X), np.concatenate(Y)
            s = (x * y).sum() / (x * x).sum()  # through origin: no gain at PV=0
            resid = y - s * x
            r2 = 1 - (resid**2).sum() / (y**2).sum()
            if best is None or r2 > best[2]:
                best = (s, tau_h, r2, len(y))
        s, tau_h, r2, n = best
        print(f"{room:8s} {s:14.3f} {tau_h:11.1f} {r2:8.3f} {n:6d}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--parquet", default="data/processed/june.parquet", type=Path)
    args = ap.parse_args()
    df = pd.read_parquet(args.parquet)
    if df.index.tz is None:
        df.index = df.index.tz_localize("UTC")
    fit(df)


if __name__ == "__main__":
    main()
