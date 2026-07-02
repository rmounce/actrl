"""Identify the compressor spin-up lag from recorded power-step transients.

The closed-loop simulator (sim/closed_loop.py) initially treated a
compressor-increment change as an instantaneous change in electrical power
and delivered heat. In reality the inverter ramps: this script measures how
fast, by finding clean steps in the controller's compressor-speed estimate
(input_number.aircon_comp_speed, logged ~20 s) and fitting a first-order
time constant to the outdoor-unit power response (Shelly ch1, ~13 s).

A "clean" step is one where the speed estimate is constant for PRE_HOLD
before the step and for POST_HOLD after it, so the power trace in the fit
window is responding to exactly one command change. Defrost windows are
excluded (power spikes there are not commanded steps).

Usage:
    uv run python analysis/lag_fit.py [--data-dir data] [--min-step 3]

Output: one line per event (time, step, fitted tau, r^2) and a summary.
The fitted tau feeds HvacParams.lag_tau_s in sim/hvac.py.
"""
from __future__ import annotations

import argparse
import gzip
import io
from pathlib import Path

import numpy as np
import pandas as pd

PRE_HOLD = pd.Timedelta("4min")
POST_HOLD = pd.Timedelta("8min")
FIT_WINDOW = pd.Timedelta("8min")


def _load_series(day_dir: Path, fname: str) -> pd.Series:
    path = day_dir / fname
    if not path.exists():
        return pd.Series(dtype=float)
    with gzip.open(path, "rt") as f:
        df = pd.read_csv(io.StringIO(f.read()))
    df = df[df["field"] == "value"]
    s = pd.Series(
        pd.to_numeric(df["value"], errors="coerce").values,
        index=pd.to_datetime(df["time"], format="ISO8601", utc=True),
    ).dropna()
    return s[~s.index.duplicated()]


def load_month(data_dir: Path) -> tuple[pd.Series, pd.Series]:
    speeds, powers = [], []
    for day_dir in sorted((data_dir / "raw").iterdir()):
        if not day_dir.is_dir():
            continue
        speeds.append(_load_series(day_dir, "input_number__aircon_comp_speed.csv.gz"))
        powers.append(
            _load_series(
                day_dir, "sensor__power__shellyem_ec64c9c6932b_channel_1_power.csv.gz"
            )
        )
    return pd.concat(speeds).sort_index(), pd.concat(powers).sort_index()


def find_steps(speed: pd.Series, min_step: float) -> list[tuple[pd.Timestamp, float, float]]:
    """Times where the speed estimate jumps by >= min_step after PRE_HOLD of
    one constant value and stays at the new value for POST_HOLD."""
    events = []
    changes = speed[speed.diff().abs() >= min_step]
    for t, new in changes.items():
        pre = speed[(speed.index >= t - PRE_HOLD) & (speed.index < t)]
        post = speed[(speed.index > t) & (speed.index <= t + POST_HOLD)]
        if len(pre) < 8 or len(post) < 16:
            continue
        if pre.nunique() == 1 and post.nunique() == 1 and post.iloc[0] == new:
            events.append((t, float(pre.iloc[0]), float(new)))
    return events


def fit_tau(power: pd.Series, t0: pd.Timestamp) -> tuple[float, float] | None:
    """Fit P(t) = P1 + (P0 - P1) * exp(-t/tau) over FIT_WINDOW after t0.
    Grid search over tau, closed-form endpoints. Returns (tau_s, r2)."""
    pre = power[(power.index >= t0 - PRE_HOLD) & (power.index < t0)]
    seg = power[(power.index >= t0) & (power.index <= t0 + FIT_WINDOW)]
    if len(pre) < 10 or len(seg) < 20:
        return None
    p0 = pre.median()
    y = seg.values.astype(float)
    ts = np.asarray((seg.index - t0).total_seconds())
    best = None
    for tau in np.arange(5, 400, 5):
        basis = np.exp(-ts / tau)
        # y ~ p1 + (p0 - p1)*basis  ->  y - p0*basis = p1*(1 - basis)
        a = 1.0 - basis
        denom = (a * a).sum()
        if denom == 0:
            continue
        p1 = ((y - p0 * basis) * a).sum() / denom
        resid = y - (p1 + (p0 - p1) * basis)
        ss_res = (resid**2).sum()
        ss_tot = ((y - y.mean()) ** 2).sum()
        r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 0.0
        if best is None or ss_res < best[2]:
            best = (tau, r2, ss_res, p1)
    if best is None:
        return None
    tau, r2, _, p1 = best
    if abs(p1 - p0) < 80:  # step too small in power terms to trust
        return None
    return float(tau), float(r2)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default="data", type=Path)
    ap.add_argument("--min-step", default=3.0, type=float)
    args = ap.parse_args()

    speed, power = load_month(args.data_dir)
    events = find_steps(speed, args.min_step)
    taus = []
    print(f"{len(events)} clean speed steps (>= {args.min_step})")
    for t, old, new in events:
        fit = fit_tau(power, t)
        if fit is None:
            continue
        tau, r2 = fit
        keep = r2 > 0.7
        taus += [tau] if keep else []
        flag = "" if keep else "  (rejected r2)"
        print(f"  {t}  {old:4.0f} -> {new:4.0f}  tau={tau:5.0f}s  r2={r2:.2f}{flag}")
    if taus:
        arr = np.array(taus)
        print(
            f"\nn={len(arr)}  median tau={np.median(arr):.0f}s  "
            f"IQR=[{np.percentile(arr, 25):.0f}, {np.percentile(arr, 75):.0f}]s"
        )
    else:
        print("no usable fits")


if __name__ == "__main__":
    main()
