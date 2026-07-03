"""Identify the minutes-scale heat-delivery lag (docs/calibration.md "Time
delays" — Ryan, 2026-07-03): the delay between a compressor-speed change
(visible immediately in electrical power, tau ~ 20 s, analysis/lag_fit.py)
and the "elbow" it produces in the room-temperature trajectory. This bundles
refrigerant/coil dynamics, duct transport, and average_temperature sensor
smoothing into one lumped lag on delivered heat Q — not separately
identifiable from this data, so we fit one dead-time + first-order-lag pair
per room (and pooled) directly against clean compressor-speed steps.

Method: reuse lag_fit.find_steps for clean +/-1 increment steps (constant
speed for PRE_HOLD before, POST_HOLD after). For each event and room, fit a
local temperature slope dT/dt [K/h] in sliding windows around the step via
short OLS regressions (average_temperature updates every ~10 s, dense enough
for this). Sign-flip by the step direction so warming and cooling steps
superpose constructively, subtract the pre-step baseline slope, and average
across events to get an excess-dT/dt(t) curve. Fit
excess(t) = A * (1 - exp(-(t - d)/tau)) for t >= d, else 0, grid-searching
dead time d and lag tau.

Usage:
    uv run python analysis/heat_lag_fit.py [--data-dir data] [--min-step 1]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lag_fit import _load_series, find_steps  # noqa: E402

ROOMS = ("bed_1", "bed_2", "bed_3", "kitchen", "study")

PRE_HOLD = pd.Timedelta("4min")
POST_HOLD = pd.Timedelta("8min")
PRE_WINDOW = pd.Timedelta("6min")   # baseline slope window before t0
POST_WINDOW = pd.Timedelta("20min")  # elbow-search window after t0
SLOPE_HALF_WIN = pd.Timedelta("90s")  # local regression half-window
STEP_MIN = pd.Timedelta("1min")       # spacing of slope-curve samples
MIN_SLOPE_SAMPLES = 4


def load_temps(data_dir: Path) -> dict[str, pd.Series]:
    out = {r: [] for r in ROOMS}
    for day_dir in sorted((data_dir / "raw").iterdir()):
        if not day_dir.is_dir():
            continue
        for r in ROOMS:
            out[r].append(
                _load_series(day_dir, f"sensor__temperature__{r}_average_temperature.csv.gz")
            )
    return {r: pd.concat(s).sort_index() for r, s in out.items()}


def local_slope(temp: pd.Series, center: pd.Timestamp) -> float | None:
    """OLS slope [K/h] of temp vs time in a window centered on `center`."""
    seg = temp[(temp.index >= center - SLOPE_HALF_WIN) & (temp.index <= center + SLOPE_HALF_WIN)]
    if len(seg) < MIN_SLOPE_SAMPLES:
        return None
    ts = (seg.index - center).total_seconds().values.astype(float)
    y = seg.values.astype(float)
    ts_mean, y_mean = ts.mean(), y.mean()
    denom = ((ts - ts_mean) ** 2).sum()
    if denom == 0:
        return None
    slope_per_s = ((ts - ts_mean) * (y - y_mean)).sum() / denom
    return slope_per_s * 3600.0  # K/h


def event_curve(temp: pd.Series, t0: pd.Timestamp, sign: float) -> pd.Series | None:
    """Sign-flipped, baseline-subtracted dT/dt(t-t0) curve, 1-min grid, from
    -PRE_WINDOW to +POST_WINDOW. None if insufficient data."""
    offsets = np.arange(
        -PRE_WINDOW.total_seconds(), POST_WINDOW.total_seconds() + 1, STEP_MIN.total_seconds()
    )
    slopes = []
    for off in offsets:
        c = t0 + pd.Timedelta(seconds=off)
        s = local_slope(temp, c)
        slopes.append(s)
    if sum(s is not None for s in slopes) < len(offsets) * 0.7:
        return None
    curve = pd.Series(slopes, index=offsets, dtype=float) * sign
    baseline = curve[curve.index < 0].mean()
    if pd.isna(baseline):
        return None
    return curve - baseline


def superpose(
    temps: dict[str, pd.Series], events: list[tuple], room: str | None
) -> tuple[pd.Series, int]:
    """Average excess-dT/dt curve across events, for one room or (room=None)
    pooled across all rooms."""
    curves = []
    rooms = [room] if room else ROOMS
    for t0, old, new in events:
        sign = np.sign(new - old)
        for r in rooms:
            c = event_curve(temps[r], t0, sign)
            if c is not None:
                curves.append(c)
    if not curves:
        return pd.Series(dtype=float), 0
    return pd.concat(curves, axis=1).mean(axis=1), len(curves)


def fit_deadtime_lag(curve: pd.Series) -> tuple[float, float, float, float] | None:
    """Fit excess(t) = A*(1 - exp(-(t-d)/tau)) for t>=d else 0.
    Returns (dead_time_s, tau_s, A, r2)."""
    t = curve.index.values.astype(float)
    y = curve.values.astype(float)
    mask_all = t >= 0  # only fit the post-step half; pre-step should be ~0
    t, y = t[mask_all], y[mask_all]
    ss_tot = ((y - y.mean()) ** 2).sum()
    if ss_tot == 0:
        return None
    best = None
    for d in np.arange(0, 480, 15):
        post = t >= d
        if post.sum() < 5:
            continue
        for tau in np.arange(15, 900, 15):
            basis = np.zeros_like(t)
            basis[post] = 1.0 - np.exp(-(t[post] - d) / tau)
            denom = (basis[post] ** 2).sum()
            if denom == 0:
                continue
            a = (y[post] * basis[post]).sum() / denom
            pred = a * basis
            ss_res = ((y - pred) ** 2).sum()
            if best is None or ss_res < best[0]:
                best = (ss_res, d, tau, a)
    if best is None:
        return None
    ss_res, d, tau, a = best
    r2 = 1 - ss_res / ss_tot
    return float(d), float(tau), float(a), float(r2)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default="data", type=Path)
    ap.add_argument("--min-step", default=1.0, type=float)
    args = ap.parse_args()

    speed = pd.concat(
        _load_series(d, "input_number__aircon_comp_speed.csv.gz")
        for d in sorted((args.data_dir / "raw").iterdir())
        if d.is_dir()
    ).sort_index()
    all_events = find_steps(speed, args.min_step)
    events = [(t, o, n) for t, o, n in all_events if abs(n - o) == 1]
    print(f"{len(events)} clean single-increment steps (of {len(all_events)} total)")

    temps = load_temps(args.data_dir)

    print(f"\n{'room':<10} {'n':>4} {'dead time':>10} {'tau':>6} {'plateau':>9} {'r2':>5}")
    for r in ROOMS:
        curve, n = superpose(temps, events, r)
        if curve.empty:
            print(f"{r:<10} insufficient data")
            continue
        fit = fit_deadtime_lag(curve)
        if fit is None:
            print(f"{r:<10} {n:>4}  fit failed")
            continue
        d, tau, a, r2 = fit
        print(f"{r:<10} {n:>4} {d:9.0f}s {tau:5.0f}s {a:8.3f} K/h {r2:5.2f}")

    pooled_curve, n = superpose(temps, events, None)
    if not pooled_curve.empty:
        fit = fit_deadtime_lag(pooled_curve)
        if fit:
            d, tau, a, r2 = fit
            print(f"\n{'pooled':<10} {n:>4} {d:9.0f}s {tau:5.0f}s {a:8.3f} K/h {r2:5.2f}")


if __name__ == "__main__":
    main()
