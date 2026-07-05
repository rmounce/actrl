"""Pre-heating ramp strategy study (docs/tuning.md follow-up).

statctrl currently ramps morning targets slowly (+0.1 K per ~3 min) so the
unit rides near minimum power, on the basis that min power has the best
COP. The competing view: heating earlier holds the house warm for longer,
accumulating envelope losses — starting later at higher power might use
less energy overall. This script evaluates alternative morning-warmup
schedules in the calibrated closed loop.

Method: for each room with a recorded morning target ramp (rise group of
the ffilled target_temp_low, net >= 0.3 K, first one before noon), replace
the controller-visible low with a synthetic strategy; the *scoring*
requirement is identical for every strategy — night band until the
recorded ramp's end minute D (the deadline), full final target from D on —
so strategies compete on cost for the same comfort outcome.

Strategies (per transformed room, night value n, final f, deadline D):
  recorded          the archived schedule (statctrl behaviour), unchanged
  step<L>           hold n until D-L minutes, then step straight to f
                    (actrl's own internal target smoothing + faithful mode
                    turn this into a hard, high-power warmup)
  ramp<R>x<L>       start at D-L, ramp at R K/min up to f

Scored over 00:00-12:00 local: energy_kwh_am; deg-min below the
requirement band; lateness (first minute after D where feels >= f, per
room, worst room); defrost episodes; above-min fraction (fan-noise/air-
charge proxy); peak increment. Whole-day energy for reference.

Usage:
  uv run python analysis/preheat.py --date 2026-06-22 \
      [--strategy step90 ...] [--eslope 0.04] [--out csv]

--eslope rotates the hvac efficiency slope e_per_kw about 1.8 kW (same
pivot as the 2026-07-04 sensitivity grid) to bracket the min-power-COP
uncertainty the strategy question hinges on.
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import replace
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
from sim.hvac import Hvac, HvacParams  # noqa: E402
from sim.solar import AZ_NE, AZ_NW, vertical_irradiance  # noqa: E402
from analysis.replay_day import load_day, feels_like_offsets, CYCLES_PER_MIN  # noqa: E402

LOCAL_TZ = ZoneInfo("Australia/Adelaide")
RISE_GAP_MIN = 15
RISE_STEP_K = 0.3
AM_END_MIN = 12 * 60
E_SLOPE_PIVOT_KW = 1.8


def detect_ramp(low: np.ndarray) -> tuple[int, int, float, float] | None:
    """(start, deadline, night, final) of the first morning rise group of
    the ffilled low series, or None. Deadline = minute of the group's last
    rise (when the recorded schedule reaches its final value)."""
    rise_idx = [i for i in range(1, min(len(low), AM_END_MIN)) if low[i] > low[i - 1]]
    k = 0
    while k < len(rise_idx):
        j = k
        while j + 1 < len(rise_idx) and rise_idx[j + 1] - rise_idx[j] <= RISE_GAP_MIN:
            j += 1
        start, end = rise_idx[k], rise_idx[j]
        night, final = low[start - 1], low[end]
        if final - night >= RISE_STEP_K:
            return start, end, float(night), float(final)
        k = j + 1
    return None


def synth_low(low: np.ndarray, ramp, strategy: str) -> np.ndarray:
    """Controller-visible low under `strategy` ('recorded', 'stepL',
    'rampRxL' with R in K/min, L in minutes before the deadline)."""
    if strategy == "recorded" or ramp is None:
        return low
    start, deadline, night, final = ramp
    out = low.copy()
    if strategy.startswith("step"):
        lead = int(strategy[4:])
        s = max(0, deadline - lead)
        out[:s] = night
        out[s:deadline] = final
        # from deadline on, keep the recorded values (already = final)
        return out
    if strategy.startswith("ramp"):
        rate_s, lead_s = strategy[4:].split("x")
        rate, lead = float(rate_s), int(lead_s)
        s = max(0, deadline - lead)
        out[:s] = night
        for m in range(s, deadline):
            out[m] = min(final, night + rate * (m - s))
        return out
    raise ValueError(f"unknown strategy {strategy!r}")


def run(day: pd.DataFrame, strategy: str, eslope: float | None):
    lows_rec = {
        r: day[f"climate.{r}_aircon.target_temp_low"].astype(float).ffill().to_numpy()
        for r in ROOMS
    }
    highs = {
        r: day[f"climate.{r}_aircon.target_temp_high"].astype(float).ffill().to_numpy()
        for r in ROOMS
    }
    ramps = {r: detect_ramp(lows_rec[r]) for r in ROOMS}
    lows_ctl = {r: synth_low(lows_rec[r], ramps[r], strategy) for r in ROOMS}

    hvac = None
    if eslope is not None:
        base = HvacParams()
        e_at_pivot = base.e0 + base.e_per_kw * (E_SLOPE_PIVOT_KW - base.e_ref_p_kw)
        hvac = Hvac(
            replace(
                base,
                e0=e_at_pivot + eslope * (base.e_ref_p_kw - E_SLOPE_PIVOT_KW),
                e_per_kw=eslope,
            )
        )

    first = day.iloc[0]
    setpoint = float(first["climate.m5atom_climate.temperature"])
    targets = {r: (float(lows_ctl[r][0]), float(highs[r][0])) for r in ROOMS}
    temps = {r: float(first[f"{r}_average_temperature"]) for r in ROOMS}
    world = base_world(setpoint=setpoint)
    room_climates(world, "heat_cool", targets, temps)
    loop = ClosedLoop(world, temps, setpoint=setpoint, hvac=hvac)

    tout = day["temperature_adelaide"].ffill().to_numpy()
    pv = (day["power_pv_5m"].fillna(0).clip(lower=0) / 1000.0).to_numpy()
    cloud = day["_cloudiness"].to_numpy()
    sun_ne = np.array([vertical_irradiance(ts, AZ_NE) * c for ts, c in zip(day.index, cloud)])
    sun_nw = np.array([vertical_irradiance(ts, AZ_NW) * c for ts, c in zip(day.index, cloud)])
    offs = feels_like_offsets(day)
    sp = day["climate.m5atom_climate.temperature"].ffill().to_numpy()

    defrost_min = 0
    rows = []
    for i in range(len(day) * CYCLES_PER_MIN):
        m = i // CYCLES_PER_MIN
        updates = {
            "climate.m5atom_climate": {"attributes": {"temperature": float(sp[m])}},
            **{
                f"climate.{r}_aircon": {
                    "attributes": {
                        "target_temp_low": float(lows_ctl[r][m]),
                        "target_temp_high": float(highs[r][m]),
                    }
                }
                for r in ROOMS
            },
        }
        rows.append(
            loop.step(
                t_out=float(tout[m]), updates=updates, pv_kw=float(pv[m]),
                sun_ne=float(sun_ne[m]), sun_nw=float(sun_nw[m]),
                ctrl_offsets={r: float(offs[r][m]) for r in ROOMS},
            )
        )
        if loop._defrost.active and i % CYCLES_PER_MIN == 0:
            defrost_min += 1
    loop.close()
    sim = pd.DataFrame(rows[CYCLES_PER_MIN - 1 :: CYCLES_PER_MIN], index=day.index)
    return sim, ramps, lows_rec, offs, defrost_min


def score(day, sim, ramps, lows_rec, offs, defrost_min) -> dict:
    am = np.arange(len(day)) < AM_END_MIN
    p = sim["p_kw"].to_numpy()
    out = {
        "energy_am": float(p[am].sum() / 60.0),
        "energy_day": float(p.sum() / 60.0),
        "defrost_min": defrost_min,
        "peak_incr": int(sim["increment"].max()),
        "abovemin_am": float((sim["increment"].to_numpy()[am] > 0).mean()),
    }
    deg_min = 0.0
    late_worst = 0.0
    for r in ROOMS:
        feels = sim[f"Tm_{r}"].to_numpy() + offs[r]
        ramp = ramps[r]
        req = lows_rec[r].copy()
        if ramp is not None:
            start, deadline, night, final = ramp
            req[:deadline] = night
            req[deadline:] = np.maximum(req[deadline:], final)
            reach = np.nonzero(feels[deadline:] >= final)[0]
            late_worst = max(late_worst, float(reach[0]) if len(reach) else float("inf"))
        deg_min += float(np.clip(req[am] - feels[am], 0, None).sum())
    out["deg_min_below_req"] = deg_min / len(ROOMS)
    out["late_min_worst"] = late_worst
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--date", required=True)
    ap.add_argument("--parquet", default="data/processed/june.parquet", type=Path)
    ap.add_argument("--strategy", action="append", default=None)
    ap.add_argument("--eslope", type=float, default=None)
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()
    strategies = args.strategy or [
        "recorded", "step150", "step120", "step90", "step60",
        "ramp0.1x60", "ramp0.05x90",
    ]
    day = load_day(args.parquet, args.date)
    results = []
    for s in strategies:
        sim, ramps, lows_rec, offs, dfr = run(day, s, args.eslope)
        m = score(day, sim, ramps, lows_rec, offs, dfr)
        results.append({"strategy": s, **m})
        print(
            f"{s:<12} am {m['energy_am']:5.2f} kWh  day {m['energy_day']:5.2f}  "
            f"late {m['late_min_worst']:5.0f} min  degmin {m['deg_min_below_req']:6.1f}  "
            f"defrost {m['defrost_min']:3d} min  abovemin {m['abovemin_am']:.2f}  peak {m['peak_incr']}"
        )
    if args.out:
        pd.DataFrame(results).to_csv(args.out, index=False)
        print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
