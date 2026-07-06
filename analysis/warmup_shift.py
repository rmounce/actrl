"""Price-aware warmup start time (Phase C component 2, the spike money).

Plain-language version: every morning the house must be warmed to the day
targets by the schedule deadline. That energy is going to be bought
regardless -- the only choice is WHEN. Starting earlier costs a little
more heat (the house leaks while held warm: ~UA x extra-warm-hours) but
buys at the earlier hour's price. Starting later risks buying the whole
warmup inside a morning price spike. So each candidate start time has an
expected cost = (energy profile it implies) x (price forecast); pick the
cheapest. No thresholds -- on a flat morning the leak penalty makes the
usual just-in-time start cheapest; the start only moves when the forecast
spread pays for the leak.

Coarse cost model (decision rule, deployable in statctrl):
    warm phase: from t_s, house heats at capacity until day target
        reached: rate = (e(Pmax,Tout)*Pmax - UA*dT/1000) / C_eff [K/h]
    hold phase: maintenance power UA*(T_day - Tout)/(e*1000) [kW] until
        deadline.
    cost(t_s) = sum over half-hours of P(t) * price_used(t)
    price_used = E[actual | predispatch] (analysis/fc_calibration.json)
        from the latest vintage available at the decision time (00:30),
        the same signal a live statctrl would have.

Validation: the chosen t_s is applied by time-shifting the recorded
morning target ramp in the day frame, replayed through the FULL sim, and
scored on ACTUAL prices + comfort vs the ORIGINAL schedule -- same
accounting as preheat_price.py. Arms: baseline / shift-only / shift +
continuous price offset (k=2, tau=20; analysis/price_offset.py).

Usage:
    uv run python analysis/warmup_shift.py [--dates 2026-06-22,2026-06-23,2026-06-24,2026-06-09]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

_ROOT = Path(__file__).resolve().parent.parent
for p in (str(_ROOT), str(_ROOT / "tests")):
    if p not in sys.path:
        sys.path.insert(0, p)

import sim.closed_loop as cl  # noqa: E402  (hassapi stub)
from sim.hvac import Hvac  # noqa: E402
from analysis.comfort import score_day  # noqa: E402
from analysis.price_cost import load_prices  # noqa: E402
from analysis.price_offset import load_predispatch, offset_series, retailize  # noqa: E402
from analysis.replay_day import load_day, replay  # noqa: E402
from analysis.tune import build_comfort_frame  # noqa: E402

LOCAL_TZ = "Australia/Adelaide"
UA_KW_PER_K = 0.160  # docs/calibration.md envelope fit
DECISION_HH = 0.5    # coarse decision made at 00:30 local


def morning_schedule(day: pd.DataFrame) -> tuple[np.ndarray, dict]:
    """(local fractional hour array, per-room morning info).

    rise_end = first minute the recorded low reaches its 06:00-11:00 max
    (statctrl ramps +0.1K/3min, so this is the scheduled comfort deadline).
    """
    local = day.index.tz_convert(LOCAL_TZ)
    hh = (local.hour + local.minute / 60.0).to_numpy()
    info = {}
    for r in cl.ROOMS:
        low = day[f"climate.{r}_aircon.target_temp_low"].ffill().to_numpy(float)
        am = (hh >= 6.0) & (hh < 11.0)
        day_level = float(np.nanmax(low[am]))
        reach = np.flatnonzero((low >= day_level - 1e-9) & (hh >= 4.0) & (hh < 12.0))
        rise_end = float(hh[reach[0]]) if len(reach) else 7.5
        night_level = float(np.nanmin(low[(hh >= 1.0) & (hh < 5.0)]))
        info[r] = {"day_level": day_level, "night_level": night_level,
                   "rise_end": rise_end}
    return hh, info


def coarse_costs(day: pd.DataFrame, pre: pd.DataFrame, fc_cal: dict,
                 deadline_hh: float, t_bulk0: float, t_day: float,
                 candidates: np.ndarray) -> dict[float, float]:
    """Expected cost of each candidate warmup start, per the 00:30 vintage."""
    hvac = Hvac()
    dec_utc = day.index[0] + pd.Timedelta(hours=DECISION_HH)
    vintages = pre[pre.run_time <= dec_utc]
    run = vintages.run_time.max()
    fc = vintages[vintages.run_time == run].set_index("time").rrp.sort_index()
    fc_retail = pd.Series(
        np.interp(retailize(fc.to_numpy(float), fc.index),
                  fc_cal["knots_fc_retail"], fc_cal["knots_e_actual_retail"]),
        index=fc.index)
    tout = day["temperature_adelaide"].ffill()

    local0 = day.index[0]

    def price_at(hh_local: float) -> float:
        ts = local0 + pd.Timedelta(hours=hh_local)
        idx = fc_retail.index.searchsorted(ts, side="right") - 1
        return float(fc_retail.iloc[max(0, min(idx, len(fc_retail) - 1))])

    def tout_at(hh_local: float) -> float:
        ts = local0 + pd.Timedelta(hours=hh_local)
        idx = tout.index.searchsorted(ts, side="right") - 1
        return float(tout.iloc[max(0, idx)])

    p_max = hvac.power_kw(16)
    c_eff = hvac.params.c_eff_kwh_per_k

    def warm_rate(temp: float, to: float) -> float:
        """Net house-bulk warm rate at capacity [K/h]: efficiency() * P is
        already the delivered house rate (sim.hvac house_q), minus the
        envelope loss UA*dT/C_eff."""
        return hvac.efficiency(p_max, to) * p_max - UA_KW_PER_K * (temp - to) / c_eff

    def hold_power(temp: float, to: float) -> float:
        """Maintenance electrical power [kW]: e(p)*p = UA*dT/C_eff."""
        q_rate = UA_KW_PER_K * max(temp - to, 0.0) / c_eff
        p = q_rate / max(hvac.efficiency(1.0, to), 0.1)
        return q_rate / max(hvac.efficiency(p, to), 0.1)

    costs = {}
    for t_s in candidates:
        cost, t, temp = 0.0, float(t_s), t_bulk0
        while t < deadline_hh + 2.0:  # hold a little past deadline
            to = tout_at(t)
            if temp < t_day:  # warm phase at capacity
                p_kw = p_max
                temp += max(warm_rate(temp, to), 0.05) * 0.5
            else:  # hold phase: maintenance
                p_kw = hold_power(temp, to)
            cost += p_kw * 0.5 * price_at(t)
            t += 0.5
            if t >= deadline_hh and temp < t_day:
                # did not reach the day target by the deadline: infeasible
                cost = float("inf")
                break
        costs[float(t_s)] = cost
    return costs


def shifted_day(day: pd.DataFrame, shift_h: float) -> pd.DataFrame:
    """Pull the pre-noon target ramp earlier by shift_h (targets only)."""
    if shift_h <= 0:
        return day
    out = day.copy()
    local = day.index.tz_convert(LOCAL_TZ)
    hh = (local.hour + local.minute / 60.0).to_numpy()
    n_shift = int(round(shift_h * 60))
    morning = hh < 12.0
    for r in cl.ROOMS:
        for col in (f"climate.{r}_aircon.target_temp_low",
                    f"climate.{r}_aircon.target_temp_high"):
            v = day[col].ffill().to_numpy(float)
            shifted = np.roll(v, -n_shift)
            shifted[-n_shift:] = v[-1]
            nv = v.copy()
            # earlier ramp = max(original, time-shifted) in the morning so
            # the night floor is never lowered, only the rise moved forward
            nv[morning] = np.maximum(v[morning], shifted[morning])
            out[col] = nv
    return out


def run_arm(day: pd.DataFrame, ctrl: pd.DataFrame, prices: pd.DataFrame) -> dict:
    sim = replay(ctrl)
    price = prices["general_price"].reindex(day.index, method="ffill").to_numpy()
    p_kw = np.asarray(sim["p_kw"], float)
    m = score_day(build_comfort_frame(day, sim))
    return {"cost": float((p_kw / 60.0 * price).sum()),
            "kwh": float(p_kw.sum() / 60.0),
            "deg_min_below": m["deg_min_below"],
            "time_in_band": m["time_in_band"], "starts": m["starts"]}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dates", default="2026-06-22,2026-06-23,2026-06-24,2026-06-09")
    ap.add_argument("--k", type=float, default=2.0)
    ap.add_argument("--tau", type=float, default=20.0)
    ap.add_argument("--parquet", default=_ROOT / "data/processed/june.parquet", type=Path)
    ap.add_argument("--cache", default=_ROOT / "data/prices_sa1.parquet", type=Path)
    ap.add_argument("--fc-cal", default=_ROOT / "analysis/out/fc_calibration.json",
                    type=Path)
    args = ap.parse_args()
    fc_cal = json.loads(args.fc_cal.read_text())

    print(f"{'arm':>18}{'date':>12}{'cost$':>8}{'kwh':>7}{'degmin_blw':>11}"
          f"{'in_band':>8}{'starts':>7}{'note':>22}", flush=True)
    for date in args.dates.split(","):
        day = load_day(args.parquet, date)
        prices = load_prices(date, date, args.cache)
        pre = load_predispatch(day.index[0], day.index[-1])
        hh, info = morning_schedule(day)

        # whole-house coarse decision over rooms with a REAL morning rise
        # (study never heats; flat schedules would corrupt the deadline)
        rising = {r: i for r, i in info.items()
                  if i["day_level"] > i["night_level"] + 0.5}
        if not rising:
            print(f"{'(no morning rise)':>18}{date:>12}", flush=True)
            continue
        deadline = float(np.median([i["rise_end"] for i in rising.values()]))
        t_day = float(np.mean([i["day_level"] for i in rising.values()]))
        # bulk start state ~ mean recorded room temp at the decision time
        temps0 = np.mean([[day[f"{r}_average_temperature"].astype(float).ffill()
                           .iloc[int(DECISION_HH * 60)]] for r in cl.ROOMS])
        candidates = np.arange(max(DECISION_HH + 0.5, deadline - 8.0),
                               deadline - 0.49, 0.5)
        costs = coarse_costs(day, pre, fc_cal, deadline, float(temps0), t_day,
                             candidates)
        t_star = min(costs, key=costs.get)
        # default just-in-time start = latest feasible candidate
        feasible = [t for t, c in costs.items() if np.isfinite(c)]
        t_jit = max(feasible) if feasible else t_star
        shift = max(0.0, t_jit - t_star)
        note = f"start {t_star:.1f}h (jit {t_jit:.1f}h)"

        b = run_arm(day, day, prices)
        print(f"{'baseline':>18}{date:>12}{b['cost']:>8.2f}{b['kwh']:>7.2f}"
              f"{b['deg_min_below']:>11.1f}{b['time_in_band']:>8.3f}"
              f"{b['starts']:>7.0f}{'-':>22}", flush=True)

        sd = shifted_day(day, shift)
        m = run_arm(day, sd, prices)
        print(f"{'shift':>18}{date:>12}{m['cost']:>8.2f}{m['kwh']:>7.2f}"
              f"{m['deg_min_below']:>11.1f}{m['time_in_band']:>8.3f}"
              f"{m['starts']:>7.0f}{note:>22}", flush=True)

        off = offset_series(day, prices, pre, args.k, args.tau, 8.0, 1.5, 0.75,
                            fc_cal)
        combo = sd.copy()
        offv = off.to_numpy()
        for r in cl.ROOMS:
            low = combo[f"climate.{r}_aircon.target_temp_low"].ffill().to_numpy(float) + offv
            high = combo[f"climate.{r}_aircon.target_temp_high"].ffill().to_numpy(float)
            combo[f"climate.{r}_aircon.target_temp_low"] = low
            combo[f"climate.{r}_aircon.target_temp_high"] = np.maximum(high, low + 0.5)
        m2 = run_arm(day, combo, prices)
        print(f"{'shift+offset':>18}{date:>12}{m2['cost']:>8.2f}{m2['kwh']:>7.2f}"
              f"{m2['deg_min_below']:>11.1f}{m2['time_in_band']:>8.3f}"
              f"{m2['starts']:>7.0f}{'-':>22}", flush=True)


if __name__ == "__main__":
    main()
