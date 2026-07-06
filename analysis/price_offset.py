"""Continuous price-pressure target offset (Phase C design, no thresholds).

Ryan's steer: no hard-coded trigger thresholds. Single continuous rule,
mirroring the existing grid-surplus offset design (continuous K offset +
clamps, one gain as the preference knob):

    offset(t) = k * ( max_{0<h<=H} p_hat(t+h|t) * exp(-h/tau) - p(t) )
    clamped to [-shave_max, +bank_max]

- p(t): retail import price now ($/kWh, tariffed).
- p_hat(t+h|t): retail price forecast for t+h using the latest AEMO
  predispatch run available AT time t (vintage -- no hindsight).
- exp(-h/tau): banked-heat retention; tau is physics (a banked degree
  decays at the house's free-running loss rate), not a tuning constant.
- k [K per $/kWh]: the comfort-vs-money exchange rate. The ONLY
  preference parameter.

Positive offset = pre-heat (future dearer than now, discounted by
retention); negative = shave (now is the expensive hour). Flat prices
give ~0. All of Phase B's discrete machinery (decision times, windows,
$300/$1 thresholds, fixed 1K/0.5K) emerges from the price curve.

Scored like preheat_price.py: full sim, cost = sim p_kw x actual retail
price, comfort vs ORIGINAL recorded targets.

Usage:
    uv run python analysis/price_offset.py --dates 2026-06-22,2026-06-24,2026-06-09
        [--ks 1,2,4] [--taus 1.5,2.5,4.0] [--horizon 8]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

_ROOT = Path(__file__).resolve().parent.parent
_SLOP = Path.home() / "src" / "ai-energy-forecast-slop"
for p in (str(_ROOT), str(_ROOT / "tests"), str(_ROOT / "tools"), str(_SLOP)):
    if p not in sys.path:
        sys.path.insert(0, p)

import sim.closed_loop as cl  # noqa: E402  (hassapi stub)
import export_history as eh  # noqa: E402
import tariff_utils as tu  # noqa: E402
from analysis.comfort import score_day  # noqa: E402
from analysis.price_cost import load_prices, _influx_args  # noqa: E402
from analysis.replay_day import load_day, replay  # noqa: E402
from analysis.tune import build_comfort_frame  # noqa: E402

LOCAL_TZ = "Australia/Adelaide"


def load_predispatch(day_utc0: pd.Timestamp, day_utc1: pd.Timestamp) -> pd.DataFrame:
    """All SA1 predispatch rows whose targets fall in [day0-?, day1+H]."""
    t0 = (day_utc0 - pd.Timedelta("1h")).strftime("%Y-%m-%dT%H:%M:%SZ")
    t1 = (day_utc1 + pd.Timedelta("9h")).strftime("%Y-%m-%dT%H:%M:%SZ")
    q = ("SELECT rrp, run_time FROM \"rp_30m\".\"aemo_predispatch_forecast\" "
         f"WHERE region = 'SA1' AND time >= '{t0}' AND time < '{t1}'")
    r = eh.influx_query(*_influx_args(), q)
    s = r["results"][0]["series"][0]
    df = pd.DataFrame(s["values"], columns=s["columns"])
    df["time"] = pd.to_datetime(df["time"], utc=True)
    df["run_time"] = pd.to_datetime(df["run_time"], utc=True)
    return df


def retailize(rrp_mwh: np.ndarray, when: pd.DatetimeIndex) -> np.ndarray:
    """Wholesale $/MWh -> retail import $/kWh via the tariff profile."""
    profile = json.loads((_SLOP / "tariff_profile.json").read_text())
    frame = tu.tariffed_price_frame_from_wholesale_mwh(
        pd.Series(rrp_mwh, index=when),
        timezone=LOCAL_TZ,
        general_tariff_map=profile["general_tariff"],
        feed_in_tariff_map=profile["feed_in_tariff"],
        network_loss_factor=profile["network_loss_factor"],
        gst_rate=1.1,
    )
    return frame["general_price"].to_numpy()


def offset_series(day: pd.DataFrame, prices: pd.DataFrame, pre: pd.DataFrame,
                  k: float, tau_h: float, horizon_h: float,
                  bank_max: float, shave_max: float,
                  fc_cal: dict | None = None) -> pd.Series:
    """Per-minute offset [K] from vintage forecasts. Updated each 30 min.

    fc_cal (analysis/fc_calibration.py output) maps forecast retail price
    through E[actual | forecast] before valuation -- restores the fat-tail
    expected value that raw predispatch magnitude understates (Phase B).
    """
    p_now = prices["general_price"].reindex(day.index, method="ffill")
    halfhours = pd.date_range(day.index[0].floor("30min"), day.index[-1],
                              freq="30min", tz="UTC")
    off_hh = {}
    for t in halfhours:
        vintages = pre[pre.run_time <= t]
        if vintages.empty:
            off_hh[t] = 0.0
            continue
        run = vintages.run_time.max()
        fc = vintages[vintages.run_time == run].set_index("time").rrp.sort_index()
        fut = fc[(fc.index > t) & (fc.index <= t + pd.Timedelta(hours=horizon_h))]
        if fut.empty:
            off_hh[t] = 0.0
            continue
        fut_retail = retailize(fut.to_numpy(float), fut.index)
        if fc_cal is not None:
            fut_retail = np.interp(fut_retail,
                                   fc_cal["knots_fc_retail"],
                                   fc_cal["knots_e_actual_retail"])
        h = (fut.index - t).total_seconds() / 3600.0
        best_fut = float(np.max(fut_retail * np.exp(-h / tau_h)))
        now = float(p_now.reindex([t], method="ffill").iloc[0])
        off_hh[t] = float(np.clip(k * (best_fut - now), -shave_max, bank_max))
    hh = pd.Series(off_hh)
    return hh.reindex(day.index, method="ffill").fillna(0.0)


def run_arm(day: pd.DataFrame, offsets: pd.Series | None,
            prices: pd.DataFrame) -> dict:
    ctrl = day
    if offsets is not None:
        ctrl = day.copy()
        off = offsets.to_numpy()
        for r in cl.ROOMS:
            low = day[f"climate.{r}_aircon.target_temp_low"].ffill().to_numpy(float) + off
            high = day[f"climate.{r}_aircon.target_temp_high"].ffill().to_numpy(float)
            ctrl[f"climate.{r}_aircon.target_temp_low"] = low
            ctrl[f"climate.{r}_aircon.target_temp_high"] = np.maximum(high, low + 0.5)
    sim = replay(ctrl)
    price = prices["general_price"].reindex(day.index, method="ffill").to_numpy()
    p_kw = np.asarray(sim["p_kw"], float)
    m = score_day(build_comfort_frame(day, sim))  # ORIGINAL targets
    return {"cost": float((p_kw / 60.0 * price).sum()),
            "kwh": float(p_kw.sum() / 60.0),
            "deg_min_below": m["deg_min_below"],
            "time_in_band": m["time_in_band"], "starts": m["starts"]}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dates", default="2026-06-22,2026-06-24,2026-06-09")
    ap.add_argument("--ks", default="1,2,4")
    ap.add_argument("--taus", default="2.5")
    ap.add_argument("--horizon", type=float, default=8.0)
    ap.add_argument("--bank-max", type=float, default=1.5)
    ap.add_argument("--shave-max", type=float, default=0.75)
    ap.add_argument("--parquet", default=_ROOT / "data/processed/june.parquet", type=Path)
    ap.add_argument("--cache", default=_ROOT / "data/prices_sa1.parquet", type=Path)
    ap.add_argument("--fc-cal", default=None, type=Path,
                    help="fc_calibration.json to value forecasts at E[actual|fc]")
    args = ap.parse_args()
    fc_cal = json.loads(args.fc_cal.read_text()) if args.fc_cal else None

    print(f"{'arm':>16}{'date':>12}{'cost$':>8}{'kwh':>7}{'degmin_blw':>11}"
          f"{'in_band':>8}{'starts':>7}{'off_range':>16}", flush=True)
    for date in args.dates.split(","):
        day = load_day(args.parquet, date)
        prices = load_prices(date, date, args.cache)
        pre = load_predispatch(day.index[0], day.index[-1])
        b = run_arm(day, None, prices)
        print(f"{'baseline':>16}{date:>12}{b['cost']:>8.2f}{b['kwh']:>7.2f}"
              f"{b['deg_min_below']:>11.1f}{b['time_in_band']:>8.3f}"
              f"{b['starts']:>7.0f}{'-':>16}", flush=True)
        for tau in (float(x) for x in args.taus.split(",")):
            for k in (float(x) for x in args.ks.split(",")):
                off = offset_series(day, prices, pre, k, tau, args.horizon,
                                    args.bank_max, args.shave_max, fc_cal)
                m = run_arm(day, off, prices)
                rng = f"[{off.min():+.2f},{off.max():+.2f}]"
                print(f"{f'k={k} tau={tau}':>16}{date:>12}{m['cost']:>8.2f}"
                      f"{m['kwh']:>7.2f}{m['deg_min_below']:>11.1f}"
                      f"{m['time_in_band']:>8.3f}{m['starts']:>7.0f}{rng:>16}",
                      flush=True)


if __name__ == "__main__":
    main()
