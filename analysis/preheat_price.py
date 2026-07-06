"""Phase A of the price-aware scheduling study (docs/ideas.md #6):
hindsight-oracle pre-heat strategies scored in the full sim against real
prices.

Strategy family (statctrl-shaped: only the target trajectories fed to the
controller change; production logic untouched):

- bank: from --bank-start until the spike start, every room's heat target
  is raised to at least (its recorded 09:00 day-level + bank_k) -- an
  early warm-up that banks heat in the cheap pre-spike hours.
- coast/shave: during [spike_start, spike_end] targets revert to the
  recorded schedule minus shave_k -- shave_k = 0 just coasts on banked
  heat (unit re-runs only if temps fall to the recorded targets);
  shave_k > 0 tolerates deeper droop before burning spike-priced energy.

Comfort is scored against the ORIGINAL recorded targets
(tune.build_comfort_frame with the unmodified day), so pre-heating can
only help comfort and any real droop below the family's intended band is
charged to the strategy. Cost = sim p_kw x retail import price
(analysis/price_cost.load_prices). Baseline = recorded targets through
the same sim (apples-to-apples; sim June energy error is -2.5% median).

The spike window defaults to auto-detection: half-hours whose price
exceeds 3x the day median, padded to a contiguous window.

Usage:
    uv run python analysis/preheat_price.py --date 2026-06-22
        [--banks 0.5,1.0] [--bank-starts 2.5,4.0] [--shaves 0,0.5]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

_ROOT = Path(__file__).resolve().parent.parent
for p in (str(_ROOT), str(_ROOT / "tests")):
    if p not in sys.path:
        sys.path.insert(0, p)

import sim.closed_loop as cl  # noqa: E402  (hassapi stub)
from analysis.comfort import score_day  # noqa: E402
from analysis.price_cost import load_prices  # noqa: E402
from analysis.replay_day import load_day, replay  # noqa: E402
from analysis.tune import build_comfort_frame  # noqa: E402

LOCAL_TZ = "Australia/Adelaide"


def detect_spike(day: pd.DataFrame, prices: pd.DataFrame) -> tuple[float, float]:
    """Contiguous local-hour window of half-hours priced > 3x day median."""
    pr = prices["general_price"].reindex(day.index, method="ffill")
    local = day.index.tz_convert(LOCAL_TZ)
    hh = local.hour + local.minute / 60.0
    hot = (pr > 3 * float(pr.median())).to_numpy()
    if not hot.any():
        return (float("nan"), float("nan"))
    return (float(hh[hot].min()), float(hh[hot].max()) + 0.5 / 60)


def synth_targets(day: pd.DataFrame, bank_start: float, bank_k: float,
                  shave_k: float, spike: tuple[float, float]) -> pd.DataFrame:
    out = day.copy()
    local = day.index.tz_convert(LOCAL_TZ)
    hh = (local.hour + local.minute / 60.0).to_numpy()
    s0, s1 = spike
    banking = (hh >= bank_start) & (hh < s0)
    shaving = (hh >= s0) & (hh < s1)
    day_level_i = int(np.argmax(hh >= 9.0))
    for r in cl.ROOMS:
        low = day[f"climate.{r}_aircon.target_temp_low"].ffill().to_numpy(float).copy()
        high = day[f"climate.{r}_aircon.target_temp_high"].ffill().to_numpy(float)
        day_level = low[day_level_i]
        low[banking] = np.maximum(low[banking], day_level + bank_k)
        low[shaving] = low[shaving] - shave_k
        out[f"climate.{r}_aircon.target_temp_low"] = low
        # keep the band width; banked lows must not invert the band
        out[f"climate.{r}_aircon.target_temp_high"] = np.maximum(high, low + 0.5)
    return out


def run_arm(day_orig: pd.DataFrame, day_ctrl: pd.DataFrame,
            prices: pd.DataFrame) -> dict:
    sim = replay(day_ctrl)
    price = prices["general_price"].reindex(day_orig.index, method="ffill").to_numpy()
    p_kw = np.asarray(sim["p_kw"], float)
    cost = float((p_kw / 60.0 * price).sum())
    frame = build_comfort_frame(day_orig, sim)  # ORIGINAL targets
    m = score_day(frame)
    return {"cost": cost, "kwh": float(p_kw.sum() / 60.0),
            "deg_min_below": m["deg_min_below"],
            "time_in_band": m["time_in_band"],
            "starts": m["starts"]}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--date", default="2026-06-22")
    ap.add_argument("--banks", default="0.5,1.0")
    ap.add_argument("--bank-starts", default="2.5,4.0")
    ap.add_argument("--shaves", default="0,0.5")
    ap.add_argument("--spike-start", type=float, default=None)
    ap.add_argument("--spike-end", type=float, default=None)
    ap.add_argument("--parquet", default=_ROOT / "data/processed/june.parquet", type=Path)
    ap.add_argument("--cache", default=_ROOT / "data/prices_sa1.parquet", type=Path)
    args = ap.parse_args()

    day = load_day(args.parquet, args.date)
    d0 = pd.Timestamp(args.date).strftime("%Y-%m-%d")
    prices = load_prices(d0, d0, args.cache)

    spike = (args.spike_start, args.spike_end)
    if spike[0] is None:
        spike = detect_spike(day, prices)
    print(f"{args.date}: spike window {spike[0]:.2f}h - {spike[1]:.2f}h local")

    hdr = f"{'arm':>26}{'cost$':>8}{'kwh':>7}{'degmin_blw':>11}{'in_band':>8}{'starts':>7}"
    print(hdr, flush=True)
    base = run_arm(day, day, prices)
    print(f"{'baseline (recorded tgts)':>26}{base['cost']:>8.2f}{base['kwh']:>7.2f}"
          f"{base['deg_min_below']:>11.1f}{base['time_in_band']:>8.3f}"
          f"{base['starts']:>7.0f}", flush=True)

    for bank_start in (float(x) for x in args.bank_starts.split(",")):
        for bank_k in (float(x) for x in args.banks.split(",")):
            for shave_k in (float(x) for x in args.shaves.split(",")):
                ctrl = synth_targets(day, bank_start, bank_k, shave_k, spike)
                m = run_arm(day, ctrl, prices)
                name = f"bank{bank_k}@{bank_start}h shave{shave_k}"
                print(f"{name:>26}{m['cost']:>8.2f}{m['kwh']:>7.2f}"
                      f"{m['deg_min_below']:>11.1f}{m['time_in_band']:>8.3f}"
                      f"{m['starts']:>7.0f}", flush=True)


if __name__ == "__main__":
    main()
