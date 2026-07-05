"""Refit the kitchen lead against cold-morning warmup onsets (lead_q_h).

docs/calibration.md "kitchen cold-morning warmup onset": the recorded
kitchen sensor runs 0.3-1 K/h ahead of the sim during 06:00-07:00 on cold
mornings (gap scales with ramp intensity; ~zero on mild mornings). The
constant kitchen lead (0.45) was fitted on midday MIN-POWER cycling and is
a sharp loop threshold (0.40 flips the 4/4 midday starts match), so the
constant cannot simply be raised. This grids the new q-sensitive term
(sim/house.py lead_eff = lead_h*(1+lead_q_h*q)) for the kitchen only:
at min-power q the multiplier stays ~1 (cycling texture preserved by
construction), at warmup q it boosts the onset.

Scored per candidate, from the same replays:
- onset gap [K/h] = (rec - sim) kitchen rise 06:00-07:00, per day
  (want ~0 on cold mornings AND still ~0 on the mild control day)
- 06-22 cycling criterion: unit starts + on-fraction, midday 10:00-16:00
  and evening 17:00-22:00, sim vs recorded (must not regress: 4/4 midday,
  ~6/7 evening, 26%/26% on-frac)
- kitchen whole-day temp RMSE vs recorded

Usage:
    uv run python analysis/kitchen_onset_refit.py [--ks 0,0.2,0.4,0.6]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from unittest import mock

import numpy as np

_ROOT = Path(__file__).resolve().parent.parent
for p in (str(_ROOT), str(_ROOT / "tests")):
    if p not in sys.path:
        sys.path.insert(0, p)

import sim.house as sh  # noqa: E402
from analysis.replay_day import load_day, replay  # noqa: E402

COLD_DAYS = ["2026-06-22", "2026-06-25", "2026-06-26", "2026-06-29"]
MILD_DAY = "2026-06-08"  # control: gap already ~0, must stay ~0
CYCLE_DAY = "2026-06-22"


def onset_gap(day, sim) -> float:
    rec = day["kitchen_average_temperature"].astype(float).ffill().to_numpy()
    st = np.asarray(sim["Tm_kitchen"], float)
    return float((rec[420] - rec[360]) - (st[420] - st[360]))


def cycle_stats(day, sim) -> dict:
    local = day.index.tz_convert("Australia/Adelaide")
    hh = (local.hour + local.minute / 60.0).to_numpy()
    rec_on = (day["power.outdoor_unit"].clip(lower=0) > 300).to_numpy()
    sim_on = np.asarray(sim["p_kw"], float) > 0.05
    out = {}
    for name, m in (("mid", (hh >= 10) & (hh < 16)), ("eve", (hh >= 17) & (hh < 22))):
        out[f"{name}_starts"] = (
            int(np.sum(np.diff(sim_on[m].astype(int)) == 1)),
            int(np.sum(np.diff(rec_on[m].astype(int)) == 1)),
        )
        out[f"{name}_onfrac"] = (float(sim_on[m].mean()), float(rec_on[m].mean()))
    return out


def kit_rmse(day, sim) -> float:
    rec = day["kitchen_average_temperature"].astype(float).ffill().to_numpy()
    st = np.asarray(sim["Tm_kitchen"], float)
    return float(np.sqrt(np.nanmean((st - rec) ** 2)))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ks", default="0,0.2,0.4,0.6",
                    help="comma list of lead_q:tau_q pairs (bare number = lead_q only)")
    ap.add_argument("--parquet", default=_ROOT / "data/processed/june.parquet", type=Path)
    args = ap.parse_args()
    ks = []
    for tok in args.ks.split(","):
        lq, _, tq = tok.partition(":")
        ks.append((float(lq), float(tq) if tq else 0.0))
    days = {d: load_day(args.parquet, d) for d in COLD_DAYS + [MILD_DAY]}

    print(f"{'lq:tq':>9} | onset gap K/h: " + " ".join(f"{d[5:]:>6}" for d in COLD_DAYS)
          + f" {MILD_DAY[5:]:>6} | 06-22 mid s|r  eve s|r  onfrac | kitRMSE")
    for lq, tq in ks:
        with mock.patch.dict(sh._DEFAULT_LEAD_Q, {"kitchen": lq}), \
                mock.patch.dict(sh._DEFAULT_TAU_Q, {"kitchen": tq}):
            gaps, rmses, cyc = [], [], None
            for d, day in days.items():
                sim = replay(day)
                gaps.append(onset_gap(day, sim))
                rmses.append(kit_rmse(day, sim))
                if d == CYCLE_DAY:
                    cyc = cycle_stats(day, sim)
        ms, es = cyc["mid_starts"], cyc["eve_starts"]
        mo = cyc["mid_onfrac"]
        print(f"{f'{lq}:{tq}':>9} | " + " ".join(f"{g:>+6.2f}" for g in gaps)
              + f" | {ms[0]}|{ms[1]}      {es[0]}|{es[1]}   "
              + f"{mo[0]:.2f}|{mo[1]:.2f} | {np.median(rmses):.3f}", flush=True)


if __name__ == "__main__":
    main()
