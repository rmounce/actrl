"""Forecast-price calibration g(fc) = E[actual | forecast] (Phase C).

The continuous price-offset rule (analysis/price_offset.py) under-banks
because predispatch systematically understates spike magnitude (Phase B:
$500/MWh forecast max vs $20.3k actual). The fix that preserves the
no-thresholds design: value future half-hours at their CONDITIONAL MEAN
actual price given the forecast, fitted from the predispatch archive.
The fat tail (a $300 forecast occasionally realising $16/kWh) makes
g convex from data alone -- the "insurance" behaviour of Phase B's
discrete trigger emerges with no trigger.

Pairs: for every half-hour target since --start, the predispatch vintage
closest to --lead hours before the target (the horizon at which the
bank decision actually looks), paired with the actual dispatch price.
Both sides converted to retail $/kWh (the offset rule's units). Fit =
mean actual per forecast bin (log-spaced), monotone-enforced, saved as
interp knots.

Validation: fit on pre-2026 data only, report June 2026 performance of
the same bins.

Usage:
    uv run python analysis/fc_calibration.py [--start 2025-03-23] [--lead 3]
        [--out analysis/out/fc_calibration.json]
"""
from __future__ import annotations

import argparse
import json
import sys
from datetime import date, timedelta
from pathlib import Path

import numpy as np
import pandas as pd

_ROOT = Path(__file__).resolve().parent.parent
_SLOP = Path.home() / "src" / "ai-energy-forecast-slop"
for p in (str(_ROOT), str(_ROOT / "tools"), str(_SLOP)):
    if p not in sys.path:
        sys.path.insert(0, p)

import export_history as eh  # noqa: E402
from analysis.price_cost import _influx_args  # noqa: E402
from analysis.price_offset import retailize  # noqa: E402


def day_pairs(ia, d: date, lead_h: float) -> list[dict]:
    """(fc, actual) wholesale pairs for one UTC day at ~lead_h vintage."""
    t0 = f"{d.isoformat()}T00:00:00Z"
    t1 = f"{(d + timedelta(days=1)).isoformat()}T00:00:00Z"
    q1 = ("SELECT rrp, run_time FROM \"rp_30m\".\"aemo_predispatch_forecast\" "
          f"WHERE region = 'SA1' AND time >= '{t0}' AND time < '{t1}'")
    r1 = eh.influx_query(*ia, q1)
    res = r1["results"][0]
    if "series" not in res:
        return []
    fc = pd.DataFrame(res["series"][0]["values"], columns=res["series"][0]["columns"])
    fc["time"] = pd.to_datetime(fc["time"], utc=True)
    fc["run_time"] = pd.to_datetime(fc["run_time"], utc=True)
    fc["lead"] = (fc.time - fc.run_time).dt.total_seconds() / 3600.0

    q2 = ("SELECT price FROM \"rp_30m\".\"aemo_dispatch_sa1_30m\" "
          f"WHERE time >= '{t0}' AND time < '{t1}'")
    r2 = eh.influx_query(*ia, q2)
    res2 = r2["results"][0]
    if "series" not in res2:
        return []
    act = pd.DataFrame(res2["series"][0]["values"], columns=res2["series"][0]["columns"])
    act["time"] = pd.to_datetime(act["time"], utc=True)
    act = act.set_index("time").price

    out = []
    for t, g in fc.groupby("time"):
        if t not in act.index:
            continue
        g = g[(g.lead >= lead_h - 1.0) & (g.lead <= lead_h + 1.0)]
        if g.empty:
            continue
        row = g.iloc[(g.lead - lead_h).abs().argmin()]
        out.append({"time": t, "fc": float(row.rrp), "actual": float(act[t])})
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", default="2025-03-23")
    ap.add_argument("--end", default="2026-07-05")
    ap.add_argument("--lead", type=float, default=3.0)
    ap.add_argument("--out", default=_ROOT / "analysis/out/fc_calibration.json",
                    type=Path)
    ap.add_argument("--fit-end", default="2026-06-01",
                    help="fit/holdout split; set past today to fit on ALL data "
                    "(insurance-premium mode -- see docs/pricing.md)")
    ap.add_argument("--hours", default=None,
                    help="local-hour window 'a-b' to condition on (e.g. 5-9)")
    ap.add_argument("--months", default=None,
                    help="comma-separated months to condition on (e.g. 5,6,7,8)")
    args = ap.parse_args()
    ia = _influx_args()

    rows = []
    d = date.fromisoformat(args.start)
    while d <= date.fromisoformat(args.end):
        rows.extend(day_pairs(ia, d, args.lead))
        d += timedelta(days=1)
    df = pd.DataFrame(rows)
    # DST changeover days make the local-time tariff mapping ambiguous;
    # drop them (4 calendar days/year, negligible)
    df = df[~df.time.dt.strftime("%m-%d").isin(["04-05", "04-06", "10-04", "10-05"])]
    local = df.time.dt.tz_convert("Australia/Adelaide")
    if args.hours:
        a, b = (float(x) for x in args.hours.split("-"))
        lh = local.dt.hour + local.dt.minute / 60.0
        df = df[(lh >= a) & (lh < b)]
    if args.months:
        df = df[local.dt.month.isin([int(m) for m in args.months.split(",")])]
    print(f"{len(df)} (forecast, actual) pairs at ~{args.lead}h lead")

    # retail both sides (ToU adder is deterministic per time-of-day, so it
    # rides along identically on fc and actual)
    when = pd.DatetimeIndex(df.time)
    df["fc_r"] = retailize(df.fc.to_numpy(float), when)
    df["act_r"] = retailize(df.actual.to_numpy(float), when)

    fit = df[df.time < pd.Timestamp(args.fit_end, tz="UTC")]
    hold = df[df.time >= pd.Timestamp(args.fit_end, tz="UTC")]

    edges = np.array([-np.inf, 0.15, 0.20, 0.25, 0.30, 0.40, 0.55, 0.80,
                      1.20, np.inf])
    knots_x, knots_y, knots_up = [], [], []
    print(f"\n{'fc bin $/kWh':>18}{'n_fit':>7}{'E[act|fc] fit':>14}"
          f"{'E[(act-fc)+]':>13}{'n_jun':>7}{'E[act|fc] jun':>14}")
    for lo, hi in zip(edges[:-1], edges[1:]):
        m_fit = fit[(fit.fc_r > lo) & (fit.fc_r <= hi)]
        m_hold = hold[(hold.fc_r > lo) & (hold.fc_r <= hi)]
        if len(m_fit) < 20:
            continue
        x = float(m_fit.fc_r.mean())
        y = float(m_fit.act_r.mean())
        # upside surprise: the insurance term. Pools ALL positive residuals
        # in the bin, so it is far stabler than the mean (docs/pricing.md
        # Phase C fork).
        up = float((m_fit.act_r - m_fit.fc_r).clip(lower=0).mean())
        knots_x.append(x)
        knots_y.append(y)
        knots_up.append(up)
        jn = f"{m_hold.act_r.mean():>14.3f}" if len(m_hold) else f"{'-':>14}"
        print(f"({lo:>6.2f},{hi:>6.2f}]{len(m_fit):>7}{y:>14.3f}"
              f"{up:>13.3f}{len(m_hold):>7}{jn}")

    # enforce monotone non-decreasing knots (isotonic-lite)
    for i in range(1, len(knots_y)):
        knots_y[i] = max(knots_y[i], knots_y[i - 1])

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(
        {"lead_h": args.lead, "fit_end": args.fit_end,
         "conditioning": {"hours": args.hours, "months": args.months},
         "knots_fc_retail": knots_x, "knots_e_actual_retail": knots_y,
         "knots_upside_retail": knots_up},
        indent=1))
    print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
