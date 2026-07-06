"""Phase B of the price-aware scheduling study (docs/pricing.md):
was the 06-22 spike knowable in time, and how good is a predispatch-based
bank trigger over two winters?

Part 1 -- lead-time curve for 2026-06-22: for every AEMO predispatch run
(vintage), the forecast max SA1 rrp over the morning spike window
(05:30-08:00 local). Answers "at bank-decision time (~02:00-02:30) what
did the planner know?".

Part 2 -- trigger evaluation over the whole predispatch archive
(2025-03-23 ->): each morning, take the latest run at/before 02:00 local,
forecast max rrp over 05:00-09:00 local, trigger if > threshold; compare
with the actual dispatch max over the same window. Confusion counts +
value proxy per event:

    hit value  ~= warmup_kwh * (actual window mean retail - night retail)
    false-alarm cost ~= bank_kwh * night retail   (heat is retained, most
    of the banked energy displaces later heating -- treat 50% as waste)

Both proxies are crude; the point is the ASYMMETRY (misses cost tens of
dollars, false alarms cents), so the trigger can be greedy.

Usage:
    uv run python analysis/spike_forecast.py [--start 2025-03-23]
        [--thresholds 300,500,1000,3000]
"""
from __future__ import annotations

import argparse
import os
import sys
from datetime import date, timedelta
from pathlib import Path

import numpy as np
import pandas as pd

_ROOT = Path(__file__).resolve().parent.parent
for p in (str(_ROOT), str(_ROOT / "tools")):
    if p not in sys.path:
        sys.path.insert(0, p)

import export_history as eh  # noqa: E402

LOCAL_UTC_OFFSET_H = 9.5  # ACST; winter mornings only, no DST in the window


def _influx_args():
    for line in (_ROOT / "tools" / "influx.env").read_text().splitlines():
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            k, v = line.split("=", 1)
            os.environ.setdefault(k, v)
    url = os.environ.get("INFLUX_URL", eh.DEFAULT_INFLUX_URL)
    return (url, os.environ.get("INFLUX_USER", eh.DEFAULT_INFLUX_USER),
            os.environ["INFLUX_PASSWORD"],
            os.environ.get("INFLUX_DB", eh.DEFAULT_INFLUX_DB))


def _series(res):
    r = res["results"][0]
    return r.get("series", [{}])[0] if "series" in r else None


def leadtime_curve(args_influx, day_local: str, w0_h: float, w1_h: float) -> pd.DataFrame:
    """All vintages' forecast max rrp over the local window [w0_h, w1_h)."""
    d = pd.Timestamp(day_local)
    t0 = (d + pd.Timedelta(hours=w0_h - LOCAL_UTC_OFFSET_H)).strftime("%Y-%m-%dT%H:%M:%SZ")
    t1 = (d + pd.Timedelta(hours=w1_h - LOCAL_UTC_OFFSET_H)).strftime("%Y-%m-%dT%H:%M:%SZ")
    q = ("SELECT rrp, run_time FROM \"rp_30m\".\"aemo_predispatch_forecast\" "
         f"WHERE region = 'SA1' AND time >= '{t0}' AND time < '{t1}'")
    s = _series(eh.influx_query(*args_influx, q))
    df = pd.DataFrame(s["values"], columns=s["columns"])
    df["run_time"] = pd.to_datetime(df["run_time"], utc=True)
    curve = df.groupby("run_time").rrp.max().sort_index()
    window_start_utc = pd.Timestamp(t0, tz="UTC")
    out = pd.DataFrame({"fc_max_rrp": curve})
    out["lead_h"] = (window_start_utc - out.index).total_seconds() / 3600.0
    return out


def morning_row(args_influx, d: date) -> dict | None:
    """One morning's decision-time forecast + actual over 05:00-09:00 local."""
    day = pd.Timestamp(d.isoformat())
    t0 = (day + pd.Timedelta(hours=5 - LOCAL_UTC_OFFSET_H)).strftime("%Y-%m-%dT%H:%M:%SZ")
    t1 = (day + pd.Timedelta(hours=9 - LOCAL_UTC_OFFSET_H)).strftime("%Y-%m-%dT%H:%M:%SZ")
    dec = (day + pd.Timedelta(hours=2 - LOCAL_UTC_OFFSET_H)).strftime("%Y-%m-%dT%H:%M:%SZ")
    dec_lo = (day + pd.Timedelta(hours=0.5 - LOCAL_UTC_OFFSET_H)).strftime("%Y-%m-%dT%H:%M:%SZ")

    # run_time is a TAG: InfluxQL has no range operators for tags, so fetch
    # all vintages for the target window and pick the decision-time run
    # client-side.
    q1 = ("SELECT rrp, run_time FROM \"rp_30m\".\"aemo_predispatch_forecast\" "
          f"WHERE region = 'SA1' AND time >= '{t0}' AND time < '{t1}'")
    s1 = _series(eh.influx_query(*args_influx, q1))
    if s1 is None:
        return None
    df = pd.DataFrame(s1["values"], columns=s1["columns"])
    df = df[(df.run_time >= dec_lo) & (df.run_time <= dec)]
    if df.empty:
        return None
    latest = df.run_time.max()
    fc_max = float(df[df.run_time == latest].rrp.max())

    q2 = ("SELECT max(price), mean(price) FROM \"rp_30m\".\"aemo_dispatch_sa1_30m\" "
          f"WHERE time >= '{t0}' AND time < '{t1}'")
    s2 = _series(eh.influx_query(*args_influx, q2))
    if s2 is None:
        return None
    act_max, act_mean = float(s2["values"][0][1]), float(s2["values"][0][2])
    return {"date": d.isoformat(), "fc_max": fc_max,
            "act_max": act_max, "act_mean": act_mean}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", default="2025-03-23")
    ap.add_argument("--end", default="2026-07-05")
    ap.add_argument("--thresholds", default="300,500,1000,3000")
    ap.add_argument("--csv", default=_ROOT / "analysis/out/spike_trigger.csv", type=Path)
    args = ap.parse_args()
    ia = _influx_args()

    print("=== Part 1: 2026-06-22 lead-time curve (fc max rrp over 05:30-08:00) ===")
    curve = leadtime_curve(ia, "2026-06-22", 5.5, 8.0)
    for lead in [40, 30, 24, 18, 12, 8, 6, 4, 3, 2.5, 2, 1]:
        sub = curve[curve.lead_h >= lead]
        if len(sub):
            v = sub.iloc[-1]
            print(f"  lead >= {lead:>4}h: run {sub.index[-1]}  fc_max ${v.fc_max_rrp:8.0f}/MWh")
    print(f"  actual max over window: query dispatch (see Part 2 row for 2026-06-22)")

    print("\n=== Part 2: 02:00-decision trigger vs actual, all mornings ===", flush=True)
    rows = []
    d = date.fromisoformat(args.start)
    end = date.fromisoformat(args.end)
    while d <= end:
        r = morning_row(ia, d)
        if r:
            rows.append(r)
        d += timedelta(days=1)
    df = pd.DataFrame(rows)
    df.to_csv(args.csv, index=False)
    print(f"{len(df)} mornings evaluated -> {args.csv}")

    spike_def = 1000.0  # actual max > $1000/MWh in the window = a real spike morning
    actual_spikes = df.act_max > spike_def
    print(f"actual spike mornings (max > ${spike_def:.0f}/MWh in 05-09): "
          f"{int(actual_spikes.sum())} "
          f"({df.loc[actual_spikes, 'date'].tolist()})")
    print(f"\n{'trigger $/MWh':>14}{'fires':>7}{'hits':>6}{'misses':>8}{'false+':>8}"
          f"{'hit rate':>9}")
    for th in (float(x) for x in args.thresholds.split(",")):
        fires = df.fc_max > th
        hits = int((fires & actual_spikes).sum())
        misses = int((~fires & actual_spikes).sum())
        fp = int((fires & ~actual_spikes).sum())
        rate = hits / max(1, int(actual_spikes.sum()))
        print(f"{th:>14.0f}{int(fires.sum()):>7}{hits:>6}{misses:>8}{fp:>8}"
              f"{rate:>9.2f}")
    # what the false alarms would have cost vs the misses
    print("\nmiss dates by trigger threshold:")
    for th in (float(x) for x in args.thresholds.split(",")):
        missed = df[(df.fc_max <= th) & actual_spikes]
        if len(missed):
            print(f"  ${th:.0f}: {missed[['date', 'fc_max', 'act_max']].to_string(index=False)}")


if __name__ == "__main__":
    main()
