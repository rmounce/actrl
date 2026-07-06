"""Phase A kill-test: what did June's recorded HVAC consumption cost, and
how much of that cost is movable? (docs/ideas.md #6, price-aware scheduling.)

Loads SA1 30-min dispatch prices from InfluxDB (rp_30m.aemo_dispatch_sa1_30m,
written by ~/src/ai-energy-forecast-slop ingest), converts to retail
import/export $/kWh with that repo's own tariff_utils (network loss factor,
ToU network tariff, GST on import leg), and costs the recorded HVAC power
(Shelly: power.outdoor_unit + power.indoor_unit, 1-min archive).

Bounds reported per day and June-wide:
- actual: recorded energy x import price (all-import accounting; PV
  self-consumption credit is a Phase A refinement, so this slightly
  overstates absolute cost but comparisons between schedules are fair)
- flat:   same energy at the June volume-weighted mean price (what cost
  would be if timing didn't matter -- actual minus flat = timing exposure)
- oracle_shift: same DAILY energy packed into that day's cheapest
  half-hours subject to a per-half-hour power cap (max recorded draw) --
  physics-free hard ceiling on rescheduling value
- bucket split: cost share in peak (17:00-20:59) / solar (10:00-15:59) /
  off-peak network-tariff windows

Usage:
    uv run python analysis/price_cost.py [--start 2026-06-01 --end 2026-06-30]
        [--cache data/prices_sa1.parquet]
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np
import pandas as pd

_ROOT = Path(__file__).resolve().parent.parent
_SLOP = Path.home() / "src" / "ai-energy-forecast-slop"
for p in (str(_ROOT), str(_ROOT / "tools"), str(_SLOP)):
    if p not in sys.path:
        sys.path.insert(0, p)

import export_history as eh  # noqa: E402
import tariff_utils as tu  # noqa: E402

LOCAL_TZ = "Australia/Adelaide"


def _influx_args():
    env = _ROOT / "tools" / "influx.env"
    for line in env.read_text().splitlines():
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            k, v = line.split("=", 1)
            os.environ.setdefault(k, v)
    url = os.environ.get("INFLUX_URL", eh.DEFAULT_INFLUX_URL)
    return (url, os.environ.get("INFLUX_USER", eh.DEFAULT_INFLUX_USER),
            os.environ["INFLUX_PASSWORD"],
            os.environ.get("INFLUX_DB", eh.DEFAULT_INFLUX_DB))


def load_prices(start: str, end: str, cache: Path | None) -> pd.DataFrame:
    """30-min retail price frame (general_price/feed_in_price $/kWh), UTC index."""
    if cache and cache.exists():
        df = pd.read_parquet(cache)
        have = df.index.tz_convert(LOCAL_TZ)
        if str(have.min().date()) <= start and str(have.max().date()) >= end:
            return df
    q = ("SELECT price FROM \"rp_30m\".\"aemo_dispatch_sa1_30m\" "
         f"WHERE time >= '{start}' - 1d AND time <= '{end}' + 2d")
    r = eh.influx_query(*_influx_args(), q)
    s = r["results"][0]["series"][0]
    raw = pd.DataFrame(s["values"], columns=s["columns"])
    raw["time"] = pd.to_datetime(raw["time"], utc=True)
    wholesale = raw.set_index("time")["price"].astype(float)

    import json
    profile = json.loads((_SLOP / "tariff_profile.json").read_text())
    frame = tu.tariffed_price_frame_from_wholesale_mwh(
        wholesale,
        timezone=LOCAL_TZ,
        general_tariff_map=profile["general_tariff"],
        feed_in_tariff_map=profile["feed_in_tariff"],
        network_loss_factor=profile["network_loss_factor"],
        gst_rate=1.1,
    )
    if cache:
        cache.parent.mkdir(parents=True, exist_ok=True)
        frame.to_parquet(cache)
    return frame


def hvac_power_kw(parquet: Path, date: str) -> pd.Series:
    """Recorded HVAC electrical power [kW] at 1-min for a local day."""
    from analysis.replay_day import load_day
    day = load_day(parquet, date)
    p = (day["power.outdoor_unit"].clip(lower=0).fillna(0)
         + day["power.indoor_unit"].clip(lower=0).fillna(0)) / 1000.0
    return p


def day_costs(p_kw: pd.Series, prices: pd.DataFrame) -> dict:
    price = prices["general_price"].reindex(
        p_kw.index, method="ffill")  # 30-min ffill onto 1-min grid
    kwh_min = p_kw / 60.0
    actual = float((kwh_min * price).sum())
    kwh = float(kwh_min.sum())

    # oracle shift: pack the day's energy into cheapest half-hours,
    # capped at the day's max observed half-hour draw
    halfhour = kwh_min.groupby(pd.Grouper(freq="30min")).sum()
    ph = prices["general_price"].reindex(halfhour.index, method="ffill")
    cap = float(halfhour.max())
    remaining, cost = kwh, 0.0
    for pr in np.sort(ph.dropna().to_numpy()):
        if remaining <= 0:
            break
        take = min(cap, remaining)
        cost += take * pr
        remaining -= take
    oracle = float(cost)

    local = p_kw.index.tz_convert(LOCAL_TZ)
    hours = local.hour
    peak = (hours >= 17) & (hours < 21)
    solar = (hours >= 10) & (hours < 16)
    return {
        "kwh": kwh,
        "actual": actual,
        "oracle_shift": oracle,
        "peak_cost": float((kwh_min * price)[peak].sum()),
        "solar_cost": float((kwh_min * price)[solar].sum()),
        "peak_kwh": float(kwh_min[peak].sum()),
        "mean_price": actual / kwh if kwh > 0 else float("nan"),
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", default="2026-06-01")
    ap.add_argument("--end", default="2026-06-30")
    ap.add_argument("--parquet", default=_ROOT / "data/processed/june.parquet", type=Path)
    ap.add_argument("--cache", default=_ROOT / "data/prices_sa1.parquet", type=Path)
    args = ap.parse_args()

    prices = load_prices(args.start, args.end, args.cache)
    vw_mean = None  # volume-weighted mean computed after the day loop

    rows = {}
    dates = pd.date_range(args.start, args.end, freq="D").strftime("%Y-%m-%d")
    for date in dates:
        try:
            p = hvac_power_kw(args.parquet, date)
        except (SystemExit, Exception):  # noqa: BLE001 archive gaps
            continue
        rows[date] = day_costs(p, prices)
    df = pd.DataFrame.from_dict(rows, orient="index")

    total_kwh = df.kwh.sum()
    vw_mean = (df.actual.sum() / total_kwh) if total_kwh else float("nan")
    flat_price = float(prices["general_price"].mean())
    print(f"{len(df)} days costed, {total_kwh:.1f} kWh HVAC")
    print(f"actual cost           ${df.actual.sum():7.2f}  "
          f"(volume-weighted {100*vw_mean:.1f} c/kWh)")
    print(f"flat-price equivalent ${total_kwh * flat_price:7.2f}  "
          f"(time-avg import price {100*flat_price:.1f} c/kWh)")
    print(f"oracle-shift bound    ${df.oracle_shift.sum():7.2f}  "
          f"(same daily kWh, cheapest half-hours, capped at max draw)")
    print(f"peak-window (17-21) cost ${df.peak_cost.sum():.2f} "
          f"({100*df.peak_cost.sum()/df.actual.sum():.0f}% of cost, "
          f"{df.peak_kwh.sum():.1f} kWh = {100*df.peak_kwh.sum()/total_kwh:.0f}% of energy)")
    print(f"solar-window (10-16) cost ${df.solar_cost.sum():.2f} "
          f"({100*df.solar_cost.sum()/df.actual.sum():.0f}% of cost)")
    print("\nper-day ($):")
    df["save_ceiling"] = df.actual - df.oracle_shift
    print(df[["kwh", "actual", "oracle_shift", "save_ceiling", "mean_price"]]
          .round(2).to_string())
    out = _ROOT / "analysis/out/price_cost_june.csv"
    out.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(out)
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
