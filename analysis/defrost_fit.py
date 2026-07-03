"""Characterise defrost episodes for the simulator's defrost model
(docs/ideas.md #3, "defrost emulation" — the last un-modelled piece flagged
in docs/calibration.md's whole-day replay results).

Detection (docs/calibration.md "Defrost", 2026-07-02): actrl forces
`aircon_comp_speed` to max during defrost (compressor on, outdoor fan off),
so `comp_speed >= 15` while `power.outdoor_unit > 300 W` labels defrost.
This script finds each raw episode (the forced-max signal itself, not the
padded overhead window `sysid_june.defrost_mask` uses for energy
accounting) and reports:

- Duration of the forced-max segment itself (the actual defrost).
- Recovery duration/energy: elevated power after the segment ends, until
  it drops back to the pre-defrost running level.
- Trigger conditions: outdoor temp, and cumulative compressor-on time at
  cold outdoor temp since the previous episode (the frost-accumulation
  proxy the model needs for triggering).
- Power/indoor-coil-temp shape during the episode, for the simulator's
  power/Q profile during defrost.

Usage:
    uv run python analysis/defrost_fit.py [--parquet data/processed/june.parquet]
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

COLD_THRESHOLD_C = 7.5  # docs/calibration.md: defrost episodes were 3.9-7.1 C
RECOVERY_MAX = pd.Timedelta("30min")  # search window after the segment ends
RUNNING_W = 300.0  # power.outdoor_unit threshold for "compressor running"


def raw_episodes(df: pd.DataFrame) -> list[tuple[pd.Timestamp, pd.Timestamp]]:
    """Contiguous (start, end) runs of the forced-max defrost signal."""
    sig = ((df["aircon_comp_speed"] >= 15) & (df["power.outdoor_unit"] > RUNNING_W)).fillna(False)
    grp = (sig != sig.shift()).cumsum()
    out = []
    for _, seg in df.index.to_series().groupby(grp):
        if sig.loc[seg.iloc[0]]:
            out.append((seg.iloc[0], seg.iloc[-1]))
    return out


def prior_cold_run_minutes(df: pd.DataFrame, start: pd.Timestamp, prev_end: pd.Timestamp | None) -> float:
    """Minutes of compressor-on time at outdoor temp < COLD_THRESHOLD_C
    since the previous episode ended (or since the compressor last turned
    off before `start` if this is the first episode of a run)."""
    running = df["power.outdoor_unit"] > RUNNING_W
    cold = df["temperature_adelaide"] < COLD_THRESHOLD_C
    window_start = prev_end if prev_end is not None else start - pd.Timedelta("6h")
    seg = (running & cold)[(df.index > window_start) & (df.index < start)]
    return float(seg.sum())  # 1-min grid -> minutes


def recovery_stats(df: pd.DataFrame, end: pd.Timestamp, baseline_w: float) -> tuple[float, float]:
    """(recovery_minutes, extra_kwh) — time and energy for outdoor power to
    settle back within 10% of `baseline_w` after the defrost segment ends."""
    seg = df["power.outdoor_unit"][(df.index > end) & (df.index <= end + RECOVERY_MAX)]
    if seg.empty or baseline_w <= 0:
        return 0.0, 0.0
    within = seg <= baseline_w * 1.1
    if not within.any():
        return float(len(seg)), float((seg - baseline_w).clip(lower=0).sum() / 60 / 1000)
    settle_idx = within.idxmax()
    pre_settle = seg[seg.index < settle_idx]
    minutes = float(len(pre_settle))
    extra_kwh = float((pre_settle - baseline_w).clip(lower=0).sum() / 60 / 1000)
    return minutes, extra_kwh


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--parquet", default="data/processed/june.parquet", type=Path)
    args = ap.parse_args()

    df = pd.read_parquet(args.parquet)
    episodes = raw_episodes(df)
    print(f"{len(episodes)} raw defrost episodes\n")

    durations, gaps, temps, powers, coil_mins, recov_mins, recov_kwh = [], [], [], [], [], [], []
    prev_end = None
    for start, end in episodes:
        dur_min = (end - start).total_seconds() / 60 + 1  # inclusive of both samples
        tout = df["temperature_adelaide"][(df.index >= start) & (df.index <= end)].mean()
        p_during = df["power.outdoor_unit"][(df.index >= start) & (df.index <= end)].mean()
        coil_min = df["m5atom_inside_coil_inlet_temp"][(df.index >= start) & (df.index <= end)].min()
        gap_min = prior_cold_run_minutes(df, start, prev_end)

        pre = df["power.outdoor_unit"][(df.index >= start - pd.Timedelta("10min")) & (df.index < start)]
        baseline_w = pre[pre > RUNNING_W].median() if (pre > RUNNING_W).any() else np.nan
        r_min, r_kwh = recovery_stats(df, end, baseline_w) if not np.isnan(baseline_w) else (np.nan, np.nan)

        local = start.tz_convert("Australia/Adelaide")
        print(
            f"  {local}  dur={dur_min:4.0f}min  Tout={tout:5.1f}C  "
            f"P={p_during:5.0f}W  coil_min={coil_min:5.1f}C  "
            f"prior_cold_run={gap_min:5.0f}min  recovery={r_min:4.0f}min/{r_kwh:.2f}kWh"
        )
        durations.append(dur_min)
        temps.append(tout)
        powers.append(p_during)
        coil_mins.append(coil_min)
        if gap_min > 0:
            gaps.append(gap_min)
        if not np.isnan(r_min):
            recov_mins.append(r_min)
            recov_kwh.append(r_kwh)
        prev_end = end

    def stats(label, arr):
        a = np.array(arr)
        print(f"{label}: n={len(a)} median={np.median(a):.1f} IQR=[{np.percentile(a,25):.1f}, {np.percentile(a,75):.1f}]")

    print()
    stats("Duration (min)", durations)
    stats("Outdoor temp during (C)", temps)
    stats("Power during (W)", powers)
    stats("Min indoor coil inlet temp (C)", coil_mins)
    stats("Prior cold compressor-on run before trigger (min)", gaps)
    stats("Recovery duration (min)", recov_mins)
    stats("Recovery extra energy (kWh)", recov_kwh)


if __name__ == "__main__":
    main()
