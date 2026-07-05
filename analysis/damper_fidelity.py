"""Multi-zone damper fidelity pass (room-PID tuning groundwork).

The June-wide scorecard validated cycle texture and energy; this validates
the OTHER controller -- the per-room PIDs whose outputs become damper
positions -- against the recorded weekday double-ramp mornings (bed_1 and
kitchen ramp simultaneously), the data-rich multi-zone regime. Before the
sim can be trusted to retune room_kp/room_ki (currently detuned low after
real-world hunting), it has to reproduce how zones share the supply air.

Per qualifying morning (05:00-11:00 local), recorded vs sim per room:
mean damper position, moves/h (>5 pt steps at 1-min cadence, the hunting
proxy), position-trace RMSE, and the bed_1->kitchen handoff time (first
minute after 06:30 that bed_1 sits below 50% for 15 min straight, having
been >90% earlier -- the morning's authority transfer). Also, per
room-day, the mean simulated-vs-recorded temperature error alongside the
damper-position error: if damper mismatch tracks temp mismatch, the gap is
plant-side (rooms reading cold/warm), not a controller-behaviour
divergence -- which changes what a tuning sweep can be trusted for.

Usage:
    uv run python analysis/damper_fidelity.py [--days ...] [--csv out.csv]
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

from analysis.replay_day import load_day, replay  # noqa: E402
from sim.closed_loop import ROOMS  # noqa: E402

LOCAL_TZ = "Australia/Adelaide"
WIN_LO, WIN_HI = 5, 11  # local hours


def june_weekdays() -> list[str]:
    days = pd.date_range("2026-06-01", "2026-06-30", freq="D")
    return [d.strftime("%Y-%m-%d") for d in days if d.dayofweek < 5]


def double_ramp(day: pd.DataFrame, local) -> bool:
    """Both bed_1 and kitchen step their low target up during 04:00-10:00."""
    morning = (local.hour >= 4) & (local.hour < 10)
    for r in ("bed_1", "kitchen"):
        low = day[f"climate.{r}_aircon.target_temp_low"].ffill()
        if not (low[morning].diff() > 0).any():
            return False
    return True


def handoff_minute(b1: np.ndarray, hours: np.ndarray) -> float:
    """First local hour (>=6.5) where bed_1 stays <50% for 15 consecutive
    minutes, given it exceeded 90% earlier in the window; NaN if never."""
    if np.nanmax(b1) < 90:
        return float("nan")
    below = b1 < 50
    run = 0
    for i in range(len(b1)):
        if hours[i] < 6.5:
            continue
        run = run + 1 if below[i] else 0
        if run >= 15:
            return float(hours[i - 14])
    return float("nan")


def one_day(parquet: Path, date: str, ctrl_noise: dict | None = None) -> list[dict] | None:
    day = load_day(parquet, date)
    local = day.index.tz_convert(LOCAL_TZ)
    if not double_ramp(day, local):
        return None
    if ctrl_noise is not None:
        # same deterministic day-distinct seeding as tune.score_one_day
        ctrl_noise = {**ctrl_noise, "seed": int(date.replace("-", ""))}
    sim = replay(day, ctrl_noise=ctrl_noise)
    win = (local.hour >= WIN_LO) & (local.hour < WIN_HI)
    hours = (local.hour + local.minute / 60.0)[win].to_numpy()
    # Dampers only actuate while the unit runs (they freeze at their last
    # position when it stops), so positions are only commensurable on
    # minutes where BOTH the recorded and the simulated unit are running —
    # otherwise a run-decision timing divergence (the known mild-day
    # residual) reads as hours of stale-damper "error" (06-08: sim stopped
    # 45 min early, froze bed_1 at 100 through its 08:30 setback).
    rec_on = (day["power.outdoor_unit"].clip(lower=0)[win] > 300).to_numpy()
    sim_on = np.asarray(sim["p_kw"], dtype=float)[win] > 0
    both_on = rec_on & sim_on
    n_hours = both_on.sum() / 60.0
    if n_hours < 1.0:
        return None  # not enough overlapping runtime to compare

    rows = []
    handoffs = {}
    for r in ROOMS:
        rec = day[f"damper.{r}"].ffill().astype(float)[win].to_numpy()
        simd = np.asarray(sim[f"damper_{r}"], dtype=float)[win] * 100.0
        if np.isnan(rec[both_on]).all():
            continue  # bed_2/bed_3 have empty-result days (docs/data.md)
        rec_t = day[f"{r}_average_temperature"].astype(float)[win].to_numpy()
        sim_t = np.asarray(sim[f"Tm_{r}"], dtype=float)[win]
        if r in ("bed_1",):
            handoffs["rec"] = handoff_minute(rec, hours)
            handoffs["sim"] = handoff_minute(simd, hours)
        # moves counted on each system's own running minutes (a move while
        # the other is off is still real activity), rates per running hour
        rows.append({
            "date": date, "room": r,
            "rec_mean": np.nanmean(rec[both_on]), "sim_mean": np.nanmean(simd[both_on]),
            "rec_moves_h": np.nansum(np.abs(np.diff(rec)[both_on[1:]]) > 5) / n_hours,
            "sim_moves_h": np.nansum(np.abs(np.diff(simd)[both_on[1:]]) > 5) / n_hours,
            "pos_rmse": float(np.sqrt(np.nanmean((simd[both_on] - rec[both_on]) ** 2))),
            "damper_bias": float(np.nanmean(simd[both_on] - rec[both_on])),
            "temp_bias": float(np.nanmean(sim_t[both_on] - rec_t[both_on])),
            "overlap_h": n_hours,
            # whole-day cycle texture, same for every room row -- lets a
            # plant-side refit (heat-split weights) watch for cycling
            # regressions in the same replay
            "sim_starts": int(np.sum(np.diff((np.asarray(sim["p_kw"], float) > 0
                                              ).astype(int)) == 1)),
            "rec_starts": int(np.sum(np.diff((day["power.outdoor_unit"].clip(lower=0)
                                              .to_numpy() > 300).astype(int)) == 1)),
            "sim_kwh": float(np.asarray(sim["p_kw"], float).sum() / 60.0),
        })
    for row in rows:
        row["handoff_rec"] = handoffs.get("rec", float("nan"))
        row["handoff_sim"] = handoffs.get("sim", float("nan"))
    return rows


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--days", default=None, help="comma-separated; default June weekdays")
    ap.add_argument("--parquet", default=_ROOT / "data/processed/june.parquet", type=Path)
    ap.add_argument("--csv", type=Path, help="write per-room-day rows")
    ap.add_argument("--noise-sigma", type=float, default=None,
                    help="controller-read sensor noise std [K] (measured 10 s "
                    "floor ~0.012, docs/tuning.md); texture check for whether "
                    "noise reproduces the recorded damper jitter")
    ap.add_argument("--noise-tau", type=float, default=15.0,
                    help="noise correlation time [s] (measured 10-20 s)")
    args = ap.parse_args()
    days = args.days.split(",") if args.days else june_weekdays()
    ctrl_noise = ({"sigma": args.noise_sigma, "tau_s": args.noise_tau}
                  if args.noise_sigma else None)

    all_rows: list[dict] = []
    for d in days:
        try:
            rows = one_day(args.parquet, d, ctrl_noise)
        except (SystemExit, Exception) as e:  # noqa: BLE001 -- archive-gap days
            print(f"{d}  skipped: {e}", flush=True)
            continue
        if rows is None:
            print(f"{d}  no double ramp, skipped", flush=True)
            continue
        all_rows.extend(rows)
        b1 = next(r for r in rows if r["room"] == "bed_1")
        print(f"{d}  ok  handoff rec {b1['handoff_rec']:.2f}h sim {b1['handoff_sim']:.2f}h",
              flush=True)

    df = pd.DataFrame(all_rows)
    if args.csv:
        df.to_csv(args.csv, index=False)

    print(f"\n=== per-room medians over {df['date'].nunique()} double-ramp mornings ===")
    print(f"{'room':<9}{'rec_mean':>9}{'sim_mean':>9}{'rec_mv/h':>9}{'sim_mv/h':>9}"
          f"{'rmse':>7}{'dmp_bias':>9}{'tmp_bias':>9}")
    for r in ROOMS:
        g = df[df.room == r]
        if g.empty:
            continue
        print(f"{r:<9}{g.rec_mean.median():>9.1f}{g.sim_mean.median():>9.1f}"
              f"{g.rec_moves_h.median():>9.2f}{g.sim_moves_h.median():>9.2f}"
              f"{g.pos_rmse.median():>7.1f}{g.damper_bias.median():>+9.1f}"
              f"{g.temp_bias.median():>+9.2f}")

    ho = df[df.room == "bed_1"][["date", "handoff_rec", "handoff_sim"]].dropna()
    if len(ho):
        diff = ho.handoff_sim - ho.handoff_rec
        print(f"\nbed_1->kitchen handoff (n={len(ho)}): "
              f"sim - rec median {diff.median():+.2f} h, IQR "
              f"[{diff.quantile(0.25):+.2f}, {diff.quantile(0.75):+.2f}]")

    # Is damper mismatch downstream of temp mismatch? Correlate per room-day.
    sub = df.dropna(subset=["damper_bias", "temp_bias"])
    if len(sub) > 3:
        c = np.corrcoef(sub.temp_bias, sub.damper_bias)[0, 1]
        print(f"\ncorr(temp_bias, damper_bias) over {len(sub)} room-days: {c:+.2f}"
              "\n(strongly negative => sim opens dampers where its rooms read cold"
              " => plant-side gap, controller behaving consistently)")


if __name__ == "__main__":
    main()
