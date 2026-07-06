"""Damper position -> airflow curve calibration (contrast-shaping follow-up).

The sim maps damper position linearly to flow share
(sim/closed_loop.py: flows = dampers * AIRFLOW_WEIGHTS). Ryan's 2023
intuition (commit 6fa96b8, "i don't know fluid dynamics but it seems
nonlinear") was that real registers deliver flow concavely in position --
if true, position-space contrast shaping (damper_share_gamma,
docs/tuning.md) delivers less real contrast than the sim predicts, and
the effective flow curve should be modelled as x**beta with beta != 1.

Method: one-step-ahead prediction on RECORDED data, no closed loop.
Steady windows (all dampers stable +-5 pts, outdoor power steady, indoor
fan on to exclude defrost, >=15 min inside a run) give per-room measured
temperature slopes [K/h]. Each window's slope is predicted from the
calibrated house physics (sim RoomParams: outdoor/coupling losses,
baseline gain, orientation solar) plus HVAC heat allocated by

    q_r = scale * e(P, Tout) * P * share_r(beta) / mass_share_r
    share_r(beta) = x_r**beta * w_r / sum_j x_j**beta * w_j

x**beta pins the endpoints (0 and 1 unchanged -- the closed/full states
the June calibration was anchored on) and bends only the mid-range, so
beta is identified purely by the ~13-21% of running minutes with
mid-range dampers. `scale` absorbs any residual level error in the
e-model so it cannot masquerade as curvature. Grid SSE over (beta,
scale); per-room profiles as a consistency check.

beta < 1 = concave (flow arrives early in travel, Ryan's hunch);
beta > 1 = convex. Sensor caveat: the measured node leads the bulk
during q changes, so the first 5 window minutes are dropped and slopes
are fitted on minutes 5..35 of each steady span.

Usage:
    uv run python analysis/damper_flow_fit.py [--month 2026-06]
        [--csv analysis/out/damper_flow_windows.csv]
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

import sim.closed_loop as cl  # noqa: E402  (hassapi stub + weights)
from sim.house import HouseParams  # noqa: E402
from sim.hvac import Hvac  # noqa: E402
from sim.solar import AZ_NE, AZ_NW, vertical_irradiance  # noqa: E402
from analysis.replay_day import load_day  # noqa: E402

ROOMS = cl.ROOMS
MIN_SPAN = 15  # minutes of steady state required
DROP_HEAD = 5  # sensor-lead settling minutes dropped from each span
MAX_FIT = 30  # cap on fitted minutes per span (limits weather drift)


def day_windows(day: pd.DataFrame, params: HouseParams, hvac: Hvac) -> list[dict]:
    n = len(day)
    x = {r: (day[f"damper.{r}"].ffill().fillna(0).to_numpy(float) / 100.0)
         for r in ROOMS}
    tm = {r: day[f"{r}_average_temperature"].astype(float).ffill().to_numpy()
          for r in ROOMS}
    p_out = day["power.outdoor_unit"].clip(lower=0).ffill().fillna(0).to_numpy() / 1000.0
    p_in = day["power.indoor_unit"].clip(lower=0).ffill().fillna(0).to_numpy()
    tout = day["temperature_adelaide"].ffill().to_numpy(float)
    cloud = day["_cloudiness"].to_numpy(float)
    sun_ne = np.array([vertical_irradiance(ts, AZ_NE) * c
                       for ts, c in zip(day.index, cloud)])
    sun_nw = np.array([vertical_irradiance(ts, AZ_NW) * c
                       for ts, c in zip(day.index, cloud)])

    running = p_out > 0.3
    med_p = np.median(p_out[running]) if running.any() else 0.0
    stable = running & (p_in > 50)
    # break spans on damper moves > 5 pts or outdoor power jumps
    brk = np.zeros(n, dtype=bool)
    for r in ROOMS:
        brk[1:] |= np.abs(np.diff(x[r])) > 0.05
    brk[1:] |= np.abs(np.diff(p_out)) > 0.25 * max(med_p, 0.6)
    brk |= ~stable

    windows = []
    i = 0
    while i < n:
        if brk[i] or not stable[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and not brk[j + 1]:
            j += 1
        span = slice(i + DROP_HEAD, min(j + 1, i + DROP_HEAD + MAX_FIT))
        if span.stop - span.start >= MIN_SPAN - DROP_HEAD:
            t_min = np.arange(span.stop - span.start, dtype=float)
            row: dict = {"ts": day.index[span.start], "n_min": len(t_min)}
            # efficiency() is scalar; vectorise manually
            e = np.maximum(
                hvac.params.e_floor,
                hvac.params.e0
                + hvac.params.e_per_kw * (p_out[span] - hvac.params.e_ref_p_kw)
                + hvac.params.e_per_k * (tout[span] - hvac.params.e_ref_tout))
            row["q_house"] = float(np.mean(e * p_out[span]))
            for r in ROOMS:
                pr = params.rooms[r]
                others = np.mean([tm[o][span] for o in ROOMS if o != r], axis=0)
                loss = (pr.a * (tout[span] - tm[r][span])
                        + pr.c * (others - tm[r][span])
                        + pr.gain
                        + pr.s_ne * sun_ne[span] + pr.s_nw * sun_nw[span])
                slope = np.polyfit(t_min / 60.0, tm[r][span], 1)[0]
                row[f"x_{r}"] = float(np.mean(x[r][span]))
                row[f"slope_{r}"] = float(slope)
                row[f"loss_{r}"] = float(np.mean(loss))
            windows.append(row)
        i = j + 1
    return windows


def shares(row: dict, beta: float) -> dict[str, float]:
    f = {r: (row[f"x_{r}"] ** beta) * cl.AIRFLOW_WEIGHTS[r] for r in ROOMS}
    tot = sum(f.values())
    if tot <= 0:
        return {r: 0.0 for r in ROOMS}
    return {r: f[r] / tot for r in ROOMS}


def sse(rows: list[dict], beta: float, scale: float) -> float:
    mass_mean = np.mean([cl.MASS_WEIGHTS[r] for r in ROOMS])
    tot = 0.0
    for row in rows:
        sh = shares(row, beta)
        for r in ROOMS:
            q_r = scale * row["q_house"] * sh[r] / (cl.MASS_WEIGHTS[r] / mass_mean)
            pred = row[f"loss_{r}"] + q_r
            tot += (row[f"slope_{r}"] - pred) ** 2
    return tot


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--month", default="2026-06")
    ap.add_argument("--parquet", default=_ROOT / "data/processed/june.parquet",
                    type=Path)
    ap.add_argument("--csv", type=Path, default=None)
    args = ap.parse_args()

    params, hvac = HouseParams(), Hvac()
    rows: list[dict] = []
    for dnum in range(1, 31):
        date = f"{args.month}-{dnum:02d}"
        try:
            day = load_day(args.parquet, date)
        except (SystemExit, Exception):  # noqa: BLE001 archive gaps
            continue
        rows.extend(day_windows(day, params, hvac))
    print(f"{len(rows)} steady windows")
    mid = [r for r in rows if any(0.15 < r[f"x_{q}"] < 0.85 for q in ROOMS)]
    print(f"{len(mid)} with at least one mid-range damper (carry the beta signal)")
    if args.csv:
        pd.DataFrame(rows).to_csv(args.csv, index=False)

    betas = np.round(np.arange(0.3, 2.51, 0.05), 2)
    scales = np.round(np.arange(0.6, 1.41, 0.02), 2)
    best = None
    prof = {}
    for b in betas:
        s_sse = [(sse(rows, b, s), s) for s in scales]
        v, s = min(s_sse)
        prof[b] = (v, s)
        if best is None or v < best[0]:
            best = (v, b, s)
    v0, b0, s0 = best
    print(f"\nbest: beta={b0:.2f} scale={s0:.2f} sse={v0:.4f}")
    print(f"beta=1.00 (linear): sse={prof[1.0][0]:.4f} at scale={prof[1.0][1]:.2f} "
          f"-> {100 * (prof[1.0][0] - v0) / prof[1.0][0]:+.1f}% worse than best")
    print("\nbeta profile (sse at best scale, relative to minimum):")
    for b in [0.3, 0.5, 0.7, 0.85, 1.0, 1.2, 1.5, 2.0, 2.5]:
        v, s = prof[b]
        print(f"  beta {b:>4}: sse/min {v / v0:6.3f}  (scale {s:.2f})")

    # Direct pairwise estimator, scale-free: in a window with a mid-range
    # room m and a fully-open reference f (x_f >= 0.97, share x^beta == x_f),
    # the mass/duct-normalised heat ratio is x_m**beta, so
    # beta = ln(ratio) / ln(x_m) per window. No e-model level dependence
    # (q_house cancels), no loss-model level dependence beyond each room's
    # own loss subtraction.
    print("\npairwise implied beta (mid-range room vs full-open reference):")
    mass_mean = np.mean([cl.MASS_WEIGHTS[q] for q in ROOMS])
    est = []
    for row in rows:
        refs = [r for r in ROOMS if row[f"x_{r}"] >= 0.97]
        if not refs:
            continue
        f = max(refs, key=lambda r: row[f"slope_{r}"] - row[f"loss_{r}"])
        qf = (row[f"slope_{f}"] - row[f"loss_{f}"]) * cl.MASS_WEIGHTS[f] / cl.AIRFLOW_WEIGHTS[f]
        if qf < 0.3:  # reference must be receiving unambiguous heat [K/h]
            continue
        for m in ROOMS:
            xm = row[f"x_{m}"]
            if not (0.15 < xm < 0.85):
                continue
            qm = (row[f"slope_{m}"] - row[f"loss_{m}"]) * cl.MASS_WEIGHTS[m] / cl.AIRFLOW_WEIGHTS[m]
            if qm <= 0.02:
                continue
            beta_i = float(np.log(qm / qf) / np.log(xm))
            est.append({"room": m, "x": xm, "beta": beta_i,
                        "w": abs(np.log(xm))})
    if est:
        e_df = pd.DataFrame(est)
        b = e_df.beta.to_numpy()
        print(f"  n={len(e_df)}  median {np.median(b):.2f}  "
              f"IQR [{np.percentile(b, 25):.2f}, {np.percentile(b, 75):.2f}]")
        for room, g in e_df.groupby("room"):
            print(f"  {room:>8}: n={len(g):>3} median {g.beta.median():.2f} "
                  f"IQR [{g.beta.quantile(.25):.2f}, {g.beta.quantile(.75):.2f}] "
                  f"x median {g.x.median():.2f}")
    else:
        print("  no usable pairs")

    print("\nper-room beta profile (rooms fitted alone, best scale each):")
    for r in ROOMS:
        sub = [w for w in rows if 0.15 < w[f"x_{r}"] < 0.85]
        if len(sub) < 10:
            print(f"  {r:>8}: only {len(sub)} mid-range windows, skipped")
            continue

        def sse_room(rows_, b, s, room=r):
            mass_mean = np.mean([cl.MASS_WEIGHTS[q] for q in ROOMS])
            t = 0.0
            for row in rows_:
                sh = shares(row, b)
                q_r = s * row["q_house"] * sh[room] / (cl.MASS_WEIGHTS[room] / mass_mean)
                t += (row[f"slope_{room}"] - (row[f"loss_{room}"] + q_r)) ** 2
            return t

        pr = {b: min((sse_room(sub, b, s), s) for s in scales) for b in betas}
        vb, bb = min((v[0], b) for b, v in pr.items())
        print(f"  {r:>8}: beta {bb:.2f} (n={len(sub)}; linear/min sse "
              f"{pr[1.0][0] / vb:.3f})")


if __name__ == "__main__":
    main()
