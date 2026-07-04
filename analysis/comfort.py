"""Comfort/cost metric functions over replayed telemetry (docs/tasks/014).

Pure functions over a per-minute pandas.DataFrame for one local day, no
data/ or sim/ access -- fully unit-testable with synthetic frames
(tests/test_comfort.py). `analysis/tune.py` builds the frame this module
consumes from `analysis/replay_day.py`'s replay output plus the recorded
target-band columns.

Expected columns (for each room r in `rooms`):
    feels_{r}   simulated feels-like temperature [C]
    low_{r}     recorded target_temp_low (feels-like units) [C]
    high_{r}    recorded target_temp_high (feels-like units) [C]
plus whole-house `p_kw` (HVAC electrical power) and `increment`
(compressor increment; >0 means stepped above minimum).
"""
from __future__ import annotations

import numpy as np
import pandas as pd

DEFAULT_ROOMS = ("bed_1", "bed_2", "bed_3", "study", "kitchen")

RISE_STEP_K = 0.3  # minimum net low-band increase to count as a rise event
# statctrl ramps schedule changes +0.1 K every ~3 min rather than stepping,
# so rise minutes within this gap of each other merge into one event.
RISE_GAP_MIN = 15
# after a rise event first reaches its target, look this far ahead for the
# overshoot peak above the stepped-to low.
OVERSHOOT_WINDOW_MIN = 120
OSC_PROMINENCE_K = 0.15  # oscillation-scan prominence threshold (analysis/oscillation_scan.py)
OSC_SMOOTH_WIN = 3  # minutes, moving-average window before extrema counting


def _extrema_count(values: np.ndarray, min_prominence: float) -> int:
    """Count alternating local extrema in `values` clearing `min_prominence`.

    Same two-pass definition as analysis/oscillation_scan.py's
    count_extrema: first find every local peak/trough (a sample not
    strictly below both neighbours, or not strictly above both), then walk
    those candidates keeping only ones that differ from the last *kept*
    extremum by at least `min_prominence` -- this discards small wiggles
    riding on top of a larger swing.
    """
    n = len(values)
    if n < 3:
        return 0
    extrema_idx = []
    for i in range(1, n - 1):
        if (values[i] > values[i - 1] and values[i] >= values[i + 1]) or (
            values[i] < values[i - 1] and values[i] <= values[i + 1]
        ):
            extrema_idx.append(i)
    if len(extrema_idx) < 2:
        return 0
    count = 0
    last_kept = extrema_idx[0]
    for idx in extrema_idx[1:]:
        if abs(values[idx] - values[last_kept]) >= min_prominence:
            count += 1
            last_kept = idx
    return count


def _rise_events(low: np.ndarray, feels: np.ndarray) -> list[tuple[int, float]]:
    """(event_start_minute, target_low) pairs for room-r rise events.

    A rise event is a group of low-band increases whose members are within
    RISE_GAP_MIN minutes of each other (statctrl ramps schedule changes
    +0.1 K every ~3 min rather than stepping, so a morning warmup is many
    small non-consecutive rises -- gap-tolerant grouping merges them into
    one event; a single >= RISE_STEP_K step is the degenerate one-member
    group). The event counts when its net rise (low after the group's last
    rise minus low just before its first) is >= RISE_STEP_K and the room
    hadn't already reached the final target when the group started
    (`feels` at the first rise minute is below the stepped-to value). The
    event is anchored at the group's first rise minute and targets the
    group's final value.
    """
    n = len(low)
    rise_idx = [i for i in range(1, n) if low[i] > low[i - 1]]
    events: list[tuple[int, float]] = []
    k = 0
    while k < len(rise_idx):
        j = k
        while j + 1 < len(rise_idx) and rise_idx[j + 1] - rise_idx[j] <= RISE_GAP_MIN:
            j += 1
        start = rise_idx[k]
        before = low[start - 1]
        target = low[rise_idx[j]]
        if target - before >= RISE_STEP_K and feels[start] < target:
            events.append((start, float(target)))
        k = j + 1
    return events


def _rise_time(feels: np.ndarray, start: int, target: float) -> float:
    """Minutes from `start` until `feels` first reaches `target`, else NaN."""
    n = len(feels)
    for t in range(start, n):
        if feels[t] >= target:
            return float(t - start)
    return float("nan")


def score_day(df: pd.DataFrame, rooms=DEFAULT_ROOMS) -> dict:
    """Per-day comfort/cost metrics dict (docs/tasks/014).

    - time_in_band: mean over rooms of the fraction of minutes with
      low <= feels <= high.
    - deg_min_below / deg_min_above: mean over rooms of the day's integral
      of (low - feels) clipped >= 0, resp. (feels - high) clipped >= 0,
      in K*min (one-minute rows, so the integral is just the per-minute
      sum) -- discomfort *magnitude*, not just minutes out of band.
    - rise_events / rise_time_med / rise_events_unmet: see _rise_events /
      _rise_time. rise_time_med is the median rise time over all rooms'
      events that reach the new low before day-end; events that never
      reach it are excluded from the median but counted in
      rise_events_unmet. rise_events counts every detected event
      (met + unmet).
    - overshoot_max: max over rooms/minutes of (feels - high) clipped >= 0,
      the day's single worst hard overshoot [K]. In heating season the
      heat_cool band is wide, so this is usually 0 -- the tuning-relevant
      number is overshoot_rise_max.
    - overshoot_rise_max: max over met rise events of the peak
      (feels - event target) within OVERSHOOT_WINDOW_MIN minutes of first
      reaching the target [K] -- how far the room shoots past a schedule
      step-up before settling. 0.0 when there are no met events.
    - osc_per_h: oscillation count of the kitchen feels-like trace,
      restricted to unit-on minutes (p_kw > 0), after a 3-minute moving
      average, normalised per running hour (0 when there are no running
      minutes).
    - energy_kwh: sum(p_kw) / 60 (one-minute rows of kW).
    - starts: count of p_kw rising edges (off -> on).
    - abovemin_frac: fraction of minutes with increment > 0.
    """
    time_in_band = []
    deg_min_below = []
    deg_min_above = []
    all_rise_times: list[float] = []
    rise_events_total = 0
    rise_events_unmet = 0
    overshoot_max = 0.0
    overshoot_rise_max = 0.0

    for r in rooms:
        feels = df[f"feels_{r}"].to_numpy(dtype=float)
        low = df[f"low_{r}"].to_numpy(dtype=float)
        high = df[f"high_{r}"].to_numpy(dtype=float)

        in_band = (feels >= low) & (feels <= high)
        time_in_band.append(float(np.mean(in_band)))

        below = np.clip(low - feels, 0.0, None)
        above = np.clip(feels - high, 0.0, None)
        deg_min_below.append(float(np.sum(below)))
        deg_min_above.append(float(np.sum(above)))

        overshoot_max = max(overshoot_max, float(np.max(above)))

        for start, target in _rise_events(low, feels):
            rise_events_total += 1
            rt = _rise_time(feels, start, target)
            if np.isnan(rt):
                rise_events_unmet += 1
            else:
                all_rise_times.append(rt)
                reach = start + int(rt)
                window = feels[reach : reach + OVERSHOOT_WINDOW_MIN]
                overshoot_rise_max = max(
                    overshoot_rise_max, float(np.max(window - target))
                )

    on = df["p_kw"].to_numpy(dtype=float) > 0
    # osc_per_h is specifically the kitchen trace (the room with the
    # highest-gain, most cycling-prone dynamics -- analysis/oscillation_scan.py);
    # tests may score fewer rooms than the full house, so treat a missing
    # kitchen column the same as "no running minutes" -> 0.0.
    if on.any() and "feels_kitchen" in df.columns:
        kitchen_feels = df["feels_kitchen"].to_numpy(dtype=float)
        smoothed = (
            pd.Series(kitchen_feels).rolling(OSC_SMOOTH_WIN, min_periods=1).mean().to_numpy()
        )
        sub = smoothed[on]
        n_extrema = _extrema_count(sub, OSC_PROMINENCE_K)
        running_hours = float(on.sum()) / 60.0
        osc_per_h = n_extrema / running_hours if running_hours > 0 else 0.0
    else:
        osc_per_h = 0.0

    p_kw = df["p_kw"].to_numpy(dtype=float)
    on_p = p_kw > 0
    starts = int(np.sum(on_p[1:] & ~on_p[:-1]))
    increment = df["increment"].to_numpy(dtype=float)

    return {
        "time_in_band": float(np.mean(time_in_band)),
        "deg_min_below": float(np.mean(deg_min_below)),
        "deg_min_above": float(np.mean(deg_min_above)),
        "rise_events": int(rise_events_total),
        "rise_time_med": float(np.median(all_rise_times)) if all_rise_times else float("nan"),
        "rise_events_unmet": int(rise_events_unmet),
        "overshoot_max": float(overshoot_max),
        "overshoot_rise_max": float(overshoot_rise_max),
        "osc_per_h": float(osc_per_h),
        "energy_kwh": float(np.sum(p_kw) / 60.0),
        "starts": starts,
        "abovemin_frac": float(np.mean(increment > 0)),
    }


def summarize(day_scores: list[dict]) -> dict:
    """Medians of each metric across days (NaN-safe), plus `days` count."""
    if not day_scores:
        return {"days": 0}
    keys = day_scores[0].keys()
    out: dict = {"days": len(day_scores)}
    for k in keys:
        values = [float(d[k]) for d in day_scores]
        out[k] = float(np.nanmedian(values))
    return out
