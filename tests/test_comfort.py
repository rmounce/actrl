"""Synthetic-data unit tests for analysis/comfort.py (docs/tasks/014).

No data/ access, no replay -- hand-built per-minute frames only.
"""
from __future__ import annotations

import math

import numpy as np
import pandas as pd
import pytest

from analysis.comfort import score_day, summarize

ROOMS = ("a", "b", "c")


def _frame(n, rooms=ROOMS, low=18.0, high=22.0, feels=20.0, p_kw=0.0, increment=0.0):
    """A flat, fully in-band frame of `n` minutes for `rooms`, overridable
    per-column so individual tests only need to poke the columns they
    care about."""
    data = {}
    for r in rooms:
        data[f"feels_{r}"] = np.full(n, feels, dtype=float)
        data[f"low_{r}"] = np.full(n, low, dtype=float)
        data[f"high_{r}"] = np.full(n, high, dtype=float)
    data["p_kw"] = np.full(n, p_kw, dtype=float)
    data["increment"] = np.full(n, increment, dtype=float)
    return pd.DataFrame(data)


def test_time_in_band_and_deg_min_fully_in_band():
    df = _frame(60)
    scores = score_day(df, rooms=ROOMS)
    assert scores["time_in_band"] == pytest.approx(1.0)
    assert scores["deg_min_below"] == pytest.approx(0.0)
    assert scores["deg_min_above"] == pytest.approx(0.0)


def test_time_in_band_and_deg_min_known_minutes():
    # 3 rooms, 10 minutes each. Room a: 4 minutes below band by 1K each.
    # Room b: fully in band. Room c: 2 minutes above band by 0.5K each.
    n = 10
    df = _frame(n)
    a_feels = df["feels_a"].to_numpy().copy()
    a_feels[:4] = 17.0  # 1K below low=18
    df["feels_a"] = a_feels
    c_feels = df["feels_c"].to_numpy().copy()
    c_feels[:2] = 22.5  # 0.5K above high=22
    df["feels_c"] = c_feels

    scores = score_day(df, rooms=ROOMS)

    # in-band minutes: a=6/10, b=10/10, c=8/10 -> mean = 24/30 = 0.8
    assert scores["time_in_band"] == pytest.approx((6 + 10 + 8) / 30)
    # deg_min_below: a contributes 4*1.0=4.0, b=0, c=0 -> mean over rooms = 4/3
    assert scores["deg_min_below"] == pytest.approx(4.0 / 3)
    # deg_min_above: a=0, b=0, c contributes 2*0.5=1.0 -> mean over rooms = 1/3
    assert scores["deg_min_above"] == pytest.approx(1.0 / 3)


def test_rise_event_single_step():
    n = 60
    df = _frame(n, rooms=("a",), low=18.0, high=22.0, feels=18.0)
    low = df["low_a"].to_numpy().copy()
    feels = df["feels_a"].to_numpy().copy()
    # Schedule steps low 18 -> 19 at minute 10; room is below the new low.
    low[10:] = 19.0
    feels[:10] = 18.0
    # Room chases the new low, reaching it exactly at minute 40.
    feels[10:40] = 18.5
    feels[40:] = 19.0
    df["low_a"] = low
    df["feels_a"] = feels

    scores = score_day(df, rooms=("a",))
    assert scores["rise_events"] == 1
    assert scores["rise_events_unmet"] == 0
    assert scores["rise_time_med"] == pytest.approx(30.0)


def test_rise_event_unmet_excluded_from_median_but_counted():
    n = 60
    df = _frame(n, rooms=("a",), low=18.0, high=25.0, feels=18.0)
    low = df["low_a"].to_numpy().copy()
    feels = df["feels_a"].to_numpy().copy()
    low[10:] = 19.0
    feels[:] = 18.0  # never reaches 19.0 by end of day
    df["low_a"] = low
    df["feels_a"] = feels

    scores = score_day(df, rooms=("a",))
    assert scores["rise_events"] == 1
    assert scores["rise_events_unmet"] == 1
    assert math.isnan(scores["rise_time_med"])


def test_rise_event_ramp_merges_to_one_event():
    n = 80
    df = _frame(n, rooms=("a",), low=18.0, high=25.0, feels=18.0)
    low = df["low_a"].to_numpy().copy()
    feels = df["feels_a"].to_numpy().copy()
    # Ramp: low rises 0.1/min for minutes 10..14 inclusive (5 steps of
    # 0.1K, net 0.5K >= the 0.3K threshold), then holds at the final value.
    for i, m in enumerate(range(10, 15)):
        low[m:] = 18.0 + 0.1 * (i + 1)
    final_target = low[-1]
    assert final_target == pytest.approx(18.5)
    feels[:10] = 18.0
    feels[10:50] = 18.2  # below target throughout the ramp
    feels[50:] = final_target  # reaches the final stepped-to low at minute 50

    df["low_a"] = low
    df["feels_a"] = feels

    scores = score_day(df, rooms=("a",))
    assert scores["rise_events"] == 1
    assert scores["rise_events_unmet"] == 0
    # anchored at the ramp's first minute (10), reaches target at minute 50
    assert scores["rise_time_med"] == pytest.approx(40.0)


def test_overshoot_max_on_crafted_spike():
    n = 30
    df = _frame(n, rooms=("a", "b"), low=18.0, high=22.0, feels=20.0)
    b_feels = df["feels_b"].to_numpy().copy()
    b_feels[15] = 23.5  # 1.5K hard overshoot spike
    df["feels_b"] = b_feels

    scores = score_day(df, rooms=("a", "b"))
    assert scores["overshoot_max"] == pytest.approx(1.5)


def test_osc_per_h_triangle_wave_and_zero_when_off():
    n = 120  # 2 hours
    # Triangle wave with period 20 minutes, amplitude well above the 0.15K
    # prominence threshold, on the kitchen feels-like trace.
    t = np.arange(n)
    period = 20
    tri = 2.0 * np.abs((t % period) - period / 2) / (period / 2) - 1.0  # in [-1, 1]
    kitchen_feels = 20.0 + tri  # amplitude 1K, well above prominence

    df = _frame(n, rooms=("kitchen",), low=18.0, high=22.0, feels=20.0)
    df["feels_kitchen"] = kitchen_feels

    # All-zero power -> no running minutes -> osc_per_h is 0 regardless of
    # the triangle wave underneath.
    off_scores = score_day(df, rooms=("kitchen",))
    assert off_scores["osc_per_h"] == pytest.approx(0.0)

    # Unit running the whole 2 hours -> extrema counted, normalised by the
    # 2 running hours.
    df_on = df.copy()
    df_on["p_kw"] = 1.0
    on_scores = score_day(df_on, rooms=("kitchen",))
    assert on_scores["osc_per_h"] > 0.0


def test_energy_starts_abovemin():
    n = 6
    df = _frame(n, rooms=("a",))
    df["p_kw"] = [0.0, 1.0, 1.0, 0.0, 2.0, 2.0]
    df["increment"] = [0.0, 0.0, 5.0, 0.0, 0.0, 3.0]

    scores = score_day(df, rooms=("a",))
    assert scores["energy_kwh"] == pytest.approx(sum([0.0, 1.0, 1.0, 0.0, 2.0, 2.0]) / 60.0)
    assert scores["starts"] == 2  # off->on at minute 1, and again at minute 4
    assert scores["abovemin_frac"] == pytest.approx(2 / 6)


def test_summarize_medians_nan_safe():
    day_scores = [
        {"time_in_band": 0.9, "rise_time_med": 10.0, "days_marker": 1},
        {"time_in_band": 0.8, "rise_time_med": float("nan"), "days_marker": 2},
        {"time_in_band": 1.0, "rise_time_med": 30.0, "days_marker": 3},
    ]
    summary = summarize(day_scores)
    assert summary["days"] == 3
    assert summary["time_in_band"] == pytest.approx(0.9)
    # nanmedian over [10.0, 30.0] (NaN excluded) = 20.0
    assert summary["rise_time_med"] == pytest.approx(20.0)


def test_statctrl_style_ramp_merges_to_one_event():
    # statctrl ramps +0.1 K every 3 minutes (non-consecutive rise minutes):
    # 16.0 -> 16.6 over 6 rises starting at minute 30. One event, targeting
    # the final 16.6, anchored at the first rise minute.
    n = 240
    df = _frame(n, low=16.0, high=24.0, feels=15.0)
    low = df["low_a"].to_numpy().copy()
    for k in range(6):
        low[30 + 3 * k :] = 16.0 + 0.1 * (k + 1)
    df["low_a"] = low
    feels = df["feels_a"].to_numpy().copy()
    feels[90:] = 16.7  # reaches the 16.6 target at minute 90
    df["feels_a"] = feels

    scores = score_day(df, rooms=ROOMS)
    assert scores["rise_events"] == 1
    assert scores["rise_events_unmet"] == 0
    assert scores["rise_time_med"] == pytest.approx(60.0)  # minute 30 -> 90


def test_overshoot_rise_max_measures_peak_above_event_target():
    # Step 18 -> 19 at minute 10; feels reaches 19 at minute 40, peaks at
    # 19.8 at minute 60, sags back. overshoot_rise_max = 0.8 even though
    # the band high (22) is never crossed (overshoot_max stays 0).
    n = 240
    df = _frame(n, low=18.0, high=22.0, feels=18.5)
    low = df["low_a"].to_numpy().copy()
    low[10:] = 19.0
    df["low_a"] = low
    feels = df["feels_a"].to_numpy().copy()
    feels[:40] = 18.5
    feels[40:60] = np.linspace(19.0, 19.8, 20)
    feels[60:] = 19.2
    df["feels_a"] = feels

    scores = score_day(df, rooms=ROOMS)
    assert scores["rise_events"] == 1
    assert scores["overshoot_rise_max"] == pytest.approx(0.8)
    assert scores["overshoot_max"] == pytest.approx(0.0)
