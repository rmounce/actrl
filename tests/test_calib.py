"""Offline tests for calib.py. No live data required -- synthetic
data/raw/<day>/<measurement>__<entity>.csv.gz fixtures are written to
tmp_path by the tests themselves. Real exported data is never committed.
"""
import gzip
import sys
from datetime import date
from pathlib import Path

import pandas as pd
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import calib  # noqa: E402


# --------------------------------------------------------------------------
# Fixture helpers
# --------------------------------------------------------------------------


def write_day_csv(data_dir: Path, day: str, measurement: str, entity_id: str, rows: list[tuple[str, str, str]]) -> Path:
    """Write a synthetic data/raw/<day>/<measurement>__<entity>.csv.gz file.
    rows: list of (time_iso, field, value_str), header added automatically."""
    path = data_dir / "raw" / day / f"{measurement}__{entity_id}.csv.gz"
    path.parent.mkdir(parents=True, exist_ok=True)
    with gzip.open(path, "wt", newline="") as f:
        f.write("time,field,value\n")
        for t, field, v in rows:
            f.write(f"{t},{field},{v}\n")
    return path


# --------------------------------------------------------------------------
# Parsing
# --------------------------------------------------------------------------


def test_parse_csv_text_drops_header():
    text = "time,field,value\n2026-06-01T00:00:00Z,value,1.5\n"
    rows = calib.parse_csv_text(text)
    assert rows == [("2026-06-01T00:00:00Z", "value", "1.5")]


def test_parse_csv_text_empty():
    assert calib.parse_csv_text("") == []
    assert calib.parse_csv_text("time,field,value\n") == []


def test_rows_for_field_filters_and_parses_mixed_subsecond_precision():
    rows = [
        ("2026-06-05T00:00:05.93548Z", "value", "1.1"),
        ("2026-06-05T00:04:14.568804Z", "value", "2.2"),
        ("2026-06-05T23:59:58.96719Z", "value", "3.3"),
        ("2026-06-05T00:00:00Z", "value", "4.4"),
        ("2026-06-05T00:00:01Z", "other_field", "99"),
    ]
    points = calib.rows_for_field(rows, "value")
    assert len(points) == 4
    for ts, _ in points:
        assert isinstance(ts, pd.Timestamp)
        assert ts.tzinfo is not None
        assert str(ts.tzinfo) == "UTC"
    # sub-microsecond precision preserved correctly
    assert points[0][0] == pd.Timestamp("2026-06-05T00:00:05.935480Z")
    assert points[1][0] == pd.Timestamp("2026-06-05T00:04:14.568804Z")


def test_points_to_series_dedup_keeps_last_and_sorts():
    points = [
        (pd.Timestamp("2026-06-05T00:00:02Z"), 1.0),
        (pd.Timestamp("2026-06-05T00:00:01Z"), 2.0),
        (pd.Timestamp("2026-06-05T00:00:01Z"), 3.0),  # duplicate ts, keep last
    ]
    s = calib.points_to_series(points)
    assert list(s.index) == [pd.Timestamp("2026-06-05T00:00:01Z"), pd.Timestamp("2026-06-05T00:00:02Z")]
    assert list(s.values) == [3.0, 1.0]


def test_points_to_series_empty():
    s = calib.points_to_series([])
    assert s.empty
    assert str(s.index.tz) == "UTC"


# --------------------------------------------------------------------------
# Grid
# --------------------------------------------------------------------------


def test_build_grid_one_day_1min():
    grid = calib.build_grid(date(2026, 6, 1), date(2026, 6, 1), "1min")
    assert len(grid) == 1440
    assert grid[0] == pd.Timestamp("2026-06-01T00:00:00Z")
    assert grid[-1] == pd.Timestamp("2026-06-01T23:59:00Z")
    assert str(grid.tz) == "UTC"


def test_build_grid_multi_day():
    grid = calib.build_grid(date(2026, 6, 1), date(2026, 6, 3), "1min")
    assert len(grid) == 3 * 1440


def test_cap_periods_exact_and_rounding():
    assert calib.cap_periods(pd.Timedelta("15min"), "1min") == 15
    assert calib.cap_periods(pd.Timedelta("2min"), "1min") == 2
    assert calib.cap_periods(pd.Timedelta("15min"), "5min") == 3
    assert calib.cap_periods(pd.Timedelta("30sec"), "1min") == 1  # min 1


# --------------------------------------------------------------------------
# bin_mean
# --------------------------------------------------------------------------


def test_bin_mean_averages_within_bin_and_leaves_empty_bins_nan():
    grid = calib.build_grid(date(2026, 6, 1), date(2026, 6, 1), "1min")
    points = [
        (pd.Timestamp("2026-06-01T00:00:10Z"), 10.0),
        (pd.Timestamp("2026-06-01T00:00:40Z"), 20.0),
        # bin 00:01 has no points -> NaN
        (pd.Timestamp("2026-06-01T00:02:05Z"), 5.0),
    ]
    raw = calib.points_to_series(points)
    binned = calib.bin_mean(raw, grid, "1min")
    assert binned.loc["2026-06-01T00:00:00Z"] == 15.0
    assert pd.isna(binned.loc["2026-06-01T00:01:00Z"])
    assert binned.loc["2026-06-01T00:02:00Z"] == 5.0


def test_bin_mean_all_empty_returns_all_nan():
    grid = calib.build_grid(date(2026, 6, 1), date(2026, 6, 1), "1min")
    raw = calib.points_to_series([])
    binned = calib.bin_mean(raw, grid, "1min")
    assert binned.isna().all()
    assert len(binned) == len(grid)


# --------------------------------------------------------------------------
# temperature kind: bin mean + <=15min interpolation cap
# --------------------------------------------------------------------------


def test_temperature_series_bridges_gap_within_15min():
    grid = calib.build_grid(date(2026, 6, 1), date(2026, 6, 1), "1min")
    points = [
        (pd.Timestamp("2026-06-01T00:00:00Z"), 10.0),
        # 15-minute gap (bins 1..15 empty), next point at bin 16
        (pd.Timestamp("2026-06-01T00:16:00Z"), 26.0),
    ]
    raw = calib.points_to_series(points)
    s = calib.temperature_series(raw, grid, "1min")
    # linear ramp from 10 at t=0 to 26 at t=16 => +1 per minute
    assert s.loc["2026-06-01T00:08:00Z"] == pytest.approx(18.0)
    assert not s.iloc[:17].isna().any()


def test_temperature_series_does_not_bridge_gap_over_15min():
    grid = calib.build_grid(date(2026, 6, 1), date(2026, 6, 1), "1min")
    points = [
        (pd.Timestamp("2026-06-01T00:00:00Z"), 10.0),
        # 16-minute gap: bins 1..16 empty (17 apart) -- exceeds the cap
        (pd.Timestamp("2026-06-01T00:17:00Z"), 27.0),
    ]
    raw = calib.points_to_series(points)
    s = calib.temperature_series(raw, grid, "1min")
    assert pd.isna(s.loc["2026-06-01T00:08:00Z"])


# --------------------------------------------------------------------------
# power kind: clamp before mean, no fill
# --------------------------------------------------------------------------


def test_power_series_clamps_negative_before_binning():
    grid = calib.build_grid(date(2026, 6, 1), date(2026, 6, 1), "1min")
    points = [
        (pd.Timestamp("2026-06-01T00:00:10Z"), -7.0),  # CT offset, clamp to 0
        (pd.Timestamp("2026-06-01T00:00:40Z"), 3.0),
    ]
    raw = calib.points_to_series(points)
    s = calib.power_series(raw, grid, "1min")
    assert s.loc["2026-06-01T00:00:00Z"] == pytest.approx(1.5)  # mean(0, 3), not mean(-7, 3)


def test_power_series_gaps_stay_nan():
    grid = calib.build_grid(date(2026, 6, 1), date(2026, 6, 1), "1min")
    points = [(pd.Timestamp("2026-06-01T00:00:10Z"), 5.0)]
    raw = calib.points_to_series(points)
    s = calib.power_series(raw, grid, "1min")
    assert pd.isna(s.loc["2026-06-01T00:05:00Z"])


# --------------------------------------------------------------------------
# controller kind: bin mean + ffill up to 2 min
# --------------------------------------------------------------------------


def test_controller_series_ffills_up_to_2min_only():
    grid = calib.build_grid(date(2026, 6, 1), date(2026, 6, 1), "1min")
    points = [(pd.Timestamp("2026-06-01T00:00:10Z"), 5.0)]
    raw = calib.points_to_series(points)
    s = calib.controller_series(raw, grid, "1min")
    assert s.loc["2026-06-01T00:00:00Z"] == 5.0
    assert s.loc["2026-06-01T00:01:00Z"] == 5.0
    assert s.loc["2026-06-01T00:02:00Z"] == 5.0
    assert pd.isna(s.loc["2026-06-01T00:03:00Z"])  # gap > 2 min: app wasn't running


# --------------------------------------------------------------------------
# state kind: unlimited ffill, seeding
# --------------------------------------------------------------------------


def test_state_series_unlimited_ffill_no_seed_nan_before_first_point():
    grid = calib.build_grid(date(2026, 6, 1), date(2026, 6, 1), "1min")
    points = [(pd.Timestamp("2026-06-01T00:30:00Z"), 42.0)]
    raw = calib.points_to_series(points)
    s = calib.state_series(raw, grid, seed=None)
    assert pd.isna(s.loc["2026-06-01T00:00:00Z"])
    assert s.loc["2026-06-01T00:30:00Z"] == 42.0
    assert s.loc["2026-06-01T23:59:00Z"] == 42.0  # unlimited ffill, held for rest of day


def test_state_series_seed_fills_before_first_observation():
    grid = calib.build_grid(date(2026, 6, 1), date(2026, 6, 1), "1min")
    points = [(pd.Timestamp("2026-06-01T00:30:00Z"), 42.0)]
    raw = calib.points_to_series(points)
    seed = (pd.Timestamp("2026-05-20T00:00:00Z"), 7.0)
    s = calib.state_series(raw, grid, seed=seed)
    assert s.loc["2026-06-01T00:00:00Z"] == 7.0
    assert s.loc["2026-06-01T00:29:00Z"] == 7.0
    assert s.loc["2026-06-01T00:30:00Z"] == 42.0


def test_state_series_no_points_and_no_seed_is_all_nan():
    grid = calib.build_grid(date(2026, 6, 1), date(2026, 6, 1), "1min")
    raw = calib.points_to_series([])
    s = calib.state_series(raw, grid, seed=None)
    assert s.isna().all()


def test_state_series_absent_day_file_means_unchanged_via_load_range(tmp_path):
    # Day 1 has a damper move; day 2 has no file at all for that entity --
    # per docs/data.md this means "did not change", not "missing".
    write_day_csv(
        tmp_path, "2026-06-01", "cover__damper", "bed_1",
        [("2026-06-01T05:00:00Z", "current_position", "10")],
    )
    # no file written for 2026-06-02
    df = calib.load_range("2026-06-01", "2026-06-02", data_dir=tmp_path, freq="1min", seed_days=1)
    assert df.loc["2026-06-02T12:00:00Z", "damper.bed_1"] == 10.0


# --------------------------------------------------------------------------
# scan_seed
# --------------------------------------------------------------------------


def test_scan_seed_finds_most_recent_point_n_days_back(tmp_path):
    write_day_csv(
        tmp_path, "2026-05-25", "cover__damper", "bed_1",
        [("2026-05-25T03:00:00Z", "current_position", "20"), ("2026-05-25T09:00:00Z", "current_position", "25")],
    )
    write_day_csv(
        tmp_path, "2026-05-20", "cover__damper", "bed_1",
        [("2026-05-20T00:00:00Z", "current_position", "1")],
    )
    seed = calib.scan_seed(tmp_path, date(2026, 6, 1), "cover__damper", "bed_1", "current_position", seed_days=14)
    assert seed == (pd.Timestamp("2026-05-25T09:00:00Z"), 25.0)


def test_scan_seed_not_found_returns_none(tmp_path):
    seed = calib.scan_seed(tmp_path, date(2026, 6, 1), "cover__damper", "bed_1", "current_position", seed_days=14)
    assert seed is None


def test_scan_seed_out_of_window_not_found(tmp_path):
    write_day_csv(
        tmp_path, "2026-05-01", "cover__damper", "bed_1",
        [("2026-05-01T00:00:00Z", "current_position", "1")],
    )
    seed = calib.scan_seed(tmp_path, date(2026, 6, 1), "cover__damper", "bed_1", "current_position", seed_days=14)
    assert seed is None  # 2026-05-01 is > 14 days before 2026-06-01


# --------------------------------------------------------------------------
# slow kind: interpolate onto grid, no extrapolation
# --------------------------------------------------------------------------


def test_slow_series_interpolates_between_points():
    grid = calib.build_grid(date(2026, 6, 1), date(2026, 6, 1), "1min")
    points = [
        (pd.Timestamp("2026-06-01T00:00:00Z"), 10.0),
        (pd.Timestamp("2026-06-01T00:30:00Z"), 40.0),
    ]
    raw = calib.points_to_series(points)
    s = calib.slow_series(raw, grid)
    assert s.loc["2026-06-01T00:15:00Z"] == pytest.approx(25.0)


def test_slow_series_no_extrapolation_beyond_last_point():
    grid = calib.build_grid(date(2026, 6, 1), date(2026, 6, 1), "1min")
    points = [(pd.Timestamp("2026-06-01T00:00:00Z"), 10.0), (pd.Timestamp("2026-06-01T00:10:00Z"), 20.0)]
    raw = calib.points_to_series(points)
    s = calib.slow_series(raw, grid)
    assert pd.isna(s.loc["2026-06-01T23:00:00Z"])
    assert pd.isna(s.loc["2026-06-01T00:00:00Z"]) is False


# --------------------------------------------------------------------------
# End-to-end load_range
# --------------------------------------------------------------------------


def test_load_range_produces_utc_index_and_expected_columns(tmp_path):
    write_day_csv(
        tmp_path, "2026-06-01", "sensor__temperature", "bed_1_average_temperature",
        [("2026-06-01T00:00:10Z", "value", "20.0"), ("2026-06-01T00:00:40Z", "value", "20.5")],
    )
    write_day_csv(
        tmp_path, "2026-06-01", "sensor__power", "shellyem_ec64c9c6932b_channel_1_power",
        [("2026-06-01T00:00:05Z", "value", "-3.0")],
    )
    df = calib.load_range("2026-06-01", "2026-06-01", data_dir=tmp_path, freq="1min", seed_days=1)
    assert isinstance(df.index, pd.DatetimeIndex)
    assert str(df.index.tz) == "UTC"
    assert len(df) == 1440
    assert "bed_1_average_temperature" in df.columns
    assert "power.outdoor_unit" in df.columns
    assert df.loc["2026-06-01T00:00:00Z", "bed_1_average_temperature"] == pytest.approx(20.25)
    assert df.loc["2026-06-01T00:00:00Z", "power.outdoor_unit"] == 0.0  # clamped


def test_load_range_all_column_specs_present_even_when_no_data(tmp_path):
    df = calib.load_range("2026-06-01", "2026-06-01", data_dir=tmp_path, freq="1min", seed_days=1)
    assert set(df.columns) == {colname for colname, _kind in calib.COLUMN_SPECS.values()}
    assert df.isna().all().all()


# --------------------------------------------------------------------------
# Coverage report
# --------------------------------------------------------------------------


def test_gap_runs_over_counts_runs_exceeding_threshold():
    grid = calib.build_grid(date(2026, 6, 1), date(2026, 6, 1), "1min")
    s = pd.Series(1.0, index=grid)
    s.iloc[10:20] = float("nan")  # 10 min gap, not > 30 min
    s.iloc[100:140] = float("nan")  # 40 min gap, > 30 min
    assert calib.gap_runs_over(s, "1min", pd.Timedelta("30min")) == 1


def test_coverage_report_basic(tmp_path):
    write_day_csv(
        tmp_path, "2026-06-01", "sensor__temperature", "bed_1_average_temperature",
        [("2026-06-01T00:00:10Z", "value", "20.0")],
    )
    df = calib.load_range("2026-06-01", "2026-06-01", data_dir=tmp_path, freq="1min", seed_days=1)
    report = calib.coverage_report(df, "1min")
    assert "bed_1_average_temperature" in report.index
    row = report.loc["bed_1_average_temperature"]
    assert row["coverage_pct"] > 0
    assert row["first"] == pd.Timestamp("2026-06-01T00:00:00Z")
