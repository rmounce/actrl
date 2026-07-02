"""Offline, pure-function tests for tools/export_history.py.

Must not require a live InfluxDB. A real-database smoke test is guarded
behind the RUN_LIVE_INFLUX_TESTS env var and skipped otherwise, so `uv run
pytest` passes with no network access.
"""
import os
import sys
from datetime import date
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

import export_history as eh  # noqa: E402


# --------------------------------------------------------------------------
# day_ranges
# --------------------------------------------------------------------------


def test_day_ranges_single_day():
    d = date(2026, 6, 1)
    assert eh.day_ranges(d, d) == [
        ("2026-06-01", "2026-06-01T00:00:00Z", "2026-06-02T00:00:00Z")
    ]


def test_day_ranges_multi_day_chunked_per_utc_day():
    days = eh.day_ranges(date(2026, 6, 1), date(2026, 6, 3))
    assert [d[0] for d in days] == ["2026-06-01", "2026-06-02", "2026-06-03"]
    # each chunk is exactly one day, contiguous, no overlap
    for day_str, start_iso, end_iso in days:
        assert start_iso.endswith("T00:00:00Z")
        assert end_iso.endswith("T00:00:00Z")


def test_day_ranges_month_boundary():
    days = eh.day_ranges(date(2026, 1, 30), date(2026, 2, 1))
    assert [d[0] for d in days] == ["2026-01-30", "2026-01-31", "2026-02-01"]


def test_day_ranges_rejects_end_before_start():
    with pytest.raises(ValueError):
        eh.day_ranges(date(2026, 6, 5), date(2026, 6, 1))


# --------------------------------------------------------------------------
# output_path / sanitize_component
# --------------------------------------------------------------------------


def test_output_path_layout():
    p = eh.output_path(Path("data"), "2026-06-01", "sensor__temperature", "bed_1_average_temperature")
    assert p == Path("data/raw/2026-06-01/sensor__temperature__bed_1_average_temperature.csv.gz")


def test_output_path_sanitizes_unsafe_characters():
    p = eh.output_path(Path("data"), "2026-06-01", "m easurement", "weird/entity:id")
    assert p.name == "m_easurement__weird_entity_id.csv.gz"


def test_sanitize_component_blank_falls_back_to_unknown():
    assert eh.sanitize_component("") == "unknown"
    assert eh.sanitize_component("   ") == "unknown"


# --------------------------------------------------------------------------
# query construction
# --------------------------------------------------------------------------


def test_build_select_query_with_entity_filter():
    q = eh.build_select_query(
        "rp_raw", "sensor__temperature", ["value"],
        "2026-06-01T00:00:00Z", "2026-06-02T00:00:00Z", "bed_1_average_temperature",
    )
    assert q == (
        'SELECT "value" FROM "rp_raw"."sensor__temperature" '
        "WHERE time >= '2026-06-01T00:00:00Z' AND time < '2026-06-02T00:00:00Z' "
        "AND entity_id = 'bed_1_average_temperature' GROUP BY *"
    )


def test_build_select_query_whole_measurement_no_entity_filter():
    q = eh.build_select_query(
        "rp_30m", "temperature_adelaide", ["mean_value"],
        "2026-06-01T00:00:00Z", "2026-06-02T00:00:00Z", None,
    )
    assert "entity_id" not in q
    assert q.startswith('SELECT "mean_value" FROM "rp_30m"."temperature_adelaide" WHERE')
    assert q.endswith("GROUP BY *")


def test_build_select_query_multiple_fields():
    q = eh.build_select_query(
        "rp_raw", "climate", ["current_temperature", "temperature"],
        "2026-06-01T00:00:00Z", "2026-06-02T00:00:00Z", "bed_1_aircon",
    )
    assert 'SELECT "current_temperature", "temperature" FROM' in q


def test_build_select_query_rejects_empty_fields():
    with pytest.raises(ValueError):
        eh.build_select_query("rp_raw", "climate", [], "a", "b", None)


def test_build_select_query_escapes_quotes_in_entity_id():
    q = eh.build_select_query("rp_raw", "climate", ["value"], "a", "b", "o'brien")
    assert r"entity_id = 'o\'brien'" in q


def test_build_field_keys_query():
    assert eh.build_field_keys_query("rp_raw", "climate") == 'SHOW FIELD KEYS FROM "rp_raw"."climate"'


def test_build_query_url():
    url = eh.build_query_url("http://localhost:8086", "hass", "SHOW MEASUREMENTS")
    assert url.startswith("http://localhost:8086/query?")
    assert "db=hass" in url
    assert "SHOW+MEASUREMENTS" in url or "SHOW%20MEASUREMENTS" in url


def test_build_query_url_strips_trailing_slash():
    url = eh.build_query_url("http://localhost:8086/", "hass", "q")
    assert url.startswith("http://localhost:8086/query?")


# --------------------------------------------------------------------------
# response parsing
# --------------------------------------------------------------------------


def test_numeric_field_names_filters_to_float():
    result = {
        "results": [
            {
                "series": [
                    {
                        "columns": ["fieldKey", "fieldType"],
                        "values": [
                            ["value", "float"],
                            ["friendly_name_str", "string"],
                            ["temperature", "float"],
                        ],
                    }
                ]
            }
        ]
    }
    assert eh.numeric_field_names(result) == ["temperature", "value"]


def test_numeric_field_names_empty_result():
    assert eh.numeric_field_names({"results": [{"statement_id": 0}]}) == []


def test_rows_from_select_result_empty():
    assert eh.rows_from_select_result({"results": [{"statement_id": 0}]}) == []


def test_rows_from_select_result_single_series():
    result = {
        "results": [
            {
                "series": [
                    {
                        "name": "climate",
                        "tags": {"entity_id": "bed_1_aircon"},
                        "columns": ["time", "temperature"],
                        "values": [["2026-06-01T00:00:00Z", 21.5]],
                    }
                ]
            }
        ]
    }
    rows = eh.rows_from_select_result(result)
    assert len(rows) == 1
    tags, columns, values = rows[0]
    assert tags == {"entity_id": "bed_1_aircon"}
    assert columns == ["time", "temperature"]
    assert values == [["2026-06-01T00:00:00Z", 21.5]]


# --------------------------------------------------------------------------
# series_suffix
# --------------------------------------------------------------------------


def test_series_suffix_prefers_explicit_entity_id():
    assert eh.series_suffix({"entity_id": "other"}, "bed_1") == "bed_1"


def test_series_suffix_falls_back_to_tag_entity_id():
    assert eh.series_suffix({"entity_id": "adelaide_west_terrace"}, None) == "adelaide_west_terrace"


def test_series_suffix_falls_back_to_other_tags():
    assert eh.series_suffix({"entity_id": "", "location": "Adelaide"}, None) == "location-Adelaide"


def test_series_suffix_falls_back_to_all():
    assert eh.series_suffix({}, None) == "all"


# --------------------------------------------------------------------------
# env / config
# --------------------------------------------------------------------------


def test_parse_env_file_basic():
    text = "INFLUX_URL=http://localhost:8086\n# comment\n\nINFLUX_USER=readonly\n"
    assert eh.parse_env_file(text) == {
        "INFLUX_URL": "http://localhost:8086",
        "INFLUX_USER": "readonly",
    }


def test_resolve_config_env_var_overrides_file():
    file_text = "INFLUX_PASSWORD=fromfile\nINFLUX_URL=http://fromfile:8086\n"
    cfg = eh.resolve_config(file_text, {"INFLUX_PASSWORD": "fromenv"})
    assert cfg["INFLUX_PASSWORD"] == "fromenv"
    assert cfg["INFLUX_URL"] == "http://fromfile:8086"


def test_resolve_config_defaults_when_no_file_or_env():
    cfg = eh.resolve_config(None, {})
    assert cfg["INFLUX_URL"] == eh.DEFAULT_INFLUX_URL
    assert cfg["INFLUX_USER"] == eh.DEFAULT_INFLUX_USER
    assert cfg["INFLUX_DB"] == eh.DEFAULT_INFLUX_DB
    assert "INFLUX_PASSWORD" not in cfg


def test_load_entities_config_skips_comment_keys(tmp_path):
    p = tmp_path / "entities.json"
    p.write_text('{"_comment": "x", "climate": {"rp": "rp_raw", "entities": null}}')
    cfg = eh.load_entities_config(p)
    assert cfg == {"climate": {"rp": "rp_raw", "entities": None}}


# --------------------------------------------------------------------------
# CSV writing
# --------------------------------------------------------------------------


def test_write_series_csv_long_format(tmp_path):
    import csv
    import gzip

    path = tmp_path / "out.csv.gz"
    columns = ["time", "value", "extra"]
    values = [
        ["2026-06-01T00:00:00Z", 21.5, None],
        ["2026-06-01T00:05:00Z", 21.7, 3],
    ]
    n = eh.write_series_csv(path, columns, values)
    assert n == 3  # one None skipped
    with gzip.open(path, "rt", newline="") as f:
        rows = list(csv.reader(f))
    assert rows[0] == ["time", "field", "value"]
    assert rows[1] == ["2026-06-01T00:00:00Z", "value", "21.5"]
    assert rows[2] == ["2026-06-01T00:05:00Z", "value", "21.7"]
    assert rows[3] == ["2026-06-01T00:05:00Z", "extra", "3"]


# --------------------------------------------------------------------------
# date range resolution (argparse-adjacent, pure)
# --------------------------------------------------------------------------


def test_resolve_date_range_default_days():
    args = eh.parse_args(["--days", "2"])
    today = date(2026, 7, 2)
    start, end = eh.resolve_date_range(args, today)
    assert (start, end) == (date(2026, 7, 1), date(2026, 7, 2))


def test_resolve_date_range_explicit_start_end():
    args = eh.parse_args(["--start", "2026-06-01", "--end", "2026-06-05"])
    today = date(2026, 7, 2)
    start, end = eh.resolve_date_range(args, today)
    assert (start, end) == (date(2026, 6, 1), date(2026, 6, 5))


# --------------------------------------------------------------------------
# Live smoke test -- guarded, skipped unless explicitly enabled
# --------------------------------------------------------------------------


@pytest.mark.skipif(
    not os.environ.get("RUN_LIVE_INFLUX_TESTS"),
    reason="set RUN_LIVE_INFLUX_TESTS=1 and INFLUX_PASSWORD to run against a live InfluxDB",
)
def test_live_influx_query_smoke():
    password = os.environ["INFLUX_PASSWORD"]
    result = eh.influx_query(
        eh.DEFAULT_INFLUX_URL, eh.DEFAULT_INFLUX_USER, password, eh.DEFAULT_INFLUX_DB,
        "SHOW MEASUREMENTS",
    )
    assert "results" in result
