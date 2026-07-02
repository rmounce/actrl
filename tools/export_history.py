#!/usr/bin/env python3
"""Export HA history from InfluxDB into per-day gitignored CSVs for
simulator calibration (docs/ideas.md #3).

Stdlib only. See docs/data.md for schema notes and usage. Idempotent:
running twice with the same date range skips files that already exist
(use --force to re-fetch).

Credentials: read from the environment (INFLUX_URL, INFLUX_USER,
INFLUX_DB, INFLUX_PASSWORD) or from a gitignored tools/influx.env file
(see tools/influx.env.example). Never write the password to a tracked
file.
"""
from __future__ import annotations

import argparse
import base64
import csv
import gzip
import json
import re
import sys
import urllib.error
import urllib.parse
import urllib.request
from datetime import date, datetime, timedelta, timezone
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
REPO_ROOT = TOOLS_DIR.parent
DEFAULT_ENTITIES_CONFIG = TOOLS_DIR / "export_entities.json"
DEFAULT_ENV_FILE = TOOLS_DIR / "influx.env"
DEFAULT_DATA_DIR = REPO_ROOT / "data"

DEFAULT_INFLUX_URL = "http://localhost:8086"
DEFAULT_INFLUX_USER = "readonly"
DEFAULT_INFLUX_DB = "hass"


class InfluxError(RuntimeError):
    """Connection, auth, or query error talking to InfluxDB."""


# --------------------------------------------------------------------------
# Pure helpers (unit tested offline in tests/test_export_history.py)
# --------------------------------------------------------------------------


def parse_env_file(text: str) -> dict:
    """Parse simple KEY=VALUE lines (as in tools/influx.env). Ignores blank
    lines and lines starting with '#'."""
    env = {}
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            continue
        key, _, value = line.partition("=")
        env[key.strip()] = value.strip()
    return env


def resolve_config(env_file_text: str | None, environ: dict) -> dict:
    """Merge influx.env file contents with real environment variables.
    Real environment variables win over the file."""
    cfg = {}
    if env_file_text is not None:
        cfg.update(parse_env_file(env_file_text))
    for key in ("INFLUX_URL", "INFLUX_USER", "INFLUX_DB", "INFLUX_PASSWORD"):
        if key in environ:
            cfg[key] = environ[key]
    cfg.setdefault("INFLUX_URL", DEFAULT_INFLUX_URL)
    cfg.setdefault("INFLUX_USER", DEFAULT_INFLUX_USER)
    cfg.setdefault("INFLUX_DB", DEFAULT_INFLUX_DB)
    return cfg


def day_ranges(start: date, end: date) -> list[tuple[str, str, str]]:
    """Inclusive [start, end] UTC day range, chunked per day.

    Returns list of (day_str, start_iso, end_iso) where [start_iso, end_iso)
    covers one UTC calendar day.
    """
    if end < start:
        raise ValueError(f"end {end} before start {start}")
    days = []
    d = start
    while d <= end:
        day_str = d.isoformat()
        next_day = d + timedelta(days=1)
        start_iso = f"{day_str}T00:00:00Z"
        end_iso = f"{next_day.isoformat()}T00:00:00Z"
        days.append((day_str, start_iso, end_iso))
        d = next_day
    return days


def sanitize_component(value: str) -> str:
    """Sanitize a string for use as a filename component."""
    value = value.strip() or "unknown"
    return re.sub(r"[^A-Za-z0-9_.-]", "_", value)


def output_path(data_dir: Path, day_str: str, measurement: str, suffix: str) -> Path:
    fname = f"{sanitize_component(measurement)}__{sanitize_component(suffix)}.csv.gz"
    return data_dir / "raw" / day_str / fname


def build_field_keys_query(rp: str, measurement: str) -> str:
    return f'SHOW FIELD KEYS FROM "{rp}"."{measurement}"'


def build_select_query(
    rp: str,
    measurement: str,
    fields: list[str],
    start_iso: str,
    end_iso: str,
    entity_id: str | None,
) -> str:
    if not fields:
        raise ValueError("fields must be non-empty")
    field_list = ", ".join(f'"{f}"' for f in fields)
    measurement_ref = f'"{rp}"."{measurement}"'
    where = f"time >= '{start_iso}' AND time < '{end_iso}'"
    if entity_id is not None:
        escaped = entity_id.replace("'", r"\'")
        where += f" AND entity_id = '{escaped}'"
    return f"SELECT {field_list} FROM {measurement_ref} WHERE {where} GROUP BY *"


def build_query_url(base_url: str, db: str, q: str) -> str:
    params = urllib.parse.urlencode({"db": db, "q": q})
    return f"{base_url.rstrip('/')}/query?{params}"


def series_suffix(tags: dict, fallback_entity_id: str | None) -> str:
    """Derive a filename suffix from a query result series' tags."""
    if fallback_entity_id:
        return fallback_entity_id
    entity_id = tags.get("entity_id")
    if entity_id:
        return entity_id
    non_empty = {k: v for k, v in tags.items() if v}
    if not non_empty:
        return "all"
    return "_".join(f"{k}-{v}" for k, v in sorted(non_empty.items()))


def numeric_field_names(field_keys_result: dict) -> list[str]:
    """Given the parsed JSON of a SHOW FIELD KEYS response, return the
    sorted list of fieldType=float field names."""
    names = []
    for result in field_keys_result.get("results", []):
        for series in result.get("series", []):
            columns = series.get("columns", [])
            try:
                key_idx = columns.index("fieldKey")
                type_idx = columns.index("fieldType")
            except ValueError:
                continue
            for row in series.get("values", []):
                if row[type_idx] == "float":
                    names.append(row[key_idx])
    return sorted(set(names))


def rows_from_select_result(select_result: dict) -> list[tuple[dict, list, list]]:
    """Given the parsed JSON of a SELECT ... GROUP BY * response, return a
    list of (tags, columns, values) per series. Empty list if no data."""
    out = []
    for result in select_result.get("results", []):
        for series in result.get("series", []):
            out.append((series.get("tags", {}), series["columns"], series["values"]))
    return out


# --------------------------------------------------------------------------
# Network I/O
# --------------------------------------------------------------------------


def influx_query(base_url: str, user: str, password: str, db: str, q: str, timeout: float = 30.0) -> dict:
    url = build_query_url(base_url, db, q)
    req = urllib.request.Request(url)
    credentials = base64.b64encode(f"{user}:{password}".encode()).decode("ascii")
    req.add_header("Authorization", f"Basic {credentials}")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            data = json.load(resp)
    except urllib.error.HTTPError as e:
        body = e.read().decode("utf-8", "replace") if e.fp else ""
        raise InfluxError(f"HTTP {e.code} querying InfluxDB: {body or e.reason}") from e
    except urllib.error.URLError as e:
        raise InfluxError(f"Could not reach InfluxDB at {base_url}: {e.reason}") from e
    for result in data.get("results", []):
        if "error" in result:
            raise InfluxError(f"InfluxDB query error: {result['error']} (query: {q})")
    return data


# --------------------------------------------------------------------------
# Export logic
# --------------------------------------------------------------------------


def write_series_csv(path: Path, columns: list[str], values: list[list]) -> int:
    """Write one series (wide rows) as long-format (time, field, value)
    rows to a gzip CSV. Returns the number of data rows written."""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    n = 0
    with gzip.open(tmp_path, "wt", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "field", "value"])
        for row in values:
            time_val = row[0]
            for idx in range(1, len(columns)):
                v = row[idx]
                if v is None:
                    continue
                writer.writerow([time_val, columns[idx], v])
                n += 1
    tmp_path.replace(path)
    return n


def export_measurement(
    *,
    base_url: str,
    user: str,
    password: str,
    db: str,
    measurement: str,
    cfg: dict,
    days: list[tuple[str, str, str]],
    data_dir: Path,
    force: bool,
    field_cache: dict,
    log,
) -> None:
    rp = cfg.get("rp", "rp_raw")
    entities = cfg.get("entities")

    fields = cfg.get("fields")
    if fields is None:
        cache_key = (rp, measurement)
        if cache_key not in field_cache:
            fk_result = influx_query(
                base_url, user, password, db, build_field_keys_query(rp, measurement)
            )
            field_cache[cache_key] = numeric_field_names(fk_result)
        fields = field_cache[cache_key]
    if not fields:
        log(f"WARNING: no numeric fields found for {rp}.{measurement}, skipping")
        return

    for day_str, start_iso, end_iso in days:
        targets = entities if entities is not None else [None]
        for entity_id in targets:
            expected_path = None
            if entity_id is not None:
                expected_path = output_path(data_dir, day_str, measurement, entity_id)
                if expected_path.exists() and not force:
                    log(f"skip (exists): {expected_path}")
                    continue

            q = build_select_query(rp, measurement, fields, start_iso, end_iso, entity_id)
            try:
                result = influx_query(base_url, user, password, db, q)
            except InfluxError:
                raise

            series_list = rows_from_select_result(result)
            if not series_list:
                who = entity_id or measurement
                log(f"WARNING: empty result for {rp}.{measurement} entity={who} on {day_str}")
                continue

            for tags, columns, values in series_list:
                suffix = series_suffix(tags, entity_id)
                out_path = output_path(data_dir, day_str, measurement, suffix)
                if out_path.exists() and not force:
                    log(f"skip (exists): {out_path}")
                    continue
                n = write_series_csv(out_path, columns, values)
                log(f"wrote {n} rows -> {out_path}")


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--days", type=int, default=30, help="Export the last N UTC days (default 30).")
    p.add_argument("--start", type=str, help="ISO date (YYYY-MM-DD), overrides --days start.")
    p.add_argument("--end", type=str, help="ISO date (YYYY-MM-DD), overrides --days end (default: today).")
    p.add_argument("--force", action="store_true", help="Re-fetch and overwrite existing files.")
    p.add_argument(
        "--entities-config",
        type=Path,
        default=DEFAULT_ENTITIES_CONFIG,
        help="Path to export_entities.json (default: tools/export_entities.json).",
    )
    p.add_argument(
        "--data-dir",
        type=Path,
        default=DEFAULT_DATA_DIR,
        help="Output data directory (default: data/, gitignored).",
    )
    p.add_argument(
        "--env-file",
        type=Path,
        default=DEFAULT_ENV_FILE,
        help="Path to influx.env (default: tools/influx.env).",
    )
    return p.parse_args(argv)


def resolve_date_range(args: argparse.Namespace, today: date) -> tuple[date, date]:
    end = date.fromisoformat(args.end) if args.end else today
    if args.start:
        start = date.fromisoformat(args.start)
    else:
        start = end - timedelta(days=args.days - 1)
    return start, end


def load_entities_config(path: Path) -> dict:
    with open(path) as f:
        raw = json.load(f)
    return {k: v for k, v in raw.items() if not k.startswith("_")}


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    env_file_text = None
    if args.env_file.exists():
        env_file_text = args.env_file.read_text()
    import os

    cfg = resolve_config(env_file_text, dict(os.environ))
    password = cfg.get("INFLUX_PASSWORD")
    if not password:
        print(
            "ERROR: INFLUX_PASSWORD not set (env var or tools/influx.env). "
            "See tools/influx.env.example.",
            file=sys.stderr,
        )
        return 2

    try:
        entities_config = load_entities_config(args.entities_config)
    except (OSError, json.JSONDecodeError) as e:
        print(f"ERROR: could not load {args.entities_config}: {e}", file=sys.stderr)
        return 2

    start, end = resolve_date_range(args, datetime.now(timezone.utc).date())
    days = day_ranges(start, end)

    def log(msg: str) -> None:
        print(msg, file=sys.stderr)

    log(f"Exporting {len(days)} day(s) [{start} .. {end}] to {args.data_dir}")

    field_cache: dict = {}
    try:
        for measurement, mcfg in entities_config.items():
            export_measurement(
                base_url=cfg["INFLUX_URL"],
                user=cfg["INFLUX_USER"],
                password=password,
                db=cfg["INFLUX_DB"],
                measurement=measurement,
                cfg=mcfg,
                days=days,
                data_dir=args.data_dir,
                force=args.force,
                field_cache=field_cache,
                log=log,
            )
    except InfluxError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1

    log("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
