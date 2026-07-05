"""Oracle A/B for ideas.md #1 -- 'bulk-state estimator in the controller'.

The room sensors read the fast measured-air lead node Tm, not the bulk
mass T (docs/tasks/009): while heating Tm = T + lead*q; after a stop Tm
sags toward T. This script bounds the payoff of controlling on the bulk
state by feeding the controller the *true* bulk T (a perfect de-leaded
estimator) while comfort is still scored on Tm+offset (the experienced
signal) -- so the comparison is the best achievable case for any real
estimator. Only what _write_room_temps presents to actrl changes; plant,
HVAC and scoring are untouched (same A/B channel as the tuning sweeps).

Findings recorded in docs/tuning.md ("Bulk-state estimator -- oracle A/B").

Usage:
    uv run python analysis/oracle_bulk_estimator.py \
        [--days 2026-06-09,2026-06-22,...]
"""
from __future__ import annotations

import argparse
import statistics
import sys
from pathlib import Path
from unittest import mock

_ROOT = Path(__file__).resolve().parent.parent
for p in (str(_ROOT), str(_ROOT / "tests")):
    if p not in sys.path:
        sys.path.insert(0, p)

import sim.closed_loop as cl  # noqa: E402
from analysis.tune import score_one_day  # noqa: E402

DEFAULT_DAYS = ["2026-06-09", "2026-06-21", "2026-06-22", "2026-06-27", "2026-06-30"]
KEYS = [
    "time_in_band", "deg_min_below", "deg_min_above", "overshoot_max",
    "overshoot_rise_max", "osc_per_h", "energy_kwh", "starts",
    "abovemin_frac", "rise_time_med",
]

_orig = cl.ClosedLoop._write_room_temps


def _oracle_write(self):
    """Present the true bulk node house.temps (not temps_measured) to the
    controller. Everything else -- the feels-like ctrl_offsets, the
    telemetry, the Tm-based scoring -- is unchanged."""
    for room in cl.ROOMS:
        t = self.house.temps[room] + self._ctrl_offsets.get(room, 0.0)
        self.world.update(f"sensor.{room}_average_temperature", {"state": f"{t:.2f}"})


def run(days, parquet, oracle: bool) -> dict[str, dict]:
    patch = _oracle_write if oracle else _orig
    out: dict[str, dict] = {}
    with mock.patch.object(cl.ClosedLoop, "_write_room_temps", patch):
        for d in days:
            try:
                out[d] = score_one_day(parquet, d, {})
            except Exception as e:  # noqa: BLE001
                print(f"  {d} skipped: {e}")
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--days", default=",".join(DEFAULT_DAYS))
    ap.add_argument("--parquet", default=_ROOT / "data/processed/june.parquet", type=Path)
    args = ap.parse_args()
    days = [d.strip() for d in args.days.split(",") if d.strip()]

    base = run(days, args.parquet, oracle=False)
    orc = run(days, args.parquet, oracle=True)
    days = [d for d in days if d in base and d in orc]

    print(f"\n{'metric':<20}{'day':<12}{'base':>10}{'oracle':>10}{'delta':>10}")
    for k in KEYS:
        for d in days:
            b, o = base[d][k], orc[d][k]
            print(f"{k:<20}{d:<12}{b:>10.3f}{o:>10.3f}{o - b:>+10.3f}")
        bm = statistics.median(base[d][k] for d in days)
        om = statistics.median(orc[d][k] for d in days)
        print(f"{k:<20}{'MEDIAN':<12}{bm:>10.3f}{om:>10.3f}{om - bm:>+10.3f}\n")


if __name__ == "__main__":
    main()
