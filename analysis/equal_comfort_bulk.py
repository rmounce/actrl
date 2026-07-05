"""Equal-comfort re-run for ideas.md #1 -- the oracle's open question (a).

The oracle A/B (docs/tuning.md "Bulk-state estimator") showed the win is a
comfort/energy rebalance: deg_min_below -58% for +9% energy. This sweep
isolates the pure cycling/energy benefit by spending the saved cold-time
back: the oracle arm additionally reads every room `delta` K warm (the
controller believes rooms are delta warmer, equivalent to lowering its
targets by delta -- scoring bands are untouched), swept until the median
deg_min_below returns to the baseline's. At that matched-comfort point the
remaining deltas (energy, osc_per_h, starts, abovemin_frac) are the
estimator's free benefit.

Usage:
    uv run python analysis/equal_comfort_bulk.py \
        [--days 2026-06-09,...] [--deltas 0.0,0.1,0.2,0.3,0.4]
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
    "time_in_band", "deg_min_below", "deg_min_above", "overshoot_rise_max",
    "osc_per_h", "energy_kwh", "starts", "abovemin_frac", "rise_time_med",
]

_orig = cl.ClosedLoop._write_room_temps


def make_oracle_write(delta: float):
    """Oracle bulk-T controller reads, biased +delta K (reads warm =>
    heats less; equivalent to targets lowered by delta)."""

    def _write(self):
        for room in cl.ROOMS:
            t = self.house.temps[room] + delta + self._ctrl_offsets.get(room, 0.0)
            self.world.update(
                f"sensor.{room}_average_temperature", {"state": f"{t:.2f}"}
            )

    return _write


def run(days, parquet, write_fn) -> dict[str, dict]:
    out: dict[str, dict] = {}
    with mock.patch.object(cl.ClosedLoop, "_write_room_temps", write_fn):
        for d in days:
            try:
                out[d] = score_one_day(parquet, d, {})
            except Exception as e:  # noqa: BLE001
                print(f"  {d} skipped: {e}")
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--days", default=",".join(DEFAULT_DAYS))
    ap.add_argument("--deltas", default="0.0,0.1,0.2,0.3,0.4")
    ap.add_argument("--parquet", default=_ROOT / "data/processed/june.parquet", type=Path)
    args = ap.parse_args()
    days = [d.strip() for d in args.days.split(",") if d.strip()]
    deltas = [float(x) for x in args.deltas.split(",")]

    print("running baseline ...", flush=True)
    arms: dict[str, dict[str, dict]] = {"base": run(days, args.parquet, _orig)}
    for delta in deltas:
        name = f"or+{delta:.1f}"
        print(f"running {name} ...", flush=True)
        arms[name] = run(days, args.parquet, make_oracle_write(delta))
    days = [d for d in days if all(d in a for a in arms.values())]

    names = list(arms)
    print(f"\nper-day, then MEDIAN and SUM over {len(days)} days:")
    print(f"{'metric':<20}{'day':<12}" + "".join(f"{n:>10}" for n in names))
    for k in KEYS:
        for d in days:
            print(f"{k:<20}{d:<12}" + "".join(f"{arms[n][d][k]:>10.3f}" for n in names))
        print(f"{k:<20}{'MEDIAN':<12}" + "".join(
            f"{statistics.median(arms[n][d][k] for d in days):>10.3f}" for n in names))
        # Medians hide cold-day dominance (they pick the middle day); the
        # summed column is the honest whole-period energy/comfort total.
        print(f"{k:<20}{'SUM':<12}" + "".join(
            f"{sum(arms[n][d][k] for d in days):>10.3f}" for n in names) + "\n")


if __name__ == "__main__":
    main()
