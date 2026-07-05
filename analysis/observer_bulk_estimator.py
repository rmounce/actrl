"""Realizable-observer A/B for ideas.md #1 -- follow-up to the oracle run.

The oracle A/B (analysis/oracle_bulk_estimator.py, docs/tuning.md) bounded
the payoff of controlling on the bulk temperature by feeding the
controller the true bulk T. This script tests whether an observer built
ONLY from controller-observable signals recovers that bound:

    T_est = Tm + tau_meas * dTm/dt - lead * q_room

which inverts the fitted lead-node ODE (sim/house.py, docs/tasks/009)
exactly when the derivative and q are perfect. The realizable version
degrades each input to what production actrl could actually have:

- Tm quantized to 0.01 K (the sensor write resolution) at the 10 s cycle,
  so the derivative comes from a first-order-filtered finite difference
  (--deriv-tau, default 120 s) rather than the true dTm/dt. Quantization
  alone puts ~3.6 K/h of noise on the raw 10 s difference; the filter
  trades that against lag on the post-stop sag (tau_meas is 13-17 min,
  so a ~2 min filter should be tolerable).
- q_room from the PREVIOUS cycle's electrical power (production reads
  power.outdoor_unit) through the fitted COP model and the commanded
  damper shares -- same distribution formula as the plant, but with no
  knowledge of the heat-delivery dead time/lag or of defrost (during
  defrost the proxy sees ~1 kW and infers heat that isn't delivered).
- tau_meas/lead are the fitted per-room constants (a deployed observer
  would ship with the calibration values).

Comfort is scored on Tm + offset in every arm, as in the oracle run.
Recovery fraction reported per metric: (observer - base) / (oracle - base).

Usage:
    uv run python analysis/observer_bulk_estimator.py \
        [--days 2026-06-09,...] [--deriv-tau 120] [--arms base,oracle,observer]
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
DT_H = 10.0 / 3600.0

_orig = cl.ClosedLoop._write_room_temps


def _oracle_write(self):
    """Perfect estimator: present the true bulk node (upper bound)."""
    for room in cl.ROOMS:
        t = self.house.temps[room] + self._ctrl_offsets.get(room, 0.0)
        self.world.update(f"sensor.{room}_average_temperature", {"state": f"{t:.2f}"})


def make_observer_write(deriv_tau_s: float, bias: float = 0.0):
    """`bias` [K] is added to what the controller reads (reads warm =>
    heats less) -- trims the observer's systematic cold bias (derivative
    filter lag on warmups + defrost-blind q proxy) back toward
    equal-energy, per the analysis/equal_comfort_bulk.py sweep."""
    alpha = 10.0 / deriv_tau_s

    def _observer_write(self):
        st = getattr(self, "_obs_state", None)
        if st is None:
            st = self._obs_state = {
                r: {"tm": round(self.house.temps_measured[r], 2), "d": 0.0}
                for r in cl.ROOMS
            }
        # q proxy from the previous cycle's telemetry (power-meter latency);
        # no q on the very first cycle, and none inferable while off.
        if self.history:
            prev = self.history[-1]
            p_kw = prev["p_kw"]
            q_kw = self.hvac.cop(p_kw, prev["t_out"]) * p_kw if p_kw > 0 else 0.0
        else:
            q_kw = 0.0
        q_rooms = (
            self._room_q(q_kw / self.hvac.params.c_eff_kwh_per_k)
            if q_kw
            else {r: 0.0 for r in cl.ROOMS}
        )
        for room in cl.ROOMS:
            s = st[room]
            tm_q = round(self.house.temps_measured[room], 2)  # sensor resolution
            s["d"] += alpha * ((tm_q - s["tm"]) / DT_H - s["d"])
            s["tm"] = tm_q
            p = self.house.params.rooms[room]
            t_est = tm_q + p.tau_meas_h * s["d"] - p.lead_h * q_rooms[room]
            t = t_est + bias + self._ctrl_offsets.get(room, 0.0)
            self.world.update(
                f"sensor.{room}_average_temperature", {"state": f"{t:.2f}"}
            )

    return _observer_write


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
    ap.add_argument("--parquet", default=_ROOT / "data/processed/june.parquet", type=Path)
    ap.add_argument("--deriv-tau", type=float, default=120.0,
                    help="derivative low-pass time constant [s]")
    ap.add_argument("--bias", type=float, default=0.0,
                    help="warm trim [K] added to the observer's reads")
    ap.add_argument("--arms", default="base,oracle,observer")
    args = ap.parse_args()
    days = [d.strip() for d in args.days.split(",") if d.strip()]
    wanted = [a.strip() for a in args.arms.split(",") if a.strip()]

    writers = {
        "base": _orig,
        "oracle": _oracle_write,
        "observer": make_observer_write(args.deriv_tau, args.bias),
    }
    results: dict[str, dict[str, dict]] = {}
    for arm in wanted:
        print(f"running {arm} ...", flush=True)
        results[arm] = run(days, args.parquet, writers[arm])
    days = [d for d in days if all(d in results[a] for a in wanted)]

    header = f"{'metric':<20}{'day':<12}" + "".join(f"{a:>10}" for a in wanted)
    if {"base", "oracle", "observer"} <= set(wanted):
        header += f"{'recovery':>10}"
    print("\n" + header)
    for k in KEYS:
        meds = {}
        for a in wanted:
            meds[a] = statistics.median(results[a][d][k] for d in days)
        for d in days:
            line = f"{k:<20}{d:<12}" + "".join(
                f"{results[a][d][k]:>10.3f}" for a in wanted
            )
            print(line)
        line = f"{k:<20}{'MEDIAN':<12}" + "".join(f"{meds[a]:>10.3f}" for a in wanted)
        if {"base", "oracle", "observer"} <= set(wanted):
            gap = meds["oracle"] - meds["base"]
            rec = (meds["observer"] - meds["base"]) / gap if abs(gap) > 1e-9 else float("nan")
            line += f"{rec:>10.2f}"
        print(line + "\n")


if __name__ == "__main__":
    main()
