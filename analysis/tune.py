"""Comfort/cost tuning CLI (docs/tasks/014): score a candidate controller
config against replayed June days.

Thin glue over the existing pieces -- day loading/replay is
`analysis/replay_day.py` unchanged, the metrics are `analysis/comfort.py`,
and a config override is applied for the whole build+replay via
`analysis/ctrl_overrides.py`. This module does no sweeping itself (that's
a later task); it prints one config's per-day metrics table plus the
across-day medians.

Usage:
    uv run python analysis/tune.py --days 2026-06-21,2026-06-22 \
        [--parquet data/processed/june.parquet] \
        [--set actrl.global_ki=0.0005 --set control.global_deadband_ki=0.02] \
        [--out metrics.csv]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import pandas as pd

_ROOT = Path(__file__).resolve().parent.parent
for p in (str(_ROOT), str(_ROOT / "tests")):
    if p not in sys.path:
        sys.path.insert(0, p)

from analysis.comfort import score_day, summarize  # noqa: E402
from analysis.ctrl_overrides import ctrl_overrides  # noqa: E402
from analysis.replay_day import feels_like_offsets, load_day, replay  # noqa: E402
from sim.closed_loop import ROOMS  # noqa: E402


def parse_overrides(pairs: list[str]) -> dict[str, object]:
    """"module.attr=literal" strings -> {key: value}, preserving the
    existing constant's int/float type (docs/tasks/014 "Parses --set
    values..."). Imports actrl/control locally (only when --set is used,
    never at module import time) purely to read the current attr's type;
    the actual override application/typo-guard happens in ctrl_overrides.
    """
    import actrl  # noqa: E402
    import control  # noqa: E402

    modules = {"actrl": actrl, "control": control}
    out: dict[str, object] = {}
    for pair in pairs:
        key, sep, literal = pair.partition("=")
        if not sep:
            raise SystemExit(f"--set {pair!r} must be KEY=VALUE, e.g. actrl.global_ki=0.0005")
        prefix, dot, attr = key.partition(".")
        if not dot or prefix not in modules:
            raise SystemExit(f"--set {key!r}: module must be 'actrl' or 'control'")
        if not hasattr(modules[prefix], attr):
            raise SystemExit(f"--set {key!r}: no such attribute (typo?)")
        current = getattr(modules[prefix], attr)
        value: object = float(literal)
        if isinstance(current, int) and not isinstance(current, bool) and value.is_integer():
            value = int(value)
        out[key] = value
    return out


def build_comfort_frame(day: pd.DataFrame, sim: pd.DataFrame) -> pd.DataFrame:
    """Assemble the per-minute frame analysis/comfort.score_day expects:
    feels_{r} = sim Tm_{r} + the recorded feels-like offset (offsets are
    exogenous -- computed from recorded temp/humidity, see
    replay_day.feels_like_offsets); low_{r}/high_{r} from the recorded,
    ffilled target-band columns; p_kw/increment straight from the sim
    frame."""
    offsets = feels_like_offsets(day)
    frame = pd.DataFrame(index=sim.index)
    for r in ROOMS:
        frame[f"feels_{r}"] = sim[f"Tm_{r}"].to_numpy() + offsets[r]
        frame[f"low_{r}"] = day[f"climate.{r}_aircon.target_temp_low"].ffill().to_numpy()
        frame[f"high_{r}"] = day[f"climate.{r}_aircon.target_temp_high"].ffill().to_numpy()
    frame["p_kw"] = sim["p_kw"].to_numpy()
    frame["increment"] = sim["increment"].to_numpy()
    return frame


def score_one_day(
    parquet: Path, date: str, overrides: dict[str, object], ctrl_noise: dict | None = None
) -> dict:
    day = load_day(parquet, date)
    if ctrl_noise is not None:
        # Deterministic but day-distinct seed so reruns reproduce exactly
        # while days don't share a noise realisation.
        ctrl_noise = {**ctrl_noise, "seed": int(date.replace("-", ""))}
    # The override must wrap ClosedLoop construction (actrl builds its PIDs
    # once at initialize()) AND the whole replay (control.py re-reads its
    # globals every cycle) -- see analysis/ctrl_overrides.py docstring.
    # replay_day.replay() does both (it calls build_loop() internally),
    # so wrapping this single call satisfies both requirements.
    with ctrl_overrides(overrides):
        sim = replay(day, ctrl_noise=ctrl_noise)
    frame = build_comfort_frame(day, sim)
    return score_day(frame)


def _fmt(value: object) -> str:
    return f"{value:.3f}" if isinstance(value, float) else str(value)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--days", required=True, help="comma-separated local dates, e.g. 2026-06-21,2026-06-22")
    ap.add_argument("--parquet", default="data/processed/june.parquet", type=Path)
    ap.add_argument(
        "--set",
        dest="overrides",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="controller constant override, e.g. actrl.global_ki=0.0005 "
        "(repeatable)",
    )
    ap.add_argument("--out", type=Path, help="write per-day metrics CSV (+ a final median row)")
    ap.add_argument(
        "--noise-sigma",
        type=float,
        default=None,
        help="synthetic controller-read sensor noise std [K] (measured floor "
        "~0.006; stress above it -- the sub-minute band is unobserved)",
    )
    ap.add_argument(
        "--noise-tau",
        type=float,
        default=60.0,
        help="noise correlation time [s] for the AR(1) model (<=0 = white per 10 s cycle)",
    )
    args = ap.parse_args()

    ctrl_noise = {"sigma": args.noise_sigma, "tau_s": args.noise_tau} if args.noise_sigma else None
    overrides = parse_overrides(args.overrides)
    dates = [d.strip() for d in args.days.split(",") if d.strip()]

    rows: dict[str, dict] = {}
    columns: list[str] | None = None
    for date in dates:
        try:
            metrics = score_one_day(args.parquet, date, overrides, ctrl_noise)
        except (SystemExit, Exception) as exc:  # noqa: BLE001
            # Same broadened skip pattern as analysis/scorecard.py: some
            # archived days have full row counts but mid-day column
            # outages that crash or corrupt the replay downstream.
            print(f"{date}  skipped: {exc}")
            sys.stdout.flush()
            continue
        rows[date] = metrics
        if columns is None:
            columns = list(metrics.keys())
        print(f"{date}  " + " ".join(f"{c}={_fmt(metrics[c])}" for c in columns))
        sys.stdout.flush()

    if not rows:
        print("\nno days scored")
        return

    summary = summarize(list(rows.values()))
    print("\nsummary (median over scored days):")
    for k, v in summary.items():
        print(f"  {k:<18} {_fmt(v)}")

    if args.out:
        table = pd.DataFrame.from_dict(rows, orient="index")
        table.index.name = "date"
        table.loc["median"] = {c: summary[c] for c in table.columns}
        table.to_csv(args.out)
        print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
