"""Capture whole-cycle golden fixtures by running every scenario through the
headless harness and journaling all service calls and entity writes.

Run: `uv run python tests/capture_cycle_fixtures.py`
Goldens define current behaviour; regenerate only when a behaviour change is
intended and reviewed.
"""

import json
from pathlib import Path

from hvac_harness import run_scenario
from scenarios import ALL_SCENARIOS

FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures" / "cycles"


def main():
    FIXTURES_DIR.mkdir(parents=True, exist_ok=True)
    for factory in ALL_SCENARIOS:
        scenario = factory()
        journal = run_scenario(scenario)
        path = FIXTURES_DIR / f"{scenario['name']}.json"
        with open(path, "w", encoding="utf-8") as f:
            json.dump(journal, f, indent=1)
        print(f"{scenario['name']}: {len(journal)} journal entries -> {path.name}")


if __name__ == "__main__":
    main()
