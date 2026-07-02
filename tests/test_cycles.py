"""Whole-cycle golden tests: replay each scenario through the headless
harness and require the journal (service calls + entity writes) to match the
committed fixture exactly."""

import json
from pathlib import Path

import pytest

from hvac_harness import run_scenario
from scenarios import ALL_SCENARIOS

FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures" / "cycles"


@pytest.mark.parametrize("factory", ALL_SCENARIOS, ids=lambda f: f.__name__)
def test_scenario_matches_golden(factory):
    scenario = factory()
    journal = json.loads(json.dumps(run_scenario(scenario)))
    with open(FIXTURES_DIR / f"{scenario['name']}.json") as f:
        golden = json.load(f)
    assert len(journal) == len(golden)
    for i, (got, want) in enumerate(zip(journal, golden)):
        assert got == want, f"journal entry {i} diverged: {got} != {want}"
