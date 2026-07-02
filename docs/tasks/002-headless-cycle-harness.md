# 002: Headless whole-cycle harness and golden scenario tests

Status: done
Branch: (implemented directly on master by the spec author)

## Goal

A FakeHass harness that runs `actrl.Actrl.initialize()` + `main()` cycles
without Home Assistant, journaling every service call and entity write, with
deterministic scenarios captured as golden fixtures. Zero product-code
change. Enables safe refactoring (task 003) and is the seed of the future
simulator harness (ideas.md §3).

## Outcome

Implemented directly by Claude Fable (not delegated — full context made
direct implementation cheaper than spec + review):

- `tests/hvac_harness.py` — FakeWorld/FakeEntity/HarnessActrl implementing
  the AppDaemon API subset actrl uses (`get_state` incl. domain queries and
  attributes, `get_entity`, `call_service` with instant-actuation emulation
  for covers/climate/number, `run_every` capture, `log` capture,
  `time.sleep` patched out).
- `tests/scenarios.py` — six deterministic scenarios: heat_approach,
  stays_off, cool_grid_surplus, window_open_heat, defrost_heat, manual_mode.
- `tests/capture_cycle_fixtures.py` — regenerates goldens (only when a
  behaviour change is intended and reviewed).
- `tests/test_cycles.py` — replays scenarios, requires exact journal match.

## Log

- 2026-07-02: implemented, 6 scenario goldens captured, 11 tests green
  (5 unit + 6 cycle). Merged as part of master commit 1a43bf7.
