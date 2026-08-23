# 015: Reduce actrl log volume

Status: done
Branch: task/015-log-volume

## Goal

Keep operational events visible at INFO while moving ten-second control-loop telemetry
to DEBUG and logging pause state only when it changes.

## Background

`actrl` is the largest repeated Docker journal producer. Journald write amplification
makes low-value INFO messages disproportionately expensive on the root NVMe mirror.
Manual mode currently repeats its unchanged pause message every ten seconds.

## Steps

- Log pause entry and exit once without changing reset/status behaviour.
- Use `input_boolean.actrl_debug_logging` to switch this app between INFO and DEBUG
  at runtime without a reload.
- Mark cycle summaries, target ramp progress, PID/capacity telemetry, forecast and
  price summaries, and unchanged damper observations DEBUG.
- Preserve actuator actions, mode transitions, warnings and failures at INFO or above.
- Test log levels and unchanged golden control journals.

## Out of scope / do not modify

- Control decisions, constants, HA entities or service calls.
- `statctrl.py` and the deployed `appdaemon/` copy.
- Docker's journald driver.

## Acceptance criteria (runnable)

- `pytest -q`
- `git diff --check`
- `git status --short`

## Questions

None.

## Log

- 2026-08-23: task created from measured journal-volume review; implementation started.
- 2026-08-23: implementation complete; 212 tests passed, one unrelated test skipped;
  golden control journals unchanged. Awaiting review/merge/deployment.
- 2026-08-23: added the requested HA runtime DEBUG toggle; 213 tests passed with one
  unrelated skip and golden control journals unchanged.
- 2026-08-23: reviewed, fast-forwarded to master, and deployed through `deploy.sh`
  while HA Manual Mode was on. AppDaemon hot reload succeeded; pause logging reduced
  to one entry and the HA DEBUG toggle was verified on/off without app reload.
