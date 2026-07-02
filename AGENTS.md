# Agent Notes

## Collaboration

- Keep worktree clean: check `git status` before finishing.
- Own your files: commit, ignore, or remove generated files.
- Inherited dirty/untracked state: ask before substantial work.
- Scratch/question files: do not commit unless explicitly requested.
- User preference change: offer to update `AGENTS.md`.
- Commit regularly; avoid noisy commits.
- Rename/refactor own code: migrate all call sites; no back-compat aliases. Don't accrue self-made tech debt.

## Project Documentation

- Keep `docs/` and `README.md` current.
- Behaviour change affecting docs: update docs in same work.
- Caveman compression: short bullets, concrete facts, decisions, commands, paths, status.
- Detailed implementation analysis: `docs/actrl.md`, `docs/statctrl.md`.
- Improvement proposals and roadmap: `docs/ideas.md`.

## External Systems

- Document confirmed black-box behaviour promptly.
- Midea AC quirks (follow-me protocol, ramp flags, purge cycle, defrost) are
  hard-won reverse engineering: preserve the comments in `actrl.py` and keep
  `docs/actrl.md` in sync. Never simplify a step sequence without evidence.
- Cover device/API quirks, HA entity lifecycle, mode/setpoint semantics, operational limits.
- Record concrete facts: date/context, command/service, observed state, remaining uncertainty.

## This Repository

- AppDaemon apps for Home Assistant; no standalone Python project (no venv,
  no requirements.txt). `hassapi` exists only in the AppDaemon runtime.
- Active apps: `actrl.py` (whole-of-house aircon control), `statctrl.py`
  (per-room setpoint scheduling). `archive/` holds retired experiments.
- `appdaemon/` is a gitignored deployed copy of the apps. Top-level files are
  the source of truth; a difference means undeployed work. Do not edit
  `appdaemon/` directly. Deploy process: confirm with user before assuming.
- Control code runs a real house's HVAC. Changes to control logic need user
  review before deployment; prefer adding tests/simulation over live trial.

## Python Environment

- Package manager: `uv`, not raw `pip` (if/when tooling is added).
- Target Python: whatever the AppDaemon container runs; keep code dependency-free
  beyond stdlib + AppDaemon APIs unless agreed otherwise.
