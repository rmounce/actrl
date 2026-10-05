# 016: Separate XYE requested fan setting from device feedback

Status: review
Branch: task/016-xye-requested-fan-feedback
Implementation repo: ~/src/esphome
Base: xye-units-switch (verify against origin before starting)

## Goal

- Fix command/feedback mixing in midea_xye now; capture passive diagnostics
  for the next natural heating event. No need to wait for heating to recur
  before implementing and testing the state separation.
- Work in the ESPHome fork, not actrl or the reusable device baseline.

## Background

- Located 2026-10-06:
  - `~/src/esphome`: origin `git@github.com:rmounce/esphome.git`;
    clean; main checkout on unrelated `arlec-fan-light-component`.
  - Component: `esphome/components/midea_xye/{air_conditioner.cpp,
    air_conditioner.h,climate.py}` on `xye-units-switch`.
  - Local origin tracking tip: `e5181b27bfdea208354edafc9eb5d3ba2d3c23fe`
    (`Parse C6 response like C4`). Remote freshness not checked.
  - `~/src/esphome-midea-xye-atoms3`: clean public baseline;
    origin `https://github.com/rmounce/esphome-midea-xye-atoms3.git`;
    `packages/midea-xye.yaml` pins that exact component revision.
  - `/opt/dockerfiles/esphome/config`: tracked installation config, dirty
    with unrelated work. `hvac-xye.yaml` includes `aa-midea-xye-hvac.yaml`,
    which references `rmounce/esphome`, branch `xye-units-switch`, refresh 10s.
    ATOM S3/Tail485, retained HA name `m5atom`. Read local AGENTS.md before
    future installation-config work. Public baseline must stay generic.
- Evidence: `~/actrl/docs/actrl.md`, section
  "Indoor fan feedback: temperature-gated speed, 2026-10-06".
  - 06:53–07:03 indoor draw ~50W despite feedback code labelled off.
  - 07:03:11 feedback became low at coil 32°C; indoor draw ramped ~50→70W
    over the following seconds and stayed ~70W through 07:13.
  - Earlier low-report transition near coil 31–32°C; return to off-report
    at 27.5°C. Temperature-gated reduced-speed regime is a hypothesis;
    exact thresholds, RPM and airflow unknown.
- Code: control() immediately publishes requested fan_mode; C0 overwrites
  the same field from RX byte 9. setACParams() reads the mutable field when
  constructing C3; off hits the default auto branch. Potential unintended
  TX auto fallback has not been demonstrated with a captured bus trace.
- Fan low nibble: zero, 4, 2, 1 map to off, low, medium, high; auto bit 0x80.
  hvac_action=idle/heating is derived from the same nibble; it is not
  independent compressor evidence. Preserve the full byte, including
  unknown combinations; do not call raw zero physical fan stopped.
- actrl requested-speed fix deployed 07:35 on 2026-10-06. It removes
  report-driven airflow inflation/hysteresis. Firmware ambiguity remains.

## Steps

1. Read applicable repo guidance. Verify clean state and current XYE base.
   Create a dedicated worktree/branch from xye-units-switch; leave the
   unrelated main checkout and installation config untouched.
2. Separate explicit requested fan setting from observed feedback. Build
   every C3 fan byte from stored requested state; subsequent C0 packets
   must not overwrite it. Climate fan_mode should consistently represent
   the requested setting. Account for startup before any explicit command:
   define/document initialization behavior without transmitting new commands
   merely to initialize monitoring state.
3. Preserve protocol full-auto HEAT_COOL semantics, off-mode behavior,
   Fahrenheit support, mode command queuing and inhibit callback behavior.
   Do not silently reinterpret physical zero-code behavior. Verify partial
   calls (target/mode/preset only) retain the proper fan request, including
   when feedback arrives between queueing and transmission.
4. Add optional diagnostic exposure of the full C0 fan byte and the actual
   emitted C3 fan byte; log changes/commands rather than every unchanged poll.
   Raw zero is a code, not an assertion of no motion. If existing fan_speed
   text diagnostics are retained, document their code mapping limitations.
   Keep current hvac_action behavior unless separately reviewed; document
   that its heating/idle labels derive from the fan code.
5. Add deterministic regressions and a minimal compile fixture with all new
   diagnostic options enabled. Document an installation adoption patch and
   passive logging recipe, including timestamps, coil temperature, pressure,
   protect flags and correlation with Shelly indoor/outdoor power. Heating
   availability is not an acceptance prerequisite.
6. Return component diff, tests, immutable revision after publication approval,
   and proposed local config changes for review. Pin the reviewed revision
   in future deployment rather than following a moving branch. Public
   baseline revision/options can be updated afterward as separate work.

## Out of scope / do not modify

- No OTA, live fan commands, firmware deployment, merge or push.
- No actrl behavior changes or edits to appdaemon/.
- No modifications to dirty `/opt/dockerfiles/esphome/config` in this task.
- No inferred RPM/airflow calibration or speculative temperature thresholds.
- No Home Assistant token repair or unrelated Arlec changes.
- No installation-specific Home Assistant imports in the public baseline.

## Acceptance criteria (runnable)

From the implementation worktree; add the named regression files/fixture:

- `uv run pytest -q tests/components/midea_xye/test_requested_fan.py`
  must execute compiled component behavior (or invoke a native C++ harness),
  not just search source or mirror the algorithm in Python. Cover:
  - request low; C0 zero; climate request stays low, raw report stays zero,
    next actual C3 TX fan byte remains low;
  - medium/high and auto requests survive differing feedback;
  - feedback between command queueing and TX construction;
  - target-only and mode-only calls preserve request appropriately;
  - HEAT_COOL auto behavior, off behavior, startup defaults;
  - unknown/full-byte feedback is preserved by diagnostics;
  - no monitoring-only command emission, bounded diagnostic logging.
- `uv run python -m esphome compile tests/components/midea_xye/fan_feedback.yaml`
  uses local component source, compiles all optional diagnostics on the
  target-compatible ESP32 Arduino platform, contains no installation secrets.
- `git diff --check`
- Record exact commands/results and limitations in the handover log. Tests
  and compile must pass before Status review. Never weaken acceptance to
  source-text assertions. If toolchain constraints block native testing,
  record the question and stop rather than claim behavioral validation.

## Questions

- None blocking the proposed implementation. Precise raw zero semantics and
  coil thresholds intentionally remain open for passive measurement.

## Log

- 2026-10-06: repos located; component implementation/handover selected.
  Installed config dirty; no external-repo edits or deployment performed.

- 2026-10-06: origin fetched; XYE local/remote base both
  `e5181b27bfdea208354edafc9eb5d3ba2d3c23fe`. Dedicated worktrees:
  `/tmp/esphome-task-016` (implementation), `/tmp/actrl-task-016` (this log),
  both on `task/016-xye-requested-fan-feedback`; main checkouts untouched.
- 2026-10-06: stored request separated from C0 feedback; Auto startup without
  C3, full-auto normalization, optional full-byte C0/C3 sensors/logs, pending
  partial-call guards implemented. Nine native scenarios pass. ESP32 fixture
  compile ongoing; adoption patch drafted, checked against isolated config
  copy, never applied. No live actions, deployment, merge or push.

- 2026-10-06: implementation review commit `2cef7ea8357dae6d22df3401d90c3ce8f0946597` (local only).
  Diff: `git -C /tmp/esphome-task-016 diff e5181b27bfdea208354edafc9eb5d3ba2d3c23fe HEAD`.
  Startup fan request Auto; HEAT_COOL resets manual request; OFF retains it.
  Climate no longer advertises uncommandable fan OFF. C0 raw zero remains a
  code, action derivation unchanged. Queued target/preset/swing requests now
  retain intent across feedback; C4 Fahrenheit target guarded likewise.
- 2026-10-06: acceptance commands from `/tmp/esphome-task-016`:
  - `uv run pytest -q tests/components/midea_xye/test_requested_fan.py`:
    PASS, 9 scenarios in 2.82s, g++ compiles production component. Includes
    RX during queued C3, partial calls, C6/mode queueing, unknown/auto bytes,
    invalid CRC rejection, startup polling, OFF/full-auto, Fahrenheit,
    action labels, bounded diagnostics and pressure confirmation guard.
  - `uv run python -m esphome compile tests/components/midea_xye/fan_feedback.yaml`:
    PASS, ESP32-S3 Arduino 3.3.7, platform espressif32 55.3.37, 48.77s.
    RAM 36,904/327,680 B; flash 771,451/1,835,008 B. Local component,
    all new diagnostics, existing Fahrenheit switch and inhibit callback API.
  - `git diff --check` and full review-diff whitespace check: PASS.
  - Adoption patch `git apply --check` against isolated current config copy:
    PASS. Placeholder SHA intentionally pending review/publication;
    installation remains untouched.
  - Python test driver Ruff check: PASS.
- 2026-10-06: validation setup required sandbox escalation for git metadata,
  uv cache and PlatformIO toolchain/cache downloads. Initial native shim
  include/type errors and fixture ambiguous optional initializers corrected;
  all required final checks pass. Logs `/tmp/xye016-pytest.log`,
  `/tmp/xye016-compile.log`.
- 2026-10-06: review docs/recipe in component README; proposed config patch:
  `/tmp/esphome-task-016/tests/components/midea_xye/installation-adoption.patch`.
  Removes routine UART dumps, adds byte sensors/component DEBUG, replaces
  moving ref with future reviewed SHA; no installation edits performed.
- 2026-10-06: limits: native harness uses framework/UART/scheduler shims;
  real ClimateCall validation/callback dispatch and hardware behavior are
  not exercised at runtime. Target fixture compiles real framework and
  callback API. Python coverage report is not C++ coverage. No natural
  heating capture or bus evidence acquired; recipe ready for future approved
  adoption. No push, merge, OTA, deployment, live fan commands, actrl behavior
  changes or public-baseline edits. Main checkouts left clean.
