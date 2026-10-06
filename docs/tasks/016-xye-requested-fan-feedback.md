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

- 2026-10-06: user requested upstream cross-check. Fetched mdrobnak branches,
  exciton dev/midea_xye, HomeOps branches, wtahler main/reorg; no code change.
  Relevant tips: mdrobnak units_switch `361989b965dcf5943c4a38c2d94db775a6480b40`,
  delays_updated `2e85a12a32c3969ab30eacceb8cd92bd16a5bbbf`;
  exciton dev `160bdca396214e0d0e7cffb5bc66fe367923063a`,
  midea_xye `fb187532f7b293bdd419dbb20577c9bd04f47d31`;
  HomeOps main `2c648a7a34e3fd3c80f68fe3ed640bfa27c73ab2`,
  fan-sync branch `2bce083287252d61b4098750d7e3aecb3e83534c`;
  wtahler main `b0055042ae4a1d2366f73e7d2a5b8950dcc5cd07`,
  reorg `55a66567df25add9ee6889dc28b82ba13cff7442`.
- 2026-10-06: mdrobnak units_switch/delays_updated already leave requested
  climate fan unchanged by C0 feedback; C3 uses that retained field. Earlier
  exciton code overwrites manual requests from C0 (protects Auto only).
  Our C0 overwrite was introduced in `ddae4debc385bf62ad5eb08b034408ef8d829fb6`
  (2026-02-21); `b17009e1d` subsequently rewrote the same assignment. Initial
  blame attribution to b170 was refined by reviewing its parent diff.
- 2026-10-06: HomeOps main likewise excludes C0 feedback from command state;
  optional numeric fan_speed diagnostic is a decoded level, not raw full byte.
  Opt-in `sync_fan_mode_from_device` (default false) reads C4 byte 17 as
  thermostat-commanded fan speed, with post-SET grace; merged feature
  `9243612f27c9999b459d470184b009312bedcf83` (#124, 2026-05-23).
  C4 field research: `cbf0210a71231f6260331d70260aa4e7121a9bb6` (#122),
  issue https://github.com/HomeOps/ESPHome-Midea-XYE/issues/120.
  Validate byte 17 passively on this unit before adopting its semantics;
  upstream zero=idle terminology is not proof of physical stopped motion here.
- 2026-10-06: HomeOps has substantial protocol/API refactoring; replacement
  is separate migration work. Its single queuedCommand is not our multi-C6
  queue/confirmed-off safety behavior. Our explicit request field, raw-byte
  diagnostics and compiled regressions remain useful in this fork. wtahler
  is a separate YAML/lambda implementation, not a drop-in component patch.
  This was source review only; upstream builds/live behavior were not tested.

- 2026-10-06: user authorized deployment after HVAC stopped. Publication,
  installation adoption and OTA now authorized for this fix, superseding
  the original deployment exclusions for this deployment only.
- 2026-10-06: published reviewed component
  `2cef7ea8357dae6d22df3401d90c3ce8f0946597` to origin branch
  `task/016-xye-requested-fan-feedback`; no merge to moving XYE branch.
  Installation `aa-midea-xye-hvac.yaml` pins full SHA, adds full-byte C0/C3
  sensors, removes UART poll dumps and enables component DEBUG logs.
  Local inhibit callback and bespoke actrl/display packages preserved.
- 2026-10-06: direct HA pre-OTA check: climate OFF; compressor/outdoor fan
  OFF; indoor Shelly draw 4.64W (2026-10-05 21:56 UTC). No fan/mode/pressure
  test commands issued. No HA token repair; used existing config loader's
  secret overlay without printing/copying credentials.
- 2026-10-06: deployment build PASS:
  `docker exec esphome esphome compile /config/hvac-xye.yaml` using installed
  ESPHome 2026.7.3; copied XYE source matches reviewed source exactly.
  RAM 109,395/341,760 B; flash 1,001,635/3,932,160 B;
  config hash `0x1dd04a00`, build time `2026-10-05 21:54:20 +0000`.
- 2026-10-06: OTA PASS:
  `docker exec esphome esphome upload /config/hvac-xye.yaml --device 172.23.17.109`;
  1,001,744-byte image; upload 4.38s. Controller reconnected; HA status ON
  and version/config hash/build time match deployment. Climate stays OFF,
  compressor/outdoor fan OFF, pressure 2, error/protect flags 0.
  Requested fan AUTO reflects documented startup default.
- 2026-10-06: live diagnostics verified:
  `sensor.hvac_xye_m5atom_c0_fan_feedback_byte` = 128 (`0x80`),
  `sensor.hvac_xye_m5atom_c3_fan_command_byte` = unknown (no C3 emitted
  since reboot). Indoor power ~5W. No interpretation of zero-code motion
  made; next natural heating event and C4 byte-17 validation remain passive
  follow-up work, not a deployment acceptance prerequisite.
- 2026-10-06: inventory appended verified hvac-xye row, MAC
  `48:CA:43:B5:EF:70`, ATOM S3/Tail485, deployed revision/version/OTA/IP;
  existing rows untouched. Inventory was already untracked and remains so.
  Only HVAC package committed; unrelated inherited dirty config preserved.
  Public baseline and actrl behavior unchanged.
- 2026-10-06: logs:
  `/tmp/hvac-xye-20261006-deploy-build.log`,
  `/tmp/hvac-xye-20261006-ota.log`,
  `/tmp/hvac-xye-20261006-post-ota.log`,
  `/tmp/hvac-xye-20261006-verified-state.jsonl`.
  Short passive API log capture successful; no protocol errors observed.

- 2026-10-06: installation commit `053c12e` records only the deployed HVAC
  package. Commit initially failed on root-owned `.git/objects/7f`; identified
  exact failed path with strace and corrected ownership of that directory.
  No unrelated config changes committed; inherited untracked inventory retained.

- 2026-10-07: passive next-morning review completed; requested low retained
  despite C0 zero/low changes; last emitted C3 byte remained 0x04 with no
  recorded auto fallback. Repeated coil ~32°C rising/~27.5°C falling
  transitions and nonzero indoor draw confirm distinct reported/commanded
  states. C4/C6 byte 17 captured 0x04 while C0 zero; low-only corroboration
  of upstream requested-speed field, not full mapping validation. Details
  in docs/actrl.md, "Post-deployment passive recheck, 2026-10-07".
  No live commands, code changes or deployment. Chart/data under /tmp.

- 2026-10-07: reviewed prior-day cooling 16:38–17:54 Adelaide.
  Low/medium/low C3 requests followed by matching C0 within 3–5s;
  brief initial AUTO followed by LOW, source not captured. No C0 zero;
  C0 LOW persists after shutdown despite standby power. Climate action
  idle throughout actual cooling confirms existing derivation limitation.
  Detailed timings/power/limits in docs/actrl.md cooling review section.
  Passive history only; no code changes, control commands or deployment.
