# Task handover convention

Specs for implementation work delegated between agents. One file per task:
`NNN-short-slug.md`, numbered in creation order.

## Rules

- Implementer works on a branch named `task/NNN-short-slug`, never on
  `master`. Never deploy (never touch `appdaemon/`).
- Respect the task's **Out of scope / do not modify** list absolutely. If a
  task seems to require violating it, stop and ask (see Questions below).
- Acceptance criteria are runnable commands. The task is not done until all
  of them pass. Do not weaken, skip, or delete acceptance tests to make
  them pass.
- Implementer updates the **Status** line and appends to the **Log** section
  as work proceeds (short dated bullets: what was done, what's left).
- Blocked or ambiguous: write the question under **Questions**, set Status
  to `blocked`, commit, and stop. The spec author answers in the same file.
- Reviewer (spec author or user) merges to master; the implementer does not.

## Status values

`draft` → `ready` → `in-progress` → `blocked` (if stuck) → `review` → `done`

## Template

```markdown
# NNN: Title

Status: ready
Branch: task/NNN-short-slug

## Goal
## Background
## Steps
## Out of scope / do not modify
## Acceptance criteria (runnable)
## Questions
## Log
```
