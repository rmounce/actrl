"""Context manager for temporarily overriding controller module constants
(docs/tasks/014), so `analysis/tune.py` can score candidate PID/deadband
configurations without ever editing production source.

Usage::

    with ctrl_overrides({"actrl.global_ki": 0.0005,
                          "control.global_deadband_ki": 0.02}):
        ... build the ClosedLoop AND run the whole replay here ...

IMPORTANT -- the `with` block must wrap *both* ClosedLoop construction and
the entire replay, not just one or the other:

- `actrl`'s `initialize()` (called once, inside `ClosedLoop.__init__`,
  sim/closed_loop.py) builds its PID objects from the module-level
  constants at that moment -- override after construction and the PIDs
  already baked in the old gains.
- `control.py`'s globals (e.g. `global_deadband_ki`) are read at *call*
  time on every cycle, not cached -- override only during construction and
  every subsequent replay step reads the restored, original value.

So the override must still be in effect for every `loop.step(...)` call
across the whole day, i.e. it wraps `replay_day.build_loop` AND
`replay_day.replay`.

Nesting two `ctrl_overrides` context managers is NOT supported (the second
enter would save/restore against the first's already-overridden values,
which is unlikely to be what's intended) -- do not attempt it.
"""
from __future__ import annotations

import sys
from contextlib import contextmanager
from pathlib import Path

_ROOT = Path(__file__).resolve().parent.parent
for p in (str(_ROOT), str(_ROOT / "tests")):
    if p not in sys.path:
        sys.path.insert(0, p)

_MODULE_NAMES = ("actrl", "control")


def _resolve_module(prefix: str):
    import actrl  # noqa: F401
    import control  # noqa: F401

    modules = {"actrl": actrl, "control": control}
    if prefix not in modules:
        raise ValueError(
            f"unknown ctrl_overrides module {prefix!r}; expected one of {_MODULE_NAMES}"
        )
    return modules[prefix]


@contextmanager
def ctrl_overrides(overrides: dict[str, object]):
    """Temporarily setattr module-level constants on `actrl`/`control`.

    Keys are "<module>.<attr>" where module is "actrl" or "control" (any
    other prefix raises ValueError). Each attr must already exist on the
    module (AttributeError otherwise -- a typo guard, since these are
    plain module globals with no schema to validate against). Original
    values are restored on exit, including when the with-block raises.
    """
    saved: list[tuple[object, str, object]] = []
    try:
        for key, value in overrides.items():
            prefix, _, attr = key.partition(".")
            if not _:
                raise ValueError(f"ctrl_overrides key {key!r} must be '<module>.<attr>'")
            module = _resolve_module(prefix)
            if not hasattr(module, attr):
                raise AttributeError(
                    f"{prefix}.{attr} does not exist -- check for a typo in the override key"
                )
            saved.append((module, attr, getattr(module, attr)))
            setattr(module, attr, value)
        yield
    finally:
        for module, attr, old_value in saved:
            setattr(module, attr, old_value)
