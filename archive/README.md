Archived AppDaemon experiments.

These files are kept for reference but are not part of the active room HVAC
control setup.

- `statoptim.py`: EMHASS/MPC-style HVAC forecast planner. Retired because the
  house has low thermal mass and the planning value did not justify the extra
  control complexity.
- `statoptimctrl.py`: companion controller that consumed `statoptim` forecasts
  and wrote room climate setpoints. Not active in AppDaemon; current room
  setpoint scheduling is handled by `statctrl.py`.
