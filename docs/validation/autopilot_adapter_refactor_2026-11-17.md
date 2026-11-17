# Autopilot adapter refactor validation — 2026-11-17

The factory creates an `AutopilotAdapter` implemented by `PX4Adapter` or
`ArduPilotAdapter`. Both implement `initialize`, `update`, `motor_commands`,
`get_diagnostics` and `close`. Shared identity/logging and diagnostic data live
in `sim/autopilot/base`. The manager contains no controller-specific branches,
sensor scheduling, readiness logic or protocol-private diagnostic access.

Validation against the local executables:

- All unit tests plus real ArduCopter JSON protocol integration: **36 passed**.
  Includes abstract contract enforcement, lazy factory imports, construction
  without connections, PX4 heartbeat failure cleanup, GPS cadence, settling,
  mocap behavior, explicit simulation clocks and diagnostic control snapshots.
- Existing PX4 takeoff/hover test: **passed in 42.84 seconds on retry**, including
  its 15-second hover checks. The initial run armed and climbed to 1.687 m but
  missed the existing 2 m deadline. The check has shown timing/startup variation
  in earlier validation as well; the refactor does not change flight tuning.
- Real Isaac Sim 6.0.1 + ArduCopter reference Quad-X run via the new factory and
  manager: normal arming, takeoff altitude threshold maintained for three
  seconds, LAND and automatic disarm **passed**. No arming checks bypassed.
- ROS 2 package build into isolated `/tmp` build/install directories: **passed**.
- Flake8 for adapter modules, compatibility imports, simplified vehicle manager
  and new contract tests; diff whitespace checks: **passed**.
- Both old backend imports resolve to the relocated protocol implementations.

User configuration edits and staged files were preserved. The reference GPU
flight used a captured temporary configuration; it does not validate unrelated
custom-world or spawn-position changes made concurrently.

No shared Isaac Python editable installation was changed. Flight processes were
shut down after validation. This refactor remains uncommitted for review.
