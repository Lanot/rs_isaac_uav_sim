---
title: Architecture
---

# Architecture

This page walks the simulator's components, the per-step pipeline, and the
coordinate frames the codebase juggles between.

## Components

```
   PX4 SITL                    rs_isaac_uav_sim
  ──────────                  ──────────────────
                            ┌──────────────────────┐
   ┌─────────┐   HIL_SENSOR │  scene_mavlink_sim   │
   │  px4    │ <─────────── │  (Isaac Sim app)     │
   │         │   HIL_GPS    │                      │
   │  EKF    │ <─────────── │  ┌──────────────┐    │
   │         │              │  │ DroneSimMgr  │    │
   │  ATT    │   HIL_       │  │              │    │
   │  CTRL   │   ACTUATOR   │  │  vehicle.py  │    │
   │         │   _CONTROLS  │  │              │    │
   │  ALLOC  │ ───────────> │  └──────┬───────┘    │
   └─────────┘              │         │            │
                            │  ┌──────▼───────┐    │
                            │  │ MAVLink HIL  │    │
                            │  │ backend      │    │
                            │  │ (TCP server) │    │
                            │  └──────────────┘    │
                            └──────────────────────┘
```

- **`scene_mavlink_sim.py`** — the Isaac Sim entry point. Boots a
  `SimulationApp`, configures the renderer, instantiates a `World` (PhysX),
  loads the iris USD asset for each drone, and runs the main loop.

- **`DroneSimManager` (`rs_isaac_uav_sim/sim/vehicle.py`)** — the per-step
  orchestrator. Reads the rigid-body state from the GPU view via
  `get_world_poses` + `get_velocities`, calls each autopilot adapter,
  computes per-rotor thrusts and torques, and applies
  forces back to PhysX through `apply_forces_and_torques_at_pos`.

- **`PX4MavlinkHIL` (`rs_isaac_uav_sim/sim/autopilot/px4/backend.py`)** — one
  instance per drone. Owns a TCP server socket, sends `HIL_SENSOR`,
  `HIL_GPS`, `HIL_STATE_QUATERNION` to PX4, and reads back
  `HIL_ACTUATOR_CONTROLS` to drive the rotors. Ported from
  [PegasusSimulator](https://github.com/PegasusSimulator/PegasusSimulator)
  (BSD-3-Clause, © Marcelo Jacinto), credited inline in the source.

- **Sensors (`rs_isaac_uav_sim/sim/sensors.py`)** — IMU, barometer,
  magnetometer, GPS, and visual-odometry sims with PX4-compatible noise
  models, biases, and random walks. Each sensor instance owns its own state.

- **`QuadrotorDynamics` (`rs_isaac_uav_sim/sim/dynamics.py`)** — pure-numpy
  per-rotor thrust + torque + drag math. Decoupled from PhysX; PhysX only
  sees a single 3-vector force and 3-vector torque per drone applied at the
  body center.

- **`px4_iris.yaml` (`config/`)** — physical parameters of the drone (mass,
  inertia, rotor positions, thrust/torque coefficients, sensor noise) plus
  the GPS reference origin. The `default.yaml` file gives you a generic
  configuration to start from.

## Per-step pipeline

The Isaac Sim main loop in `scene_mavlink_sim.py` is just:

```python
while simulation_app.is_running():
    manager.step(physics_dt)        # our work
    world.step(render=False)        # PhysX integration
    if not args.headless:
        if _render_step >= _render_every_n:
            world.render()          # 50 Hz throttled
            _render_step = 0
```

Inside `DroneSimManager.step()`:

1. **GPU readback** — `get_world_poses(usd=False)` + `get_velocities()` are
   batched into a single `torch.cat(...).cpu().numpy()` so only one CUDA
   stream sync happens per step.

2. **Per-drone loop** — populate `VehicleState`, call
   `adapter.update(state, dt, simulation_time)`, read `adapter.motor_commands`
   in rad/s, compute body-frame force/torque and rotate them to the world frame.
   The selected adapter owns sensor generation and protocol exchange.

3. **GPU upload** — copy the (N×3) world-frame force and torque arrays into
   pre-allocated CUDA tensors with a non-blocking `copy_()` and call
   `apply_forces_and_torques_at_pos`.

4. **PhysX `world.step()`** — runs outside our `step()`, integrates the
   articulation forward by `physics_dt`.

PX4 sends HIL sensors each step and receives actuator messages without blocking,
retaining the last command when none is available. ArduPilot waits for a fresh
servo frame and exchanges JSON state before allowing the step to continue.
Both receive the same absolute physics timestamp from the manager; the adapter
contract does not force the two transports to use the same synchronization model.

## Autopilot adapters

```text
sim/autopilot/
  factory.py                 create_autopilot_adapter(...)
  base/
    adapter.py               AutopilotAdapter abstract class
    diagnostics.py           AdapterDiagnostics shared snapshot
  px4/
    adapter.py               PX4Adapter: sensors, settling, GPS, readiness
    backend.py               PX4 MAVLink HIL transport and motor scaling
  ardupilot/
    adapter.py               ArduPilotAdapter: JSON lifecycle and truth exchange
    backend.py               ArduPilot JSON UDP transport and PWM scaling
```

Every adapter implements the same contract:

| Member | Contract |
|---|---|
| `initialize()` | Open transport and perform controller-specific readiness checks. |
| `update(state, dt, simulation_time)` | Process one ENU/FLU physics state; both time arguments use seconds. |
| `motor_commands` | Rotor angular velocities in rad/s in configured rotor order. |
| `get_diagnostics()` | Shared `AdapterDiagnostics` snapshot; unknown armed/mode values stay `None`. |
| `close()` | Release resources safely, including after partial initialization. |

Construction opens no connections. The factory lazily imports only the selected
implementation, and returns an `AutopilotAdapter`. The manager holds typed
`adapters`, calls this contract, and contains no PX4/ArduPilot branches or
transport-private attribute access.

PX4's adapter owns one sensor set per drone, the original 250-step settling
window, first-frame GPS then 50 Hz GPS updates, heartbeat readiness and HIL truth.
ArduPilot's adapter validates its navigation mode and exchanges truth with its
own SITL sensor simulation. Its physics link does not carry GCS arming/mode data.
Shared vehicle state, physics and frame conversions stay in `sim/`, because
these are also used by the physics engine itself. Only shared adapter contracts
and diagnostics belong in `autopilot/base/`.

The old `sim.mavlink_backend` and `sim.ardupilot_backend` modules remain thin
compatibility imports. New code should import the factory or the protocol's
package. The low-level PX4 backend accepts explicit simulation time; its legacy
no-argument update still obtains the clock from Isaac when needed.

## Coordinate frames

The codebase juggles three frames:

| frame | axes | used by |
|---|---|---|
| **ENU world**  | x=East, y=North, z=Up    | Isaac Sim, USD assets |
| **FLU body**   | x=Forward, y=Left, z=Up  | dynamics, rotor positions |
| **NED / FRD**  | x=North, y=East, z=Down  | MAVLink, PX4, HIL_STATE_QUATERNION |

`VehicleState` stores everything in ENU/FLU and exposes `get_attitude_ned_frd()`
and `get_linear_velocity_ned()` for the MAVLink path. `state.py` is the single
source of truth for the conversion math; `test_state_rotations.py` covers it.
