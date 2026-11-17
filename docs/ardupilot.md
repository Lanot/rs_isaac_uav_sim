# ArduPilot SITL support

This branch adds an experimental ArduCopter Quad-X integration alongside PX4.
PX4 remains the default (`--autopilot px4`). Both controllers are selected by
`sim/autopilot/factory.py` and implement the same abstract adapter contract.
The PX4 launch file, Iris YAML and flight demo keep their existing behavior.

## Design and scope

The adapter is in `sim/autopilot/ardupilot/adapter.py` and its JSON transport is
in `sim/autopilot/ardupilot/backend.py`. Shared contracts and diagnostic data
are in `sim/autopilot/base/`; `vehicle.py` uses only the adapter interface. See
[adapter architecture](architecture.md#autopilot-adapters).

ArduPilot's [JSON physics interface](https://ardupilot.org/dev/docs/sitl-with-JSON.html)
is separate from MAVLink. ArduCopter sends binary motor PWM over UDP; Isaac
replies to the packet's source address with timestamp, FRD specific force and
gyro, NED position and velocity, and a scalar-first FRD-to-NED quaternion.
MAVLink commands and telemetry use ArduCopter's own GCS connection.

The implementation supports 16- and 32-output servo packets, configurable
four-channel mapping and PWM endpoints, one UDP port per vehicle, repeated
packet reply caching, and a bounded wait for every new controller frame.
Physics stays at 500 Hz; ArduCopter runs its control loop at 250 Hz so the
500 Hz gyro stream meets its sample-rate arming requirement. The servo packet's requested rate is advisory;
ArduPilot can advertise its startup rate before ramping to SIM_RATE_HZ.
Timestamps, not the advertised rate, govern simulation time.

A timeout, backwards frame counter, peer change, or backwards physics clock
stops the run and clears motor demand. Restart both processes after a world
reset or SITL restart. A duplicate frame is answered again without advancing
physics. A 32-bit counter wrap is supported. Waiting is synchronous, so a
missing controller can pause the UI for up to `ardupilot.timeout` seconds.

The JSON path sends truth and lets ArduPilot synthesize its own GPS, compass,
barometer and sensor noise. The YAML `sensor` noise parameters belong to PX4's
HIL path and do not tune ArduPilot sensors. `mocap` is explicitly unsupported:
the existing `/fmu/in/vehicle_visual_odometry` pipeline is PX4-specific. ROS
truth odometry can still be published with `ros2_namespaces`.

The example is a symmetric Quad-X with Iris mass/inertia and visual asset.
Rotor geometry is deliberately distinct from PX4's asymmetric Iris allocation.
It uses motor order front-right, rear-left, front-left, rear-right, and yaw
reaction signs `[-1, -1, +1, +1]` in FLU. PWM maps linearly to rotor angular
speed, with quadratic thrust; `MOT_THST_EXPO=1`, PWM endpoints, motor functions,
and spin limits in the `.parm` file match this model. Physical flight tuning
and other airframes need separate validation. Ground-truth position is relative
to the Isaac world origin; the launcher's home is taken from `gps_origin`.

## Build and launch

Build this isolated checkout in its own colcon workspace, or use explicit build
and install directories so that the existing PX4 workspace stays intact:

```bash
source /opt/ros/jazzy/setup.bash
# Run from this repository. Set Isaac's Python only at runtime to avoid an
# editable pip install changing a shared Isaac environment during the build.
env -u ISAAC_SIM_PYTHON colcon build --base-paths . \
  --build-base /tmp/rs_isaac_ap_build \
  --install-base /tmp/rs_isaac_ap_install \
  --packages-select rs_isaac_uav_sim
source /tmp/rs_isaac_ap_install/setup.bash
export ISAAC_SIM_PYTHON=/home/tolik/sim/IsaacSim-6.0.1/python.sh
export ARDUCOPTER_BINARY=/home/tolik/sim/ardupilot/build/sitl/bin/arducopter
ros2 launch rs_isaac_uav_sim ardupilot_sitl.launch.py headless:=true
```

Each run gets a fresh `/tmp/rs_isaac_ardupilot_*` directory containing its copied
parameter file, `eeprom.bin`, and logs. These are retained for inspection.
Copying the parameter file also avoids ArduPilot interpreting commas in this
workspace's path as defaults-file separators. The launcher starts SERIAL0 as
nonblocking TCP; attach a GCS to TCP 5760 (instance 0). Use `instance:=N` to
move MAVLink/RC ports by `10*N`. The JSON destination is set explicitly from
YAML after the instance argument, so changing the instance does not silently
change the simulator UDP port.

Launch arguments: `arducopter_binary`, `config_file`, `parameter_file`,
`headless`, `verbose`, `world_usd`, `ros2_namespaces`, `instance`. Shut down
with Ctrl-C; either child's exit shuts down the other child. No global process
kill is used. This launcher starts one drone. For multiple drones, start the
scene directly with `--autopilot ardupilot --num_drones N --config ...`, then
start matching ArduCopter instances with distinct `--sim-port-out` ports:
9002, 9012, ... by default. Multiple-drone flight is not validated.

For a GCS supporting MAVProxy commands, connect to the simulator's MAVLink TCP
port, then request normal GUIDED arming and takeoff:

```text
mode guided
arm throttle
takeoff 2
```

For a repeatable check against an already running local simulation:

```bash
python3 scripts/ardupilot_flight_check.py --connection tcp:127.0.0.1:5760
```

This requests normal arming, a 2 m takeoff, at least three seconds above
1.5 m, and landing followed by automatic disarm. Initial EKF/GPS startup can
take over a minute; the default timeout is 240 seconds. Use this script only
with the simulation you launched.

Do not reuse the PX4 demo: its mode IDs, readiness checks and takeoff commands
are specific to PX4. No arming checks are disabled by this integration.

## Automated checks

```bash
python3 -m pytest -q test/unit
ARDUCOPTER_BINARY=/home/tolik/sim/ardupilot/build/sitl/bin/arducopter \
  python3 -m pytest -q test/integration/test_ardupilot_sitl_json.py
```

The real-SITL check uses an ephemeral UDP port, instance 91, temporary state,
and 100 stationary truth exchanges. It verifies protocol acceptance, not
flight. The unit suite covers frames, specific force, packet validation,
16/32-channel output, PWM scaling, timeout shutdown, duplicate retransmission,
restart handling, advisory rates, balanced thrust/yaw signs, and the original
PX4 linear-thrust scaling. Tests require local UDP socket access.

## Local validation, 2026-11-17

The local Isaac Sim 6.0.1 / ArduCopter experiment completed normal arming,
GUIDED takeoff to about 2 m, the three-second altitude check, LAND and automatic
disarm. All 25 CPU/protocol tests passed, and the existing PX4 takeoff plus
15-second hover regression passed separately. See the
[assessment and captured results](ardupilot_assessment.md) for exact scope.
