# ArduPilot feasibility and implementation assessment

Assessed 2026-11-17. Parent: `dev_6.0.1_custom` at
`88eacf80cf7b3374c35cc454833979e110d02ff0`. Implementation branch:
`feat/ardupilot-json-sitl`, in an independent local clone. The original checkout
at `/home/tolik/workspaces/isaac_ros-dev/src/rs_isaac_uav_sim` stays on its
original branch and is not edited.

## Result

ArduCopter SITL support is feasible and has been demonstrated with the local
Isaac Sim 6.0.1 installation and ArduCopter binary. The final experiment used
normal arming checks, GUIDED takeoff to 2 m, at least three seconds above 1.5 m,
LAND, and automatic disarm. This is an experimental Quad-X integration, not
validation of every ArduPilot airframe, mission mode, or external navigation
configuration. The captured [flight transcript](validation/ardupilot_flight_2026-11-17.log)
contains the accepted arm/takeoff commands and takeoff/landing pass markers.

## Existing customization and interfaces

Compared with local `main`, the custom branch changes five files, adding
58 lines and removing 17: custom environment USD forwarding through
`launch/px4_sitl.launch.py` and `isaac_run/scene_mavlink_sim.py`; YAML spawn
`positions` and `scene_camera_view`; and ROS logger output in `sim/vehicle.py`.
These features remain available on the implementation branch.

This is a standalone Isaac simulator, not a live Pegasus extension. Its
`sim/mavlink_backend.py` is credited as a Pegasus port; sensors are also derived
from Pegasus defaults. Adding an upstream Pegasus backend to an extension
list would not wire it into this manager. Upstream Pegasus already documents
[experimental ArduPilot integration](https://pegasussimulator.github.io/PegasusSimulator/source/features/ardupilot.html),
but this fork needs its own adapter and launch path.

PX4's simulation link is `HIL_SENSOR`/`HIL_GPS` to the autopilot and
`HIL_ACTUATOR_CONTROLS` back to the simulator. It uses TCP ports
`4560 + vehicle_id`; this backend listens with `tcpin`. Motor outputs are
normalized floats, converted to angular speed using the configured PX4 thrust
model. Its current `update()` receives nonblocking and holds the last output;
the README's lockstep wording is stronger than this implementation guarantees.

ArduPilot's supported [JSON simulator interface](https://ardupilot.org/dev/docs/sitl-with-JSON.html)
uses UDP binary PWM output and JSON state input. MAVLink remains the GCS and
telemetry interface. Changing the MAVLink autopilot identifier or TCP port
would therefore be insufficient.

The reusable pieces are the GPU rigid-body state/force path, ENU/FLU to NED/FRD
conversions, rotor thrust/torque computation, scene assets, world selection,
and ROS truth odometry. `QuadrotorDynamics` assumes four rotors. Its mass and
inertia configuration fields are not applied to PhysX by the existing manager;
the USD asset controls physical mass/inertia. Custom airframes must reconcile
both. The new profile retains the Iris USD and its nominal mass/inertia, with
symmetric commanded thrust-arm geometry.

The PX4 Iris profile is asymmetric front/rear and is coupled to its PX4 motor
allocation. A new symmetric ArduCopter Quad-X profile avoids silently treating
these as interchangeable. ArduPilot receives PWM in microseconds, not PX4's
normalized output. Its quadratic ESC/thrust model and motor parameters are
configured together. Reaction torque signs and balanced force are tested.

## Exact implementation changes

| File | Change |
|---|---|
| `rs_isaac_uav_sim/sim/ardupilot_backend.py` | New standalone JSON/UDP backend: 16/32 channels, frame validation, PWM mapping, body/world conversions, specific force, scalar-first quaternion, timestamps, duplicate replies, timeout/restart handling. |
| `rs_isaac_uav_sim/sim/config.py` | Add validated `ArduPilotParams` and load the `ardupilot` YAML block. Existing configuration defaults are preserved. |
| `rs_isaac_uav_sim/sim/vehicle.py` | Select backend; bind UDP without a MAVLink heartbeat for ArduPilot; exchange truth and PWM before applying forces; separate diagnostics. Existing PX4 sensor/update path remains the default. |
| `isaac_run/scene_mavlink_sim.py` | Add `--autopilot px4\|ardupilot`; forward configuration; close manager/app on failures. Keep custom worlds, positions, camera and ROS options. |
| `config/ardupilot_quad_x.yaml` | Symmetric Quad-X geometry, rotor order/signs, Iris asset, transport, PWM endpoints, GPS home. |
| `config/ardupilot_quad_x.parm` | Match ArduCopter Quad-X output functions, PWM endpoints, quadratic thrust and valid idle/minimum spin. Use 500 Hz simulation and 250 Hz control loop. |
| `launch/ardupilot_sitl.launch.py` | Separate one-vehicle launcher, fresh temporary runtime state, defaults-file copy, configurable binary/profile/world/ROS/instance, nonblocking GCS TCP port and paired process shutdown. |
| `scripts/ardupilot_flight_check.py` | Repeatable GCS takeoff/altitude/landing check for an already running local SITL. |
| `CMakeLists.txt` | Install flight check and register CPU/real-SITL protocol tests; directory installation already includes new YAML/launch files. |
| `setup.py` | Include new launch/config files; repair the stale setuptools scene-script path to `isaac_run/scene_mavlink_sim.py`. |
| `test/unit/test_ardupilot_json.py` | Protocol, coordinate, force, PWM and PX4 scaling regression checks. |
| `test/integration/test_ardupilot_sitl_json.py` | Optional real ArduCopter test: 100 stationary JSON exchanges, temporary state, ephemeral UDP, instance 91. |
| `README.md`, `docs/ardupilot.md` | Entry point, launch instructions, architecture, configuration and limitations. |

No edits to `sim/mavlink_backend.py`, `sim/dynamics.py`, `sim/state.py`,
`sim/sensors.py`, the PX4 launcher, PX4 Iris YAML, PX4 demo, or existing PX4
flight test.

## Validation and remaining scope

- CPU unit suite plus real ArduCopter JSON integration: **25 passed**
  ([result](validation/ardupilot_tests_2026-11-17.log)).
- Real protocol peer: `/home/tolik/sim/ardupilot/build/sitl/bin/arducopter`;
  adjacent source revision reports `913e171`. No rebuild or installation of
  ArduPilot was needed.
- Isolated ROS 2 package build: passed. Build/install state is under `/tmp`,
  without an editable pip install into the user's Isaac environment.
- Python compilation, diff whitespace checks, and flake8 for the new backend,
  launcher, flight-check script, configuration and new tests: passed. This is
  not a claim that all inherited project lint violations are fixed. The existing
  pyright test could not run its checker: pyright is absent and its npx
  fallback failed to reach registry.npmjs.org (EAI_AGAIN); type checking is
  unverified, rather than passed.
- GPU experiment: Isaac Sim 6.0.1, RTX 5080 Laptop GPU, driver 595.99.02;
  normal arm accepted, relative altitude reached 2.043 m, takeoff threshold
  maintained, LAND and automatic disarm confirmed. Isaac truth altitude was
  about 2.0 m. Landing ended around ENU [-0.51, 1.12, 0.008] m: this was a short
  functional check, not a long-duration hover/drift acceptance test.
- Existing PX4 takeoff-and-hover regression on the final branch: **passed**
  in 45.44 seconds, including the test's 15-second hover/stability checks.
  See [result](validation/px4_branch_2026-11-17.log). A clean parent-revision
  comparison failed its climb assertion in 47.66 seconds
  ([parent result](validation/px4_parent_2026-11-17.log)); earlier exploratory
  runs also encountered readiness/heartbeat timeouts. This does not imply the
  ArduPilot changes fix PX4 startup variability. The final standalone PX4
  run is the passing regression result.

The initial flight trials identified two real configuration problems: minimum
motor spin below armed idle, and a 500 Hz control loop requiring at least
900 Hz gyro samples. Both are corrected without disabling arming checks.
ArduPilot also advertises a startup simulation rate before settling to its
requested rate; the protocol treats this as advisory, with Isaac timestamps
authoritative. EKF/GPS startup can take over a minute.

Initial delivery scope: one local ArduCopter Quad-X. Multiple UDP backends are
implemented but multiple-vehicle flight is untested. ArduPlane, helicopters,
ESC lag, aerodynamic fidelity, missions, sustained hover, remote transport,
and external navigation/mocap need follow-up validation. PX4's external
visual-odometry topics cannot be reused as an ArduPilot navigation input.
The JSON path supplies truth and uses ArduPilot's own sensor simulation;
PX4 sensor-noise YAML settings do not apply to it.

See [setup and usage](ardupilot.md) for commands and full transport behavior.
