# Measuring and improving real-time factor

The simulator advances physics and MAVLink sensors at 500 Hz. Each physics step
has a 2 ms wall-time budget for real-time execution. Rendering runs every ten
steps (50 Hz in simulation time); a ROS bridge graph, when requested, is evaluated
on every physics step. A 500 Hz graph can be a substantial cost even when only
one drone is simulated. The measurements below retain all these rates.

## CPU versus CUDA physics

CPU physics can outperform CUDA for a small number of rigid bodies when the
control/sensor calculation is in NumPy and MAVLink. The existing implementation
reads state into CPU memory on every step and submits a new wrench back to the
physics device. CUDA adds readback, dispatch and synchronization costs to this
otherwise CPU-facing loop. It is not guaranteed to be faster for one drone.

Use identical launch arguments, world, drone count and vehicle activity for each
comparison. Add these arguments to your existing launch command:

```bash
# Existing CUDA backend, with native simulator RTF and phase timing enabled:
ros2 launch rs_isaac_uav_sim px4_sitl.launch.py physics_device:=cuda profile:=true

# Next run, change only the physics/tensor device:
ros2 launch rs_isaac_uav_sim px4_sitl.launch.py physics_device:=cpu profile:=true
```

Keep any existing `config_file:=`, `world_usd:=`, `ros2_namespaces:=`, airframe and
connection arguments. The defaults preserve CUDA and the existing render quality.
CPU mode uses Torch CPU views/buffers, CPU dynamics and MBP broadphase. It enables
USD writeback so visible transforms remain current. Rendering still uses the GPU.

Direct entrypoint:

```bash
"$ISAAC_SIM_PYTHON" isaac_run/scene_mavlink_sim.py \
  --num_drones 1 --physics_device cpu --profile
```

The existing `launch/scene_mavlink_sim.py` launch entrypoint accepts the same
performance arguments. Start PX4 normally when using the sim-only entrypoint.

## Read the measurements

Every five wall seconds the simulator prints:

```text
SIM_PERF rtf=... physics_hz=... loop_hz=...
SIM_PROFILE vehicle.step total_ms=... calls=... avg_ms=... max_ms=...
SIM_PROFILE physics.step ...
SIM_PROFILE ros2.graph ...
SIM_PROFILE app.render ...
SIM_PROFILE vehicle.readback ...
SIM_PROFILE vehicle.force_submit ...
SIM_PROFILE mavlink.update ...
SIM_PROFILE mavlink.ground_truth ...
```

RTF uses Isaac's actual `world.current_time` change divided by monotonic wall
time, independent of PX4/DDS timestamps. `physics_hz` derives from the simulation
time change and the configured physics timestep. Phase times are inclusive:
`vehicle.step` contains readback, sensor/math work, MAVLink and force submission.
Do not add nested rows to their parent. CUDA phase timing may charge the wait
for preceding GPU work to the next CPU-facing read; this is not a GPU kernel
profiler and deliberately adds no explicit device synchronization.

Allow shader compilation, scene loading and PX4 initialization to finish. Compare
several stable windows rather than one sample. For a 500 Hz step loop,
`vehicle.step.avg_ms + physics.step.avg_ms + ros2.graph.avg_ms`, plus the
amortized render/UI work, must fit in about 2 ms. `app.render` includes a Kit
application update and should not be interpreted as pure GPU rendering time.

## Optional render comparison

Only after establishing device/phase results, compare the same run with:

```bash
ros2 launch rs_isaac_uav_sim px4_sitl.launch.py \
  physics_device:=cpu profile:=true render_quality:=fast
```

The fast preset disables reflections, indirect diffuse and ambient occlusion.
It retains the existing shadows, antialiasing and exposure settings. It changes
visual appearance, not physics rate or the MAVLink sensor cadence.

## Changes in this patch

- Select the physics device consistently in World, PhysX flags and manager buffers.
- Avoid redundant state getter clones: the existing concatenation creates an
  owned snapshot before stepping physics again.
- Add opt-in phase profiling and a separate fast render preset.
- Preserve the existing 500 Hz physics/sensor loop, ROS graph cadence, forces,
  frame conversions, sensor noise and PX4 command handling.

Offline regression checks cover state frame conversion and submitted wrenches
using real Torch CPU tensors with fake simulator views/transport, snapshot
ownership, launch argument forwarding, profiler resets and exception propagation.
Real GPU-host launch, GUI transforms, ROS odometry and flight validation are still
required. No RTF improvement is claimed until the patched simulator is measured.
