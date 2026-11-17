# Copyright 2026 ROBOTSIX
# Author: Damien SIX (damien@robotsix.net)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""DroneSimManager: orchestrates N drones using USD asset references."""

import collections
import math
import os
from typing import Any, Optional

from ament_index_python.packages import get_package_share_directory

from isaacsim.core.api import SimulationContext

import numpy as np

from rclpy.logging import get_logger

from scipy.spatial.transform import Rotation

from .autopilot import AutopilotAdapter, create_autopilot_adapter
from .config import (
    ArduPilotParams, GPSOrigin, MAVLINK_UPDATE_RATE, MavlinkParams, QuadrotorParams, SensorParams,
)
from .dynamics import QuadrotorDynamics
from .state import VehicleState

# Resolve the package's installed assets directory at import time so the path
# follows the colcon install layout instead of hard-coded developer paths.
ASSETS_DIR = os.path.join(
    get_package_share_directory('rs_isaac_uav_sim'), 'assets'
)
QUADROTOR_ASSET_PATH = os.path.join(
    ASSETS_DIR, 'basic_quadrotor', 'basic_quadrotor.usda'
)

# Path suffix inside the USD asset from its defaultPrim to the articulation root prim.
# physics.usda applies PhysicsArticulationRootAPI to:
#   <defaultPrim>/Geometry/base_link
_BASE_LINK_SUFFIX = '/Geometry/base_link'


class DroneSimManager:
    """
    Manages N quadrotor drones loaded from the basic_quadrotor USD asset.

    Uses Isaac Sim's ArticulationView tensor API (GPU) for state reading and
    force application. Each drone is spawned as a USD reference under
    /World/{namespace}/basic_quadrotor and the ArticulationView targets the
    articulation root prim at
    /World/{namespace}/basic_quadrotor/Geometry/base_link.

    Args:
    ----
    namespaces
        List of drone name strings, e.g. ['drone_0', 'drone_1'].
    quad_params
        Physical parameters for the quadrotor model.
    sensor_params
        Sensor noise parameters.
    mavlink_params
        MAVLink connection parameters.
    gps_origin
        GPS reference origin for sensor simulation.

    """

    def __init__(
        self,
        namespaces: list,
        quad_params: Optional[QuadrotorParams] = None,
        sensor_params: Optional[SensorParams] = None,
        mavlink_params: Optional[MavlinkParams] = None,
        gps_origin: Optional[GPSOrigin] = None,
        verbose: bool = False,
        autopilot: str = 'px4',
        ardupilot_params: Optional[ArduPilotParams] = None,
    ):
        """Build physical components and factory-selected adapters without connecting."""
        self._logger = get_logger(self.__class__.__name__)

        self.namespaces = namespaces
        self._verbose = verbose
        self.num_drones = len(namespaces)
        self.quad_params = quad_params or QuadrotorParams()
        self.sensor_params = sensor_params or SensorParams()
        self.mavlink_params = mavlink_params or MavlinkParams()
        self.gps_origin = gps_origin or GPSOrigin()

        # Per-drone simulation components
        self.states = [VehicleState() for _ in range(self.num_drones)]
        self.dynamics = [QuadrotorDynamics(self.quad_params) for _ in range(self.num_drones)]
        self.adapters: list[AutopilotAdapter] = [
            create_autopilot_adapter(
                autopilot, vehicle_id=i + 1,
                quad_params=self.quad_params, sensor_params=self.sensor_params,
                mavlink_params=self.mavlink_params, gps_origin=self.gps_origin,
                ardupilot_params=ardupilot_params, verbose=verbose, logger=self._logger,
            )
            for i in range(self.num_drones)
        ]

        # GPU tensor views — initialized after world.reset()
        self._prims = None        # Articulation view (sleep threshold only)
        # RigidPrim view on frame child prim (world pose and velocity reading);
        # frame has PhysicsRigidBodyAPI and returns reliable physics buffer data,
        # unlike the articulation root base_link)
        self._frame_view = None
        self._torch: Any = None
        self._device = 'cuda'

        # Diagnostic divider: print at ~1 Hz
        self._diag_counter = 0
        self._diag_divider = max(1, round(MAVLINK_UPDATE_RATE / 1.0))

        # Loop timing tracking (populated by record_loop_timing)
        self._loop_durations: collections.deque = collections.deque(maxlen=500)
        self._t_last_loop_begin: float | None = None
        self._t_last_loop_end: float | None = None

        # Render timing tracking (populated by record_render_timing; GUI only)
        self._render_durations: collections.deque = collections.deque(maxlen=100)

    @staticmethod
    def _grid_positions(num_drones: int, spacing: float = 2.0,
                        height: float = 0.3) -> list:
        """
        Compute grid spawn positions for N drones.

        Drones are arranged in a square grid with the given spacing.

        Args:
        ----
        num_drones
            Total number of drones.
        spacing
            Distance between adjacent drones in metres.
        height
            Spawn height above ground in metres.

        Returns
        -------
        list
            List of [x, y, z] positions, one per drone.

        """
        cols = math.ceil(math.sqrt(num_drones))
        positions = []
        for i in range(num_drones):
            row = i // cols
            col = i % cols
            positions.append([col * spacing, row * spacing, height])
        return positions

    def spawn_drones(self, world, positions: Optional[list] = None):
        """
        Spawn each drone as a USD reference and register physics views.

        Creates /World/{namespace}/basic_quadrotor for each drone using
        create_prim() with the QUADROTOR_ASSET_PATH reference. Then registers
        the Articulation and RigidPrim views with world.scene so that
        world.reset() initialises them in the correct order using a single
        consistent simulation view.

        Must be called BEFORE world.reset().

        Args:
        ----
        world:
            Isaac Sim World instance.
        positions:
            List of [x, y, z] spawn positions. If None, a default grid layout
            is computed automatically.

        """
        import torch
        import isaacsim.core.utils.prims as prim_utils
        from isaacsim.core.prims import Articulation, RigidPrim

        self._torch = torch

        if positions is None:
            positions = self._grid_positions(self.num_drones)

        if len(positions) < self.num_drones:
            raise ValueError(
                f'Expected {self.num_drones} spawn positions, got {len(positions)}'
            )

        if self.quad_params.usd_asset:
            asset_path = os.path.join(ASSETS_DIR, self.quad_params.usd_asset)
        else:
            asset_path = QUADROTOR_ASSET_PATH

        for i, ns in enumerate(self.namespaces):
            drone_prim_path = f'/World/{ns}/basic_quadrotor'
            pos = positions[i]
            # orientation: 90° CCW about Z so drone faces North (ENU forward=+Y)
            # → NED yaw = 0° at spawn. Isaac Sim quaternion format: (w, x, y, z)
            _s45 = float(np.sin(np.pi / 4))
            prim_utils.create_prim(
                prim_path=drone_prim_path,
                prim_type='Xform',
                position=(float(pos[0]), float(pos[1]), float(pos[2])),
                orientation=(_s45, 0.0, 0.0, _s45),
                usd_path=asset_path,
            )
            self._logger.info(
                f'[VEHICLE] Spawned {ns} at '
                f'({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) '
                f'from {asset_path}'
            )

        # Register Articulation and RigidPrim views with the World scene BEFORE
        # world.reset(). World.reset() calls _on_physics_ready on all registered
        # objects in order using a single valid simulation view, avoiding the
        # 'simulation view invalidated' crash that occurs when views are
        # constructed and initialised manually after reset().
        base_link_expr = f'/World/*/basic_quadrotor{_BASE_LINK_SUFFIX}'
        self._prims = Articulation(
            prim_paths_expr=base_link_expr,
            name='drone_base_links',
            reset_xform_properties=False,
        )
        world.scene.add(self._prims)

        frame_expr = '/World/*/basic_quadrotor/Geometry/base_link/frame'
        self._frame_view = RigidPrim(
            prim_paths_expr=frame_expr,
            name='drone_frames_rigid',
            reset_xform_properties=False,
        )
        world.scene.add(self._frame_view)

    def initialize(self):
        """
        Post-reset setup: disable sleep and initialize autopilot adapters.

        Must be called AFTER world.reset(). Physics views are already
        initialised by the World during reset(); this method only performs
        post-reset configuration that requires valid physics handles.
        """
        torch = self._torch
        msg = 'spawn_drones() must be called before on_physics_ready()'
        assert self._prims is not None, msg
        assert torch is not None, msg

        # Disable PhysX sleep so force application remains active at rest
        self._prims.set_sleep_thresholds(  # type: ignore
            torch.zeros(self.num_drones, device=self._device)
        )

        # Pre-allocate the force/torque CUDA tensors that step() reuses on
        # every iteration. Constructing fresh torch.tensor(...) each step
        # forces a host->device copy of a 3-element numpy buffer through
        # python; copy_(...) on a pre-allocated tensor reuses the buffer
        # and avoids that allocation/dispatch each frame.
        self._forces_tensor = torch.zeros(  # type: ignore
            (self.num_drones, 3), dtype=torch.float32, device=self._device
        )
        self._torques_tensor = torch.zeros(  # type: ignore
            (self.num_drones, 3), dtype=torch.float32, device=self._device
        )

        self._logger.info(
            f'[VEHICLE] ArticulationView initialized for {self.num_drones} drone(s).')

        for adapter in self.adapters:
            adapter.initialize()

    def step(self, dt: float):
        """
        Execute one physics step for all drones.

        Pipeline:

        1. Read world poses and velocities from GPU via RigidPrim view.
        2. Convert to ENU/FLU VehicleState.
        3. Update each adapter with state and the authoritative physics clock.
        4. Read adapter motor angular velocities in rad/s.
        5. Compute body-frame forces/torques from motor commands.
        6. Rotate forces/torques to world frame and apply via GPU tensor API.

        Args:
        ----
        dt:
            Physics timestep in seconds.

        """
        if self._prims is None or not self._prims.is_physics_handle_valid():
            return

        torch = self._torch

        # --- 1. Read state from GPU ---
        # Batch the three tensors into one before transferring to CPU.
        # A single .cpu() call amortises the CUDA stream synchronisation
        # overhead (~40% faster transfer vs three separate calls).
        positions_t, orientations_t = self._frame_view.get_world_poses(usd=False)  # type: ignore
        velocities_t = self._frame_view.get_velocities()  # type: ignore

        _batch = torch.cat([positions_t, orientations_t, velocities_t], dim=1).cpu().numpy()
        positions_np = _batch[:, :3]          # (N, 3)
        orientations_np = _batch[:, 3:7]      # (N, 4) [w, x, y, z]
        velocities_np = _batch[:, 7:]         # (N, 6) [vx,vy,vz,wx,wy,wz]

        # Prepare output buffers
        forces_world = np.zeros((self.num_drones, 3))
        torques_world = np.zeros((self.num_drones, 3))

        simulation_time = SimulationContext.instance().current_time

        for i in range(self.num_drones):
            state = self.states[i]

            # --- 2. Populate VehicleState (ENU world / FLU body) ---
            state.position = positions_np[i]
            # Isaac Sim quaternion convention: scalar-first [w, x, y, z]
            # scipy uses scalar-last [x, y, z, w]
            qw, qx, qy, qz = orientations_np[i]
            state.attitude = np.array([qx, qy, qz, qw])
            state.linear_velocity = velocities_np[i, :3]
            # Angular velocity from get_velocities() is in world ENU frame.
            # Sensors expect body FLU frame:
            # omega_body = R_body_to_world^{-1} * omega_world
            rot = Rotation.from_quat(state.attitude)
            state.angular_velocity = rot.inv().apply(velocities_np[i, 3:])

            adapter = self.adapters[i]
            adapter.update(state, dt, simulation_time)

            # --- 5. Compute forces/torques in body frame ---
            motor_omega = adapter.motor_commands  # (4,) [rad/s]
            force_body, torque_body = self.dynamics[i].compute_forces_and_torques(
                motor_omega, state
            )

            # --- 6. Rotate body FLU -> world ENU ---
            forces_world[i] = rot.apply(force_body)
            torques_world[i] = rot.apply(torque_body)

        # Apply forces and torques via GPU tensor API (_frame_view has
        # PhysicsRigidBodyAPI). Reuse the pre-allocated tensors instead
        # of constructing fresh ones each step — copy_() avoids the
        # per-step allocation + dispatch cost of torch.tensor(...).
        self._forces_tensor.copy_(  # type: ignore
            torch.from_numpy(forces_world.astype(np.float32, copy=False)),  # type: ignore
            non_blocking=True,
        )
        self._torques_tensor.copy_(  # type: ignore
            torch.from_numpy(torques_world.astype(np.float32, copy=False)),  # type: ignore
            non_blocking=True,
        )
        self._frame_view.apply_forces_and_torques_at_pos(  # type: ignore
            self._forces_tensor, self._torques_tensor, is_global=True
        )

        # Diagnostics — opt-in via the manager's verbose flag
        if self._verbose:
            self._diag_counter += 1
            if (self._diag_counter % self._diag_divider) == 0:
                self._print_diagnostics(positions_np, velocities_np, forces_world)

    def record_loop_timing(self, t_begin: float, t_end: float) -> None:
        """Record wall-clock timing for a full simulation loop iteration."""
        self._t_last_loop_begin = t_begin
        self._t_last_loop_end = t_end
        self._loop_durations.append((t_end - t_begin) * 1000.0)

    def record_render_timing(self, t_begin: float, t_end: float) -> None:
        """Record wall-clock timing for a single world.render() call."""
        self._render_durations.append((t_end - t_begin) * 1000.0)

    def _print_diagnostics(self, positions_np, velocities_np, forces_world):
        # --- Loop timing stats ---
        loop_samples = list(self._loop_durations)
        self._loop_durations.clear()

        if loop_samples:
            l_avg = sum(loop_samples) / len(loop_samples)
            l_min = min(loop_samples)
            l_max = max(loop_samples)
            loop_str = f'avg={l_avg:.2f}ms  min={l_min:.2f}ms  max={l_max:.2f}ms'
        else:
            loop_str = 'N/A'
        t_begin_str = (
            f'{self._t_last_loop_begin:.6f}' if self._t_last_loop_begin is not None else 'N/A'
        )
        t_end_str = (
            f'{self._t_last_loop_end:.6f}' if self._t_last_loop_end is not None else 'N/A'
        )

        render_samples = list(self._render_durations)
        self._render_durations.clear()
        if render_samples:
            r_avg = sum(render_samples) / len(render_samples)
            r_min = min(render_samples)
            r_max = max(render_samples)
            render_str = f'avg={r_avg:.2f}ms  min={r_min:.2f}ms  max={r_max:.2f}ms'
        else:
            render_str = 'N/A (headless)'

        self._logger.info(f'[DIAG t={SimulationContext.instance().current_time:.1f}s]')
        self._logger.info(f'  Loop  : {loop_str}  (begin={t_begin_str}  end={t_end_str})')
        self._logger.info(f'  Render: {render_str}')
        for i, adapter in enumerate(self.adapters):
            diagnostics = adapter.get_diagnostics()
            state = self.states[i]
            rpy = Rotation.from_quat(state.attitude).as_euler('xyz', degrees=True)
            self._logger.info(
                f'  {diagnostics.name} drone{i}: armed={diagnostics.armed} '
                f'mode={diagnostics.mode} raw={diagnostics.raw_controls} '
                f'omega={diagnostics.motor_commands} rad/s '
                f'pos={positions_np[i]} vz={velocities_np[i, 2]:.3f} m/s '
                f'rpy={rpy} deg Fz={forces_world[i, 2]:.2f} N')
            if diagnostics.timing:
                self._logger.info(f'  [TIMING] {diagnostics.timing}')
            for sensor, data in diagnostics.sensors.items():
                self._logger.info(f'  [SENSORS] {sensor}: {data}')

    def close(self):
        """Close all autopilot adapters."""
        for adapter in self.adapters:
            adapter.close()
