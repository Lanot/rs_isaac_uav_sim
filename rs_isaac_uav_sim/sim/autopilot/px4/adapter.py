# Copyright {YEAR} {COMPANY}
# {LICENSE}
"""PX4 sensor pipeline, readiness and diagnostics behind a common contract."""

import numpy as np

from .backend import PX4MavlinkHIL
from ..base import AdapterDiagnostics, AutopilotAdapter
from ...config import GPSOrigin, MAVLINK_UPDATE_RATE, MavlinkParams, QuadrotorParams, SensorParams
from ...sensors import BarometerSensor, GPSSensor, IMUSensor, MagnetometerSensor
from ...state import VehicleState

_STABLE_VEL_MS = 0.05
_STABLE_OMEGA_RPS = 0.05
_STABLE_STEPS = int(0.5 * MAVLINK_UPDATE_RATE)


class PX4Adapter(AutopilotAdapter):
    """Own the existing PX4 HIL sensor and startup behavior for one drone."""

    def __init__(self, vehicle_id: int, quad_params: QuadrotorParams,
                 sensor_params: SensorParams, params: MavlinkParams,
                 gps_origin: GPSOrigin, *, verbose: bool, logger):
        """Configure independent sensors, readiness tracking and HIL transport."""
        super().__init__(vehicle_id, logger=logger)
        self._localization_mode = sensor_params.localization_mode
        self._imu = IMUSensor(sensor_params)
        self._barometer = BarometerSensor(sensor_params, gps_origin)
        self._magnetometer = MagnetometerSensor(sensor_params, gps_origin)
        self._gps = (GPSSensor(sensor_params, gps_origin)
                     if self._localization_mode == 'gps' else None)
        self._backend = PX4MavlinkHIL(
            vehicle_id, params, quad_params, localization_mode=self._localization_mode,
            verbose=verbose,
        )
        self._gps_counter = 0
        self._gps_divider = max(1, round(MAVLINK_UPDATE_RATE / 50.0))
        self._stable_count = 0
        self._stabilized = False
        self._sensor_data: dict[str, dict] = {}

    def initialize(self) -> None:
        """Open HIL TCP and require a PX4 heartbeat before stepping physics."""
        try:
            self._backend.connect()
            self._logger.info(
                f'[VEHICLE] MAVLink vehicle {self._vehicle_id} '
                f'listening on TCP port {self._backend.port}')
            self._logger.info(
                f'[VEHICLE] Waiting for PX4 heartbeat on vehicle {self._vehicle_id}...')
            if not self._backend.wait_for_heartbeat(timeout=30.0):
                raise RuntimeError(
                    f'Vehicle {self._vehicle_id}: no heartbeat within 30 s — '
                    f'is PX4 SITL running on port {self._backend.port}?')
        except BaseException:
            self.close()
            raise

    def update(self, state: VehicleState, dt: float, simulation_time: float) -> None:
        """Generate HIL sensors, receive actuators and send truth at physics time."""
        if (np.linalg.norm(state.linear_velocity) < _STABLE_VEL_MS
                and np.linalg.norm(state.angular_velocity) < _STABLE_OMEGA_RPS):
            self._stable_count += 1
        else:
            self._stable_count = 0
        if not self._stabilized and self._stable_count >= _STABLE_STEPS:
            self._stabilized = True
            self._logger.info(
                f'[VEHICLE] Drone {self._vehicle_id - 1} stabilized at '
                f't={simulation_time:.2f}s — switching to real sensor data')
        baro = self._barometer.update(state, dt)
        mag = self._magnetometer.update(state, dt)
        imu = self._imu.update(state, dt) if self._stabilized else {
            'angular_velocity': np.zeros(3),
            'linear_acceleration': np.array([0.0, 0.0, -9.80665]),
        }
        self._sensor_data.update(imu=imu, barometer=baro, magnetometer=mag)
        self._backend.update_sensors(imu, baro, mag)
        self._gps_counter += 1
        if self._gps is not None and (
                self._gps_counter == 1 or self._gps_counter % self._gps_divider == 0):
            gps = self._gps.update(state, dt * self._gps_divider)
            self._backend.update_gps(gps)
            self._sensor_data['gps'] = gps
        self._backend.update(simulation_time)
        self._backend.send_ground_truth(state, self._backend.current_utime)

    @property
    def motor_commands(self) -> np.ndarray:
        """Return rotor angular velocities in rad/s."""
        return self._backend.motor_commands

    def get_diagnostics(self) -> AdapterDiagnostics:
        """Return protocol details without exposing them to the vehicle manager."""
        backend = self._backend
        return AdapterDiagnostics(
            name='PX4', vehicle_id=self._vehicle_id,
            raw_controls=backend.raw_controls.copy(), motor_commands=self.motor_commands.copy(),
            armed=bool(backend.raw_mode & 0x80), mode=backend.raw_mode,
            sensors={name: dict(data) for name, data in self._sensor_data.items()},
            timing={
                'latency_ms': backend.px4_latency_stats,
                'sensor_sent_mono': backend.t_last_sensor_sent_mono,
                'actuator_received_mono': backend.t_last_actuator_recv_mono,
                'actuator_interval_ms': backend.last_actuator_interval_ms,
                'actuator_simdt_us': backend.last_actuator_simdt_us,
                'sync_offset_us': backend.time_sync_offset_us,
            },
        )

    def close(self) -> None:
        """Close the HIL transport safely even if readiness failed."""
        self._backend.close()
