# Copyright {YEAR} {COMPANY}
# {LICENSE}
"""ArduPilot-specific behavior behind the common autopilot contract."""

import numpy as np

from .backend import ArduPilotJSON
from ..base import AdapterDiagnostics, AutopilotAdapter
from ...config import ArduPilotParams, QuadrotorParams, SensorParams
from ...state import VehicleState


class ArduPilotAdapter(AutopilotAdapter):
    """Supply physics truth to ArduPilot's own simulated sensors over JSON/UDP."""

    def __init__(self, vehicle_id: int, quad_params: QuadrotorParams,
                 sensor_params: SensorParams, params: ArduPilotParams, *, logger):
        """Validate navigation mode and configure the transport without opening it."""
        if sensor_params.localization_mode != 'gps':
            raise ValueError('ArduPilot mocap mode is not implemented')
        super().__init__(vehicle_id, logger=logger)
        self._backend = ArduPilotJSON(vehicle_id, params, quad_params)

    def initialize(self) -> None:
        """Bind JSON UDP; the first update waits for servo output, not a heartbeat."""
        self._backend.connect()
        self._logger.info(
            f'[VEHICLE] ArduPilot JSON listening on UDP {self._backend.port}')

    def update(self, state: VehicleState, dt: float, simulation_time: float) -> None:
        """Exchange truth and PWM while preserving JSON lockstep behavior."""
        self._backend.exchange(state, simulation_time, dt)

    @property
    def motor_commands(self) -> np.ndarray:
        """Return rotor angular velocities in rad/s."""
        return self._backend.motor_commands

    def get_diagnostics(self) -> AdapterDiagnostics:
        """Return controls; arming/mode are unavailable on the JSON physics link."""
        return AdapterDiagnostics(
            name='ArduPilot', vehicle_id=self._vehicle_id,
            raw_controls=self._backend.raw_controls.copy(),
            motor_commands=self.motor_commands.copy(),
        )

    def close(self) -> None:
        """Release JSON UDP and clear motor output."""
        self._backend.close()
