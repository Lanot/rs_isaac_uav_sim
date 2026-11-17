# Copyright {YEAR} {COMPANY}
# {LICENSE}
"""Abstract contract shared by every autopilot adapter."""

from abc import ABC, abstractmethod

import numpy as np

from .diagnostics import AdapterDiagnostics
from ...state import VehicleState


class AutopilotAdapter(ABC):
    """Bridge a canonical ENU/FLU vehicle state to an autopilot.

    Construction must not open connections. Call initialize after resetting
    the physics world, update once before each physics step, then close on
    shutdown (including partial initialization failures). Adapters own sensor
    models, transport readiness, actuator conversion and protocol diagnostics.
    """

    def __init__(self, vehicle_id: int, *, logger):
        """Keep the identity and logger shared by all implementations."""
        self._vehicle_id = vehicle_id
        self._logger = logger

    @abstractmethod
    def initialize(self) -> None:
        """Open the transport and perform autopilot-specific startup checks."""

    @abstractmethod
    def update(self, state: VehicleState, dt: float, simulation_time: float) -> None:
        """Exchange state/control using seconds for dt and absolute physics time."""

    @property
    @abstractmethod
    def motor_commands(self) -> np.ndarray:
        """Return rotor angular velocities in configured rotor order, in rad/s."""

    @abstractmethod
    def get_diagnostics(self) -> AdapterDiagnostics:
        """Return a snapshot without consuming timing samples or exposing transport objects."""

    @abstractmethod
    def close(self) -> None:
        """Release transport resources; safe before initialization or after close."""
