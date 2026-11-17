# Copyright {YEAR} {COMPANY}
# {LICENSE}
"""Protocol-independent diagnostic snapshot for the simulation manager."""

from dataclasses import dataclass, field
from typing import Any

import numpy as np


@dataclass(frozen=True)
class AdapterDiagnostics:
    """Copied control values and optional protocol-specific diagnostic fields.

    Unknown arming/mode values stay None, rather than inferring them from PWM.
    Sensor values retain their existing units and reference frames.
    """

    name: str
    vehicle_id: int
    raw_controls: np.ndarray
    motor_commands: np.ndarray
    armed: bool | None = None
    mode: int | None = None
    sensors: dict[str, dict] = field(default_factory=dict)
    timing: dict[str, Any] = field(default_factory=dict)
