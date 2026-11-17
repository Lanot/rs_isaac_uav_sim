# Copyright {YEAR} {COMPANY}
# {LICENSE}
"""Create an autopilot adapter without importing the other protocol stack."""

import logging
from typing import Any

from .base import AutopilotAdapter
from ..config import ArduPilotParams, GPSOrigin, MavlinkParams, QuadrotorParams, SensorParams


def create_autopilot_adapter(
    autopilot: str,
    *,
    vehicle_id: int,
    quad_params: QuadrotorParams | None = None,
    sensor_params: SensorParams | None = None,
    mavlink_params: MavlinkParams | None = None,
    gps_origin: GPSOrigin | None = None,
    ardupilot_params: ArduPilotParams | None = None,
    verbose: bool = False,
    logger: Any = None,
) -> AutopilotAdapter:
    """Select PX4 or ArduPilot; defer network operations to initialize()."""
    quad_params = quad_params or QuadrotorParams()
    sensor_params = sensor_params or SensorParams()
    logger = logger if logger is not None else logging.getLogger(__name__)
    if autopilot == 'px4':
        from .px4 import PX4Adapter
        return PX4Adapter(
            vehicle_id, quad_params, sensor_params, mavlink_params or MavlinkParams(),
            gps_origin or GPSOrigin(), verbose=verbose, logger=logger,
        )
    if autopilot == 'ardupilot':
        from .ardupilot import ArduPilotAdapter
        return ArduPilotAdapter(
            vehicle_id, quad_params, sensor_params,
            ardupilot_params or ArduPilotParams(), logger=logger,
        )
    raise ValueError(f'Unsupported autopilot: {autopilot!r}; expected px4 or ardupilot')
