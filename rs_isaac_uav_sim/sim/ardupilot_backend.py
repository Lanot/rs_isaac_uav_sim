# Copyright {YEAR} {COMPANY}
# {LICENSE}
"""Compatibility import; JSON transport lives in autopilot.ardupilot.backend."""

from .autopilot.ardupilot.backend import ArduPilotJSON

__all__ = ['ArduPilotJSON']
