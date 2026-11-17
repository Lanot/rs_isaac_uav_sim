# Copyright {YEAR} {COMPANY}
# {LICENSE}
"""Autopilot adapters and their shared factory/interface."""

from .base import AdapterDiagnostics, AutopilotAdapter
from .factory import create_autopilot_adapter

__all__ = ['AdapterDiagnostics', 'AutopilotAdapter', 'create_autopilot_adapter']
