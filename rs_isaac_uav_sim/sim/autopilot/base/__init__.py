# Copyright {YEAR} {COMPANY}
# {LICENSE}
"""Shared autopilot contract and diagnostic data."""

from .adapter import AutopilotAdapter
from .diagnostics import AdapterDiagnostics

__all__ = ['AdapterDiagnostics', 'AutopilotAdapter']
