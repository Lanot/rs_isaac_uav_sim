# Copyright {YEAR} {COMPANY}
# {LICENSE}
"""Exercise adapter contracts, factory isolation and PX4 sensor scheduling."""

import logging
import subprocess
import sys
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
from numpy.testing import assert_allclose

import pytest

from rs_isaac_uav_sim.sim.autopilot import AutopilotAdapter, create_autopilot_adapter
from rs_isaac_uav_sim.sim.autopilot.ardupilot import ArduPilotAdapter
from rs_isaac_uav_sim.sim.autopilot.base import AdapterDiagnostics
from rs_isaac_uav_sim.sim.autopilot.px4 import adapter as px4_module
from rs_isaac_uav_sim.sim.config import ArduPilotParams, SensorParams
from rs_isaac_uav_sim.sim.state import VehicleState


@pytest.mark.parametrize('name', ['px4', 'ardupilot'])
def test_factory_contract_and_construction_without_connections(name, monkeypatch):
    """Both implementations satisfy the ABC and construct without socket calls."""
    import socket
    monkeypatch.setattr(socket, 'socket', MagicMock(side_effect=AssertionError('opened socket')))
    adapter = create_autopilot_adapter(name, vehicle_id=1)
    assert isinstance(adapter, AutopilotAdapter)
    assert_allclose(adapter.motor_commands, np.zeros(4))
    snapshot = adapter.get_diagnostics()
    assert isinstance(snapshot, AdapterDiagnostics)
    snapshot.motor_commands[:] = 99
    assert_allclose(adapter.motor_commands, 0)
    adapter.close()
    adapter.close()


def test_abstract_contract_cannot_be_instantiated():
    """Missing lifecycle/update implementations are rejected before any physics runs."""
    with pytest.raises(TypeError):
        AutopilotAdapter()


def test_factory_rejects_unknown_autopilot_and_ardupilot_mocap():
    """Configuration errors belong to factory/adapter creation, not vehicle stepping."""
    with pytest.raises(ValueError, match='Unsupported autopilot'):
        create_autopilot_adapter('unknown', vehicle_id=1)
    with pytest.raises(ValueError, match='mocap'):
        create_autopilot_adapter('ardupilot', vehicle_id=1,
                                 sensor_params=SensorParams(localization_mode='mocap'))


def test_ardupilot_import_does_not_require_px4_or_isaac():
    """JSON-only use remains independent of pymavlink and Isaac's Python runtime."""
    code = """import sys
sys.modules['pymavlink'] = None
sys.modules['isaacsim'] = None
from rs_isaac_uav_sim.sim.autopilot import create_autopilot_adapter
adapter = create_autopilot_adapter('ardupilot', vehicle_id=1)
assert 'rs_isaac_uav_sim.sim.autopilot.px4.backend' not in sys.modules
adapter.close()
"""
    root = Path(__file__).resolve().parents[2]
    subprocess.run([sys.executable, '-c', code], cwd=root, check=True,
                   capture_output=True, text=True)


@pytest.fixture
def px4(monkeypatch):
    """Use a recording HIL transport to exercise actual adapter sensor behavior."""
    backend = MagicMock()
    backend.wait_for_heartbeat.return_value = True
    backend.motor_commands = np.zeros(4)
    backend.raw_controls = np.zeros(4)
    backend.raw_mode = 0
    backend.current_utime = 123000
    monkeypatch.setattr(px4_module, 'PX4MavlinkHIL', MagicMock(return_value=backend))
    adapter = create_autopilot_adapter('px4', vehicle_id=1)
    return adapter, backend


def test_px4_lifecycle_requires_heartbeat_and_closes_on_failure(px4):
    """Readiness failures release the transport before propagating to the manager."""
    adapter, backend = px4
    backend.wait_for_heartbeat.return_value = False
    with pytest.raises(RuntimeError, match='no heartbeat'):
        adapter.initialize()
    backend.connect.assert_called_once()
    backend.wait_for_heartbeat.assert_called_once_with(timeout=30.0)
    backend.close.assert_called_once()


def test_px4_update_preserves_gps_cadence_and_ideal_startup_imu(px4):
    """GPS is sent on step one and then every ten steps, at the supplied clock."""
    adapter, backend = px4
    state = VehicleState()
    adapter.initialize()
    for step in range(1, 22):
        adapter.update(state, .002, step * .002)
    assert backend.update_gps.call_count == 3  # steps 1, 10, 20
    assert backend.update_sensors.call_count == 21
    backend.update.assert_called_with(.042)
    backend.send_ground_truth.assert_called_with(state, 123000)
    imu = backend.update_sensors.call_args.args[0]
    assert_allclose(imu['angular_velocity'], [0, 0, 0])
    assert_allclose(imu['linear_acceleration'], [0, 0, -9.80665])
    diagnostics = adapter.get_diagnostics()
    assert set(diagnostics.sensors) == {'imu', 'barometer', 'magnetometer', 'gps'}
    assert diagnostics.armed is False


def test_px4_stability_gate_uses_measured_imu_after_settling(px4):
    """Motion resets the settling window and measured IMU begins after 250 quiet steps."""
    adapter, backend = px4
    adapter._imu = MagicMock()
    measured = {'angular_velocity': np.array([1., 2., 3.]),
                'linear_acceleration': np.array([4., 5., 6.])}
    adapter._imu.update.return_value = measured
    state = VehicleState()
    for step in range(249):
        adapter.update(state, .002, step * .002)
    adapter._imu.update.assert_not_called()
    state.linear_velocity = np.array([1., 0., 0.])
    adapter.update(state, .002, .498)
    state.linear_velocity = np.zeros(3)
    for step in range(250):
        adapter.update(state, .002, .5 + step * .002)
    adapter._imu.update.assert_called_once_with(state, .002)
    assert backend.update_sensors.call_args.args[0] is measured


def test_px4_mocap_never_sends_gps(monkeypatch):
    """PX4 mocap keeps HIL sensors but leaves localization to its external bridge."""
    backend = MagicMock()
    backend.motor_commands = np.zeros(4)
    monkeypatch.setattr(px4_module, 'PX4MavlinkHIL', MagicMock(return_value=backend))
    adapter = create_autopilot_adapter(
        'px4', vehicle_id=2, sensor_params=SensorParams(localization_mode='mocap'))
    for step in range(20):
        adapter.update(VehicleState(), .002, step * .002)
    backend.update_gps.assert_not_called()
    assert backend.update_sensors.call_count == 20
    assert 'gps' not in adapter.get_diagnostics().sensors


def test_ardupilot_adapter_uses_same_clock_and_motor_interface(monkeypatch):
    """The wrapper forwards physics time/dt and leaves arming status unknown."""
    from rs_isaac_uav_sim.sim.autopilot.ardupilot import adapter as ap_module
    backend = MagicMock()
    backend.motor_commands = np.array([100., 200., 300., 400.])
    backend.raw_controls = np.array([.1, .2, .3, .4])
    constructor = MagicMock(return_value=backend)
    monkeypatch.setattr(ap_module, 'ArduPilotJSON', constructor)
    params = ArduPilotParams(base_port=19002)
    adapter = create_autopilot_adapter(
        'ardupilot', vehicle_id=3, ardupilot_params=params, logger=logging.getLogger(__name__))
    assert isinstance(adapter, ArduPilotAdapter)
    adapter.initialize()
    state = VehicleState()
    adapter.update(state, .002, 4.2)
    backend.exchange.assert_called_once_with(state, 4.2, .002)
    assert_allclose(adapter.motor_commands, [100, 200, 300, 400])
    snapshot = adapter.get_diagnostics()
    assert snapshot.armed is None and snapshot.mode is None
    assert snapshot.sensors == {}
    adapter.close()
    backend.close.assert_called_once()


def test_px4_backend_uses_explicit_clock_without_isaac(monkeypatch):
    """The real HIL serializer uses adapter time without importing SimulationContext."""
    from rs_isaac_uav_sim.sim.autopilot.px4.backend import PX4MavlinkHIL
    backend = PX4MavlinkHIL(1)
    backend._connection = MagicMock()
    backend._connection.recv_match.return_value = None
    backend._is_running = True
    backend._received_first_heartbeat = True
    backend.update_sensors({'angular_velocity': np.zeros(3),
                            'linear_acceleration': np.array([0, 0, -9.80665])}, {}, {})
    monkeypatch.setitem(sys.modules, 'isaacsim', None)
    backend.update(12.5)
    assert backend.current_utime == 12_500_000
    args = backend._connection.mav.hil_sensor_send.call_args.args
    assert args[0] == 12_500_000
    backend.close()
