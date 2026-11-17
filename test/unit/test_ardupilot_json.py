# Copyright {YEAR} {COMPANY}
# {LICENSE}
"""Protocol, frame conversion and dynamics regression checks without Isaac."""

import json
import socket
import struct
import threading
from pathlib import Path

import numpy as np
from numpy.testing import assert_allclose

import pytest

from rs_isaac_uav_sim.sim.ardupilot_backend import ArduPilotJSON
from rs_isaac_uav_sim.sim.config import ArduPilotParams, QuadrotorParams, load_config_from_yaml
from rs_isaac_uav_sim.sim.dynamics import QuadrotorDynamics
from rs_isaac_uav_sim.sim.state import VehicleState

from scipy.spatial.transform import Rotation


@pytest.fixture
def link():
    """Link."""
    backend = ArduPilotJSON(1, ArduPilotParams(base_port=0, timeout=0.1), QuadrotorParams())
    backend.connect()
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.settimeout(1)
    yield backend, client
    backend.close()
    client.close()


def packet(count=0, rate=500, channels=16, values=None):
    """Packet."""
    return struct.pack(f'<HHI{channels}H', 18458 if channels == 16 else 29569,
                       rate, count, *(values or [1500] * channels))


def send(client, backend, data):
    """Send."""
    client.sendto(data, ('127.0.0.1', backend.port))


def test_exchange_frames_scaling_and_finite_difference(link):
    """Test exchange frames scaling and finite difference."""
    backend, client = link
    state = VehicleState()
    state.position = np.array([2., 3., 4.])
    state.angular_velocity = np.array([1., 2., 3.])
    send(client, backend, packet(values=[0, 1000, 1500, 2500] + [1000] * 12))
    backend.exchange(state, 0., .002)
    wire = client.recv(4096)
    assert wire.startswith(b'\n') and wire.endswith(b'\n')
    data = json.loads(wire)
    assert_allclose(data['position'], [3, 2, -4])
    assert_allclose(data['imu']['gyro'], [1, -2, -3])
    assert_allclose(data['imu']['accel_body'], [0, 0, -9.80665])
    q = data['quaternion']
    assert_allclose(Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix(),
                    Rotation.from_quat(state.get_attitude_ned_frd()).as_matrix())
    assert_allclose(backend.motor_commands, [0, 0, 500, 1000])
    state.linear_velocity = np.array([.002, .004, .006])
    send(client, backend, packet(1, channels=32))
    backend.exchange(state, .002, .002)
    data = json.loads(client.recv(4096))
    assert_allclose(data['imu']['accel_body'], [1, -2, -12.80665])
    assert_allclose(data['velocity'], [.004, .002, -.006])


def test_duplicate_replays_identical_state_without_advancing(link):
    """Test duplicate replays identical state without advancing."""
    backend, client = link
    state = VehicleState()
    send(client, backend, packet())
    backend.exchange(state, 0., .002)
    original = client.recv(4096)
    send(client, backend, packet())
    # Server replays frame zero, then waits for the next frame before returning.

    def next_frame():
        assert client.recv(4096) == original
        send(client, backend, packet(1))
    worker = threading.Thread(target=next_frame)
    worker.start()
    backend.exchange(state, .002, .002)
    worker.join(timeout=1)
    assert not worker.is_alive()
    assert json.loads(client.recv(4096))['timestamp'] == .002


@pytest.mark.parametrize('data', [b'', bytes(40), packet(rate=0), packet() + b'0'])
def test_invalid_packet(data):
    """Test invalid packet."""
    with pytest.raises(ValueError):
        ArduPilotJSON.decode_packet(data)


def test_timeout_zeroes_motor_output(link):
    """Test timeout zeroes motor output."""
    backend, _ = link
    backend.motor_commands[:] = 100
    with pytest.raises(TimeoutError):
        backend.exchange(VehicleState(), 0., .002)
    assert_allclose(backend.motor_commands, 0)


def test_restart_fails_closed_and_advisory_rate_accepted(link):
    """Test restart and rate mismatch fail closed."""
    backend, client = link
    send(client, backend, packet(10))
    backend.exchange(VehicleState(), 0., .002)
    client.recv(4096)
    send(client, backend, packet(0))
    with pytest.raises(RuntimeError, match='restarted'):
        backend.exchange(VehicleState(), .002, .002)
    assert_allclose(backend.motor_commands, 0)
    send(client, backend, packet(11, rate=400))
    backend.exchange(VehicleState(), .002, .002)
    assert json.loads(client.recv(4096))['timestamp'] == .002


def test_invalid_packet_ignored_before_valid_frame(link):
    """Test invalid packet ignored before valid frame."""
    backend, client = link
    send(client, backend, b'bad')
    send(client, backend, packet())
    backend.exchange(VehicleState(), 0., .002)
    assert json.loads(client.recv(4096))['timestamp'] == 0.


def test_quad_x_geometry_and_px4_default_unchanged():
    """Test quad x geometry and px4 default unchanged."""
    root = Path(__file__).resolve().parents[2]
    ap = load_config_from_yaml(str(root / 'config/ardupilot_quad_x.yaml'))
    p = ap['quadrotor']
    dynamics = QuadrotorDynamics(p)
    force, torque = dynamics.compute_forces_and_torques(np.full(4, 850.), VehicleState())
    assert force[2] > p.mass * 9.80665
    assert_allclose(torque, 0, atol=1e-12)
    # AP motors 1/2 create positive FRD yaw (negative FLU), 3/4 the reverse.
    for i, sign in enumerate([-1, -1, 1, 1]):
        omega = np.zeros(4)
        omega[i] = 500.
        _, torque = dynamics.compute_forces_and_torques(omega, VehicleState())
        assert np.sign(torque[2]) == sign
    px4 = load_config_from_yaml(str(root / 'config/px4_iris.yaml'))
    px4_dyn = QuadrotorDynamics(px4['quadrotor'])
    assert_allclose(px4_dyn.scale_motor_commands(np.full(4, .25)), 550.)
    assert_allclose(dynamics.scale_motor_commands(np.full(4, .25)), 275.)


@pytest.mark.parametrize('kwargs', [{'pwm_min': 2000}, {'timeout': -1},
                                    {'motor_channels': [0, 0, 2, 3]},
                                    {'motor_channels': [0, 1, 2, 32]}])
def test_invalid_config(kwargs):
    """Test invalid config."""
    with pytest.raises(ValueError):
        ArduPilotParams(**kwargs)
