# Copyright {YEAR} {COMPANY}
# {LICENSE}
"""ArduPilot JSON SITL transport, independent of Isaac Sim and MAVLink.

Protocol: https://github.com/ArduPilot/ardupilot/tree/master/libraries/SITL/examples/JSON
MAVLink telemetry and commands remain between ArduPilot and its GCS.
"""

import json
import socket
import struct
import time

import numpy as np

from scipy.spatial.transform import Rotation

from ...config import ArduPilotParams
from ...state import flu_to_frd


class ArduPilotJSON:
    """Receive one PWM frame per physics step and reply with current truth.

    Duplicate requests replay the cached reply without advancing physics.
    A peer change or backwards frame counter requires a simulator restart.
    Timeout raises rather than continuing to apply stale motor commands.
    """

    def __init__(self, vehicle_id, params, quad_params):
        """Configure one Quad-X vehicle and its dedicated JSON UDP port."""
        self._vehicle_id = vehicle_id
        self.params = params or ArduPilotParams()
        self.quad_params = quad_params
        self.port = self.params.base_port + (vehicle_id - 1) * self.params.port_stride
        if not 0 <= self.port <= 65535:
            raise ValueError('ArduPilot instance port exceeds UDP range')
        if len(quad_params.rotor_positions) != 4 or quad_params.omega_min != 0:
            raise ValueError('ArduPilot currently requires four rotors and omega_min=0')
        self.motor_commands = np.zeros(4)
        self.raw_controls = np.zeros(4)
        self._socket = None
        self._peer = None
        self._frame = None
        self._reply = None
        self._previous_velocity = None
        self._previous_time = None

    def connect(self):
        """Bind a dedicated UDP socket without waiting for a MAVLink heartbeat."""
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._socket.bind((self.params.bind_ip, self.port))
            self.port = self._socket.getsockname()[1]
        except BaseException:
            self.close()
            raise

    @staticmethod
    def decode_packet(packet):
        """Validate little-endian 16/32-channel ArduPilot servo packets."""
        if len(packet) not in (40, 72):
            raise ValueError('Invalid servo packet length')
        magic, rate, count = struct.unpack_from('<HHI', packet)
        channels = 16 if len(packet) == 40 else 32
        if magic != {16: 18458, 32: 29569}[channels] or rate == 0:
            raise ValueError('Invalid servo packet magic or frame rate')
        return rate, count, struct.unpack_from(f'<{channels}H', packet, 8)

    def exchange(self, state, timestamp, dt):
        """Wait for fresh controls, send truth, and permit exactly one step."""
        if self._socket is None:
            raise RuntimeError('ArduPilot socket is not connected')
        deadline = time.monotonic() + self.params.timeout
        try:
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('No fresh ArduPilot servo frame before timeout')
                self._socket.settimeout(remaining)
                packet, peer = self._socket.recvfrom(2048)
                try:
                    _rate, count, pwm = self.decode_packet(packet)
                except ValueError:
                    continue
                if self._peer is not None and peer != self._peer:
                    raise RuntimeError('ArduPilot peer changed; restart the simulator')
                if self._frame is not None:
                    delta = (count - self._frame) & 0xffffffff
                    if delta == 0:
                        assert self._reply is not None
                        self._socket.sendto(self._reply, peer)
                        continue
                    if delta > 0x7fffffff:
                        raise RuntimeError('ArduPilot restarted; restart the simulator')
                # frame_rate is advisory; ArduPilot ramps it from its startup
                # default. Isaac remains authoritative at the fixed physics dt.
                if max(self.params.motor_channels) >= len(pwm):
                    raise ValueError('Configured motor channel absent from servo packet')
                if self._previous_time is not None and timestamp <= self._previous_time:
                    raise ValueError('Simulation time must increase; restart after a world reset')
                controls = np.array([pwm[c] for c in self.params.motor_channels], dtype=float)
                controls = np.clip((controls - self.params.pwm_min)
                                   / (self.params.pwm_max - self.params.pwm_min), 0, 1)
                # Raw PWM is ESC demand. This profile uses a quadratic thrust curve.
                self.raw_controls = controls
                self.motor_commands = controls * self.quad_params.omega_max
                accel = np.zeros(3) if self._previous_velocity is None else (
                    state.linear_velocity - self._previous_velocity) / dt
                specific_force = Rotation.from_quat(state.attitude).inv().apply(
                    accel - np.array([0.0, 0.0, -9.80665]))
                q = state.get_attitude_ned_frd()
                data = {
                    'timestamp': float(timestamp),
                    'imu': {'gyro': state.get_angular_velocity_frd().tolist(),
                            'accel_body': flu_to_frd(specific_force).tolist()},
                    'position': state.get_position_ned().tolist(),
                    'velocity': state.get_linear_velocity_ned().tolist(),
                    'quaternion': [float(q[3]), float(q[0]), float(q[1]), float(q[2])],
                }
                # Compact separators are required by ArduPilot's simple parser.
                reply = ('\n' + json.dumps(data, separators=(',', ':'), allow_nan=False)
                         + '\n').encode('ascii')
                self._socket.sendto(reply, peer)
                self._peer, self._frame, self._reply = peer, count, reply
                self._previous_time = timestamp
                self._previous_velocity = state.linear_velocity.copy()
                return
        except BaseException:
            self.motor_commands[:] = 0
            raise

    def close(self):
        """Release the UDP port and clear actuator demand."""
        if self._socket is not None:
            self._socket.close()
            self._socket = None
        self.motor_commands[:] = 0
