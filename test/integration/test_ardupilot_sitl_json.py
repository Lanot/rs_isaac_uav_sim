# Copyright {YEAR} {COMPANY}
# {LICENSE}
"""Opt-in real ArduCopter JSON handshake; stationary truth, no flight claim."""

import os
import shutil
import subprocess
from pathlib import Path

import numpy as np

import pytest

from rs_isaac_uav_sim.sim.autopilot.ardupilot.backend import ArduPilotJSON
from rs_isaac_uav_sim.sim.config import ArduPilotParams, QuadrotorParams
from rs_isaac_uav_sim.sim.state import VehicleState


def test_real_arducopter_json(tmp_path):
    """Validate 100 state exchanges against a real, disarmed ArduCopter."""
    binary = os.environ.get('ARDUCOPTER_BINARY')
    if not binary or not Path(binary).is_file():
        pytest.skip('Set ARDUCOPTER_BINARY to run the real SITL protocol test')
    backend = ArduPilotJSON(1, ArduPilotParams(base_port=0, timeout=20), QuadrotorParams())
    backend.connect()
    root = Path(__file__).resolve().parents[2]
    defaults = root / 'config/ardupilot_quad_x.parm'
    runtime_defaults = tmp_path / 'quad_x.parm'
    shutil.copyfile(defaults, runtime_defaults)
    log = tmp_path / 'arducopter.log'
    # High instance avoids existing GCS/RC ports; explicit simulator port comes last.
    command = [str(Path(binary).resolve()), '--instance', '91',
               '--model', 'JSON:127.0.0.1', '--defaults', str(runtime_defaults),
               '--home', '47.24926,-1.54844,0,90',
               '--sim-port-out', str(backend.port), '--rate', '500', '--speedup', '1',
               '--serial0', 'tcp:0', '--synthetic-clock']
    try:
        with log.open('w') as stream:
            process = subprocess.Popen(command, cwd=tmp_path, stdout=stream,
                                       stderr=subprocess.STDOUT)
            try:
                state = VehicleState()
                for step in range(100):
                    backend.exchange(state, step * .002, .002)
                    assert process.poll() is None, log.read_text()
                    assert np.isfinite(backend.motor_commands).all()
                assert backend._frame >= 99
            finally:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
    finally:
        backend.close()
    output = log.read_text()
    assert 'JSON received' in output, output
    assert 'accel_body' in output and 'quaternion' in output, output
