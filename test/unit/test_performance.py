# Copyright 2026 ROBOTSIX
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Offline checks for simulator-time RTF and inclusive phase reporting."""

import pytest

from rs_isaac_uav_sim.sim.performance import SimulationProfiler


def test_actual_simulator_progress_and_nested_phase_reporting():
    """RTF uses sim time, not loop count, and nested timings stay separate."""
    now = [100.0]
    lines = []
    profiler = SimulationProfiler(0.0, 0.002, clock=lambda: now[0],
                                  report=lambda text, **kwargs: lines.append(text))

    def child():
        now[0] += 0.001
        return 'value'

    def parent():
        assert profiler.call('child', child) == 'value'
        now[0] += 0.001

    profiler.call('parent', parent)
    now[0] = 105.0
    profiler.sample(2.5)
    assert 'rtf=0.500' in lines[0]
    assert 'physics_hz=250.0' in lines[0]
    assert any('parent total_ms=2.0 calls=1' in line for line in lines)
    assert any('child total_ms=1.0 calls=1' in line for line in lines)
    assert profiler.timings == {}


def test_reset_within_window_discards_pre_reset_work():
    """A reset must be caught even when the new time exceeds window start."""
    now = [0.0]
    lines = []
    profiler = SimulationProfiler(0.0, 0.002, clock=lambda: now[0],
                                  report=lambda text, **kwargs: lines.append(text))
    now[0] = 1.0
    profiler.sample(0.8)
    profiler.record('old', 0.01)
    now[0] = 2.0
    profiler.sample(0.2)
    assert profiler.timings == {}
    now[0] = 7.0
    profiler.sample(2.7)
    assert 'rtf=0.500' in lines[0]
    assert not any('old' in line for line in lines)


def test_call_preserves_exceptions():
    """Profiling must not swallow a simulator or communication failure."""
    profiler = SimulationProfiler(0.0, 0.002)

    def fail():
        raise RuntimeError('physics failed')

    with pytest.raises(RuntimeError, match='physics failed'):
        profiler.call('failure', fail)
    assert profiler.timings['failure'][1] == 1


@pytest.mark.parametrize('interval,dt', [(0, 0.002), (float('nan'), 0.002), (5, 0)])
def test_invalid_measurement_parameters(interval, dt):
    """Invalid intervals cannot produce misleading rates or divide by zero."""
    with pytest.raises(ValueError):
        SimulationProfiler(0.0, dt, interval=interval)


@pytest.mark.parametrize('profile', [False, True])
def test_cpu_step_preserves_frame_conversion_and_owns_state_snapshot(profile):
    """Exercise the actual step body with real CPU tensors and fake IO/views."""
    import ast
    from pathlib import Path
    from types import SimpleNamespace

    import numpy as np
    from numpy.testing import assert_allclose
    from scipy.spatial.transform import Rotation
    import torch

    from rs_isaac_uav_sim.sim.state import VehicleState

    # Loading only this method avoids booting Kit in an offline unit test.
    # View and transport boundaries are fakes; tensor/math operations are real.
    source = Path(__file__).resolve().parents[2] / 'rs_isaac_uav_sim/sim/vehicle.py'
    tree = ast.parse(source.read_text())
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef))
    method = next(node for node in cls.body if isinstance(node, ast.FunctionDef)
                  and node.name == 'step')
    namespace = {'np': np, 'Rotation': Rotation,
                 '_STABLE_VEL_MS': 0.05, '_STABLE_OMEGA_RPS': 0.05,
                 '_STABLE_STEPS': 250}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
    pose = torch.tensor([[1.0, 2.0, 3.0]], dtype=torch.float32)
    orientation = torch.tensor([[2**-0.5, 0.0, 0.0, 2**-0.5]], dtype=torch.float32)
    velocity = torch.tensor([[2.0, 0.0, 0.0, 0.0, 1.0, 0.0]], dtype=torch.float32)
    applied = []

    class View:
        def get_world_poses(self, *, usd, clone):
            assert not usd and not clone
            return pose, orientation

        def get_velocities(self, *, clone):
            assert not clone
            return velocity

        def apply_forces_and_torques_at_pos(self, force, torque, *, is_global):
            assert is_global
            applied.append((force.clone(), torque.clone()))

    state = VehicleState()
    sensor = SimpleNamespace(update=lambda state, dt: {})
    backend = SimpleNamespace(update_sensors=lambda *args: None, update=lambda: None,
                              send_ground_truth=lambda *args: None, current_utime=0,
                              motor_commands=np.zeros(4))
    dynamics = SimpleNamespace(compute_forces_and_torques=lambda omega, state:
                               (np.array([0.0, 0.0, 9.81]), np.array([1.0, 2.0, 3.0])))
    manager = SimpleNamespace(
        _prims=SimpleNamespace(is_physics_handle_valid=lambda: True),
        _frame_view=View(), _torch=torch, num_drones=1,
        profiler=SimulationProfiler(0.0, 0.002) if profile else None,
        _gps_counter=0, _gps_divider=10, states=[state],
        _stable_count=[0], _stabilized=[True], baro_sensors=[sensor],
        mag_sensors=[sensor], imu_sensors=[sensor], _last_imu=[{}],
        _last_baro=[{}], _last_mag=[{}], backends=[backend], dynamics=[dynamics],
        _sensor_params=SimpleNamespace(localization_mode='mocap'),
        _forces_tensor=torch.zeros((1, 3)), _torques_tensor=torch.zeros((1, 3)),
        _verbose=False)
    namespace['step'](manager, 0.002)
    assert_allclose(state.angular_velocity, [1.0, 0.0, 0.0], atol=1e-6)
    assert_allclose(applied[0][0].numpy(), [[0.0, 0.0, 9.81]], atol=1e-6)
    assert_allclose(applied[0][1].numpy(), [[-2.0, 1.0, 3.0]], atol=1e-6)
    pose.zero_()
    velocity.zero_()
    assert_allclose(state.position, [1.0, 2.0, 3.0])
    assert_allclose(state.linear_velocity, [2.0, 0.0, 0.0])
    if profile:
        assert manager.profiler.timings['vehicle.readback'][1] == 1
        assert manager.profiler.timings['vehicle.force_submit'][1] == 1


@pytest.mark.parametrize('filename', ['scene_mavlink_sim.py', 'px4_sitl.launch.py'])
def test_launch_forwards_selected_device_and_profiling(filename, monkeypatch):
    """Both launch entrypoints must pass performance options to Isaac Python."""
    import ast
    import os
    from pathlib import Path

    settings = {'physics_device': 'cpu', 'profile': 'true', 'render_quality': 'fast',
                'num_drones': '0', 'headless': 'true', 'config_file': '', 'world_usd': '',
                'base_port': '4560', 'connection_ip': 'localhost', 'verbose': 'false',
                'ros2_namespaces': '', 'extra_px4_env': '', 'airframe_file': '',
                'display_terminal': 'none', 'px4_sitl_build_dir': '/unused/px4'}

    class Config:
        def __init__(self, name):
            self.name = name

        def perform(self, context):
            return settings[self.name]

    source = Path(__file__).resolve().parents[2] / 'launch' / filename
    tree = ast.parse(source.read_text())
    tree.body = [node for node in tree.body if not isinstance(node, (ast.Import, ast.ImportFrom))]
    namespace = {'os': os, 're': __import__('re'), 'LaunchConfiguration': Config,
                 'ExecuteProcess': lambda **kwargs: kwargs,
                 'get_package_prefix': lambda name: '/fake/install'}
    monkeypatch.setenv('ISAAC_SIM_PYTHON', '/fake/isaac/python.sh')
    exec(compile(tree, str(source), 'exec'), namespace)
    actions = namespace['_launch_setup'](None)
    command = actions[0]['cmd']
    assert command[command.index('--physics_device') + 1] == 'cpu'
    assert command[command.index('--render_quality') + 1] == 'fast'
    assert '--profile' in command
    settings['profile'] = 'false'
    assert '--profile' not in namespace['_launch_setup'](None)[0]['cmd']
