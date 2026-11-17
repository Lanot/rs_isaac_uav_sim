# Copyright {YEAR} {COMPANY}
# {LICENSE}
"""Launch Isaac and a single ArduCopter JSON SITL in isolated runtime state."""

import os
import shutil
import tempfile

from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

from rs_isaac_uav_sim.sim.config import load_config_from_yaml


def _launch_setup(context, *args, **kwargs):
    python = os.environ.get('ISAAC_SIM_PYTHON')
    if not python:
        raise EnvironmentError('Set ISAAC_SIM_PYTHON to the Isaac Sim Python executable')

    def value(name):
        return LaunchConfiguration(name).perform(context)
    binary = os.path.abspath(os.path.expanduser(value('arducopter_binary')))
    if not os.path.isfile(binary) or not os.access(binary, os.X_OK):
        raise FileNotFoundError('Provide arducopter_binary:=/path/to/build/sitl/bin/arducopter')
    prefix = get_package_prefix('rs_isaac_uav_sim')
    config = value('config_file')
    cfg = load_config_from_yaml(config)
    ap = cfg['ardupilot']
    if ap.bind_ip not in ('127.0.0.1', 'localhost'):
        raise ValueError('Bundled launcher supports local SITL; use bind_ip: 127.0.0.1')
    if ap.base_port == 0:
        raise ValueError('Launcher requires a fixed UDP port')
    # Each run gets its own eeprom.bin, logs and MAVLink TCP port (instance 0 by default).
    workdir = tempfile.mkdtemp(prefix='rs_isaac_ardupilot_')
    params = value('parameter_file')
    # ArduPilot parses --defaults as a comma-separated list of paths.
    runtime_params = os.path.join(workdir, 'quad_x.parm')
    shutil.copyfile(params, runtime_params)
    scene = [python, os.path.join(prefix, 'lib', 'rs_isaac_uav_sim',
                                  'scene_mavlink_sim.py'),
             '--autopilot', 'ardupilot', '--config', config]
    if value('headless') == 'true':
        scene.append('--headless')
    if value('verbose') == 'true':
        scene.append('--verbose')
    if value('world_usd'):
        scene.extend(['--world_usd', value('world_usd')])
    if value('ros2_namespaces'):
        scene.extend(['--ros2_namespaces', value('ros2_namespaces')])
    origin = cfg['gps_origin']
    copter = [binary, '--instance', value('instance'),
              '--model', 'JSON:127.0.0.1', '--defaults', runtime_params,
              '--sim-port-out', str(ap.base_port), '--home',
              f'{origin.lat},{origin.lon},{origin.alt},90',
              '--rate', '500', '--speedup', '1', '--serial0', 'tcp:0', '--synthetic-clock']
    sim_process = ExecuteProcess(cmd=scene, output='screen')
    copter_process = ExecuteProcess(cmd=copter, cwd=workdir, output='screen')
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=process,
        on_exit=[EmitEvent(event=Shutdown(reason='Simulation process exited'))],
    )) for process in (sim_process, copter_process)]
    return handlers + [sim_process, copter_process]


def generate_launch_description():
    """Declare a separate launcher, preserving PX4 launch defaults."""
    prefix = get_package_prefix('rs_isaac_uav_sim')
    return LaunchDescription([
        DeclareLaunchArgument('arducopter_binary',
                              default_value=os.environ.get('ARDUCOPTER_BINARY', '')),
        DeclareLaunchArgument('config_file', default_value=os.path.join(
            prefix, 'share', 'rs_isaac_uav_sim', 'config', 'ardupilot_quad_x.yaml')),
        DeclareLaunchArgument('parameter_file', default_value=os.path.join(
            prefix, 'share', 'rs_isaac_uav_sim', 'config', 'ardupilot_quad_x.parm')),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('world_usd', default_value=''),
        DeclareLaunchArgument('ros2_namespaces', default_value=''),
        DeclareLaunchArgument('instance', default_value='0'),
        OpaqueFunction(function=_launch_setup),
    ])
