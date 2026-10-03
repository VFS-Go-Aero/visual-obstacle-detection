"""Shared boot sequence for ground.launch.py and flight.launch.py.

Boot order (same as by hand): cameras -> perception (point cloud + detection) -> [sphere]
                              -> loggers -> 20 s + MAVROS -> [RViz]
Each step starts only after the previous step's health check (wait_for_topic) exits.
  ground: a failed check prints a WARNING and boot keeps going, so you can debug.
  flight: a failed check prints NOT SAFE TO FLY and shuts everything down.
"""
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

# Processes that mean a stack is already up (same check as the old vfs_*.sh scripts)
RUNNING_PATTERN = ('lib/mavros/mavros_node|lib/zed_wrapper/|component_container'
                   '|install/visual_obstacle_detection/lib/')


def include(package, launch_file, **launch_arguments):
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    source = (XMLLaunchDescriptionSource(path) if not launch_file.endswith('.py')
              else PythonLaunchDescriptionSource(path))
    return IncludeLaunchDescription(source, launch_arguments=launch_arguments.items())


def check(name, topic, timeout, problem, on_done, ground, field=None, equals=None):
    """Run wait_for_topic, then the on_done actions (or stop, in flight mode)."""
    args = [topic, '--timeout', str(timeout)]
    if field:
        args += ['--field', field, '--equals', equals]
    waiter = Node(package='startup_launcher', executable='wait_for_topic',
                  name='wait_' + name, arguments=args, output='screen')

    def on_exit(event, context):
        if context.is_shutdown:  # Ctrl+C: don't start the next step
            return None
        if event.returncode == 0:
            return on_done
        if ground:
            return [LogInfo(msg=f'!! WARNING: {problem}. Continuing (ground mode).')] + on_done
        return [LogInfo(msg=f'!!! {problem}'),
                LogInfo(msg='!!! NOT SAFE TO FLY. Shutting everything down. '
                            'Full log: ~/.ros/log/latest/launch.log'),
                EmitEvent(event=Shutdown(reason='NOT SAFE TO FLY: ' + problem))]

    return [waiter, RegisterEventHandler(OnProcessExit(target_action=waiter, on_exit=on_exit))]


def boot(context, ground):
    if subprocess.run(['pgrep', '-f', RUNNING_PATTERN], capture_output=True).returncode == 0:
        msg = 'Stack already running. Run:  ros2 run startup_launcher stop   then try again.'
        return [LogInfo(msg=msg), EmitEvent(event=Shutdown(reason=msg))]

    total = 6 if ground else 4
    n = iter(range(1, total + 1))

    def step(title):
        return LogInfo(msg=f'[{next(n)}/{total}] {title}')

    # Steps are built in boot order (step() numbers them as it goes),
    # then chained back-to-front: each list is the on_done of the check before it.
    s_cams = step('Cameras (multi_zed)')
    s_perc = step('Perception (point_cloud + obstacle_detection + obstacle_to_mavlink)')
    s_marker = step('Orange camera sphere') if ground else None
    s_logs = step('Loggers')
    s_mav = step('MAVROS (20 s wait first)')
    s_rviz = step('RViz') if ground else None

    if ground:
        rviz_cfg = os.path.expanduser(LaunchConfiguration('rviz_config').perform(context))
        has_cfg = os.path.isfile(rviz_cfg)
        rviz = Node(package='rviz2', executable='rviz2', output='log',
                    arguments=['-d', rviz_cfg] if has_cfg else [],
                    additional_env={'DISPLAY': os.environ.get('DISPLAY', ':0')})
        done = [s_rviz, rviz,
                LogInfo(msg='=== GROUND STACK UP. Ctrl+C here to stop everything. ===')]
        if not has_cfg:
            done.append(LogInfo(msg='(No saved RViz layout yet: set it up once, then '
                                    f'File > Save Config As... {rviz_cfg})'))
    else:
        done = [LogInfo(msg='=== FLIGHT STACK UP. Safe to unplug the monitor. '
                            'Ctrl+C here to stop everything. ===')]

    mavros = [s_mav, TimerAction(period=20.0, actions=[
        include('mavros', 'apm.launch', fcu_url=LaunchConfiguration('fcu_url')),
        *check('mavros', '/mavros/state', 40,
               'MAVROS not connected to flight controller (obstacles will NOT reach it)',
               done, ground, field='connected', equals='True'),
    ])]

    loggers = [s_logs, Node(package='latency_logger', executable='monitor_launcher',
                            output='screen')] + mavros
    if ground:
        loggers = [s_marker, Node(package='startup_launcher', executable='camera_marker',
                                  output='log')] + loggers

    perception = [s_perc, include('visual_obstacle_detection',
                                  'visual_obstacle_detection.launch.py'),
                  *check('merged_cloud', '/merged_cloud', 30,
                         '/merged_cloud not publishing', loggers, ground)]

    zed2 = check('zed2', '/zed2/zed_node/point_cloud/cloud_registered', 90,
                 'ZED2 not publishing', perception, ground)
    zed1 = check('zed1', '/zed1/zed_node/point_cloud/cloud_registered', 90,
                 'ZED1 not publishing', zed2, ground)

    return [s_cams, include('launch_files', 'multi_zed.launch.py'), *zed1]


def generate(mode):
    ground = mode == 'ground'
    args = [DeclareLaunchArgument('fcu_url', default_value='/dev/ttyACM0:115200',
                                  description='MAVROS flight controller link')]
    if ground:
        args.append(DeclareLaunchArgument('rviz_config', default_value='~/vfs_ground.rviz',
                                          description='RViz layout (used if the file exists)'))
    return LaunchDescription(args + [OpaqueFunction(function=boot, kwargs={'ground': ground})])
