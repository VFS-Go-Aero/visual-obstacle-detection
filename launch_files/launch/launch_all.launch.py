"""
Launch the full system in order, checking each step before starting the next.

    ros2 launch launch_files launch_all.launch.py              # with RViz
    ros2 launch launch_files launch_all.launch.py rviz:=false  # without RViz
    ros2 run launch_files stop                                 # stop everything

Boot order: camera -> perception -> MAVROS -> RViz.
With RViz (ground test), a failed check prints a WARNING and the boot keeps going.
Without RViz (flight test), a failed check prints NOT SAFE TO FLY and shuts everything down.
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

FCU_URL = "/dev/ttyACM0:57600"
RVIZ_CONFIG = os.path.expanduser("~/vfs_ground.rviz")

# Any of these running means the system is already up.
RUNNING_PATTERN = (
    "lib/mavros/mavros_node|lib/zed_wrapper/|component_container"
    "|install/visual_obstacle_detection/lib/"
)


def _include(package: str, launch_file: str, **launch_arguments) -> IncludeLaunchDescription:
    """Include a Python or XML launch file from another package."""
    path = os.path.join(get_package_share_directory(package), "launch", launch_file)
    if launch_file.endswith(".py"):
        source = PythonLaunchDescriptionSource(path)
    else:
        source = XMLLaunchDescriptionSource(path)
    return IncludeLaunchDescription(
        launch_description_source=source,
        launch_arguments=launch_arguments.items(),
    )


def _check(name: str, topic: str, timeout: int, problem: str, on_done: list, strict: bool,
           field: str = None, equals: str = None) -> list:
    """Wait for a topic, then start the on_done actions (or shut down on timeout if strict)."""
    arguments = [topic, "--timeout", str(timeout)]
    if field:
        arguments += ["--field", field, "--equals", equals]
    waiter = Node(
        package="launch_files",
        executable="wait_for_topic",
        name="wait_" + name,
        arguments=arguments,
        output="screen",
    )

    def on_exit(event, context):
        if context.is_shutdown:  # Ctrl+C: don't start the next step
            return None
        if event.returncode == 0:
            return on_done
        if strict:
            return [
                LogInfo(msg=f"!!! {problem}"),
                LogInfo(msg="!!! NOT SAFE TO FLY. Shutting everything down. "
                            "Full log: ~/.ros/log/latest/launch.log"),
                EmitEvent(event=Shutdown(reason="NOT SAFE TO FLY: " + problem)),
            ]
        return [LogInfo(msg=f"!! WARNING: {problem}. Continuing.")] + on_done

    return [waiter, RegisterEventHandler(OnProcessExit(target_action=waiter, on_exit=on_exit))]


def _boot(context) -> list:
    """Build the boot chain back to front: each step is the on_done of the check before it."""
    if subprocess.run(["pgrep", "-f", RUNNING_PATTERN], capture_output=True).returncode == 0:
        msg = "System already running. Run: ros2 run launch_files stop, then try again."
        return [LogInfo(msg=msg), EmitEvent(event=Shutdown(reason=msg))]

    rviz = IfCondition(LaunchConfiguration("rviz"))
    # No RViz means a flight test: any failed check stops everything.
    strict = not rviz.evaluate(context)

    done = [
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", RVIZ_CONFIG] if os.path.isfile(RVIZ_CONFIG) else [],
            additional_env={"DISPLAY": os.environ.get("DISPLAY", ":0")},
            output="log",
            condition=rviz,
        ),
        LogInfo(msg="=== SYSTEM UP. Ctrl+C here to stop everything. ==="),
    ]

    mavros = [
        LogInfo(msg="[3/3] MAVROS (after 20 s)"),
        TimerAction(period=20.0, actions=[
            _include("mavros", "apm.launch", fcu_url=FCU_URL),
            *_check("mavros", "/mavros/state", 40,
                    "MAVROS not connected to the flight controller (obstacles will NOT reach it)",
                    done, strict, field="connected", equals="True"),
        ]),
    ]

    perception = [
        LogInfo(msg="[2/3] Perception"),
        _include("visual_obstacle_detection", "visual_obstacle_detection.launch.py"),
        *_check("merged_cloud", "/merged_cloud", 30, "/merged_cloud not publishing", [
            Node(
                package="launch_files",
                executable="camera_marker",
                output="log",
                condition=rviz,
            ),
            *mavros,
        ], strict),
    ]

    return [
        LogInfo(msg="[1/3] Camera"),
        _include("launch_files", "single_zed.launch.py"),
        *_check("zed1", "/zed1/zed_node/point_cloud/cloud_registered", 90,
                "ZED1 not publishing", perception, strict),
    ]


def generate_launch_description() -> LaunchDescription:
    """Launch the full system using ROS 2 launch descriptions."""
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Open RViz (ground test). false = flight test: any failed check shuts down",
    )

    return LaunchDescription([rviz_arg, OpaqueFunction(function=_boot)])
