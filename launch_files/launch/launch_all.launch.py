import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
try:
    from launch.launch_description_sources import AnyLaunchDescriptionSource
except ImportError:
    AnyLaunchDescriptionSource = None
try:
    from launch.launch_description_sources import XMLLaunchDescriptionSource
except ImportError:
    XMLLaunchDescriptionSource = None
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_launch_file(package_name: str, base_name: str) -> str:
    package_share = get_package_share_directory(package_name)
    candidates = glob.glob(os.path.join(package_share, "launch", f"{base_name}*"))
    if not candidates:
        raise FileNotFoundError(
            f"Launch file for {package_name} not found: {base_name}"
        )
    return candidates[0]


def _source_for_launch_file(path: str):
    if path.endswith(".py"):
        return PythonLaunchDescriptionSource(path)
    if AnyLaunchDescriptionSource is not None:
        return AnyLaunchDescriptionSource(path)
    if XMLLaunchDescriptionSource is not None:
        return XMLLaunchDescriptionSource(path)
    raise ImportError(
        "No compatible launch description source available for non-Python launch files."
    )


def generate_launch_description() -> LaunchDescription:
    """Launch the full system using ROS 2 launch descriptions."""
    logging_arg = DeclareLaunchArgument(
        "logging",
        default_value="true",
        description="Enable ROS-level debug logging",
    )
    logging_enabled = LaunchConfiguration("logging")

    latency_logger = Node(
        package="latency_logger",
        executable="monitor_launcher",
        name="latency_logger",
        output="screen",
        condition=IfCondition(logging_enabled),
    )

    mavros_launch = _find_launch_file("mavros", "apm.launch")
    multi_zed_launch = os.path.join(
        get_package_share_directory("launch_files"),
        "launch",
        "multi_zed.launch.py",
    )
    obstacle_launch = os.path.join(
        get_package_share_directory("visual_obstacle_detection"),
        "launch",
        "visual_obstacle_detection.launch.py",
    )

    return LaunchDescription(
        [
            logging_arg,
            latency_logger,
            IncludeLaunchDescription(
                launch_description_source=_source_for_launch_file(mavros_launch),
                launch_arguments={
                    "fcu_url": "/dev/ttyACM0:115200",
                }.items(),
            ),
            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource(multi_zed_launch),
            ),
            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource(obstacle_launch),
            ),
        ]
    )

