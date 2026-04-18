from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch the visual obstacle detection pipeline in order."""
    verbose_arg = DeclareLaunchArgument(
        "v",
        default_value="false",
        description="Enable verbose obstacle detection logging",
    )

    verbose = LaunchConfiguration("v")

    return LaunchDescription(
        [
            verbose_arg,
            Node(
                package="visual_obstacle_detection",
                executable="point_cloud",
                name="point_cloud",
                output="screen",
            ),
            Node(
                package="visual_obstacle_detection",
                executable="obstacle_detection",
                name="obstacle_detection",
                output="screen",
                parameters=[{"verbose": verbose}],
            ),
            Node(
                package="visual_obstacle_detection",
                executable="obstacle_to_mavlink",
                name="obstacle_to_mavlink",
                output="screen",
            ),
        ]
    )
