from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch the visual obstacle detection pipeline in order."""
    return LaunchDescription(
        [
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
            ),
            Node(
                package="visual_obstacle_detection",
                executable="obstacle_to_mavlink",
                name="obstacle_to_mavlink",
                output="screen",
            ),
        ]
    )
