#!/usr/bin/env python3
"""Launch the connected ZED camera as zed2 with an explicit base_link transform."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Build a single-camera launch description with explicit TF ownership."""
    camera_pitch_up = LaunchConfiguration("camera_pitch_up")
    zed_launch = os.path.join(
        get_package_share_directory("zed_wrapper"),
        "launch",
        "zed_camera.launch.py"
    )

    zed2 = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(zed_launch),
        launch_arguments={
            "camera_name": "zed2",
            "camera_model": "zedx",
            "serial_number": "42203370",
            "publish_tf": "false",
            "publish_map_tf": "false",
            "pub_frame_rate": "15.0",
            "log_level": "info",
        }.items(),
    )

    # Static extrinsics from drone body frame to the connected camera.
    base_to_zed2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_zed2_tf",
        arguments=[
            "--x", "0.1524", "--y", "0.0", "--z", "-0.127",
            "--roll", "0.0",
            "--pitch", PythonExpression(["-", camera_pitch_up]),
            "--yaw", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "zed2_camera_link",
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_pitch_up",
            default_value="0.0",
            description="Physical upward camera tilt in radians; converted to negative ROS pitch.",
        ),
        zed2,
        base_to_zed2,
    ])
