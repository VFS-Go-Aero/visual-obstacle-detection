#!/usr/bin/env python3
"""Launch a single ZED camera (zed1) with explicit static transform from base_link."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Build a launch description for single ZED setup with explicit TF ownership."""
    zed_launch = os.path.join(
        get_package_share_directory("zed_wrapper"),
        "launch",
        "zed_camera.launch.py"
    )

    zed1 = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(zed_launch),
        launch_arguments={
            "camera_name": "zed1",
            "camera_model": "zedx",
            "serial_number": "44659546",
            "publish_tf": "false",
            "publish_map_tf": "false",
            "pub_frame_rate": "15.0",
            "log_level": "info",
        }.items(),
    )

    # Static extrinsics from drone body frame to camera root frame.
    # Update these values after camera calibration.
    base_to_zed1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_zed1_tf",
        arguments=[
            "0.1524", "0.0", "-0.127",
            "0.0", "0.0", "0.0",
            "base_link",
            "zed1_camera_link",
            "--ros-args",
            "--log-level", "info",
        ],
    )

    return LaunchDescription([zed1, base_to_zed1])
