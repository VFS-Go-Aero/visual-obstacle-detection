#!/usr/bin/env python3
"""Launch multi-camera ZED setup using the zed_multi_camera package."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Build a launch description for multi-camera ZED setup."""
    # Path to the multi-camera launch file
    multi_camera_launch_file = os.path.join(
        get_package_share_directory("zed_multi_camera"),
        "launch",
        "zed_multi_camera.launch.py"
    )

    # Include the multi-camera launch with your parameters
    zed_multi_camera = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            multi_camera_launch_file
        ),
        launch_arguments={
            "cam_names": "[zed1,zed2]",
            "cam_models": "[zedx, zedx]",  # Adjust if different
            "cam_serials": "[44659546,42203370]",
            "cam_resolutions": "[HD720,HD720]",  # Lower res
            "cam_framerates": "[15,15]",  # Lower framerate
        }.items(),
    )

    # Static extrinsics from drone body frame to each camera optical source frame.
    # Update these values after camera calibration.
    base_to_zed1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_zed1_tf",
        arguments=[
            "0.0", "0.0", "0.0",
            "0.0", "0.0", "0.0",
            "base_link",
            "zed1_left_camera_frame",
        ],
    )

    base_to_zed2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_zed2_tf",
        arguments=[
            "0.0", "-0.31", "0.0",
            "0.0", "0.0", "0.0",
            "base_link",
            "zed2_left_camera_frame",
        ],
    )

    return LaunchDescription([zed_multi_camera, base_to_zed1, base_to_zed2])
