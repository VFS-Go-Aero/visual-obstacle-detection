#!/usr/bin/env python3
"""Launch two ZED X cameras with a static TF between them."""

import os
import math
import numpy as np

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

transformations = {
    "odom_to_zed1": {
        "position:": np.array([0.0, 0.0, 0.0]),  # x y z in meters
        "orientation": np.array([0.0, 0.0, 0.0]),  # roll pitch yaw in degrees
    },
    "odom_to_zed2": {
        "position:": np.array([0.0, -0.31, 0.0]),  # x y z in meters
        "orientation": np.array([0.0, 0.0, 0.0]),  # roll pitch yaw in degrees
    },
}


def generate_launch_description():
    """Build a launch description for dual ZED X cameras with static TF."""
    zed_launch = os.path.join(
        get_package_share_directory("zed_wrapper"),
        "launch",
        "zed_camera.launch.py"
    )

    # -----------------------------------------------
    # Camera 1 — no odom/map TF
    # -----------------------------------------------
    cam1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch),
        launch_arguments={
            "camera_name": "zed1",
            "camera_model": "zedx",
            "serial_number": "44659546",
            "publish_tf": "false",
            "publish_map_tf": "false",
        }.items()
    )

    # -----------------------------------------------
    # Camera 2 — no odom/map TF
    # -----------------------------------------------
    cam2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch),
        launch_arguments={
            "camera_name": "zed2",
            "camera_model": "zedx",
            "serial_number": "42203370",
            "publish_tf": "false",
            "publish_map_tf": "false",
        }.items()
    )

    # zed1 and zed2 are both transformed around odom (drone tracker reference).
    odom_to_zed1_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="zed1_offset_tf",
        arguments=[
            *tuple(transformations["odom_to_zed1"]["position:"]),  # x y z (meters) from odom to zed1
            *tuple(
                transformations["odom_to_zed1"]["orientation"]
                * math.pi / 180
            ),  # roll pitch yaw (radians)
            "odom",
            "zed1_camera_link",
        ],
    )

    odom_to_zed2_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="zed2_offset_tf",
        arguments=[
            *tuple(transformations["odom_to_zed2"]["position:"]),  # x y z (meters) from odom to zed2
            *tuple(
                transformations["odom_to_zed2"]["orientation"]
                * math.pi / 180
            ),  # roll pitch yaw (radians)
            "odom",
            "zed2_camera_link",
        ],
    )

    return LaunchDescription([
        Node(
            package="launch_files",
            executable="drone_pose",
            name="drone_pose",
        ),
        cam1,
        cam2,
        odom_to_zed1_tf,
        odom_to_zed2_tf,
    ])
