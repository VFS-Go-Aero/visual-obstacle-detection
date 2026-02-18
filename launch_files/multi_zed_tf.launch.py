#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    zed_launch = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'launch',
        'zed_camera.launch.py'
    )

    # -----------------------------------------------
    # Camera 1 — master, publishes odom and map TF
    # -----------------------------------------------
    cam1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch),
        launch_arguments={
            'camera_name':   'zed1',
            'camera_model':  'zedx',
            'serial_number': '44659546',
            'publish_tf':    'true',
            'publish_map_tf':'true',
        }.items()
    )

    # -----------------------------------------------
    # Camera 2 — secondary, no odom/map TF
    # -----------------------------------------------
    cam2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch),
        launch_arguments={
            'camera_name':   'zed2',
            'camera_model':  'zedx',
            'serial_number': '42203370',
            'publish_tf':    'false',   # zed1 owns odom
            'publish_map_tf':'false',
        }.items()
    )

    # -----------------------------------------------
    # YOUR TF — edit these 6 numbers any time
    # x y z roll pitch yaw (meters, radians)
    # -----------------------------------------------
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed2_offset_tf',
        arguments=[
            '0.0', '0.0', '0.05',       # 5cm above zed1
            '0.0', '0.0', '3.1416',     # 180deg yaw
            'zed1_camera_link',   # parent (root of zed1 TF tree)
            'zed2_camera_link'    # child (root of zed2 TF tree)
        ]
    )

    return LaunchDescription([cam1, cam2, static_tf]) 