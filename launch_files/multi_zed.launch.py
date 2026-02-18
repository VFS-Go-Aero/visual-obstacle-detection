import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Path to the multi-camera launch file
    multi_camera_launch_file = os.path.join(
        get_package_share_directory('zed_multi_camera'),
        'launch',
        'zed_multi_camera.launch.py'
    )

    # Include the multi-camera launch with your parameters
    zed_multi_camera = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(multi_camera_launch_file),
        launch_arguments={
            'cam_names': '[zed1,zed2]',
            'cam_models': '[zedx, zedx]',  # Adjust model names if different
            'cam_serials': '[44659546,42203370]',
            'cam_resolutions': '[HD720,HD720]',  # Lower resolution for bandwidth
            'cam_framerates': '[15,15]'          # Lower framerate helps too
            }.items()
    )

    return LaunchDescription([zed_multi_camera])