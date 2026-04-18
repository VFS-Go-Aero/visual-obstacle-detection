ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200 gcs_url:=udp://@10.186.18.209:14550
ros2 launch --packages-select launch_files multi_zed.launch.py
ros2 launch --packages-select visual_obstacle_detection single_zed.launch.py