# bash-commands

## Stereocam

**Inspect Camera Outputs**
```bash
ZED_Explorer
```

**Launch Single Camera** 
```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx
```

**Launch Dual Cameras**
```bash
ros2 launch launch_files multi_zed.launch.py
```

**Run Pointcloud Viz**
```bash
ros2 run rviz2 rviz2
```

then, do add > nodes > zed > poincloud2 to view each.

**Build Package:**
```bash
colcon build --packages-select scan_to_mavlink
```

## Mavros

**Start Mavlink Connection:** 
```bash
ros2 launch mavros apm.launch fcu_url:=///dev/ttyUSB0:57600 

ros2 launch mavros apm.launch.py fcu_url:=/dev/ttyUSB0:57600 gcs_url:=udp://@XX.XXX.XXX.X:14550
```
ros2 node list

ros2 run scan_to_mavlink scan_to_mavlink_node

ros2 topic list 

Set mode - ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'STABILIZE'}"

Arm - ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}" 

Temporary sensor publish - 

ros2 topic pub /mavros/obstacle/scan sensor_msgs/msg/LaserScan "{
  header: { frame_id: 'laser' },
  angle_min: 0.0,
  angle_max: 0.0,
  angle_increment: 0.0,
  time_increment: 0.0,
  scan_time: 0.0,
  range_min: 0.1,
  range_max: 10.0,
  ranges: [10.0],
  intensities: []
}"

Recieve sensor data - ros2 topic echo /mavros/obstacle/scan

-----------------------------------------

Mavros, Mavros-extras and Mavros-msgs

sudo apt install ros-humble-mavros-extras ros-humble-mavros-msgs
source /opt/ros/humble/setup.bash

ros2 topic echo /mavros/obstacle/send mavros_msgs/msg/ObstacleDistance3D

-----------------------------------------
