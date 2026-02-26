# bash-commands

## Downwards Rangefinder

```bash
ros2 topic pub /mavros/rangefinder_sub sensor_msgs/Range "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'rangefinder'},
  radiation_type: 1,
  field_of_view: 0.1,
  min_range: 0.2,
  max_range: 50.0,
  range: 5.0
}"
```

## LaserScan

```bash
ros2 topic pub /mavros/obstacle/send sensor_msgs/msg/LaserScan "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'},
  angle_min: -3.14159,
  angle_max: 3.14159,
  angle_increment: 0.017453,
  time_increment: 0.0,
  scan_time: 0.1,
  range_min: 0.2,
  range_max: 50.0,
  ranges: [5.0, 5.0, 5.0, 5.0, 5.0]
}"
```


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
ros2 launch launch_files multi_zed_tf.launch.py
```

**Run Pointcloud Viz**
```bash
ros2 run rviz2 rviz2
```

then, do add > by topic > /zed1/point_cloud/cloud_registered/PointCloud2

**Build Package:**
```bash
colcon build --packages-select scan_to_mavlink
colcon build --packages-select launch_files
```

## Mavros

**Start Mavlink Connection:** 
```bash
ros2 launch mavros apm.launch fcu_url:=///dev/ttyUSB0:57600 

ros2 launch mavros apm.launch.py fcu_url:=/dev/ttyUSB0:57600 gcs_url:=udp://@XX.XXX.XXX.X:14550
ros2 launch mavros apm.launch fcu_url:=/dev/ttyUSB0:57600 gcs_url:=udp://@XX.XXX.XXX.X:14550

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
