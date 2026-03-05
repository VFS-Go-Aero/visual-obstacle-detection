#!/usr/bin/env python3

import numpy as np
import struct

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, ObstacleDistance3D
from std_msgs.msg import Header

class ObstacleToMavros(Node):

    def __init__(self) -> None:
        super().__init__("obstacle_to_mavros")

        self._obstacles = self.create_subscription(
            PointCloud2,
            "/merged_cloud/obstacles",
            self._obstacle_callback,
            10
        )

        self.mavros_pub = self.create_publisher(
            ObstacleDistance3D,
            "/mavros/obstacle_distance_3d/send",
            10
        )
    
    def _obstacle_callback(self, msg: PointCloud2) -> None:
        # Convert PointCloud2 to list of (x, y, z) tuples
        points = []
        for i in range(msg.width * msg.height):
            offset = i * msg.point_step
            x = struct.unpack_from('f', msg.data, offset + msg.fields[0].offset)[0]
            y = struct.unpack_from('f', msg.data, offset + msg.fields[1].offset)[0]
            z = struct.unpack_from('f', msg.data, offset + msg.fields[2].offset)[0]
            points.append((x, y, z))

        # Publish each point as an ObstacleDistance3D message
        for idx, (x, y, z) in enumerate(points):
            obstacle_msg = ObstacleDistance3D()
            obstacle_msg.header = Header()
            obstacle_msg.header.stamp = self.get_clock().now().to_msg()
            obstacle_msg.header.frame_id = "base_link"
            obstacle_msg.sensor_type = 1
            obstacle_msg.frame = 0
            obstacle_msg.obstacle_id = idx
            obstacle_msg.position.x = x
            obstacle_msg.position.y = y
            obstacle_msg.position.z = z
            obstacle_msg.min_distance = 0.2
            obstacle_msg.max_distance = 60.0

            self.mavros_pub.publish(obstacle_msg)

def main() -> None:
    rclpy.init()
    rclpy.spin(ObstacleToMavros())
    rclpy.shutdown()
