#!/usr/bin/env python3
"""
Point Cloud Store — subscribes to the registered point cloud topics
published by two ZED X cameras (zed1, zed2) via the ZED ROS 2 wrapper,
converts each incoming PointCloud2 message into a NumPy array of (x, y, z)
points, and keeps the latest cloud for each camera in memory.

Run:  python3 point_cloud_store.py   (with cameras already launched)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
from sensor_msgs_py import point_cloud2  # ROS 2 helper to iterate PointCloud2 fields


class PointCloud(Node):
    # Names of the two ZED X cameras (must match the launch file camera_name args)
    CAMERAS = ["zed1", "zed2"]

    def __init__(self):
        super().__init__("point_cloud")

        # Dictionary holding the latest point cloud for each camera.
        # Each value is either None (no data yet) or an (N, 3) float32 NumPy array.
        self.clouds = {c: None for c in self.CAMERAS}

        # Subscribe to each camera's registered (world-frame) point cloud topic.
        for cam in self.CAMERAS:
            self.create_subscription(
                PointCloud2,
                f"/{cam}/zed_node/point_cloud/cloud_registered",
                # Default lambda capture trick: c=cam ensures each callback
                # closes over its own camera name rather than sharing the loop var.
                lambda msg, c=cam: self._cb(msg, c),
                10,  # QoS queue depth
            )

    def _cb(self, msg, cam):
        """Called every time a new point cloud is published for *cam*.

        Extracts only the (x, y, z) fields, drops NaN points (invalid depth),
        and overwrites the stored cloud so it always reflects the latest frame.
        """
        self.clouds[cam] = np.array(
            list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)),
            dtype=np.float32,
        )
        self.get_logger().info(f"[{cam}] {self.clouds[cam].shape[0]} pts")


def main():
    rclpy.init()
    rclpy.spin(PointCloud())  # Block and process callbacks until Ctrl-C
    rclpy.shutdown()


if __name__ == "__main__":
    main()
