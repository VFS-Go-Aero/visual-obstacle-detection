#!/usr/bin/env python3
"""
Point Cloud — subscribes to the registered point cloud topics
published by two ZED X cameras (zed1, zed2) via the ZED ROS 2
wrapper, converts each incoming PointCloud2 message into a NumPy
array of (x, y, z) points, and maintains a single merged cloud
from both cameras.

Run:  python3 point_cloud.py  (with cameras already launched)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class PointCloud(Node):

    def __init__(self):
        super().__init__("point_cloud")

        # Latest (N, 3) float32 array from each camera, starts empty.
        self._cloud1 = np.empty((0, 3), dtype=np.float32)
        self._cloud2 = np.empty((0, 3), dtype=np.float32)

        # Merged cloud from both cameras — always up to date.
        self.cloud = np.empty((0, 3), dtype=np.float32)

        # Explicit subscriptions — no loop, no dictionary.
        self._sub_zed1 = self.create_subscription(
            PointCloud2,
            "/zed1/zed_node/point_cloud/cloud_registered",
            self._cb_zed1,
            10,
        )
        self._sub_zed2 = self.create_subscription(
            PointCloud2,
            "/zed2/zed_node/point_cloud/cloud_registered",
            self._cb_zed2,
            10,
        )

    def _parse(self, msg):
        """Extract (x, y, z) points from a PointCloud2, dropping NaNs."""
        return np.array(
            list(point_cloud2.read_points(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )),
            dtype=np.float32,
        ).reshape(-1, 3)

    def _merge(self):
        """Concatenate the two camera clouds into self.cloud."""
        self.cloud = np.concatenate((self._cloud1, self._cloud2), axis=0)
        self.get_logger().info(f"merged cloud: {self.cloud.shape[0]} pts")

    def _cb_zed1(self, msg):
        self._cloud1 = self._parse(msg)
        self._merge()

    def _cb_zed2(self, msg):
        self._cloud2 = self._parse(msg)
        self._merge()


def main():
    """Spin the PointCloud node until shutdown."""
    rclpy.init()
    rclpy.spin(PointCloud())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
