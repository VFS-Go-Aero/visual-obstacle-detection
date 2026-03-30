#!/usr/bin/env python3
"""
Point Cloud — merged point cloud from two ZED X cameras.

Subscribes to the registered point cloud topics published by two
ZED X cameras (zed1, zed2) via the ZED ROS 2 wrapper, converts each
incoming PointCloud2 message into a NumPy array of (x, y, z) points,
and maintains a single merged cloud from both cameras.

Run:  python3 point_cloud.py  (with cameras already launched)
"""

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import std_msgs.msg as std_msg
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class PointCloud(Node):
    """
    Subscribe to two ZED X point clouds and maintain a merged cloud.

    Converts each incoming PointCloud2 message into a NumPy array of
    (x, y, z) points and concatenates the two camera clouds into a
    single array available as ``self.cloud``.
    """

    def __init__(self) -> None:
        super().__init__("point_cloud")

        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("tf_timeout_s", 0.05)

        self._target_frame = str(self.get_parameter("target_frame").value)
        self._tf_timeout = Duration(seconds=float(self.get_parameter("tf_timeout_s").value))
        self._frame_id = self._target_frame

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

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

        self._merged_pub = self.create_publisher(PointCloud2, "/merged_cloud", 10)

    def _parse(self, msg: PointCloud2) -> np.ndarray:
        """
        Extract (x, y, z) points from a PointCloud2, dropping NaNs.

        Parameters
        ----------
        msg : PointCloud2
            Incoming point cloud message.

        Returns
        -------
        np.ndarray
            Float32 array of shape (N, 3).

        """
        structured = np.array(
            list(point_cloud2.read_points(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )),
        )
        if structured.size == 0:
            return np.empty((0, 3), dtype=np.float32)
        return np.column_stack(
            [structured["x"], structured["y"], structured["z"]]
        ).astype(np.float32)

    def _merge(self) -> None:
        """Concatenate the two camera clouds and publish on /merged_cloud."""
        self.cloud = np.concatenate((self._cloud1, self._cloud2), axis=0)
        self.get_logger().info(f"merged cloud: {self.cloud.shape[0]} pts")
        header = std_msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._frame_id
        msg = point_cloud2.create_cloud_xyz32(header, self.cloud.tolist())
        self._merged_pub.publish(msg)

    def _transform_cloud(self, msg: PointCloud2) -> np.ndarray:
        """Transform a PointCloud2 into the configured target frame and parse xyz."""
        if msg.header.frame_id == self._target_frame:
            return self._parse(msg)

        try:
            tf_msg = self._tf_buffer.lookup_transform(
                self._target_frame,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=self._tf_timeout,
            )
            transformed = do_transform_cloud(msg, tf_msg)
        except TransformException as exc:
            self.get_logger().warning(
                f"TF unavailable {msg.header.frame_id}->{self._target_frame}: {exc}"
            )
            return np.empty((0, 3), dtype=np.float32)

        return self._parse(transformed)

    def _cb_zed1(self, msg: PointCloud2) -> None:
        self._cloud1 = self._transform_cloud(msg)
        self._merge()

    def _cb_zed2(self, msg: PointCloud2) -> None:
        self._cloud2 = self._transform_cloud(msg)
        self._merge()


def main() -> None:
    """Spin the PointCloud node until shutdown."""
    rclpy.init()
    rclpy.spin(PointCloud())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
