#!/usr/bin/env python3

import numpy as np
import struct

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class PointCloud(Node):

    def __init__(self) -> None:
        super().__init__("obstacle_detection")

        self._cloud1 = np.empty((0, 3), dtype=np.float32)
        self._cloud2 = np.empty((0, 3), dtype=np.float32)
        self.cloud = np.empty((0, 3), dtype=np.float32)

        # Publisher for colored obstacle cloud
        self.pub = self.create_publisher(
            PointCloud2,
            "/merged_cloud/obstacles",
            10,
        )

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

    def _parse(self, msg: PointCloud2) -> np.ndarray:
        structured = np.array(
            list(point_cloud2.read_points(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True,
            ))
        )

        if structured.size == 0:
            return np.empty((0, 3), dtype=np.float32)

        return np.column_stack(
            [structured["x"], structured["y"], structured["z"]]
        ).astype(np.float32)

    def _merge(self) -> None:
        self.cloud = np.concatenate((self._cloud1, self._cloud2), axis=0)
        self._detect_and_publish()

    def _detect_and_publish(self) -> None:

        if self.cloud.shape[0] == 0:
            return

        pts = self.cloud

        min_dist = 0.5
        max_dist = 5.0
        min_height = 0.1

        distances = np.linalg.norm(pts[:, :2], axis=1)

        obstacle_mask = (
            (distances > min_dist) &
            (distances < max_dist) &
            (pts[:, 2] > min_height)
        )

        colored_points = []

        for i, p in enumerate(pts):

            x, y, z = p

            if obstacle_mask[i]:
                r, g, b = 255, 0, 0
            else:
                r, g, b = 255, 255, 255

            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]
            colored_points.append([x, y, z, rgb])

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12,
                       datatype=PointField.UINT32, count=1),
        ]

        msg = point_cloud2.create_cloud(header, fields, colored_points)
        self.pub.publish(msg)

    def _cb_zed1(self, msg: PointCloud2) -> None:
        self._cloud1 = self._parse(msg)
        self._merge()

    def _cb_zed2(self, msg: PointCloud2) -> None:
        self._cloud2 = self._parse(msg)
        self._merge()


def main() -> None:
    rclpy.init()
    rclpy.spin(PointCloud())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
