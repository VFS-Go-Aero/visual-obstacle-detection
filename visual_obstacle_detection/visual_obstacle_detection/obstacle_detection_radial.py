#!/usr/bin/env python3

import numpy as np
import struct

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class ObstacleDetection(Node):

    def __init__(self, **kwargs) -> None:
        super().__init__("obstacle_detection")

        self.hfov = kwargs.get("hfov", 105.0)
        self.vfov = kwargs.get("vfov", 78.0)

        self.h_sections = kwargs.get("h_sections", 10)
        self.v_sections = kwargs.get("v_sections", 10)
        self._h_arc = self.hfov / self.h_sections
        self._v_arc = self.vfov / self.v_sections

        self._obstacles = np.empty((self.h_sections, self.v_sections), dtype=np.float32)

        self.max_distance = kwargs.get("max_distance", 20.0)
        self.ideal_distance = kwargs.get("ideal_distance", 12.0)

        # Publisher for colored obstacle cloud
        self.obstacle_data = self.create_publisher(
            PointCloud2,
            "/merged_cloud/obstacles",
            10,
        )

        self._point_cloud = self.create_subscription(
            PointCloud2,
            "/merged_cloud",
            self._detect_and_publish,
            10,
        )

    def _parse(self, msg: PointCloud2) -> np.ndarray:
        pts = np.array(
            list(point_cloud2.read_points(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True,
            ))
        )

        return pts.astype(np.float32) if pts.size > 0 else np.empty((0, 3), dtype=np.float32)

    def _extract_radial(self, pts: np.ndarray) -> None:
        # Placeholder for radial extraction logic
        n_pts = pts.size

        distances = np.linalg.norm(pts[:, :2], axis=1)
        angles = np.arctan2(pts[:, 1], pts[:, 0])

        return distances, angles
    
    def _assign_by_sector(self, msg: PointCloud2) -> None:
        distances, angles = self._extract_radial(self._parse(msg))
        # Placeholder for sector assignment logic

    def _detect_and_publish(self, msg: PointCloud2) -> None:
        # Placeholder for obstacle detection and publishing logic
        pass

    """
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
        self.obstacle_data.publish(msg)
    """


def main() -> None:
    rclpy.init()
    rclpy.spin(PointCloud())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
