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
import xml.etree.ElementTree as ET
from pathlib import Path
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
        self.declare_parameter("cloud_topic_zed1", "/zed1/zed_node/point_cloud/cloud_registered")
        self.declare_parameter("cloud_topic_zed2", "/zed2/zed_node/point_cloud/cloud_registered")
        self.declare_parameter(
            "exclude_boxes_xml",
            str(Path(__file__).with_name("excluded_boxes.xml")),
        )

        self._target_frame = str(self.get_parameter("target_frame").value)
        self._tf_timeout = Duration(seconds=float(self.get_parameter("tf_timeout_s").value))
        self._topic_zed1 = str(self.get_parameter("cloud_topic_zed1").value)
        self._topic_zed2 = str(self.get_parameter("cloud_topic_zed2").value)
        self._exclude_boxes_xml = str(self.get_parameter("exclude_boxes_xml").value)
        self._frame_id = self._target_frame

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._msgs_zed1 = 0
        self._msgs_zed2 = 0
        self._tf_fail_zed1 = 0
        self._tf_fail_zed2 = 0
        self._status_timer = self.create_timer(2.0, self._status_tick)

        # Latest (N, 3) float32 array from each camera, starts empty.
        self._cloud1 = np.empty((0, 3), dtype=np.float32)
        self._cloud2 = np.empty((0, 3), dtype=np.float32)

        # Merged cloud from both cameras — always up to date.
        self.cloud = np.empty((0, 3), dtype=np.float32)
        self._exclude_boxes = self._load_exclude_boxes(self._exclude_boxes_xml)

        # Explicit subscriptions — no loop, no dictionary.
        self._sub_zed1 = self.create_subscription(
            PointCloud2,
            self._topic_zed1,
            self._cb_zed1,
            10,
        )
        self._sub_zed2 = self.create_subscription(
            PointCloud2,
            self._topic_zed2,
            self._cb_zed2,
            10,
        )

        self._merged_pub = self.create_publisher(PointCloud2, "/merged_cloud", 10)

        tf_timeout_s = self._tf_timeout.nanoseconds / 1e9
        self.get_logger().info(
            "point_cloud started "
            f"[target_frame={self._target_frame}, "
            f"tf_timeout={tf_timeout_s:.3f}s, "
            f"topic_zed1={self._topic_zed1}, "
            f"topic_zed2={self._topic_zed2}, "
            f"exclude_boxes={len(self._exclude_boxes)}]"
        )

    def _load_exclude_boxes(self, xml_path: str) -> list[tuple[np.ndarray, np.ndarray]]:
        """
        Load axis-aligned exclusion boxes from XML in target_frame coordinates.

        XML format:
          <exclusion_boxes>
            <box>
              <min x="-0.5" y="-0.5" z="-0.2"/>
              <max x="0.5" y="0.5" z="0.3"/>
            </box>
          </exclusion_boxes>
        """
        path = Path(xml_path)
        if not path.exists():
            self.get_logger().warning(
                f"exclude_boxes_xml not found: {xml_path}; no exclusion filtering"
            )
            return []

        try:
            root = ET.parse(path).getroot()
        except ET.ParseError as exc:
            self.get_logger().error(
                f"Failed to parse exclusion XML {xml_path}: {exc}; no exclusion filtering"
            )
            return []

        boxes: list[tuple[np.ndarray, np.ndarray]] = []
        for idx, box in enumerate(root.findall("box"), start=1):
            min_node = box.find("min")
            max_node = box.find("max")
            if min_node is None or max_node is None:
                self.get_logger().warning(
                    f"Skipping box #{idx} in {xml_path}: missing <min> or <max>"
                )
                continue

            try:
                min_corner = np.array(
                    [
                        float(min_node.attrib["x"]),
                        float(min_node.attrib["y"]),
                        float(min_node.attrib["z"]),
                    ],
                    dtype=np.float32,
                )
                max_corner = np.array(
                    [
                        float(max_node.attrib["x"]),
                        float(max_node.attrib["y"]),
                        float(max_node.attrib["z"]),
                    ],
                    dtype=np.float32,
                )
            except (KeyError, ValueError) as exc:
                self.get_logger().warning(
                    f"Skipping box #{idx} in {xml_path}: invalid min/max values ({exc})"
                )
                continue

            lo = np.minimum(min_corner, max_corner)
            hi = np.maximum(min_corner, max_corner)
            boxes.append((lo, hi))

        self.get_logger().info(
            f"Loaded {len(boxes)} exclusion boxes from {xml_path} in frame {self._target_frame}"
        )
        return boxes

    def _apply_exclude_boxes(self, points: np.ndarray) -> np.ndarray:
        """Remove points that fall inside any configured exclusion box."""
        if points.size == 0 or not self._exclude_boxes:
            return points

        keep_mask = np.ones(points.shape[0], dtype=bool)
        for lo, hi in self._exclude_boxes:
            inside = np.all((points >= lo) & (points <= hi), axis=1)
            keep_mask &= ~inside
        return points[keep_mask]

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
        merged = np.concatenate((self._cloud1, self._cloud2), axis=0)
        self.cloud = self._apply_exclude_boxes(merged)
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
                Time(),
                timeout=self._tf_timeout,
            )
            transformed = do_transform_cloud(msg, tf_msg)
        except TransformException as exc:
            if "zed1" in msg.header.frame_id:
                self._tf_fail_zed1 += 1
            elif "zed2" in msg.header.frame_id:
                self._tf_fail_zed2 += 1
            self.get_logger().warning(
                f"TF unavailable {msg.header.frame_id}->{self._target_frame}: {exc}"
            )
            return np.empty((0, 3), dtype=np.float32)

        return self._parse(transformed)

    def _cb_zed1(self, msg: PointCloud2) -> None:
        self._msgs_zed1 += 1
        self._cloud1 = self._transform_cloud(msg)
        self._merge()

    def _cb_zed2(self, msg: PointCloud2) -> None:
        self._msgs_zed2 += 1
        self._cloud2 = self._transform_cloud(msg)
        self._merge()

    def _status_tick(self) -> None:
        """Periodic diagnostics to show whether inputs and TF are healthy."""
        if self._msgs_zed1 == 0 and self._msgs_zed2 == 0:
            self.get_logger().warning(
                f"Waiting for camera clouds on {self._topic_zed1} and {self._topic_zed2}"
            )
            return

        self.get_logger().info(
            f"status: zed1_msgs={self._msgs_zed1}, zed2_msgs={self._msgs_zed2}, "
            f"tf_fail_zed1={self._tf_fail_zed1}, tf_fail_zed2={self._tf_fail_zed2}, "
            f"merged_pts={self.cloud.shape[0]}"
        )


def main() -> None:
    """Spin the PointCloud node until shutdown."""
    rclpy.init()
    rclpy.spin(PointCloud())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
