#!/usr/bin/env python3
"""Ground plane marker from a downward rangefinder + IMU, for RViz2."""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point as PointMsg
from geometry_msgs.msg import Quaternion, Vector3
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def _quat_to_rotmat(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _shortest_arc_quat(v_from, v_to):
    a = v_from / np.linalg.norm(v_from)
    b = v_to / np.linalg.norm(v_to)
    c = float(np.dot(a, b))
    if c < -0.9999999:
        axis = np.cross(a, np.array([1.0, 0.0, 0.0]))
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(a, np.array([0.0, 1.0, 0.0]))
        axis = axis / np.linalg.norm(axis)
        return (float(axis[0]), float(axis[1]), float(axis[2]), 0.0)
    v = np.cross(a, b)
    s = math.sqrt((1.0 + c) * 2.0)
    invs = 1.0 / s
    return (float(v[0] * invs), float(v[1] * invs), float(v[2] * invs), float(s * 0.5))


class GroundPlane(Node):
    def __init__(self):
        super().__init__("ground_plane_detection")

        self.declare_parameter("rangefinder_topic", "/mavros/rangefinder/rangefinder")
        self.declare_parameter("imu_topic", "/mavros/imu/data")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("beam_direction", [0.0, 0.0, -1.0])
        self.declare_parameter("mount_offset", [0.0, 0.0, 0.0])
        self.declare_parameter("plane_size_m", 4.0)
        self.declare_parameter("publish_rate_hz", 20.0)

        self._frame_id = str(self.get_parameter("frame_id").value)
        beam_dir = np.array(self.get_parameter("beam_direction").value, dtype=np.float64)
        self._beam_dir = beam_dir / np.linalg.norm(beam_dir)
        self._mount_offset = np.array(self.get_parameter("mount_offset").value, dtype=np.float64)
        self._plane_size = float(self.get_parameter("plane_size_m").value)

        self._range = None
        self._quat = None

        self._pub = self.create_publisher(MarkerArray, "/ground_plane_detection/markers", 10)
        self.create_subscription(
            Range, str(self.get_parameter("rangefinder_topic").value), self._cb_range, qos_profile_sensor_data
        )
        self.create_subscription(
            Imu, str(self.get_parameter("imu_topic").value), self._cb_imu, qos_profile_sensor_data
        )

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / rate, self._publish)

    def _cb_range(self, msg):
        self._range = msg.range

    def _cb_imu(self, msg):
        q = msg.orientation
        self._quat = (q.x, q.y, q.z, q.w)

    def _publish(self):
        if self._range is None or self._quat is None:
            return

        rot = _quat_to_rotmat(*self._quat)
        normal_body = rot[2, :]

        plane_point = self._mount_offset + self._range * self._beam_dir
        qx, qy, qz, qw = _shortest_arc_quat(np.array([0.0, 0.0, 1.0]), normal_body)

        plane = Marker()
        plane.header.frame_id = self._frame_id
        plane.header.stamp = self.get_clock().now().to_msg()
        plane.ns = "ground_plane_detection"
        plane.id = 0
        plane.type = Marker.CUBE
        plane.action = Marker.ADD
        plane.pose.position = PointMsg(x=float(plane_point[0]), y=float(plane_point[1]), z=float(plane_point[2]))
        plane.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        plane.scale = Vector3(x=self._plane_size, y=self._plane_size, z=0.02)
        plane.color = ColorRGBA(r=0.2, g=0.8, b=0.2, a=0.6)

        array = MarkerArray()
        array.markers.append(plane)
        self._pub.publish(array)


def main():
    rclpy.init()
    node = GroundPlane()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()