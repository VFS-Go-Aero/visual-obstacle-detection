#!/usr/bin/env python3
"""Level a point cloud with respect to the ground plane.

Uses the same IMU-derived ground normal as ground_plane_detection.py. The
cloud (in the drone's body frame, e.g. base_link) is rotated so that the
ground plane becomes horizontal:

  * drone pitched nose-down  -> the ground (tilted up in the body frame) is
    rotated back down to flat, and the cloud moves with it.
  * only roll/pitch is removed; heading (yaw) is left untouched, so the cloud
    stays aligned with the drone's forward axis.

Optionally the cloud is also shifted vertically so the ground sits at z = 0,
using the downward rangefinder.

All point fields (e.g. `obstacle_id` on /merged_cloud/obstacles) are preserved;
only x, y, z are rewritten. NaNs and the cloud's width/height are kept as-is.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField, Range

_DTYPES = {
    PointField.INT8: "i1",
    PointField.UINT8: "u1",
    PointField.INT16: "i2",
    PointField.UINT16: "u2",
    PointField.INT32: "i4",
    PointField.UINT32: "u4",
    PointField.FLOAT32: "f4",
    PointField.FLOAT64: "f8",
}


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


def _rotation_from_to(v_from, v_to):
    """Shortest-arc rotation matrix taking unit vector v_from onto v_to."""
    a = v_from / np.linalg.norm(v_from)
    b = v_to / np.linalg.norm(v_to)
    c = float(np.dot(a, b))
    if c < -0.9999999:
        # Antiparallel (drone upside down): rotate 180 deg about any axis normal to a.
        axis = np.cross(a, np.array([1.0, 0.0, 0.0]))
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(a, np.array([0.0, 1.0, 0.0]))
        axis = axis / np.linalg.norm(axis)
        return 2.0 * np.outer(axis, axis) - np.eye(3)
    v = np.cross(a, b)
    vx = np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ])
    return np.eye(3) + vx + (vx @ vx) / (1.0 + c)


class PcAlign(Node):
    def __init__(self):
        super().__init__("pc_align")

        self.declare_parameter("input_topic", "/merged_cloud")
        self.declare_parameter("output_topic", "/merged_cloud/aligned")
        self.declare_parameter("imu_topic", "/mavros/imu/data")
        self.declare_parameter("rangefinder_topic", "/mavros/rangefinder/rangefinder")
        # Shift the leveled cloud vertically so the ground is at z = 0.
        self.declare_parameter("ground_at_origin", True)
        self.declare_parameter("beam_direction", [0.0, 0.0, -1.0])
        self.declare_parameter("mount_offset", [0.0, 0.0, 0.0])

        self._ground_at_origin = bool(self.get_parameter("ground_at_origin").value)
        beam_dir = np.array(self.get_parameter("beam_direction").value, dtype=np.float64)
        self._beam_dir = beam_dir / np.linalg.norm(beam_dir)
        self._mount_offset = np.array(self.get_parameter("mount_offset").value, dtype=np.float64)

        self._quat = None
        self._range = None
        self._warned_no_imu = False
        self._warned_fields = False

        self._pub = self.create_publisher(
            PointCloud2, str(self.get_parameter("output_topic").value), 1
        )
        self.create_subscription(
            Imu, str(self.get_parameter("imu_topic").value), self._cb_imu, qos_profile_sensor_data
        )
        self.create_subscription(
            Range, str(self.get_parameter("rangefinder_topic").value), self._cb_range,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2, str(self.get_parameter("input_topic").value), self._cb_cloud, 1
        )

        self.get_logger().info(
            f"pc_align started [{self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('output_topic').value}, ground_at_origin={self._ground_at_origin}]"
        )

    def _cb_imu(self, msg):
        q = msg.orientation
        self._quat = (q.x, q.y, q.z, q.w)

    def _cb_range(self, msg):
        self._range = msg.range

    @staticmethod
    def _cloud_dtype(msg):
        names, formats, offsets = [], [], []
        endian = ">" if msg.is_bigendian else "<"
        for f in msg.fields:
            if f.datatype not in _DTYPES or f.count != 1:
                continue
            names.append(f.name)
            formats.append(endian + _DTYPES[f.datatype])
            offsets.append(f.offset)
        return np.dtype({
            "names": names,
            "formats": formats,
            "offsets": offsets,
            "itemsize": msg.point_step,
        })

    def _cb_cloud(self, msg: PointCloud2):
        if self._quat is None:
            if not self._warned_no_imu:
                self.get_logger().warning("Waiting for IMU before aligning cloud")
                self._warned_no_imu = True
            return

        if msg.width * msg.height == 0:
            self._pub.publish(msg)
            return

        dtype = self._cloud_dtype(msg)
        if not all(n in dtype.names for n in ("x", "y", "z")):
            if not self._warned_fields:
                self.get_logger().error("Input cloud has no scalar x/y/z fields")
                self._warned_fields = True
            return

        # Ground normal in the body frame = world "up" expressed in body coordinates.
        rot_body_to_world = _quat_to_rotmat(*self._quat)
        normal_body = rot_body_to_world[2, :]

        # Tilt-only rotation: ground normal -> +Z. Yaw is left alone.
        r_level = _rotation_from_to(normal_body, np.array([0.0, 0.0, 1.0]))

        shift_z = 0.0
        if self._ground_at_origin and self._range is not None:
            ground_pt = self._mount_offset + self._range * self._beam_dir
            shift_z = -float((r_level @ ground_pt)[2])

        n = msg.width * msg.height
        data = np.frombuffer(bytes(msg.data), dtype=dtype, count=n).copy()

        xyz = np.stack([data["x"], data["y"], data["z"]], axis=1).astype(np.float64)
        xyz = xyz @ r_level.T
        xyz[:, 2] += shift_z

        data["x"] = xyz[:, 0]
        data["y"] = xyz[:, 1]
        data["z"] = xyz[:, 2]

        out = PointCloud2()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.is_dense = msg.is_dense
        out.data = data.tobytes()
        self._pub.publish(out)


def main():
    rclpy.init()
    node = PcAlign()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()