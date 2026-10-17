#!/usr/bin/env python3
"""
Rotate a point cloud against the drone's tilt, pivoting about the ground.

Only roll/pitch are used; yaw is untouched. All point fields (e.g.

`pitch_gain` scales the pitch correction (roll is always levelled normally).

To change while code runs: ros2 param set /pc_align pitch_gain 1.0
- this going to set it to default,
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

    a = v_from / np.linalg.norm(v_from)
    b = v_to / np.linalg.norm(v_to)
    c = float(np.dot(a, b))
    if c < -0.9999999:

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


def _scale_pitch(normal_body, gain):
    """
    Return the ground normal with its pitch multiplied by `gain` (roll kept).

    normal_body = (-sin p, cos p sin r, cos p cos r) for pitch p, roll r.
    """
    n = normal_body / np.linalg.norm(normal_body)
    pitch = math.asin(max(-1.0, min(1.0, -n[0])))
    roll = math.atan2(n[1], n[2])
    limit = math.radians(89.0)
    pitch = max(-limit, min(limit, gain * pitch))
    return np.array([
        -math.sin(pitch),
        math.cos(pitch) * math.sin(roll),
        math.cos(pitch) * math.cos(roll),
    ])


class PcAlign(Node):
    def __init__(self):
        super().__init__("pc_align")

        self.declare_parameter("input_topic", "/merged_cloud")
        self.declare_parameter("output_topic", "/merged_cloud/aligned")
        self.declare_parameter("imu_topic", "/mavros/imu/data")
        self.declare_parameter("rangefinder_topic", "/mavros/rangefinder/rangefinder")
        self.declare_parameter("beam_direction", [0.0, 0.0, -1.0])
        self.declare_parameter("mount_offset", [0.0, 0.0, 0.0])
        # Pitch correction gain: 1.0 = level, 2.0 = pitch down with the camera tilt.
        self.declare_parameter("pitch_gain", 2.0)

        beam_dir = np.array(self.get_parameter("beam_direction").value, dtype=np.float64)
        self._beam_dir = beam_dir / np.linalg.norm(beam_dir)
        self._mount_offset = np.array(self.get_parameter("mount_offset").value, dtype=np.float64)
        self._pitch_gain = float(self.get_parameter("pitch_gain").value)

        self._quat = None
        self._range = None
        self._warned_no_imu = False
        self._warned_no_range = False
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
            f"{self.get_parameter('output_topic').value}, "
            f"pitch_gain={self._pitch_gain}]"
        )

    def _cb_imu(self, msg):
        q = msg.orientation
        self._quat = (q.x, q.y, q.z, q.w)

    def _cb_range(self, msg):
        # inf/NaN (out of range) would turn the whole cloud into NaN.
        self._range = msg.range if math.isfinite(msg.range) else None

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
        normal_body = rot_body_to_world[2, :].copy()

        # Scale only the pitch part of the ground normal. Read live so
        # `ros2 param set /pc_align pitch_gain <x>` applies to the next cloud.
        gain = float(self.get_parameter("pitch_gain").value)
        if gain != self._pitch_gain:
            self._pitch_gain = gain
            self.get_logger().info(f"pitch_gain -> {gain}")
        normal_body = _scale_pitch(normal_body, gain)

        # Rotation that swings the (pitch-scaled) ground normal back to +Z.
        rot = _rotation_from_to(normal_body, np.array([0.0, 0.0, 1.0]))

        # Pivot: the ground point under the rangefinder, so the cloud's height
        # above the ground does not change when it is rotated.
        if self._range is not None:
            pivot = self._mount_offset + self._range * self._beam_dir
        else:
            pivot = np.zeros(3)
            if not self._warned_no_range:
                self.get_logger().warning(
                    "No valid rangefinder reading yet; rotating about the base_link origin"
                )
                self._warned_no_range = True

        n = msg.width * msg.height
        data = np.frombuffer(bytes(msg.data), dtype=dtype, count=n).copy()

        xyz = np.stack([data["x"], data["y"], data["z"]], axis=1).astype(np.float64)
        xyz = (xyz - pivot) @ rot.T + pivot

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
