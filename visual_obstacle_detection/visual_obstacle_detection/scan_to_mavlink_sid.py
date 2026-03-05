"""
ScanToMavlinkSid.

Subscribes to the merged point cloud, extracts XYZ obstacle coordinates,
and publishes each obstacle as an ObstacleDistance3D message to the
``/mavros/obstacle_distance_3d/send`` topic.

Obstacle criteria (configurable via ROS parameters):
  - XY distance between ``min_distance`` and ``max_distance``
  - Z (height) above ``min_height``

Up to ``max_obstacles`` closest obstacles are published per cloud frame.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from mavros_msgs.msg import ObstacleDistance3D
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


# MAV_DISTANCE_SENSOR_LASER — stereo depth camera is closest to this type.
_SENSOR_TYPE_LASER = 0

# MAV_FRAME_BODY_FRD — points are expressed relative to the vehicle body.
_MAV_FRAME_BODY_FRD = 8


class ScanToMavlinkSid(Node):
    """Extract obstacle XYZ from a merged point cloud and forward to MAVROS."""

    def __init__(self) -> None:
        super().__init__("scan_to_mavlink_sid")

        self.declare_parameter("min_distance", 0.2)
        self.declare_parameter("max_distance", 60.0)
        self.declare_parameter("min_height", 0.1)
        self.declare_parameter("max_obstacles", 5)

        self._min_distance: float = (
            self.get_parameter("min_distance").get_parameter_value().double_value
        )
        self._max_distance: float = (
            self.get_parameter("max_distance").get_parameter_value().double_value
        )
        self._min_height: float = (
            self.get_parameter("min_height").get_parameter_value().double_value
        )
        self._max_obstacles: int = (
            self.get_parameter("max_obstacles").get_parameter_value().integer_value
        )

        self._pub = self.create_publisher(
            ObstacleDistance3D,
            "/mavros/obstacle_distance_3d/send",
            10,
        )
        self._sub = self.create_subscription(
            PointCloud2,
            "/merged_cloud",
            self._cb_cloud,
            10,
        )

        self._obstacle_id: int = 0

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _cb_cloud(self, msg: PointCloud2) -> None:
        """Parse point cloud, filter obstacles, and publish to MAVROS."""
        points = self._parse(msg)
        if points.shape[0] == 0:
            return

        xy_distances = np.linalg.norm(points[:, :2], axis=1)
        mask = (
            (xy_distances >= self._min_distance)
            & (xy_distances <= self._max_distance)
            & (points[:, 2] > self._min_height)
        )

        obstacles = points[mask]
        if obstacles.shape[0] == 0:
            return

        obs_xy_dist = xy_distances[mask]
        closest_indices = np.argsort(obs_xy_dist)[: self._max_obstacles]

        for idx in closest_indices:
            self._publish_obstacle(
                frame_id=msg.header.frame_id,
                point=obstacles[idx],
                xy_dist=obs_xy_dist[idx],
            )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _parse(self, msg: PointCloud2) -> np.ndarray:
        """Return an (N, 3) float32 array of XYZ points, NaNs removed."""
        structured = np.array(
            list(
                point_cloud2.read_points(
                    msg,
                    field_names=("x", "y", "z"),
                    skip_nans=True,
                )
            )
        )
        if structured.size == 0:
            return np.empty((0, 3), dtype=np.float32)
        return np.column_stack(
            [structured["x"], structured["y"], structured["z"]]
        ).astype(np.float32)

    def _publish_obstacle(
        self, frame_id: str, point: np.ndarray, xy_dist: float
    ) -> None:
        """Publish a single ObstacleDistance3D message for one obstacle."""
        self._obstacle_id = (self._obstacle_id + 1) % 65536

        msg = ObstacleDistance3D()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.sensor_type = _SENSOR_TYPE_LASER
        msg.frame = _MAV_FRAME_BODY_FRD
        msg.obstacle_id = self._obstacle_id
        msg.position = Point(
            x=float(point[0]), y=float(point[1]), z=float(point[2])
        )
        msg.min_distance = self._min_distance
        msg.max_distance = self._max_distance
        self._pub.publish(msg)
        self.get_logger().debug(
            f"obstacle #{self._obstacle_id}: "
            f"x={point[0]:.2f} y={point[1]:.2f} z={point[2]:.2f} "
            f"d_xy={xy_dist:.2f} m"
        )


def main() -> None:
    """Spin the ScanToMavlinkSid node until shutdown."""
    rclpy.init()
    rclpy.spin(ScanToMavlinkSid())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
