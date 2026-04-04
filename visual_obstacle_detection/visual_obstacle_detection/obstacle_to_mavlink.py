import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ObstacleDistance3D
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


# MAV_FRAME_BODY_FRD from MAVLink enum (common.xml).
MAV_FRAME_BODY_FRD = 12


class ObstacleToMavlink(Node):

    def __init__(self) -> None:
        super().__init__("obstacle_to_mavlink")

        self.declare_parameter("MAX_DISTANCE", 60.0)
        self.declare_parameter("MIN_DISTANCE", 0.2)
        self.declare_parameter("FREQUENCY", 10.0)

        self._max_distance = self.get_parameter("MAX_DISTANCE").value
        self._min_distance = self.get_parameter("MIN_DISTANCE").value
        frequency = self.get_parameter("FREQUENCY").value

        self._latest_cloud: list[tuple[float, float, float, int]] | None = None

        self.create_subscription(
            PointCloud2,
            "/merged_cloud/obstacles",
            self._cb_obstacles,
            10,
        )

        self._pub_3d = self.create_publisher(
            ObstacleDistance3D,
            "/mavros/obstacle_distance_3d/send",
            10,
        )

        self.create_timer(1.0 / frequency, self._publish)

        self.get_logger().info(
            f"obstacle_to_mavlink started "
            f"[dist {self._min_distance}–{self._max_distance} m, "
            f"{frequency} Hz]"
        )

    def _cb_obstacles(self, msg: PointCloud2) -> None:
        pts = list(point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z", "obstacle_id"),
            skip_nans=True,
        ))
        if not pts:
            self._latest_cloud = None
            return

        self._latest_cloud = [
            (float(point[0]), float(point[1]), float(point[2]), int(point[3]))
            for point in pts
        ]

    def _publish(self) -> None:
        now = self.get_clock().now().to_msg()
        cloud = self._latest_cloud

        if cloud is not None and len(cloud) > 0:
            for x, y, z, obstacle_id in cloud:
                obstacle_id = int(obstacle_id % 65536)

                # Convert incoming body-centered RFU points to body FRD.
                x_frd = y
                y_frd = x
                z_frd = -z

                msg3D = ObstacleDistance3D()
                msg3D.header.stamp = now
                msg3D.header.frame_id = "base_link"
                msg3D.sensor_type = 1
                msg3D.frame = MAV_FRAME_BODY_FRD
                msg3D.obstacle_id = obstacle_id
                msg3D.position = Point(x=x_frd, y=y_frd, z=z_frd)
                msg3D.min_distance = self._min_distance
                msg3D.max_distance = self._max_distance
                self._pub_3d.publish(msg3D)

            self.get_logger().debug(
                f"Published {len(cloud)} ObstacleDistance3D"
            )


def main() -> None:
    rclpy.init()
    rclpy.spin(ObstacleToMavlink())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
