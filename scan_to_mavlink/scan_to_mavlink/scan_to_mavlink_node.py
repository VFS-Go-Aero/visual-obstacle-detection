"""
MovingFakeObstacle1D.

Publishes fake obstacle data to the ``/mavros/obstacle_distance_3d/send`` topic.
"""

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ObstacleDistance3D
from geometry_msgs.msg import Point

class MovingFakeObstacle1D(Node):
    """Publish fake obstacle data at 10 Hz."""

    def __init__(self) -> None:
        super().__init__("moving_fake_obstacle_1d")
        self.pub = self.create_publisher(
            ObstacleDistance3D,
            "/mavros/obstacle_distance_3d/send",
            10
        )
        self.timer = self.create_timer(0.1, self.publish_fake)
        self.obstacle_id = 1
        self.position = Point(x=5.0, y=0.0, z=0.0)
        self.speed = 0.05

    def publish_fake(self) -> None:
        """Publish fake obstacle data."""
        msg = ObstacleDistance3D()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.sensor_type = 1
        msg.frame = 0
        msg.obstacle_id = self.obstacle_id
        self.position.x -= self.speed
        if self.position.x < 0.2:
            self.position.x = 5.0
        msg.position = self.position
        msg.min_distance = 0.2
        msg.max_distance = 60.0
        self.pub.publish(msg)
        self.get_logger().info(f"Obstacle x={self.position.x:.2f} m")


def main() -> None:
    """Spin the MovingFakeObstacle1D node until shutdown."""
    rclpy.init()
    rclpy.spin(MovingFakeObstacle1D())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
