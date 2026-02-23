"""Publisher — simple ROS 2 node that publishes a greeting on the
``chatter`` topic once per second.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MyPublisher(Node):
    """Publish a fixed string message at 1 Hz on the ``chatter`` topic."""

    def __init__(self) -> None:
        super().__init__("my_publisher")
        self.publisher_ = self.create_publisher(String, "chatter", 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self) -> None:
        """Build and publish a greeting message."""
        msg = String()
        msg.data = "Hello from my first ROS 2 node"
        self.publisher_.publish(msg)
        self.get_logger().info(msg.data)


def main() -> None:
    """Spin the MyPublisher node until shutdown."""
    rclpy.init()
    rclpy.spin(MyPublisher())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
