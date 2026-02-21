import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyPublisher(Node):
    def __init__(self):
        super().__init__("my_publisher")
        self.publisher_ = self.create_publisher(String, "chatter", 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = "Hello from my first ROS 2 node"
        self.publisher_.publish(msg)
        self.get_logger().info(msg.data)

def main():
    rclpy.init()
    rclpy.spin(MyPublisher())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
