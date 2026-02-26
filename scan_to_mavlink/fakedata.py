import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class FakeObstaclePublisher(Node):
    def __init__(self):
        super().__init__('fake_obstacle_publisher')
        self.pub = self.create_publisher(Float32MultiArray, '/obstacles', 10)
        self.timer = self.create_timer(0.1, self.publish_data)

    def publish_data(self):
        msg = Float32MultiArray()
        msg.data = [3.0] * 72  # 2 meters in all directions
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = FakeObstaclePublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
