import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class DronePose(Node):
    def __init__(self):
        super().__init__('drone_pose')
        self.subscription = self.create_subscription(
            Odometry,
            '/mavros/local_position/odom',
            self.odom_callback,
            10
        )
        self.pose_publisher = self.create_publisher(PoseStamped, '/drone_pose', 10)

    def odom_callback(self, msg):
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose.pose
        self.pose_publisher.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DronePose())
    rclpy.shutdown()
