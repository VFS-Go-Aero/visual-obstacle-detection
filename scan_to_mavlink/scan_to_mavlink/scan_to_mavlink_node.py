#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ObstacleDistance3D
from geometry_msgs.msg import Point

class MovingFakeObstacle1D(Node):
    def __init__(self):
        super().__init__('moving_fake_obstacle_1d')

        self.pub = self.create_publisher(
            ObstacleDistance3D,
            '/mavros/obstacle/send',
            10
        )

        self.timer = self.create_timer(0.1, self.publish_fake)  # 10 Hz
        self.obstacle_id = 1

        # Only x-axis matters for PX6
        self.position = Point(x=5.0, y=0.0, z=0.0)
        self.speed = 0.05  # 0.5 m/s forward

    def publish_fake(self):
        msg = ObstacleDistance3D()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.sensor_type = 1  # 1 = LiDAR
        msg.frame = 0        # MAV_FRAME_BODY_FRD
        msg.obstacle_id = self.obstacle_id

        # Move obstacle toward PX6
        self.position.x -= self.speed
        if self.position.x < 0.2:
            self.position.x = 5.0

        msg.position = self.position
        msg.min_distance = 0.2
        msg.max_distance = 60.0

        self.pub.publish(msg)
        self.get_logger().info(f'Obstacle x={self.position.x:.2f} m')

def main():
    rclpy.init()
    node = MovingFakeObstacle1D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
