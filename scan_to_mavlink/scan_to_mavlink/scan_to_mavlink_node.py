#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ObstacleDistance3D
from geometry_msgs.msg import Point

class MovingFakeObstacle3D(Node):
    def __init__(self):
        super().__init__('moving_fake_obstacle_3d')

        self.pub = self.create_publisher(
            ObstacleDistance3D,
            '/mavros/obstacle/send',
            10
        )

        self.timer = self.create_timer(0.1, self.publish_fake)  # 10 Hz
        self.obstacle_id = 1

        # Starting position
        self.position = Point(x=5.0, y=0.0, z=0.0)  # 5 m in front
        self.speed = 0.05  # m per cycle (0.05 m * 10 Hz = 0.5 m/s)

    def publish_fake(self):
        msg = ObstacleDistance3D()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.sensor_type = ObstacleDistance3D.MAV_DISTANCE_SENSOR_LASERMAV
        msg.frame = 0  # MAV_FRAME_BODY_FRD
        msg.obstacle_id = self.obstacle_id

        # Update obstacle position
        self.position.x -= self.speed  # Move toward the drone
        if self.position.x < 0.2:      # Reset when too close
            self.position.x = 5.0

        msg.position = self.position

        msg.min_distance = 0.2
        msg.max_distance = 60.0

        self.pub.publish(msg)
        self.get_logger().info(f'Published obstacle at x={self.position.x:.2f}')

def main():
    rclpy.init()
    node = MovingFakeObstacle3D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
