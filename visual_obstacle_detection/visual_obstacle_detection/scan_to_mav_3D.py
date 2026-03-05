"""
MovingFakeObstacle1D.

Publishes fake obstacle data to the ``/mavros/obstacle_distance_3d/send`` topic.
"""

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ObstacleDistance3D
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np


class MovingFakeObstacle1D(Node):
    """Publish fake obstacle data at 10 Hz."""

    def __init__(self) -> None:
        super().__init__("moving_fake_obstacle_1d")
        
        self.sub_zed1 = self.create_subscription(
            PointCloud2,
            "/zed1/zed_node/point_cloud/cloud_registered",
            self.cb_zed1,
            10
        )
        
        self.sub_zed2 = self.create_subscription(
            PointCloud2,
            "/zed2/zed_node/point_cloud/cloud_registered",
            self.cb_zed2,
            10
        )
        
        self.pub = self.create_publisher(
            ObstacleDistance3D,
            "/mavros/obstacle_distance_3d/send",
            10
        )
        
        self._cloud1 = np.empty((0,3), dtype=np.float32)
        self._cloud2 = np.empty((0,3), dtype=np.float32)
        
        self.min_dist = 0.5
        self.max_dist = 5.0
        self.min_height = 0.1
        
        # self.timer = self.create_timer(0.1, self.publish_fake)
        # self.obstacle_id = 1
        # self.position = Point(x=5.0, y=0.0, z=0.0)
        # self.speed = 0.05
        
    def parse_cloud(self, msg: PointCloud2) -> np.ndarray:
        points = np.array(list(point_cloud2.read_points(
            msg, field_names=("x","y","z"), skip_nans=True
        )))
        if points.size == 0:
            return np.empty((0,3), dtype=np.float32)
        return points.astype(np.float32)
    
    def cb_zed1(self, msg: PointCloud2):
        self._cloud1 = self.parse_cloud(msg)
        self.publish_obstacles()

    def cb_zed2(self, msg: PointCloud2):
        self._cloud2 = self.parse_cloud(msg)
        self.publish_obstacles()

    def publish_obstacles(self):
        # Merge the two clouds
        merged = np.concatenate((self._cloud1, self._cloud2), axis=0)
        if merged.shape[0] == 0:
            return

        # Compute distance in XY plane
        distances = np.linalg.norm(merged[:, :2], axis=1)

        # Mask for obstacle points
        mask = (distances > self.min_dist) & (distances < self.max_dist) & (merged[:,2] > self.min_height)
        obstacles = merged[mask]
        
        for i, p in enumerate(obstacles):
            msg = ObstacleDistance3D()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.sensor_type = 1
            msg.frame = 0
            msg.obstacle_id = i+1
            msg.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
            msg.min_distance = float(self.min_dist)
            msg.max_distance = float(self.max_dist)
            self.pub_obstacle.publish(msg)

        if obstacles.shape[0] > 0:
            self.get_logger().info(f"Published {obstacles.shape[0]} obstacles")
    
        
        

    # def publish_fake(self) -> None:
    #     """Publish fake obstacle data."""
    #     msg = ObstacleDistance3D()
    #     msg.header.stamp = self.get_clock().now().to_msg()
    #     msg.header.frame_id = "base_link"
    #     msg.sensor_type = 1
    #     msg.frame = 0
    #     msg.obstacle_id = self.obstacle_id
    #     self.position.x -= self.speed
    #     if self.position.x < 0.2:
    #         self.position.x = 5.0
    #     msg.position = self.position
    #     msg.min_distance = 0.2
    #     msg.max_distance = 60.0
    #     self.pub.publish(msg)
    #     self.get_logger().info(f"Obstacle x={self.position.x:.2f} m")


def main() -> None:
    """Spin the MovingFakeObstacle1D node until shutdown."""
    rclpy.init()
    rclpy.spin(MovingFakeObstacle1D())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
