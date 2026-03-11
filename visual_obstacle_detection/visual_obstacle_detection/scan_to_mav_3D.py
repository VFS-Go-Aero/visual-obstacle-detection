import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ObstacleDistance3D, ObstacleDistance
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import Header

INCREMENT_ANGLE = 5.0 


class ObstaclesToMAVLink(Node):
    
    def __init__(self) -> None:
        super().__init__("scan_to_mav_3D")
        
        self.declare_parameter("INCREMENT_ANGLE", INCREMENT_ANGLE)  
        self.declare_parameter("MAX_DISTANCE", 60.0)
        self.declare_parameter("MIN_DISTANCE", 0.2)
        self.declare_parameter("FREQUENCY", 10.0)
        
        self.increment_angle = self.get_parameter("INCREMENT_ANGLE").value
        self.max_distance = self.get_parameter("MAX_DISTANCE").value    
        self.min_distance = self.get_parameter("MIN_DISTANCE").value    
        frequency = self.get_parameter("FREQUENCY").value  
        
        self.n_bins = int(360 / self.increment_angle)
        self.recent_cloud: np.ndarray | None = None
        
        self.create_subscription(
            PointCloud2,
            "/merged_cloud/obstacles",
            self._cb_obstacles,
            10,
        )
        
        self.pub_3d = self.create_publisher(
            ObstacleDistance3D,
            "/mavros/obstacle_distance_3d/send",
            10,
        )
        
        self.pub_2d = self.create_publisher(
            ObstacleDistance,
            "/mavros/obstacle_distance/send",
            10,
        )
        
        self.create_timer(1.0 / frequency, self._publish)

        self.get_logger().info(
            f"obstacles_to_mavlink started  "
            f"[{self._n_bins} bins × {self._increment}°, "
            f"dist {self._min_distance}–{self._max_distance} m, "
            f"{frequency} Hz]"
        )
        
        self.get_logger().info(
            f"obstacles_to_mavlink started  "
            f"[{self._n_bins} bins × {self._increment}°, "
            f"dist {self._min_distance}–{self._max_distance} m, "
            f"{frequency} Hz]"
        )
        
    def _cb_obstacles(self, msg: PointCloud2) -> None:
        pts = list(point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z", "rgb"),
            skip_nans=True,
        ))
        if not pts:
            self._latest_cloud = None
            return

        arr = np.array(
            [(p[0], p[1], p[2], p[3]) for p in pts],
            dtype=np.float32,
        )
        self._latest_cloud = arr
        
        
    def _publish(self) -> None:
        if self.recent_cloud is None or self.recent_cloud.shape[0] == 0:
            self._publish_in_2d()
            return
        
        cloud = self.recent_cloud
        now = self.get_clock().now().to_msg()
        
        for row in cloud:
            x,y,z, raw_rgb = float(row[0]), float(row[1]), float(row[2]), int(row[3])
            obstacle_id = int(np.frombuffer(np.float32(rgb_raw).tobytes(), dtype=np.uint32)[0])
            
            msg3D = ObstacleDistance3D()
            msg3D.header.stamp = now    
            msg3D.header.frame_id = "base_link"
            msg3D.sensor_type = 1
            msg3D.frame = 0
            msg3D.obstacle_id = obstacle_id
            msg3D.position = Point(x=x, y=y, z=z)
            msg3D.min_distance = self.min_distance
            msg3D.max_distance = self.max_distance
            self.pub_3d.publish(msg3D)
            
        self._publish_obstacle_distance(cloud, now)
        self.get_logger().debug(
            f"Published {cloud.shape[0]} ObstacleDistance3D + 1 ObstacleDistance"
        )
        
    def _publish_in_2d(self) -> None:
        msg = ObstacleDistance()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.sensor_type     = 0
        msg.frame           = 12        
        msg.increment       = self._increment
        msg.min_distance    = int(self._min_distance * 100)   # cm
        msg.max_distance    = int(self._max_distance * 100)   # cm
        msg.increment_f     = float(self._increment)
        msg.angle_offset    = 0.0
        msg.distances       = [int(self._max_distance * 100)] * self._n_bins
        self._pub_2d.publish(msg)
            
    def _publish_obstacle_distance(self, cloud: np.ndarray, stamp) -> None:
        xy = cloud[:, :2]   
        dist2d = np.linalg.norm(xy, axis=1)
        valid = (dist2d >= self._min_distance) & (dist2d <= self._max_distance)
        xy = xy[valid]
        dist2d = dist2d[valid]
        max_in_cm = int(self._max_distance * 100)
        bins_in_cm = np.full((self._n_bins,), max_in_cm, dtype=np.int16)
        
        if xy.shape[0] > 0:
            angles_deg = np.degrees(np.arctan2(xy[:, 1], xy[:, 0])) % 360.0
            bin_indices = (angles_deg / self._increment).astype(int) % self._n_bins
            
            for b in range(self._n_bins):
                mask = bin_indices == b
                if np.any(mask):
                    bins_in_cm[b] = int(dist2d[mask].min() * 100)
                    
        msg = ObstacleDistance()
        msg.header.stamp    = stamp
        msg.header.frame_id = "base_link"
        msg.sensor_type     = 0
        msg.frame           = 12 
        msg.increment       = self._increment
        msg.min_distance    = int(self._min_distance * 100)
        msg.max_distance    = int(self._max_distance * 100)
        msg.increment_f     = float(self._increment)
        msg.angle_offset    = 0.0
        msg.distances       = bins_in_cm.tolist()
        self._pub_2d.publish(msg)
        
def main() -> None:
    rclpy.init()
    rclpy.spin(ObstaclesToMAVLink())
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()

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


# def main() -> None:
#     """Spin the MovingFakeObstacle1D node until shutdown."""
#     rclpy.init()
#     rclpy.spin(MovingFakeObstacle1D())
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()
