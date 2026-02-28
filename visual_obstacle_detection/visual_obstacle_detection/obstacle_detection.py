import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header


class ObstacleDetection(Node):

    def __init__(self):
        super().__init__("obstacle_detection")

        self.subscription = self.create_subscription(
            PointCloud2,
            "/zed1/zed_node/point_cloud/cloud_registered",
            self._detect_and_publish,
            10
        )

        self.obstacle_pub = self.create_publisher(
            PointCloud2,
            "/obstacle_points",
            10
        )

        self.region_pub = self.create_publisher(
            MarkerArray,
            "/drone_regions",
            10
        )

        self.get_logger().info("Obstacle Detection Node Started")


    def _detect_and_publish(self, msg):

        
        min_dist = 0.5
        max_dist = 5.0
        min_height = 0.1

      
        x_min, x_max = -2.0, 2.0    
        y_min, y_max = 0.3, 5.0
        z_min, z_max = -1.5, 1.5    

        
        points = np.array(list(
            pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ))

        if points.shape[0] == 0:
            return

        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]

        dist = np.sqrt(x**2 + y**2 + z**2)

        
        valid = (
            (dist > min_dist) &
            (dist < max_dist) &
            (z > min_height) &
            (x > x_min) & (x < x_max) &
            (y > y_min) & (y < y_max) &
            (z > z_min) & (z < z_max)
        )

        if np.sum(valid) == 0:
            return

        filtered_points = points[valid]
        filtered_dist = dist[valid]
        x = filtered_points[:, 0]
        y = filtered_points[:, 1]
        z = filtered_points[:, 2]

 
        x_bins = np.linspace(x_min, x_max, 4)
        z_bins = np.linspace(z_min, z_max, 4)

        x_idx = np.digitize(x, x_bins) - 1
        z_idx = np.digitize(z, z_bins) - 1

        # Ensure indices are valid
        x_idx = np.clip(x_idx, 0, 2)
        z_idx = np.clip(z_idx, 0, 2)

        region_id = z_idx * 3 + x_idx

        region_blocked = [False] * 9

        for r in range(9):

            region_mask = (region_id == r)

            if np.sum(region_mask) == 0:
                continue

            region_dist = filtered_dist[region_mask]

            # Distance weighting:
            # If close obstacle exists, trigger fast
            close_points = region_dist < 2.0

            # Threshold tuning
            if r == 4:  # center region
                threshold = 20
            else:
                threshold = 40

            if np.sum(close_points) > 5 or np.sum(region_mask) > threshold:
                region_blocked[r] = True


        obstacle_points = filtered_points[
            np.isin(region_id, [i for i, b in enumerate(region_blocked) if b])
        ]

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        obstacle_cloud = pc2.create_cloud_xyz32(header, obstacle_points.tolist())
        self.obstacle_pub.publish(obstacle_cloud)

        marker_array = MarkerArray()

        region_width = (x_max - x_min) / 3
        region_height = (z_max - z_min) / 3
        region_depth = (y_max - y_min)

        for r in range(9):

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "regions"
            marker.id = r
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            row = r // 3
            col = r % 3

            marker.pose.position.x = x_min + (col + 0.5) * region_width
            marker.pose.position.y = (y_min + y_max) / 2
            marker.pose.position.z = z_min + (row + 0.5) * region_height

            marker.pose.orientation.w = 1.0

            marker.scale.x = region_width
            marker.scale.y = region_depth
            marker.scale.z = region_height

            if region_blocked[r]:
                marker.color.r = 1.0
                marker.color.g = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0

            marker.color.b = 0.0
            marker.color.a = 0.3

            marker_array.markers.append(marker)

        self.region_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()