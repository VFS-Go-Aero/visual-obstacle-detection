import rclpy
from rclpy.node import Node
import numpy as np
import struct

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header

class PointCloud(Node):

    def __init__(self) -> None:
        super().__init__("obstacle_detection")

        self._cloud1 = np.empty(    (0, 3), dtype=np.float32)
        self._cloud2 = np.empty((0, 3), dtype=np.float32)
        self.cloud = np.empty((0, 3), dtype=np.float32) 

        self.pub = self.create_publisher(PointCloud2, "/merged_cloud/obstacles", 10)
        self.region_pub = self.create_publisher(MarkerArray, "/merged_cloud/regions", 10)

        self._sub_zed1 = self.create_subscription(
            PointCloud2,
            "/zed1/zed_node/point_cloud/cloud_registered",
            self._cb_zed1,
            10,
        )

        self._sub_zed2 = self.create_subscription(
            PointCloud2,
            "/zed2/zed_node/point_cloud/cloud_registered",
            self._cb_zed2,
            10,
        )

    def _parse(self, msg: PointCloud2) -> np.ndarray:
        structured = np.array(list(pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True)))
        if structured.size == 0:
            return np.empty((0, 3), dtype=np.float32)
        return structured.astype(np.float32)

    def _merge(self) -> None:
        self.cloud = np.concatenate((self._cloud1, self._cloud2), axis=0)

    def _cb_zed1(self, msg: PointCloud2) -> None:
        self._cloud1 = self._parse(msg)
        self._merge()
        self._detect_and_publish()

    def _cb_zed2(self, msg: PointCloud2) -> None:
        self._cloud2 = self._parse(msg)
        self._merge()
        self._detect_and_publish()

    def _detect_and_publish(self) -> None:
        if self.cloud.shape[0] == 0:
            return

        min_dist, max_dist, min_height = 0.5, 5.0, 0.1
        x_min, x_max = -5.0, 5.0
        y_min, y_max = -5.0, 5.0
        z_min, z_max = -5.0, 5.0

        x = self.cloud[:,0]
        y = self.cloud[:,1]
        z = self.cloud[:,2]
        dist = np.sqrt(x**2 + y**2 + z**2)

        obstacle_mask = (dist>min_dist)&(dist<max_dist)&(z>min_height)&(x>x_min)&(x<x_max)&(y>y_min)&(y<y_max)&(z>z_min)&(z<z_max)

        colored_points = []
        for i, p in enumerate(self.cloud):
            px, py, pz = p
            if obstacle_mask[i]:
                r, g, b = 255, 0, 0
            else:
                r, g, b = 255, 255, 255
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]
            colored_points.append([px, py, pz, rgb])

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]

        cloud_msg = pc2.create_cloud(header, fields, colored_points)
        self.pub.publish(cloud_msg)

        # 3x3 regions for visualization
        marker_array = MarkerArray()
        region_width = (x_max - x_min)/3
        region_height = (z_max - z_min)/3
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
            marker.pose.position.x = x_min + (col+0.5)*region_width
            marker.pose.position.y = (y_min + y_max)/2
            marker.pose.position.z = z_min + (row+0.5)*region_height
            marker.pose.orientation.w = 1.0

            marker.scale.x = region_width
            marker.scale.y = region_depth
            marker.scale.z = region_height
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.3
            marker_array.markers.append(marker)

        self.region_pub.publish(marker_array)

def main():
    rclpy.init()
    node = PointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()