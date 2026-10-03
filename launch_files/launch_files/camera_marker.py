"""
Sphere marker on the camera frame so you can see where the camera is in RViz.

Same as the old one-liner:
    ros2 topic pub -r 2 /camera_marker visualization_msgs/msg/Marker "{... zed1_camera_link ...}"
"""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker


class CameraMarker(Node):

    def __init__(self):
        super().__init__("camera_marker")
        frame_id = self.declare_parameter("frame_id", "zed1_camera_link").value
        self.pub = self.create_publisher(Marker, "/camera_marker", 10)

        self.marker = Marker()
        self.marker.header.frame_id = frame_id
        self.marker.ns = "cam"
        self.marker.id = 0
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = self.marker.scale.y = self.marker.scale.z = 0.2
        self.marker.color.r, self.marker.color.g, self.marker.color.b, self.marker.color.a = (
            1.0, 0.5, 0.0, 1.0)

        self.create_timer(0.5, self.publish)  # 2 Hz

    def publish(self):
        self.pub.publish(self.marker)


def main():
    rclpy.init()
    node = CameraMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
