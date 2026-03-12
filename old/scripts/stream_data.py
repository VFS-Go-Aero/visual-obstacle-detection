import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix

# init ROS only if it hasn't been initialized yet
if not rclpy.ok():
    rclpy.init()

class PixhawkTest(Node):
    def __init__(self):
        super().__init__('pixhawk_test')
        self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_cb, 10)

    def state_cb(self, msg: State):
        print(f"Mode: {msg.mode}, Armed: {msg.armed}, Connected: {msg.connected}")

    def gps_cb(self, msg: NavSatFix):
        print(f"GPS: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, alt={msg.altitude:.2f}")

# create node
node = PixhawkTest()

# spin in the background without blocking
import threading
thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
thread.start()
