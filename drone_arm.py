#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode


class ArmAndModeNode(Node):
    def __init__(self):
        super().__init__('arm_and_mode_node')

        # Create service clients
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Wait for services
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/cmd/arming service...')

        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/set_mode service...')

        # Arm and set mode
        self.set_mode()
        self.arm_drone()
        

    def arm_drone(self):
        req = CommandBool.Request()
        req.value = True
        future = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.get_logger().info('Drone armed successfully.')
        else:
            self.get_logger().error('Failed to arm the drone.')

    def set_mode(self):
        req = SetMode.Request()
        req.custom_mode = 'STABILIZE'
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().mode_sent:
            self.get_logger().info('Mode changed successfully.')
        else:
            self.get_logger().error('Failed to change mode.')


def main():
    rclpy.init()
    node = ArmAndModeNode()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
