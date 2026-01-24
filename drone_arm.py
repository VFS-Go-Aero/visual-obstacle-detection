import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool


class ArmDrone(Node):
    def __init__(self):
        super().__init__('arm_drone_client')
        self.client = self.create_client(CommandBool, '/mavros/cmd/arming')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/cmd/arming service...')

        self.request = CommandBool.Request()
        self.request.value = True  

    def send_request(self):
        self.future = self.client.call_async(self.request)


def main():
    rclpy.init()
    node = ArmDrone()
    node.send_request()

    rclpy.spin_until_future_complete(node, node.future)

    if node.future.result() is not None:
        response = node.future.result()
        node.get_logger().info(f'Arming success: {response.success}')
    else:
        node.get_logger().error('Service call failed.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
