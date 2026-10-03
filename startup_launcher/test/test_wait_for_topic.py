import time

import pytest
import rclpy
from std_msgs.msg import Bool, String

from startup_launcher.wait_for_topic import wait_for_message


@pytest.fixture
def node():
    rclpy.init()
    n = rclpy.create_node('test_wait_for_topic')
    yield n
    n.destroy_node()
    rclpy.shutdown()


def start_publisher(node, topic, msg_type, msg):
    pub = node.create_publisher(msg_type, topic, 10)
    node.create_timer(0.1, lambda: pub.publish(msg))


def test_returns_true_when_message_arrives(node):
    start_publisher(node, '/vfs_test/chatter', String, String(data='hi'))
    assert wait_for_message(node, '/vfs_test/chatter', timeout=5.0)


def test_returns_false_on_timeout(node):
    start = time.monotonic()
    assert not wait_for_message(node, '/vfs_test/nobody', timeout=1.0)
    assert time.monotonic() - start < 3.0


def test_field_must_match(node):
    start_publisher(node, '/vfs_test/state', Bool, Bool(data=False))
    assert not wait_for_message(
        node, '/vfs_test/state', timeout=1.5, field='data', equals='True')


def test_field_match_succeeds(node):
    start_publisher(node, '/vfs_test/state2', Bool, Bool(data=True))
    assert wait_for_message(
        node, '/vfs_test/state2', timeout=5.0, field='data', equals='True')
