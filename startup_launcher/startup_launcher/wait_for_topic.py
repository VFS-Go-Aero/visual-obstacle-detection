"""Wait until a topic publishes a message, then exit 0 (or 1 on timeout).

    ros2 run startup_launcher wait_for_topic /merged_cloud --timeout 30
    ros2 run startup_launcher wait_for_topic /mavros/state --timeout 40 \
        --field connected --equals True

The bringup launch files chain boot steps on this node's exit code.
"""
import argparse
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message


def field_value(msg, field):
    """Return msg.<field>, where field may be dotted (e.g. 'header.frame_id')."""
    for part in field.split('.'):
        msg = getattr(msg, part)
    return msg


def wait_for_message(node, topic, timeout, field=None, equals=None):
    """Spin `node` until `topic` delivers a (matching) message or `timeout` seconds pass."""
    deadline = time.monotonic() + timeout
    received = []

    def on_msg(msg):
        if field is None or str(field_value(msg, field)) == equals:
            received.append(msg)

    sub = None
    while time.monotonic() < deadline and not received:
        if sub is None:  # topic type is unknown until a publisher shows up
            types = dict(node.get_topic_names_and_types()).get(topic)
            if types:
                # best-effort subscribers accept both reliable and best-effort publishers
                sub = node.create_subscription(
                    get_message(types[0]), topic, on_msg, qos_profile_sensor_data)
        rclpy.spin_once(node, timeout_sec=0.2)

    if sub is not None:
        node.destroy_subscription(sub)
    return bool(received)


def main(argv=None):
    argv = rclpy.utilities.remove_ros_args(sys.argv if argv is None else argv)
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('topic')
    parser.add_argument('--timeout', type=float, default=30.0)
    parser.add_argument('--field', help="message field to check, e.g. 'connected'")
    parser.add_argument('--equals', help="required value of --field, e.g. 'True'")
    args = parser.parse_args(argv[1:])

    rclpy.init()
    node = rclpy.create_node('wait_for_topic')
    try:
        ok = wait_for_message(node, args.topic, args.timeout, args.field, args.equals)
    except KeyboardInterrupt:
        ok = False
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    what = args.topic + (f' ({args.field} == {args.equals})' if args.field else '')
    print(f"{'OK' if ok else 'TIMEOUT'}: {what}", flush=True)
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
