#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image


class LatencyTracker(Node):

    def __init__(self):
        super().__init__('latency_tracker')

        self.create_subscription(
            Image, '/legacy/image_processed',
            self.on_legacy, 10)
        self.create_subscription(
            Image, '/accelerated/image_processed',
            self.on_accelerated, 10)

        self.get_logger().info(
            'Tracking latency on /legacy/image_processed '
            'and /accelerated/image_processed')

    def _measure(self, msg, label):
        now = self.get_clock().now()
        stamp = Time.from_msg(msg.header.stamp)
        dt_ns = (now - stamp).nanoseconds
        dt_ms = dt_ns / 1e6
        self.get_logger().info(
            f'[{label:>12s}]  latency: {dt_ms:8.2f} ms  '
            f'(frame {msg.header.frame_id})')

    def on_legacy(self, msg):
        self._measure(msg, 'LEGACY')

    def on_accelerated(self, msg):
        self._measure(msg, 'ACCELERATED')


def main(args=None):
    rclpy.init(args=args)
    node = LatencyTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
