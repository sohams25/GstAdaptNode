#!/usr/bin/python3

import psutil
import rclpy
from rclpy.node import Node


class CpuMonitor(Node):

    def __init__(self):
        super().__init__('cpu_monitor')

        self.container_proc_ = None
        self.timer_ = self.create_timer(1.0, self.on_timer)
        self.get_logger().info('Searching for ab_container process...')

    def _find_container(self):
        for proc in psutil.process_iter(['pid', 'cmdline']):
            try:
                cmdline = proc.info['cmdline'] or []
                joined = ' '.join(cmdline)
                if 'component_container' in joined and 'ab_container' in joined:
                    proc.cpu_percent()
                    return proc
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        return None

    def on_timer(self):
        if self.container_proc_ is None:
            self.container_proc_ = self._find_container()
            if self.container_proc_ is None:
                self.get_logger().warn('Waiting for ab_container...', throttle_duration_sec=5.0)
                return

        try:
            cpu = self.container_proc_.cpu_percent()
            mem = self.container_proc_.memory_info().rss / (1024 * 1024)
        except psutil.NoSuchProcess:
            self.container_proc_ = None
            return

        bar = self._bar(cpu)
        self.get_logger().info(
            f'[CONTAINER]  CPU: {cpu:6.1f}% {bar}  |  RAM: {mem:.0f} MB')

    @staticmethod
    def _bar(pct, width=20):
        filled = int(min(pct, 400) * width / 400)
        return '[' + '#' * filled + '.' * (width - filled) + ']'


def main(args=None):
    rclpy.init(args=args)
    node = CpuMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
