#!/usr/bin/python3

import psutil
import rclpy
from rclpy.node import Node


class CpuMonitor(Node):

    def __init__(self):
        super().__init__('cpu_monitor')

        self.legacy_proc_ = None
        self.accel_proc_ = None

        self.timer_ = self.create_timer(1.0, self.on_timer)
        self.get_logger().info('Searching for legacy_container and accel_container processes...')

    def _find_proc(self, name):
        for proc in psutil.process_iter(['pid', 'cmdline']):
            try:
                cmdline = proc.info['cmdline'] or []
                joined = ' '.join(cmdline)
                if 'component_container' in joined and name in joined:
                    proc.cpu_percent()  # prime the measurement
                    return proc
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        return None

    def on_timer(self):
        if self.legacy_proc_ is None:
            self.legacy_proc_ = self._find_proc('legacy_container')
        if self.accel_proc_ is None:
            self.accel_proc_ = self._find_proc('accel_container')

        if self.legacy_proc_ is None and self.accel_proc_ is None:
            self.get_logger().warn('Waiting for container processes...')
            return

        legacy_cpu = self._read_cpu(self.legacy_proc_, 'legacy_container')
        accel_cpu = self._read_cpu(self.accel_proc_, 'accel_container')

        bar_l = self._bar(legacy_cpu)
        bar_a = self._bar(accel_cpu)

        self.get_logger().info(
            f'[CPU UTIL]  Legacy: {legacy_cpu:6.1f}% {bar_l}  |  '
            f'Accelerated: {accel_cpu:6.1f}% {bar_a}')

    def _read_cpu(self, proc, label):
        if proc is None:
            return 0.0
        try:
            return proc.cpu_percent()
        except psutil.NoSuchProcess:
            self.get_logger().warn(f'{label} process exited')
            return 0.0

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
