#!/usr/bin/python3

import time

import cv2
import numpy as np
import psutil
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image


class VisualizeDemoNode(Node):

    DISPLAY_W = 640
    DISPLAY_H = 480

    def __init__(self):
        super().__init__('visualize_demo')

        self.create_subscription(Image, '/legacy/image_processed', self.on_legacy, 10)
        self.create_subscription(Image, '/accelerated/image_processed', self.on_accel, 10)

        self.legacy_frame_ = np.zeros((self.DISPLAY_H, self.DISPLAY_W, 3), dtype=np.uint8)
        self.accel_frame_ = np.zeros((self.DISPLAY_H, self.DISPLAY_W, 3), dtype=np.uint8)

        self.legacy_latency_ = 0.0
        self.accel_latency_ = 0.0
        self.legacy_fps_ = 0.0
        self.accel_fps_ = 0.0
        self.legacy_cpu_ = 0.0
        self.accel_cpu_ = 0.0

        self.legacy_count_ = 0
        self.accel_count_ = 0
        self.last_fps_time_ = time.monotonic()

        self.legacy_proc_ = None
        self.accel_proc_ = None

        self.timer_ = self.create_timer(0.033, self.render)
        self.cpu_timer_ = self.create_timer(1.0, self.update_cpu)

        self.get_logger().info('Visual dashboard started — waiting for frames')

    def _find_proc(self, name):
        for proc in psutil.process_iter(['pid', 'cmdline']):
            try:
                cmdline = proc.info['cmdline'] or []
                if 'component_container' in ' '.join(cmdline) and name in ' '.join(cmdline):
                    proc.cpu_percent()
                    return proc
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        return None

    def _decode_frame(self, msg):
        if msg.encoding == 'bgr8':
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        elif msg.encoding == 'rgb8':
            rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        else:
            h, w = msg.height, msg.width
            yuv_size = w * h * 3 // 2
            if len(msg.data) >= yuv_size:
                yuv = np.frombuffer(msg.data, dtype=np.uint8)[:yuv_size].reshape(h * 3 // 2, w)
                return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
            return None

    def _latency_ms(self, msg):
        now = self.get_clock().now()
        stamp = Time.from_msg(msg.header.stamp)
        return (now - stamp).nanoseconds / 1e6

    def on_legacy(self, msg):
        frame = self._decode_frame(msg)
        if frame is not None:
            self.legacy_frame_ = cv2.resize(frame, (self.DISPLAY_W, self.DISPLAY_H))
        self.legacy_latency_ = self._latency_ms(msg)
        self.legacy_count_ += 1

    def on_accel(self, msg):
        frame = self._decode_frame(msg)
        if frame is not None:
            self.accel_frame_ = cv2.resize(frame, (self.DISPLAY_W, self.DISPLAY_H))
        self.accel_latency_ = self._latency_ms(msg)
        self.accel_count_ += 1

    def update_cpu(self):
        if self.legacy_proc_ is None:
            self.legacy_proc_ = self._find_proc('legacy_container')
        if self.accel_proc_ is None:
            self.accel_proc_ = self._find_proc('accel_container')

        try:
            self.legacy_cpu_ = self.legacy_proc_.cpu_percent() if self.legacy_proc_ else 0
        except psutil.NoSuchProcess:
            self.legacy_cpu_ = 0
        try:
            self.accel_cpu_ = self.accel_proc_.cpu_percent() if self.accel_proc_ else 0
        except psutil.NoSuchProcess:
            self.accel_cpu_ = 0

        elapsed = time.monotonic() - self.last_fps_time_
        if elapsed > 0:
            self.legacy_fps_ = self.legacy_count_ / elapsed
            self.accel_fps_ = self.accel_count_ / elapsed
        self.legacy_count_ = 0
        self.accel_count_ = 0
        self.last_fps_time_ = time.monotonic()

    def render(self):
        left = self.legacy_frame_.copy()
        right = self.accel_frame_.copy()

        self._overlay(left, 'LEGACY (CPU)', self.legacy_latency_,
                      self.legacy_fps_, self.legacy_cpu_, (0, 140, 255))
        self._overlay(right, 'ACCELERATED (VA-API)', self.accel_latency_,
                      self.accel_fps_, self.accel_cpu_, (0, 255, 80))

        combined = np.hstack([left, right])
        cv2.line(combined, (self.DISPLAY_W, 0), (self.DISPLAY_W, self.DISPLAY_H),
                 (255, 255, 255), 2)

        cv2.imshow('GstAdaptNode A/B Benchmark', combined)
        cv2.waitKey(1)

    @staticmethod
    def _overlay(frame, label, latency, fps, cpu, color):
        cv2.rectangle(frame, (0, 0), (360, 110), (0, 0, 0), cv2.FILLED)
        cv2.putText(frame, label, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)
        cv2.putText(frame, f'Latency: {latency:7.1f} ms', (10, 52),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, f'FPS:     {fps:7.1f}', (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, f'CPU:     {cpu:7.1f}%', (10, 98),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)


def main(args=None):
    rclpy.init(args=args)
    node = VisualizeDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
