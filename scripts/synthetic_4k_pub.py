#!/usr/bin/python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import CameraInfo, Image


class Synthetic4kPub(Node):

    WIDTH = 3840
    HEIGHT = 2160
    FPS = 30
    ENCODING = 'bgr8'

    def __init__(self):
        super().__init__('synthetic_4k_pub')

        self.pub_ = self.create_publisher(
            Image, '/camera/image_raw', qos_profile_system_default)
        self.info_pub_ = self.create_publisher(
            CameraInfo, '/camera/camera_info', qos_profile_system_default)

        # Pre-allocate the 4K frame once (~24.9 MB). The timer callback only
        # draws a small overlay on top — no per-frame allocation.
        self.frame_ = np.zeros(
            (self.HEIGHT, self.WIDTH, 3), dtype=np.uint8)

        # Gradient background so the image isn't solid black during demos
        self.frame_[:, :, 0] = np.tile(
            np.linspace(40, 80, self.WIDTH, dtype=np.uint8), (self.HEIGHT, 1))

        # Bouncing circle state
        self.cx_ = self.WIDTH // 2
        self.cy_ = self.HEIGHT // 2
        self.dx_ = 15
        self.dy_ = 10
        self.radius_ = 80

        # Pre-build the Image message shell (reused every tick)
        self.msg_ = Image()
        self.msg_.height = self.HEIGHT
        self.msg_.width = self.WIDTH
        self.msg_.encoding = self.ENCODING
        self.msg_.step = self.WIDTH * 3
        self.msg_.is_bigendian = False

        # Synthetic pinhole camera info (focal length = width, centered principal point)
        self.info_msg_ = CameraInfo()
        self.info_msg_.width = self.WIDTH
        self.info_msg_.height = self.HEIGHT
        self.info_msg_.distortion_model = 'plumb_bob'
        self.info_msg_.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        fx = float(self.WIDTH)
        fy = float(self.WIDTH)
        cx = float(self.WIDTH) / 2.0
        cy = float(self.HEIGHT) / 2.0
        self.info_msg_.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self.info_msg_.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.info_msg_.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        self.frame_id_ = 0

        period = 1.0 / self.FPS
        self.timer_ = self.create_timer(period, self.on_timer)
        self.get_logger().info(
            f'Publishing {self.WIDTH}x{self.HEIGHT} @ {self.FPS} Hz '
            f'on /camera/image_raw')

    def on_timer(self):
        # Erase previous circle position (restore gradient under it)
        cv2.circle(self.frame_, (self.cx_, self.cy_),
                   self.radius_ + 2, (0, 0, 0), -1)
        # Restore gradient in the erased region
        x0 = max(self.cx_ - self.radius_ - 2, 0)
        x1 = min(self.cx_ + self.radius_ + 3, self.WIDTH)
        self.frame_[
            max(self.cy_ - self.radius_ - 2, 0):
            min(self.cy_ + self.radius_ + 3, self.HEIGHT),
            x0:x1, 0
        ] = np.linspace(40, 80, self.WIDTH, dtype=np.uint8)[x0:x1]

        # Advance circle
        self.cx_ += self.dx_
        self.cy_ += self.dy_
        if self.cx_ - self.radius_ <= 0 or self.cx_ + self.radius_ >= self.WIDTH:
            self.dx_ = -self.dx_
        if self.cy_ - self.radius_ <= 0 or self.cy_ + self.radius_ >= self.HEIGHT:
            self.dy_ = -self.dy_

        # Draw new circle + frame counter
        cv2.circle(self.frame_, (self.cx_, self.cy_),
                   self.radius_, (0, 255, 0), -1)
        cv2.putText(self.frame_, str(self.frame_id_),
                    (60, 120), cv2.FONT_HERSHEY_SIMPLEX,
                    3.0, (255, 255, 255), 4, cv2.LINE_AA)
        # Clear the text region next tick by overwriting with background
        # (cheaper than clearing the whole frame)

        # Stamp and publish — reuse the pre-built message shells
        stamp = self.get_clock().now().to_msg()
        self.msg_.header.stamp = stamp
        self.msg_.header.frame_id = 'synthetic_4k'
        self.msg_.data = self.frame_.tobytes()
        self.pub_.publish(self.msg_)

        self.info_msg_.header.stamp = stamp
        self.info_msg_.header.frame_id = 'synthetic_4k'
        self.info_pub_.publish(self.info_msg_)

        # Erase text for next frame (small region only)
        cv2.rectangle(self.frame_, (50, 60), (500, 140), (0, 0, 0), -1)
        x0, x1 = 50, 500
        self.frame_[60:140, x0:x1, 0] = np.linspace(
            40, 80, self.WIDTH, dtype=np.uint8)[x0:x1]

        self.frame_id_ += 1


def main(args=None):
    rclpy.init(args=args)
    node = Synthetic4kPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
