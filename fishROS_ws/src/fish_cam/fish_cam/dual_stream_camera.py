"""One-camera publisher for raw high-quality and low-res JPEG streams."""

from __future__ import annotations

import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image


class DualStreamCamera(Node):
    def __init__(self) -> None:
        super().__init__('dual_stream_camera')

        self.declare_parameter('video_device_id', 0)
        self.declare_parameter('raw_topic', '/camera/image_raw')
        self.declare_parameter('low_res_topic', '/camera/image_low/compressed')
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('capture_fps', 20.0)
        self.declare_parameter('low_res_fps', 10.0)
        self.declare_parameter('low_res_width', 640)
        self.declare_parameter('low_res_height', 480)
        self.declare_parameter('jpeg_quality', 90)
        self.declare_parameter('high_res_width', 0)
        self.declare_parameter('high_res_height', 0)

        self.video_device_id = int(self.get_parameter('video_device_id').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        capture_fps = float(self.get_parameter('capture_fps').value)
        self.low_res_period = 1.0 / float(
            self.get_parameter('low_res_fps').value
        )
        self.low_res_width = int(self.get_parameter('low_res_width').value)
        self.low_res_height = int(self.get_parameter('low_res_height').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        high_res_width = int(self.get_parameter('high_res_width').value)
        high_res_height = int(self.get_parameter('high_res_height').value)

        self.raw_publisher = self.create_publisher(
            Image,
            str(self.get_parameter('raw_topic').value),
            10,
        )
        self.low_res_publisher = self.create_publisher(
            CompressedImage,
            str(self.get_parameter('low_res_topic').value),
            10,
        )

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(self.video_device_id)
        if high_res_width > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, high_res_width)
        if high_res_height > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, high_res_height)
        self.cap.set(cv2.CAP_PROP_FPS, capture_fps)

        if not self.cap.isOpened():
            raise RuntimeError(
                f'Cannot open camera /dev/video{self.video_device_id}'
            )

        self._last_low_res_publish = 0.0
        self.timer = self.create_timer(1.0 / capture_fps, self.publish_frame)
        self.get_logger().info(
            f'Publishing /dev/video{self.video_device_id}: raw every frame, '
            f'low-res JPEG at {1.0 / self.low_res_period:.1f} Hz'
        )

    def publish_frame(self) -> None:
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Unable to read frame from camera')
            return

        stamp = self.get_clock().now().to_msg()

        raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        raw_msg.header.stamp = stamp
        raw_msg.header.frame_id = self.frame_id
        self.raw_publisher.publish(raw_msg)

        now = time.monotonic()
        if now - self._last_low_res_publish < self.low_res_period:
            return

        resized = cv2.resize(frame, (self.low_res_width, self.low_res_height))
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        ok, encoded = cv2.imencode('.jpg', resized, encode_params)
        if not ok:
            self.get_logger().error('Unable to encode low-res camera frame')
            return

        low_msg = CompressedImage()
        low_msg.header.stamp = stamp
        low_msg.header.frame_id = self.frame_id
        low_msg.format = 'jpeg'
        low_msg.data = np.asarray(encoded).tobytes()
        self.low_res_publisher.publish(low_msg)
        self._last_low_res_publish = now

    def destroy_node(self) -> bool:
        if hasattr(self, 'cap'):
            self.cap.release()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = DualStreamCamera()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
