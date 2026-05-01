"""One-camera publisher for low-res JPEG and selected high-res JPEG captures."""

from __future__ import annotations

import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


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
        self.declare_parameter('high_res_topic', '/camera/image_high/compressed')
        self.declare_parameter('high_res_width', 1920)
        self.declare_parameter('high_res_height', 1080)
        self.declare_parameter('high_res_fps', 1.0)
        self.declare_parameter('high_res_jpeg_quality', 85)
        self.declare_parameter('high_res_drain_frames', 0)
        self.declare_parameter('publish_raw', True)
        self.declare_parameter('publish_low_res', True)
        self.declare_parameter('capture_fourcc', 'MJPG')
        self.declare_parameter('high_res_selector_topic', '/fish_cam/high_res_camera')
        self.declare_parameter('selected_high_res_camera', '')

        self.video_device_id = int(self.get_parameter('video_device_id').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        low_res_fps = max(0.1, float(self.get_parameter('low_res_fps').value))
        high_res_fps = max(0.1, float(self.get_parameter('high_res_fps').value))
        self.low_profile = {
            'width': int(self.get_parameter('low_res_width').value),
            'height': int(self.get_parameter('low_res_height').value),
            'fps': low_res_fps,
            'jpeg_quality': int(self.get_parameter('jpeg_quality').value),
        }
        self.high_res_profile = {
            'width': int(self.get_parameter('high_res_width').value),
            'height': int(self.get_parameter('high_res_height').value),
            'fps': high_res_fps,
            'jpeg_quality': int(
                self.get_parameter('high_res_jpeg_quality').value
            ),
        }
        self.high_res_period = 1.0 / self.high_res_profile['fps']
        self.high_res_drain_frames = max(
            0, int(self.get_parameter('high_res_drain_frames').value)
        )
        capture_fps = float(self.get_parameter('capture_fps').value)
        if capture_fps <= 0.0:
            capture_fps = low_res_fps
        self.publish_raw = bool(self.get_parameter('publish_raw').value)
        self.publish_low_res = bool(self.get_parameter('publish_low_res').value)
        self.capture_fourcc = str(self.get_parameter('capture_fourcc').value)
        selected_high_res_camera = str(
            self.get_parameter('selected_high_res_camera').value
        )

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.raw_publisher = None
        if self.publish_raw:
            self.raw_publisher = self.create_publisher(
                Image,
                str(self.get_parameter('raw_topic').value),
                sensor_qos,
            )

        self.low_res_publisher = None
        if self.publish_low_res:
            self.low_res_publisher = self.create_publisher(
                CompressedImage,
                str(self.get_parameter('low_res_topic').value),
                sensor_qos,
            )

        self.high_res_publisher = self.create_publisher(
            CompressedImage,
            str(self.get_parameter('high_res_topic').value),
            sensor_qos,
        )

        self.bridge = CvBridge() if self.publish_raw else None
        self.cap = cv2.VideoCapture(self.video_device_id)
        if self.capture_fourcc:
            self._set_capture_fourcc()
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            raise RuntimeError(
                f'Cannot open camera /dev/video{self.video_device_id}'
            )

        self.low_res_width = 0
        self.low_res_height = 0
        self.low_res_period = 1.0
        self.jpeg_quality = 0
        self.high_res_selected = False
        self._apply_low_profile(reason='initial low-res profile')
        self._set_high_res_selected(
            self._selector_matches(selected_high_res_camera),
            reason='initial selection',
        )

        self.selector_subscription = self.create_subscription(
            String,
            str(self.get_parameter('high_res_selector_topic').value),
            self._handle_high_res_selection,
            10,
        )

        self._last_low_res_publish = 0.0
        self._last_high_res_publish = 0.0
        self.timer = self.create_timer(1.0 / capture_fps, self.publish_frame)
        self.get_logger().info(
            f'Publishing /dev/video{self.video_device_id}: '
            f'raw={self.publish_raw}, low-res JPEG={self.publish_low_res} '
            f'best-effort QoS depth 1, selected high-res JPEG '
            f'{self.high_res_profile["width"]}x{self.high_res_profile["height"]} '
            f'at {self.high_res_profile["fps"]:.1f} FPS'
        )

    def _selector_matches(self, selected_camera: str) -> bool:
        selected_camera = selected_camera.strip()
        if selected_camera.lower() == 'all':
            self.get_logger().warn(
                '"all" high-res selection ignored to protect USB bandwidth; '
                'select one camera name or /dev/video ID'
            )
            return False

        return selected_camera in {
            self.frame_id,
            self.get_name(),
            str(self.video_device_id),
            f'/dev/video{self.video_device_id}',
        }

    def _handle_high_res_selection(self, msg: String) -> None:
        self._set_high_res_selected(
            self._selector_matches(msg.data),
            reason=f'selector={msg.data.strip() or "<empty>"}',
        )

    def _set_capture_fourcc(self) -> None:
        self.cap.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*self.capture_fourcc[:4]),
        )

    def _apply_low_profile(self, reason: str, log: bool = True) -> None:
        profile = self.low_profile
        self.low_res_width = profile['width']
        self.low_res_height = profile['height']
        self.low_res_period = 1.0 / profile['fps']
        self.jpeg_quality = profile['jpeg_quality']

        if self.capture_fourcc:
            self._set_capture_fourcc()
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.low_res_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.low_res_height)
        self.cap.set(cv2.CAP_PROP_FPS, profile['fps'])
        if log:
            self.get_logger().info(
                f'{reason}: low profile '
                f'{self.low_res_width}x{self.low_res_height} '
                f'at {profile["fps"]:.1f} FPS, '
                f'JPEG quality {self.jpeg_quality}, '
                f'capture FOURCC {self.capture_fourcc}'
            )

    def _apply_high_res_profile(self) -> None:
        profile = self.high_res_profile
        if self.capture_fourcc:
            self._set_capture_fourcc()
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, profile['width'])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, profile['height'])
        self.cap.set(cv2.CAP_PROP_FPS, profile['fps'])

    def _set_high_res_selected(self, selected: bool, reason: str) -> None:
        if self.high_res_selected == selected:
            return

        self.high_res_selected = selected
        self.get_logger().info(
            f'{reason}: high-res capture '
            f'{"enabled" if selected else "disabled"}'
        )

    def _read_frame(self, context: str):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error(f'Unable to read {context} frame from camera')
            return None

        return frame

    def publish_frame(self) -> None:
        now = time.monotonic()
        if (
            self.high_res_selected
            and now - self._last_high_res_publish >= self.high_res_period
        ):
            self._publish_high_res_frame()
            return

        frame = self._read_frame('low-res')
        if frame is None:
            return

        stamp = self.get_clock().now().to_msg()

        if self.raw_publisher is not None:
            raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            raw_msg.header.stamp = stamp
            raw_msg.header.frame_id = self.frame_id
            self.raw_publisher.publish(raw_msg)

        if (
            self.low_res_publisher is None
            or now - self._last_low_res_publish < self.low_res_period
        ):
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

    def _publish_high_res_frame(self) -> None:
        self._apply_high_res_profile()

        frame = None
        for _ in range(self.high_res_drain_frames + 1):
            frame = self._read_frame('high-res')
            if frame is None:
                self._apply_low_profile(reason='restore low-res after failed high-res')
                self._last_high_res_publish = time.monotonic()
                return

        stamp = self.get_clock().now().to_msg()
        encode_params = [
            int(cv2.IMWRITE_JPEG_QUALITY),
            self.high_res_profile['jpeg_quality'],
        ]
        ok, encoded = cv2.imencode('.jpg', frame, encode_params)
        if not ok:
            self.get_logger().error('Unable to encode high-res camera frame')
            self._apply_low_profile(reason='restore low-res after encode failure')
            self._last_high_res_publish = time.monotonic()
            return

        high_msg = CompressedImage()
        high_msg.header.stamp = stamp
        high_msg.header.frame_id = self.frame_id
        high_msg.format = 'jpeg'
        high_msg.data = np.asarray(encoded).tobytes()
        self.high_res_publisher.publish(high_msg)
        self._last_high_res_publish = time.monotonic()
        self._apply_low_profile(
            reason='restore low-res after high-res frame',
            log=False,
        )

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
