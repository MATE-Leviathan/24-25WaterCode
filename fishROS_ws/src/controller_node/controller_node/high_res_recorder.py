"""Record selected high-res camera frames to a Jetson-local video file."""

from __future__ import annotations

import os
import re
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from fish_operator_msgs.msg import RecordingStatus
from fish_operator_msgs.srv import StartHighResRecording, StopHighResRecording
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


def _safe_name(value: str, fallback: str) -> str:
    value = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip())
    return value.strip("_") or fallback


class HighResRecorder(Node):
    def __init__(self) -> None:
        super().__init__("high_res_recorder")

        self.declare_parameter("output_dir", "~/high_res_recordings")
        self.declare_parameter("camera_names", ["front", "bottom", "left", "right"])
        self.declare_parameter("high_res_selector_topic", "/fish_cam/high_res_camera")
        self.declare_parameter("high_res_topic_suffix", "image_high/compressed")
        self.declare_parameter("recording_fps", 1.0)
        self.declare_parameter("video_codec", "mp4v")

        self.output_dir = os.path.expanduser(str(self.get_parameter("output_dir").value))
        self.camera_names = {
            str(camera) for camera in self.get_parameter("camera_names").value
        }
        self.topic_suffix = str(self.get_parameter("high_res_topic_suffix").value)
        self.recording_fps = max(0.1, float(self.get_parameter("recording_fps").value))
        self.video_codec = str(self.get_parameter("video_codec").value)[:4]

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.selector_pub = self.create_publisher(
            String,
            str(self.get_parameter("high_res_selector_topic").value),
            10,
        )
        self.status_pub = self.create_publisher(
            RecordingStatus,
            "high_res_recording/status",
            10,
        )
        self.start_srv = self.create_service(
            StartHighResRecording,
            "high_res_recording/start",
            self.start_recording,
        )
        self.stop_srv = self.create_service(
            StopHighResRecording,
            "high_res_recording/stop",
            self.stop_recording,
        )

        self.sensor_qos = sensor_qos
        self.subscription = None
        self.writer: Optional[cv2.VideoWriter] = None
        self.active_camera = ""
        self.mode = ""
        self.file_path = ""
        self.frame_count = 0
        self.started_at = 0.0
        self.last_error = ""

        self.status_timer = self.create_timer(1.0, self.publish_status)
        os.makedirs(self.output_dir, exist_ok=True)
        self.get_logger().info(f"High-res recordings directory: {self.output_dir}")

    def start_recording(self, request, response):
        camera = request.camera.strip()
        if not camera:
            response.success = False
            response.message = "camera is required"
            response.file_path = ""
            return response

        if self.camera_names and camera not in self.camera_names:
            response.success = False
            response.message = f"unknown camera '{camera}'"
            response.file_path = ""
            return response

        if self.active_camera:
            response.success = False
            response.message = f"already capturing {self.active_camera}"
            response.file_path = self.file_path
            return response

        mode = request.mode.strip().lower() or "video"
        if mode not in {"video", "image"}:
            response.success = False
            response.message = "mode must be 'video' or 'image'"
            response.file_path = ""
            return response

        label = _safe_name(request.label, "recording")
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        extension = "mp4" if mode == "video" else "jpg"
        self.file_path = os.path.join(
            self.output_dir,
            f"{timestamp}_{_safe_name(camera, 'camera')}_{label}.{extension}",
        )
        self.active_camera = camera
        self.mode = mode
        self.frame_count = 0
        self.started_at = time.monotonic()
        self.last_error = ""

        topic = f"/{camera}/{self.topic_suffix.lstrip('/')}"
        self.subscription = self.create_subscription(
            CompressedImage,
            topic,
            self.frame_callback,
            self.sensor_qos,
        )
        self.selector_pub.publish(String(data=camera))
        self.get_logger().info(f"Capturing {topic} to {self.file_path}")

        response.success = True
        response.message = f"{mode} capture started"
        response.file_path = self.file_path
        self.publish_status()
        return response

    def stop_recording(self, request, response):
        if not self.active_camera:
            response.success = False
            response.message = "not recording"
            response.file_path = ""
            response.frame_count = 0
            return response

        file_path = self.file_path
        frame_count = self.frame_count
        self._finish_recording(clear_error=False)

        response.success = True
        response.message = "capture stopped"
        response.file_path = file_path
        response.frame_count = frame_count
        self.publish_status()
        return response

    def frame_callback(self, msg: CompressedImage) -> None:
        if not self.active_camera:
            return

        data = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if frame is None:
            self.last_error = "failed to decode high-res frame"
            self.get_logger().error(self.last_error)
            return

        if self.mode == "image":
            if not cv2.imwrite(self.file_path, frame):
                self.last_error = f"failed to write image {self.file_path}"
                self.get_logger().error(self.last_error)
                self._finish_recording(clear_error=False)
                return

            self.frame_count = 1
            self.get_logger().info(f"Saved high-res image {self.file_path}")
            self._finish_recording(clear_error=False)
            self.publish_status()
            return

        if self.writer is None:
            height, width = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*self.video_codec)
            self.writer = cv2.VideoWriter(
                self.file_path,
                fourcc,
                self.recording_fps,
                (width, height),
            )
            if not self.writer.isOpened():
                self.last_error = f"failed to open video writer for {self.file_path}"
                self.get_logger().error(self.last_error)
                self._finish_recording(clear_error=False)
                return

        self.writer.write(frame)
        self.frame_count += 1

    def publish_status(self) -> None:
        msg = RecordingStatus()
        msg.active = bool(self.active_camera)
        msg.mode = self.mode
        msg.camera = self.active_camera
        msg.file_path = self.file_path
        msg.frame_count = self.frame_count
        msg.elapsed_sec = (
            float(time.monotonic() - self.started_at) if self.active_camera else 0.0
        )
        msg.error = self.last_error
        self.status_pub.publish(msg)

    def _finish_recording(self, clear_error: bool = True) -> None:
        self.selector_pub.publish(String(data=""))

        if self.subscription is not None:
            self.destroy_subscription(self.subscription)
            self.subscription = None

        if self.writer is not None:
            self.writer.release()
            self.writer = None

        self.active_camera = ""
        self.mode = ""
        self.started_at = 0.0
        if clear_error:
            self.last_error = ""

    def destroy_node(self) -> bool:
        if self.active_camera or self.writer is not None:
            self._finish_recording(clear_error=False)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HighResRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
