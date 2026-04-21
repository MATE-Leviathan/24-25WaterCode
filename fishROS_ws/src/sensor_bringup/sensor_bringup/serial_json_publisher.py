"""ROS 2 publisher for newline-delimited JSON from a serial device."""

from __future__ import annotations

import threading

import rclpy
from leviathan_sensor_msgs.msg import SerialReading
from rclpy.node import Node
import serial

from sensor_bringup.serial_parser import parse_serial_json_line


class SerialJsonPublisher(Node):
    """Read serial lines and publish parsed JSON plus the original line."""

    def __init__(self) -> None:
        super().__init__('serial_json_publisher')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('topic', '/sensors/serial')

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.timeout = float(self.get_parameter('timeout').value)
        topic = self.get_parameter('topic').value

        self.publisher = self.create_publisher(SerialReading, topic, 10)
        self._stop_event = threading.Event()

        try:
            self._serial = serial.Serial(
                self.port,
                self.baud,
                timeout=self.timeout,
            )
        except serial.SerialException as exc:
            self.get_logger().error(f'Failed to open {self.port}: {exc}')
            raise

        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f'Reading serial JSON from {self.port} @ {self.baud} baud'
        )

    def _read_loop(self) -> None:
        while rclpy.ok() and not self._stop_event.is_set():
            try:
                line = self._serial.readline()
            except serial.SerialException as exc:
                self.get_logger().error(f'Serial read error: {exc}')
                break

            if not line:
                continue

            raw_line = line.decode('utf-8', errors='replace').rstrip('\r\n')
            parsed = parse_serial_json_line(raw_line)

            msg = SerialReading()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'serial'
            msg.port = str(self.port)
            msg.baud = self.baud
            msg.raw_line = parsed.raw_line
            msg.raw_json = parsed.raw_json
            msg.numeric_keys = parsed.numeric_keys
            msg.numeric_values = parsed.numeric_values
            msg.string_keys = parsed.string_keys
            msg.string_values = parsed.string_values
            msg.parse_ok = parsed.parse_ok
            msg.error = parsed.error

            self.publisher.publish(msg)

    def destroy_node(self) -> bool:
        self._stop_event.set()
        if hasattr(self, '_thread') and self._thread.is_alive():
            self._thread.join(timeout=max(self.timeout, 0.1) + 0.2)
        if hasattr(self, '_serial') and self._serial.is_open:
            self._serial.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = SerialJsonPublisher()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
