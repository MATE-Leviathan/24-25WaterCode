"""Publish water-quality voltages from an ADS1115 ADC."""

from __future__ import annotations

import importlib

import rclpy
from leviathan_sensor_msgs.msg import WaterQualityVoltages
from rclpy.node import Node


class ADS1115WaterQualityPublisher(Node):
    """Read TDS, turbidity, and pH channels from one ADS1115."""

    def __init__(self) -> None:
        super().__init__('ads1115_water_quality_publisher')

        self.declare_parameter('topic', '/sensors/water_quality/voltages')
        self.declare_parameter('frame_id', 'ads1115')
        self.declare_parameter('sample_period', 0.5)
        self.declare_parameter('i2c_scl_pin', 'SCL')
        self.declare_parameter('i2c_sda_pin', 'SDA')
        self.declare_parameter('i2c_address', 0x48)
        self.declare_parameter('gain', 1.0)
        self.declare_parameter('tds_channel', 0)
        self.declare_parameter('turbidity_channel', 1)
        self.declare_parameter('ph_channel', 2)

        topic = str(self.get_parameter('topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        sample_period = float(self.get_parameter('sample_period').value)
        if sample_period <= 0.0:
            raise ValueError('sample_period must be greater than 0')

        self.publisher = self.create_publisher(WaterQualityVoltages, topic, 10)
        self._init_ads1115()
        self.timer = self.create_timer(sample_period, self.timer_callback)

        self.get_logger().info(
            f'Publishing ADS1115 water-quality voltages on {topic}'
        )

    def _init_ads1115(self) -> None:
        try:
            board = importlib.import_module('board')
            busio = importlib.import_module('busio')
            ads_module = importlib.import_module(
                'adafruit_ads1x15.ads1115'
            )
            analog_module = importlib.import_module(
                'adafruit_ads1x15.analog_in'
            )
        except ImportError as exc:
            raise RuntimeError(
                'Missing ADS1115 dependencies. Install board, busio, and '
                'adafruit-circuitpython-ads1x15 on the Jetson.'
            ) from exc

        scl_pin_name = str(self.get_parameter('i2c_scl_pin').value)
        sda_pin_name = str(self.get_parameter('i2c_sda_pin').value)
        scl_pin = self._resolve_board_pin(board, scl_pin_name)
        sda_pin = self._resolve_board_pin(board, sda_pin_name)

        address = int(self.get_parameter('i2c_address').value)
        gain = float(self.get_parameter('gain').value)
        i2c = busio.I2C(scl_pin, sda_pin)
        self.ads = ads_module.ADS1115(i2c, address=address)
        self.ads.gain = gain

        analog_in = analog_module.AnalogIn
        self.tds_channel = analog_in(
            self.ads,
            self._resolve_ads_channel(
                int(self.get_parameter('tds_channel').value)
            ),
        )
        self.turbidity_channel = analog_in(
            self.ads,
            self._resolve_ads_channel(
                int(self.get_parameter('turbidity_channel').value)
            ),
        )
        self.ph_channel = analog_in(
            self.ads,
            self._resolve_ads_channel(
                int(self.get_parameter('ph_channel').value)
            ),
        )

    @staticmethod
    def _resolve_board_pin(board, pin_name: str):
        try:
            return getattr(board, pin_name)
        except AttributeError as exc:
            raise ValueError(
                f'board.{pin_name} is not available on this platform'
            ) from exc

    @staticmethod
    def _resolve_ads_channel(channel: int) -> int:
        if channel < 0 or channel > 3:
            raise ValueError('ADS1115 channel must be between 0 and 3')
        return channel

    def timer_callback(self) -> None:
        try:
            msg = WaterQualityVoltages()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.tds_voltage = float(self.tds_channel.voltage)
            msg.turbidity_voltage = float(self.turbidity_channel.voltage)
            msg.ph_voltage = float(self.ph_channel.voltage)
            msg.tds_raw = int(self.tds_channel.value)
            msg.turbidity_raw = int(self.turbidity_channel.value)
            msg.ph_raw = int(self.ph_channel.value)
            self.publisher.publish(msg)
        except Exception as exc:
            self.get_logger().error(f'Failed to read ADS1115: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ADS1115WaterQualityPublisher()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
