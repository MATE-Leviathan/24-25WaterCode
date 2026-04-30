"""
Authors(s): Everett Tucker
Creation Date: 01/10/2024
Description: Publishes data from the inu temperature sensor
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import adafruit_bno055
import board

# This is unused for now
BAUD_RATE = 115200

class TempPub(Node):
    def __init__(self):
        super().__init__('internal_temperature_publisher')
        self.publisher = self.create_publisher(Int32, 'InternalTempData', 10)

        i2c = board.I2C()

        self.sensor = adafruit_bno055.BNO055_I2C(i2c, address=0x28)
        timer_period = 0.02  # Should be 50 Hz
        self.timer = self.create_timer(timer_period, self.publish_temp_data)

    
    def publish_temp_data(self):
        temp = self.sensor.temperature

        if not temp:
            self.get_logger().error("Failed reading the IMU Temperature Data")

        int_msg = Int32()
        int_msg.data = temp

        self.publisher.publish(int_msg)


def main(args=None):
    rclpy.init(args=args)
    temp_publisher = TempPub()
    rclpy.spin(temp_publisher)
    temp_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

