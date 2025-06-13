#!/usr/bin/env

"""
Date Created: 04/29/2025
Author(s): Larry Zhao
Description: Publishes pH data from the pH Sensor
Subscribers: None
Publishers: pH
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

class PHPub(Node):

    def __init__(self):
        super().__init__('pH_publisher')
        
        # Initialize ADS1115
        i2c = busio.I2C('GP13_I2C2_CLK', 'GP14_I2C2_DAT')
        self.ads = ADS.ADS1115(i2c)
        
        # Use Single-ended Mode (AIN0)
        self.channel = AnalogIn(self.ads, ADS.P3)
        
        self.publisher_ = self.create_publisher(Float32, 'pH', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        try:
            voltage = self.channel.voltage  # Get voltage reading
            
            msg = Float32()
            msg.data = round((-5.98*voltage)+16.1, 3) # pH from voltage
            
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published pH: {msg.data:.3f}')
        except Exception as e:
            self.get_logger().error(f'Failed to read pH sensor: {e}')


def main(args=None):
    rclpy.init(args=args)
    pH_publisher = PHPub()
    rclpy.spin(pH_publisher)
    pH_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
