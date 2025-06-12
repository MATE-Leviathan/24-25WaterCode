"""
Date Created: 04/29/2025
Author(s): Larry Zhao
Description: Publishes Depth and Temperature data from a Blue Robotics Bar02 sensor
Subscribers: None
Publishers: Depth and External Temp
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import ms5837

class Bar02Pub(Node):

    def __init__(self):
        # Creating the Publisher
        super().__init__('bar02_publisher')
        
        # Initialize MS5837 sensor on I2C bus 1 (i.e., /dev/i2c-0)
        self.sensor = ms5837.MS5837(ms5837.MODEL_02BA, 1)
        
        # Initialize sensor
        if not self.sensor.init():
            self.get_logger().error("Sensor could not be initialized")
            exit(1)
            
        self.sensor.setFluidDensity(1000) #1000 kg/m^3   pool water might be slightly denser
        
        # Publisher setup
        self.depth_pub = self.create_publisher(Float32, 'bar02/depth', 10)
        self.temp_pub = self.create_publisher(Float32, 'bar02/temp', 10)
        
        timer_period = 0.04 # 25 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)  # 2 Hz

    def timer_callback(self):
        if self.sensor.read(ms5837.OSR_2048):
            depth = round(self.sensor.depth(), 3)  # in meters, fluid density is freshwater by default
            temp = self.sensor.temperature()  # in °C
            
            depth_msg = Float32()
            depth_msg.data = depth
            
            temp_msg = Float32()
            temp_msg.data = temp
            
            self.depth_pub.publish(depth_msg)
            self.temp_pub.publish(temp_msg)
            
            #self.get_logger().info(f'Published depth: {depth:.3f} m, temp: {temp:.2f} °C')
            
        else:
            self.get_logger().warn("Failed to read from sensor")

def main(args=None):
    rclpy.init(args=args)
    bar02_publisher = Bar02Pub()
    rclpy.spin(bar02_publisher)
    bar02_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
