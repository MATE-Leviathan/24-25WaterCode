"""
Date Created: 04/27/2024
Author(s): Everett Tucker
Description: Publishes orientation data from an Adafruit BNO055 IMU
Subscribers: None
Publishers: IMU
"""


import rclpy
import busio
import time
import adafruit_bno055
import tf_transformations
from rclpy.node import Node
from sensor_msgs import Imu
from geometry_msgs import Quaternion


class IMUPub():

    def __init__(self):
        # Creating the sensor

        # There are some special arguments that go in this function
        i2c = busio.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(i2c, address=0x28)

        # Creating the Publisher
        super().__init__("imu_publisher")
        self.publisher = self.create_publisher(Imu, 'imu', 10)

        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.publishIMU)
    

    def publishIMU():
        msg = Imu()
        data = self.sensor.euler
        msg.orientation = tf_transformations.quaterion_from_euler(data[0], data[1], data[2])

        # This needs more work and some debugging before it is good

        self.publisher.publish(msg)


def main():
    rclpy.init(args=args)
    imu_publisher = IMUPub()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
