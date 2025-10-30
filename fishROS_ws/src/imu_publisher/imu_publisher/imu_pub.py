#!/usr/bin/env

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
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

from board import SCL, SDA

class IMUPub(Node):

    def __init__(self):
        # Creating the sensor

        # There are some special arguments that go in this function
        self.i2c = busio.I2C(SCL, SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c, address=0x28)

        # Creating the Publisher
        super().__init__("imu_publisher")
        self.publisher = self.create_publisher(Imu, 'imu', 10)

        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.publishIMU)
    

    def publishIMU(self):
        msg = Imu()
        data = self.sensor.euler
        print(data[0], data[1], data[2])
        quaternion_msg = Quaternion()
        # roll, pitch, yaw
        q = tf_transformations.quaternion_from_euler(data[0], data[1], data[2])

        quaternion_msg.x = q[0]
        quaternion_msg.y = q[1]
        quaternion_msg.z = q[2]
        quaternion_msg.w = q[3]
        msg.orientation = quaternion_msg
        self.publisher.publish(msg)


def main():
    rclpy.init()
    imu_publisher = IMUPub()
    rclpy.spin(imu_publisher) #imu_publisher
    imu_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
