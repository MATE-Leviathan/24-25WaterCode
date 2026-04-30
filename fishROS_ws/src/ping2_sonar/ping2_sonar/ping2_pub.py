"""
Author(s): Christopher Holley, Everett Tucker
Creation Date: 09/07/2023
Description: Handles the interfacing with the Ping2 sonar and publishing the data to the topic 'Data'
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import numpy as np
import adafruit_bno055
import board
import tf_transformations
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

# shouldn't need to change this
BAUD_RATE = 115200
# @TODO Make this use a launch file
DEVICE = "/dev/ttyUSB0"


class ImuPub(Node):
    """
    The Ping2SonarPub object represents the publisher node for the ping2 sonar. 
    It handles interfacing with the ping2 sonar and publishing the data to the topic 'Data'

    Args:
        None

    Attributes:
        publisher: ROS publisher object, publishes Ping2 messages to the topic 'Data'
        ping_sonar: Ping1D object, used to get data from the ping2 sonar
        timer: ROS timer object, used to call the read_and_publish_data function at exactly 50hz

    Special Cases:
        None
    """
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(Imu, 'IMUData', 10)

        i2c = board.I2C()

        self.sensor = adafruit_bno055.BNO055_I2C(i2c, address=0x28)
        print(f'Current Temperature: {self.sensor.temperature}')
        timer_period = 0.02 # seconds, should be 50hz
        self.timer = self.create_timer(timer_period, self.read_and_publish_imu_data)            

    """
    Reads in the most recent data from the ping2 sonar and publishes it to the topic 'Data'
    Publishes Quaternion Orientation, Linear Acceleration, and Angular Velocity

    Args:
        None

    Returns:
        None

    Raises:
        Does not raise but will log error if unable to read frame from ping2
    """
    def read_and_publish_imu_data(self):
        # Getting data from the sensor
        euler = self.sensor.euler
        l_accel = self.sensor.linear_acceleration
        a_vel = self.sensor.gyro

        # Checking data
        if not euler:
            self.get_logger().error("Failed reading IMU Euler Orientation Data")
        if not l_accel:
            self.get_logger().error("Failed reading IMU Linear Acceleration Data")
        if not a_vel:
            self.get_logger().error("Failed reading IMU Angular Velocity Data")

        imu_msg = Imu()

        # Adding quaternion orientation [x, y, z, w]
        arr = tf_transformations.quaternion_from_euler(euler[0], euler[1], euler[2])
        imu_msg.orientation = Quaternion(x=arr[0], y=arr[1], z=arr[2], w=arr[3])

        # Adding linear acceleration (m/s^2)
        linear_acceleration = Vector3()
        linear_acceleration.x = l_accel[0]
        linear_acceleration.y = l_accel[1]
        linear_acceleration.z = l_accel[2]
        imu_msg.linear_acceleration = linear_acceleration

        # Adding angluar velocity (rad/sec)
        angular_velocity = Vector3()
        angular_velocity.x = a_vel[0]
        angular_velocity.y = a_vel[1]
        angular_velocity.z = a_vel[2]
        imu_msg.angular_velocity = angular_velocity
        
        # Publish the IMU message
        self.publisher.publish(imu_msg)        


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = ImuPub()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()
