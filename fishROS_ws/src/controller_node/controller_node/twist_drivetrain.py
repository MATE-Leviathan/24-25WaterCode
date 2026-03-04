"""
Author(s): Everett Tucker
Date Created: March 6, 2024
Description: Controls the MATE ROV and Claw with a twist message and Point message, respectively
Subscribers: Point, Imu, Twist
Publishers: None
TODO:
add depth subscriber for hovering
"""

from operator import index

import rclpy
import time
import math
import busio
import serial
from board import SCL, SDA
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy

# Final Global Variables
MOTOR_PINS = [0, 1, 2, 3, 6, 7]
ONEOVERROOTTWO = 1 / math.sqrt(2)
CONTROLLER_DEADZONE = 0.05
THRUST_SCALE_FACTOR = 0.8 #0.6 #0.83375
INITAL_CLAW_Y = 0 # should actually be x rotation but I'm too lazy to change it
INITIAL_CLAW_Z = 0
SERIAL_PORT = '/dev/ttyACM0'
SERIAL_BAUD = 115200

# Dynamic Global Variables
global imu_init, orientation, linear_acceleration, angular_velocity
imu_init = False
orientation = Quaternion()
linear_acceleration = Vector3()
angular_velocity = Vector3()


class DriveRunner(Node):
    def __init__(self):
        # Creating the node and subscriber
        super().__init__("drive_runner")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,  # or small value like 5
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.twist_sub = self.create_subscription(Twist, "twist", self.twist_callback, qos)
        self.stab_sub = self.create_subscription(Twist, "stabilization", self.stabilization_callback, 10)
        
        self.stabilization = 0.0
        self.last_stabilization_time = self.get_clock().now()
        self.stabilization_timeout_sec = 0.5

        self.serial_conn = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
        self.thruster_values = [0.0] * 6
        time.sleep(3)
        self.get_logger().info(f'Using serial motor control on {SERIAL_PORT} @ {SERIAL_BAUD}')

        self.drivetrainInit()


    def drivetrainInit(self):
        # Setting thrusters to initialization angles for 7 seconds
        print("Initializing Thrusters... Make sure to hit both triggers before 6 seconds! Otherwise will not work!")
        for i in range(6):
            self.set_thruster(i, 0.0)
        self.flush_thrusters()
        time.sleep(3)
        for i in range(6):
            self.set_thruster(i, 0.0)
        self.flush_thrusters()
        print("Ready!")

    def set_thruster(self, index, value):
        value = min(max(value, -1), 1)  # Keeping it in bounds
        value = value if value < 0 else value * THRUST_SCALE_FACTOR
        self.thruster_values[index] = value
        self.get_logger().info(f'Thruster {index}: {value}')

    def _format_motor_value(self, value):
        normalized = max(0.0, min(1.0, 0.5 + (0.5 * value)))
        value_str = f"{normalized:.2f}"
        if len(value_str) == 4:
            value_str = f"0{value_str}"
        return value_str

    def flush_thrusters(self):
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.get_logger().info(f'Serial conn {self.serial_conn}, is open {self.serial_conn.is_open}')
            return

        cmd = ""
        for pin, value in zip(MOTOR_PINS, self.thruster_values):
            cmd += f"z{int(pin):02d}{self._format_motor_value(value)}x\n"
        #self.get_logger().info(f'Not running thruster {index}: {value}')
        self.serial_conn.write(cmd.encode())

    def twist_callback(self, msg):
        self.get_logger().info(f'Recieved Twist: {msg}')   
        x = msg.linear.x
        y = msg.linear.y
        z = msg.linear.z
        x_rotation = msg.angular.x
        z_rotation = msg.angular.z
        ### Horizontal Motor Writing
        if abs(x) > CONTROLLER_DEADZONE or abs(y) > CONTROLLER_DEADZONE: # Linear Movement in XY
            self.set_thruster(5, -ONEOVERROOTTWO * (x - y)) # RB
            self.set_thruster(0, ONEOVERROOTTWO * (x - y)) # LF
            self.set_thruster(3, -ONEOVERROOTTWO * (-y - x)) # RF
            self.set_thruster(2, ONEOVERROOTTWO * (-y - x)) # LB
        elif abs(z_rotation) > CONTROLLER_DEADZONE:  # Yaw (Spin)
            self.set_thruster(5, z_rotation * 0.75)
            self.set_thruster(0, z_rotation * 0.75)
            self.set_thruster(3, -z_rotation * 0.75)
            self.set_thruster(2, -z_rotation * 0.75)
        else:
            self.set_thruster(5, 0.0)
            self.set_thruster(0, 0.0)
            self.set_thruster(3, 0.0)
            self.set_thruster(2, 0.0)

        ### Vertical Motor Writing
        if abs(z) > CONTROLLER_DEADZONE:  # Linear Movement in Z
            self.set_thruster(1, -z) # LU
            self.set_thruster(4, -z) # RU
        elif abs(x_rotation) > CONTROLLER_DEADZONE:  # Roll
            self.set_thruster(1, x_rotation)
            self.set_thruster(4, -x_rotation)
        # Depth Hover with timeout
        elif (self.get_clock().now() - self.last_stabilization_time).nanoseconds * 1e-9 < self.stabilization_timeout_sec:
            self.set_thruster(1, self.stabilization)
            self.set_thruster(4, self.stabilization)
        else:
            self.set_thruster(1, 0.0)
            self.set_thruster(4, 0.0)
    
        self.flush_thrusters()

    def stabilization_callback(self, msg: Twist):
        self.stabilization = msg.linear.z
        self.last_stabilization_time = self.get_clock().now()

    def destroy_node(self):
        if self.serial_conn is not None and self.serial_conn.is_open:
            for i in range(6):
                self.set_thruster(i, 0.0)
            self.flush_thrusters()
            self.serial_conn.close()
        super().destroy_node()

class IMUSub(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(Imu, 'IMUData', self.imu_callback, 10)

    def imu_callback(self, msg):
        global orientation, linear_acceleration, angular_velocity, imu_init
        orientation = msg.orientation
        linear_acceleration = msg.linear_acceleration
        angular_velocity = msg.angular_velocity
        imu_init = True      


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    drive_runner = DriveRunner()
    imu_sub = IMUSub()
    executor.add_node(drive_runner)
    executor.add_node(imu_sub)

    # Starting the execution loop
    executor.spin()

    # Destroying Nodes
    drive_runner.destroy_node()
    imu_sub.destroy_node()

    # Shutting down the program
    rclpy.shutdown()

if __name__ == '__main__':
    main()
