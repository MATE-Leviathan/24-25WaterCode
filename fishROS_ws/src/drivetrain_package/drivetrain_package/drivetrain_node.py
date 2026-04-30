"""
Author(s): Everett Tucker
Date Created: 1/10/2024
Description: This is the main ros node for driving the robot.
TODO: 
* Create an auto stabalization system using the IMU data
* Add subscribers for the depth sensor and external temp sensor
* Test this in the water!
"""

import rclpy
import time
import math
import busio
import threading
from rclpy.node import Node
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from board import SCL, SDA
from sensor_msgs.msg import Joy, Imu
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

# Defining Dynamic Global Variables
thrusters = []
buttons = []
axes = []
joy_init = False
imu_init = False
orientation = Quaternion()
linear_acceleration = Vector3()
angular_velocity = Vector3()
sensitivity = 1

# Defining Final Global Variables
DELTA_SENSITIVITY = 0.0001  # This controls how fast the sensitivity changes
MOTOR_DEADZONE = 5  # This is the smallest difference in angle for the motors to start turning
CONTROLLER_DEADZONE = 0.01  # This is the smallest controller input that will be acted upon
ONEOVERROOTTWO = 1 / math.sqrt(2)  # Small speedup for linear movement calculations
MIDPOINT_ANGLE = 90  # This is the stationary angle
ANGLE_RANGE = 90  # This is the range from midpoint to max/min thrust

# Defining Thruster Pins
FRONTRIGHT = 0 # Index the front right thruster will be in the array
MIDRIGHT = 5 # Index the middle right thruster will be in the array
BACKRIGHT = 4 # Index the back right thruster will be in the array
FRONTLEFT = 2 # Index the front left thruster will be in the array
MIDLEFT = 3 # Index the middle left thruster will be in the array
BACKLEFT = 1 # Index the back left thruster will be in the array


class DriveRunner(Node):
    def __init__(self, node_name, use_global_arguments=False):
        super().__init__(node_name)
        threading.Thread(target=self.drive).start()
        global logger
        logger = self.get_logger()


    def drive(self):
        self.get_logger().info(f'Drive running with IMU {imu_init} and Joy {joy_init}')
        while rclpy.ok() and joy_init and imu_init:
            # Code to drive the robot goes here
            # Use global variables to get controller and sensor data

            # Changing Controller Sensitivity
            global sensitivity
            if buttons[0] == 1:
                sensitivity = max(0, sensitivity - DELTA_SENSITIVITY)
            elif buttons[1] == 1:
                sensitivity = min(1, sensitivity + DELTA_SENSITIVITY)

            # Writing joystick inputs
            # There is no manual way to adjust roll, for now

            # This if statement sets the z_rotation from both triggers
            if axes[2] < axes[5]:
                z_rotation = (axes[2] - 1) / 2
            else:
                z_rotation = -(axes[5] - 1) / 2

            logger.info(f'IMU TEST: {orientation.x}')
            logger.info(f'JOY TEST: {axes[1]}')
            write(axes[0], axes[1], axes[4], 0, 0, z_rotation)


class ControllerSub(Node):
    def __init__(self, node_name, use_global_arguments=False):
        super().__init__(node_name)
        self.subscription = self.create_subscription(Joy, 'joy', self.controller_callback, 10)

    def controller_callback(self, msg):
        global axes, buttons, joy_init
        axes = msg.axes
        buttons = msg.buttons
        joy_init = True


class IMUSub(Node):
    def __init__(self, node_name):
        super().__init__(node_name, use_global_arguments=False)
        self.subscription = self.create_subscription(Imu, 'IMUData', self.imu_callback, 10)

    def imu_callback(self, msg):
        global orientation, linear_acceleration, angular_velocity, imu_init
        orientation = msg.orientation
        linear_acceleration = msg.linear_acceleration
        angular_velocity = msg.angular_velocity
        imu_init = True


def drivetrain_init():
    # Initializing the PCA Board
    i2c_bus = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c_bus)
    pca.frequency = 450

    # Thruster Variables
    NUM_MOTORS = 6
    START_PIN = 8
    MIN_PULSE = 1340
    MAX_PULSE = 1870

    # Creating the Thrusters
    for i in range(NUM_MOTORS):
        thrusters.append(servo.Servo(pca.channels[START_PIN + i], min_pulse=MIN_PULSE, max_pulse=MAX_PULSE))

    # Initializing Thrusters
    for thruster in thrusters:
        thruster.angle = MIDPOINT_ANGLE

    time.sleep(7)


"""
Maps values in [-1, 1] to angles in [0, 180] with respect to sensitivity
@param val the float value that you want to transform into an angle
@param sensitivity should be a float in [0, 1], higher sensitivity means that controller inputs are mapped to a smaller range
"""
def map_float_to_angle(val):
    angle = (val * ANGLE_RANGE) * sensitivity
    if angle > MOTOR_DEADZONE:
        return min(angle + MIDPOINT_ANGLE, 180)
    elif angle < MOTOR_DEADZONE:
        return max(angle + MIDPOINT_ANGLE, 0)
    else:
        return MIDPOINT_ANGLE


"""
Writes linear and rotation movements in three dimensions
Each movement is given as a float in [-1, 1], which corrosponds to max_reverse and max_forward thrust, respectively
This layout is with respect to the right-hand-rule, so x is forward, y is right, and z is down by default
Furthermore, x rotation is roll, y is pitch, and z is yaw
Z rotation is clockwise by definition
X rotation is counterclockwise by definition
"""
def write(x, y, z, x_rotation, y_rotation, z_rotation):
    ### Horizontal Motor Writing
    if abs(x) > CONTROLLER_DEADZONE or abs(y) > CONTROLLER_DEADZONE:  # Linear Movement in XY
        x *= -1  # Reversal to compensate for the non-standard basis
        set_thruster(5, -1 * ONEOVERROOTTWO * (x + y)) # 5
        set_thruster(3, ONEOVERROOTTWO * (x - y)) # 3
        set_thruster(0, ONEOVERROOTTWO * (y - x)) # 0
        set_thruster(1, ONEOVERROOTTWO * (y + x)) # 1
    elif abs(z_rotation) > CONTROLLER_DEADZONE:  # Yaw (Spin)
        set_thruster(5, -z_rotation) # 5
        set_thruster(3, z_rotation) # 3
        set_thruster(1, -z_rotation) # 1
        set_thruster(0, z_rotation) # 0

    ### Vertical Motor Writing
    if abs(z) > CONTROLLER_DEADZONE:  # Linear Movement in Z
        set_thruster(2, z) # 2
        set_thruster(4, z) # 4
    elif abs(x_rotation) > CONTROLLER_DEADZONE:  # Roll
        set_thruster(2, x_rotation) # 2
        set_thruster(4, -x_rotation) # 4


"""
Sets the specified thruster with the given angle with respect to the change in acceleration
TODO: Add a compensator to prevent large changes in motor speed
This likely involves keeping track of the previous angle and only changing it by some threshold
"""
def set_thruster(index, value):
    thrusters[index].angle = map_float_to_angle(value)


def main(args=None):
    # Initializing instances
    rclpy.init(args=args)
    drivetrain_init()

    # Creating a thread executor
    executor = MultiThreadedExecutor()
    executor.add_node(ControllerSub('controller_subscriber'))
    executor.add_node(IMUSub('imu_subscriber'))
    executor.add_node(DriveRunner('drive_runner'))

    # Starting the thread
    executor.spin()

    # Shutting down
    rclpy.shutdown()


if __name__ == '__main__':
    main()
