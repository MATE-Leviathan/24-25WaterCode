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
from fish_operator_msgs.msg import ActuatorCommand
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
SERVO_LIMITS = {
    "20": (0.42, 0.54),
    "01": (0.45, 1.00),
}
ACTUATOR_DIRECTION_PIN = "21"
ACTUATOR_SPEED_PIN = "28"
ACTUATOR_SPEED = 1.0
OPERATOR_LINEAR_TIMEOUT_SEC = 0.4
SERVO_NAME_TO_PIN = {
    "servo_20": "20",
    "20": "20",
    "servo_01": "01",
    "01": "01",
}

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

        self.declare_parameter("enable_thrusters", True)
        self.declare_parameter("port", SERIAL_PORT)
        self.declare_parameter("baud", SERIAL_BAUD)

        self.enable_thrusters = bool(self.get_parameter("enable_thrusters").value)
        self.serial_port = str(self.get_parameter("port").value)
        self.serial_baud = int(self.get_parameter("baud").value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,  # or small value like 5
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.twist_sub = self.create_subscription(Twist, "twist", self.twist_callback, qos)
        self.stab_sub = self.create_subscription(Twist, "stabilization", self.stabilization_callback, 10)
        self.aux_sub = self.create_subscription(Point, "aux_control", self.aux_callback, 10)
        self.operator_aux_sub = self.create_subscription(
            ActuatorCommand,
            "operator/actuator_command",
            self.operator_aux_callback,
            10,
        )
        
        self.stabilization = 0.0
        self.last_stabilization_time = self.get_clock().now()
        self.stabilization_timeout_sec = 0.5
        self.actuator_state = "stop"
        self.operator_linear_active = False
        self.last_operator_linear_time = self.get_clock().now()
        self.servo_positions = {
            "20": None,
            "01": None,
        }
        self.servo_values = {
            "20": None,
            "01": None,
        }

        self.serial_conn = serial.Serial(self.serial_port, self.serial_baud, timeout=1)
        self.thruster_values = [0.0] * 6
        time.sleep(3)
        self.get_logger().info(
            f'Using serial actuator control on {self.serial_port} @ {self.serial_baud}'
        )

        if self.enable_thrusters:
            self.drivetrainInit()
        else:
            self.get_logger().warn(
                'Thruster output disabled; auxiliary servos and linear actuator remain enabled'
            )
        self.operator_watchdog = self.create_timer(0.1, self.operator_timeout_check)


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
        if not self.enable_thrusters:
            return

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
        if not self.enable_thrusters:
            return

        if self.serial_conn is None or not self.serial_conn.is_open:
            self.get_logger().info(f'Serial conn {self.serial_conn}, is open {self.serial_conn.is_open}')
            return

        cmd = ""
        for pin, value in zip(MOTOR_PINS, self.thruster_values):
            cmd += f"z{int(pin):02d}{self._format_motor_value(value)}x\n"
        #self.get_logger().info(f'Not running thruster {index}: {value}')
        self.serial_conn.write(cmd.encode())

    def write_aux_command(self, pin, value):
        if self.serial_conn is None or not self.serial_conn.is_open:
            return

        value = max(0.0, min(1.0, float(value)))
        self.serial_conn.write(f"z{str(pin).zfill(2)}{value:04.2f}x\n".encode())

    def set_actuator_state(self, state):
        if state == self.actuator_state:
            return

        if state == "forward":
            self.write_aux_command(ACTUATOR_DIRECTION_PIN, 1.0)
            self.write_aux_command(ACTUATOR_SPEED_PIN, ACTUATOR_SPEED)
        elif state == "back":
            self.write_aux_command(ACTUATOR_DIRECTION_PIN, 0.0)
            self.write_aux_command(ACTUATOR_SPEED_PIN, ACTUATOR_SPEED)
        else:
            self.write_aux_command(ACTUATOR_DIRECTION_PIN, 0.5)
            self.write_aux_command(ACTUATOR_SPEED_PIN, 0.0)

        self.actuator_state = state
        self.get_logger().info(f'Actuator state: {state}')

    def set_linear_actuator_value(self, value):
        value = max(-1.0, min(1.0, float(value)))
        speed = min(1.0, abs(value) * ACTUATOR_SPEED)

        if value > CONTROLLER_DEADZONE:
            self.write_aux_command(ACTUATOR_DIRECTION_PIN, 1.0)
            self.write_aux_command(ACTUATOR_SPEED_PIN, speed)
            state = "forward"
        elif value < -CONTROLLER_DEADZONE:
            self.write_aux_command(ACTUATOR_DIRECTION_PIN, 0.0)
            self.write_aux_command(ACTUATOR_SPEED_PIN, speed)
            state = "back"
        else:
            self.write_aux_command(ACTUATOR_DIRECTION_PIN, 0.5)
            self.write_aux_command(ACTUATOR_SPEED_PIN, 0.0)
            state = "stop"

        if state != self.actuator_state:
            self.actuator_state = state
            self.get_logger().info(f'Operator actuator state: {state}')

    def set_aux_servo(self, pin, at_max):
        if self.servo_positions[pin] == at_max:
            return

        servo_min, servo_max = SERVO_LIMITS[pin]
        value = servo_max if at_max else servo_min
        self.set_aux_servo_value(pin, 1.0 if at_max else 0.0)
        self.servo_positions[pin] = at_max
        state = "max" if at_max else "min"
        self.get_logger().info(f'Servo {pin} toggled to {state} ({value:.2f})')

    def set_aux_servo_value(self, pin, normalized):
        if pin not in SERVO_LIMITS:
            self.get_logger().warn(f'Unknown servo pin: {pin}')
            return

        normalized = max(0.0, min(1.0, float(normalized)))
        if self.servo_values.get(pin) == normalized:
            return

        servo_min, servo_max = SERVO_LIMITS[pin]
        value = servo_min + ((servo_max - servo_min) * normalized)
        self.write_aux_command(pin, value)
        self.servo_values[pin] = normalized
        self.servo_positions[pin] = normalized >= 0.5
        self.get_logger().info(
            f'Operator servo {pin}: {normalized:.2f} ({value:.2f})'
        )

    def twist_callback(self, msg):
        if not self.enable_thrusters:
            return

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

    def aux_callback(self, msg: Point):
        if msg.x > 0.5:
            self.set_actuator_state("forward")
        elif msg.x < -0.5:
            self.set_actuator_state("back")
        else:
            self.set_actuator_state("stop")

        self.set_aux_servo_value("20", msg.y)
        self.set_aux_servo_value("01", msg.z)

    def operator_aux_callback(self, msg: ActuatorCommand):
        actuator_type = msg.actuator_type.strip().lower()
        name = msg.name.strip()

        if actuator_type == "stop_all":
            self.operator_linear_active = False
            self.set_linear_actuator_value(0.0)
            for pin in SERVO_LIMITS:
                self.set_aux_servo_value(pin, 0.0)
            return

        if actuator_type == "servo":
            pin = SERVO_NAME_TO_PIN.get(name)
            if pin is None:
                self.get_logger().warn(f'Unknown servo command name: {name}')
                return
            self.set_aux_servo_value(pin, msg.value)
            return

        if actuator_type == "linear":
            self.last_operator_linear_time = self.get_clock().now()
            self.operator_linear_active = abs(msg.value) > CONTROLLER_DEADZONE
            self.set_linear_actuator_value(msg.value)
            return

        self.get_logger().warn(f'Unknown actuator type: {msg.actuator_type}')

    def operator_timeout_check(self):
        if not self.operator_linear_active:
            return

        elapsed = (
            self.get_clock().now() - self.last_operator_linear_time
        ).nanoseconds * 1e-9
        if elapsed > OPERATOR_LINEAR_TIMEOUT_SEC:
            self.operator_linear_active = False
            self.set_linear_actuator_value(0.0)
            self.get_logger().warn('Operator linear actuator command timed out')

    def destroy_node(self):
        if self.serial_conn is not None and self.serial_conn.is_open:
            self.set_actuator_state("stop")
            if self.enable_thrusters:
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
