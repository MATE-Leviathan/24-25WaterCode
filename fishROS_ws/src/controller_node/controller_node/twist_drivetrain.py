"""
Author(s): Everett Tucker
Date Created: March 6, 2024
Description: Controls the MATE ROV drivetrain from Twist messages.
Subscribers: Imu, Twist
Publishers: None
TODO:
add depth subscriber for hovering
"""

import rclpy
import time
import math
import serial
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy

# Final Global Variables
# Logical thruster order: LF, LU, LB, RF, RU, RB.
# Physical mapping: front left=26, mid left=03, back left=11,
# front right=15, mid right=16, back right=07.
MOTOR_PINS = ["26", "03", "11", "15", "16", "07"]
ONEOVERROOTTWO = 1 / math.sqrt(2)
CONTROLLER_DEADZONE = 0.05
THRUST_SCALE_FACTOR = 0.8 #0.6 #0.83375
SERIAL_PORT = '/dev/ttyACM0'
SERIAL_BAUD = 115200
DRIVETRAIN_LOG_PERIOD_SEC = 0.5
THRUSTER_MAX_OUTPUT = 0.8
THRUSTER_RAMP_RATE = 0.5
SERVO_LIMITS = {
    "20": (0.42, 0.60),
    "01": (0.58, 0.77),
}
ACTUATOR_DIRECTION_PIN = "21"
ACTUATOR_SPEED_PIN = "28"
ACTUATOR_SPEED = 1.0

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
        self.declare_parameter("enable_auxiliary", True)
        self.declare_parameter("port", SERIAL_PORT)
        self.declare_parameter("baud", SERIAL_BAUD)
        self.declare_parameter("thruster_max_output", THRUSTER_MAX_OUTPUT)
        self.declare_parameter("thruster_ramp_rate", THRUSTER_RAMP_RATE)
        self.declare_parameter("actuator_direction_pin", ACTUATOR_DIRECTION_PIN)
        self.declare_parameter("actuator_speed_pin", ACTUATOR_SPEED_PIN)
        self.declare_parameter("actuator_speed", ACTUATOR_SPEED)

        self.enable_thrusters = bool(self.get_parameter("enable_thrusters").value)
        self.enable_auxiliary = bool(self.get_parameter("enable_auxiliary").value)
        self.serial_port = str(self.get_parameter("port").value)
        self.serial_baud = int(self.get_parameter("baud").value)
        self.thruster_max_output = max(
            0.0, min(1.0, float(self.get_parameter("thruster_max_output").value))
        )
        self.thruster_ramp_rate = max(
            0.01, float(self.get_parameter("thruster_ramp_rate").value)
        )
        self.actuator_direction_pin = str(
            self.get_parameter("actuator_direction_pin").value
        ).zfill(2)
        self.actuator_speed_pin = str(
            self.get_parameter("actuator_speed_pin").value
        ).zfill(2)
        self.actuator_speed = max(
            0.0, min(1.0, float(self.get_parameter("actuator_speed").value))
        )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,  # or small value like 5
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.twist_sub = self.create_subscription(Twist, "twist", self.twist_callback, qos)
        self.stab_sub = self.create_subscription(Twist, "stabilization", self.stabilization_callback, 10)
        self.aux_sub = self.create_subscription(Point, "aux_control", self.aux_callback, qos)
        
        self.stabilization = 0.0
        self.last_stabilization_time = self.get_clock().now()
        self.stabilization_timeout_sec = 0.5

        self.serial_conn = serial.Serial(self.serial_port, self.serial_baud, timeout=1)
        self.thruster_values = [0.0] * 6
        self.target_thruster_values = [0.0] * 6
        self.last_thruster_ramp_time = self.get_clock().now()
        self.servo_values = {"20": None, "01": None}
        self.actuator_state = "stop"
        self.last_twist_log_time = self.get_clock().now()
        time.sleep(3)
        self.get_logger().info(
            f'Using Pico serial control on {self.serial_port} @ {self.serial_baud}'
        )
        self.get_logger().info(
            f'Thruster ramp rate: {self.thruster_ramp_rate:.2f} command units/sec'
        )
        self.get_logger().info(
            f'Thruster max output: {self.thruster_max_output:.2f}'
        )

        if self.enable_thrusters:
            self.drivetrainInit()
        else:
            self.get_logger().warn('Thruster output disabled')

        if self.enable_auxiliary:
            self.get_logger().info('Auxiliary output enabled on aux_control')
            self.stop_actuator()
        else:
            self.get_logger().warn('Auxiliary output disabled')

    def drivetrainInit(self):
        # Setting thrusters to initialization angles for 7 seconds
        self.get_logger().info(
            'Initializing thrusters. Pull both triggers during ESC calibration.'
        )
        for i in range(6):
            self.set_thruster(i, 0.0)
        self.flush_thrusters()
        time.sleep(3)
        for i in range(6):
            self.set_thruster(i, 0.0)
        self.flush_thrusters()
        self.get_logger().info('Thrusters ready')

    def set_thruster(self, index, value):
        if not self.enable_thrusters:
            return

        value = min(max(value, -1), 1)  # Keeping it in bounds
        value = value if value < 0 else value * THRUST_SCALE_FACTOR
        value = min(max(value, -self.thruster_max_output), self.thruster_max_output)
        self.target_thruster_values[index] = value

    def _format_motor_value(self, value):
        normalized = max(0.0, min(1.0, 0.5 + (0.5 * value)))
        return f"{normalized:.2f}"

    def flush_thrusters(self):
        if not self.enable_thrusters:
            return

        if self.serial_conn is None or not self.serial_conn.is_open:
            self.get_logger().warn(
                f'Serial conn {self.serial_conn}, is open {self.serial_conn.is_open}'
            )
            return

        self.ramp_thrusters()

        cmd = "z"
        for pin, value in zip(MOTOR_PINS, self.thruster_values):
            cmd += f"{int(pin):02d}{self._format_motor_value(value)}"
        cmd += "x\n"
        #self.get_logger().info(f'Not running thruster {index}: {value}')
        self.serial_conn.write(cmd.encode())

    def ramp_thrusters(self):
        now = self.get_clock().now()
        dt = (now - self.last_thruster_ramp_time).nanoseconds * 1e-9
        self.last_thruster_ramp_time = now

        if dt <= 0.0:
            return

        max_delta = self.thruster_ramp_rate * dt
        for index, target in enumerate(self.target_thruster_values):
            current = self.thruster_values[index]
            delta = target - current
            if abs(delta) <= max_delta:
                self.thruster_values[index] = target
            else:
                self.thruster_values[index] = current + math.copysign(max_delta, delta)

    def write_pico_command(self, pin, value):
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.get_logger().warn(
                f'Serial conn {self.serial_conn}, is open {self.serial_conn.is_open}'
            )
            return

        value = max(0.0, min(1.0, float(value)))
        command = f"z{str(pin).zfill(2)}{value:04.2f}x\n"
        self.serial_conn.write(command.encode())
        self.serial_conn.flush()

    def write_servo(self, pin, normalized):
        normalized = max(0.0, min(1.0, float(normalized)))
        if self.servo_values.get(pin) == normalized:
            return

        servo_min, servo_max = SERVO_LIMITS[pin]
        value = servo_min + ((servo_max - servo_min) * normalized)
        self.write_pico_command(pin, value)
        self.servo_values[pin] = normalized
        self.get_logger().info(f"Servo {pin}: {normalized:.2f} ({value:.2f})")

    def stop_actuator(self):
        self.write_pico_command(self.actuator_direction_pin, 0.5)
        self.write_pico_command(self.actuator_speed_pin, 0.0)

    def should_log_twist(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_twist_log_time).nanoseconds * 1e-9
        if elapsed < DRIVETRAIN_LOG_PERIOD_SEC:
            return False
        self.last_twist_log_time = now
        return True

    def twist_callback(self, msg):
        if not self.enable_thrusters:
            return

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

        if self.should_log_twist():
            self.get_logger().info(
                'Drive command: '
                f'x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={z_rotation:.2f}; '
                f'thrusters={",".join(f"{value:.2f}" for value in self.thruster_values)}; '
                f'target={",".join(f"{value:.2f}" for value in self.target_thruster_values)}'
            )

    def stabilization_callback(self, msg: Twist):
        self.stabilization = msg.linear.z
        self.last_stabilization_time = self.get_clock().now()

    def aux_callback(self, msg: Point):
        if not self.enable_auxiliary:
            return

        self.write_servo("20", msg.y)
        self.write_servo("01", msg.z)

        if msg.x > 0.5:
            next_state = "forward"
        elif msg.x < -0.5:
            next_state = "back"
        else:
            next_state = "stop"

        if next_state == self.actuator_state:
            return

        if next_state == "forward":
            self.write_pico_command(self.actuator_direction_pin, 1.0)
            self.write_pico_command(self.actuator_speed_pin, self.actuator_speed)
        elif next_state == "back":
            self.write_pico_command(self.actuator_direction_pin, 0.0)
            self.write_pico_command(self.actuator_speed_pin, self.actuator_speed)
        else:
            self.stop_actuator()

        self.actuator_state = next_state
        self.get_logger().info(f"Actuator state: {next_state}")

    def destroy_node(self):
        if self.serial_conn is not None and self.serial_conn.is_open:
            if self.enable_auxiliary:
                self.stop_actuator()
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
