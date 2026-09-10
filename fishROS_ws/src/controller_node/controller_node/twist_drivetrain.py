"""
Author(s): Everett Tucker
Date Created: March 6, 2024
Description: Drives the thrusters from twist messages over serial to the Pico
Subscribers: twist, stabilization
Publishers: None
"""

import rclpy
import time
import math
import serial
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy

# Final Global Variables
# Thruster IDs are indexed in the same order as MOTOR_PINS.
BACK_LEFT = 0      # z0000.50x
MIDDLE_RIGHT = 1   # z0100.50x
FRONT_RIGHT = 2    # z0200.50x
MIDDLE_LEFT = 3    # z0300.50x
BACK_RIGHT = 4     # z0600.50x
FRONT_LEFT = 5     # z0700.50x
MOTOR_PINS = [0, 1, 2, 3, 6, 7]
ONEOVERROOTTWO = 1 / math.sqrt(2)
CONTROLLER_DEADZONE = 0.05
THRUST_SCALE_FACTOR = 0.8 #0.6 #0.83375
INITAL_CLAW_Y = 0 # should actually be x rotation but I'm too lazy to change it
INITIAL_CLAW_Z = 0
SERIAL_PORT = '/dev/ttyACM1'
SERIAL_BAUD = 115200


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

        # Zero every thruster if twists stop arriving (tether drop, topside crash)
        self.declare_parameter('twist_timeout', 0.5)
        self.twist_timeout_sec = float(self.get_parameter('twist_timeout').value)
        self.last_twist_time = None
        self.twist_stale = True
        self.watchdog_timer = self.create_timer(0.1, self.watchdog)

        # /dev/ttyACM* numbering follows USB enumeration order, so on the robot
        # pass a /dev/serial/by-id/ path for the thruster Pico instead
        self.declare_parameter('port', SERIAL_PORT)
        self.declare_parameter('baud', SERIAL_BAUD)
        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)

        # write_timeout keeps a Pico that stops reading from blocking the executor
        self.serial_conn = serial.Serial(self.port, self.baud, timeout=1, write_timeout=0.1)
        self.thruster_values = [0.0] * 6
        time.sleep(3)
        self.get_logger().info(f'Using serial motor control on {self.port} @ {self.baud}')

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

    def set_thrusters_scaled(self, values):
        # Scale the group down together so combined inputs keep their ratio
        # instead of clipping one thruster
        peak = max(1.0, max(abs(v) for v in values.values()))
        for index, value in values.items():
            self.set_thruster(index, value / peak)

    def _format_motor_value(self, value):
        normalized = max(0.0, min(1.0, 0.5 + (0.5 * value)))
        value_str = f"{normalized:.2f}"
        if len(value_str) == 4:
            value_str = f"0{value_str}"
        return value_str

    def reopen_serial(self):
        try:
            self.serial_conn.open()
            self.get_logger().info(f'Reopened {self.port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open {self.port}: {e}', throttle_duration_sec=1.0)

    def flush_thrusters(self):
        if not self.serial_conn.is_open:
            self.reopen_serial()
            if not self.serial_conn.is_open:
                return

        cmd = ""
        for pin, value in zip(MOTOR_PINS, self.thruster_values):
            cmd += f"z{int(pin):02d}{self._format_motor_value(value)}x\n"
        try:
            self.serial_conn.write(cmd.encode())
        except serial.SerialTimeoutException:
            self.get_logger().warn('Pico is not reading, dropped a thruster frame', throttle_duration_sec=1.0)
        except serial.SerialException as e:
            # Pico unplugged or reset; close so the next flush tries to reopen
            self.get_logger().error(f'Thruster serial write failed: {e}', throttle_duration_sec=1.0)
            self.serial_conn.close()

    def stop_thrusters(self):
        for i in range(6):
            self.set_thruster(i, 0.0)
        self.flush_thrusters()

    def watchdog(self):
        now = self.get_clock().now()
        if self.last_twist_time is not None and \
                (now - self.last_twist_time).nanoseconds * 1e-9 < self.twist_timeout_sec:
            return
        if not self.twist_stale:
            self.get_logger().warn(f'No twist for {self.twist_timeout_sec}s, stopping thrusters')
            self.twist_stale = True
        self.stop_thrusters()

    def twist_callback(self, msg):
        self.last_twist_time = self.get_clock().now()
        if self.twist_stale:
            self.get_logger().info('Receiving twist, thrusters live')
            self.twist_stale = False
        self.get_logger().info(f'Recieved Twist: {msg}')   
        x = msg.linear.x
        y = msg.linear.y
        z = msg.linear.z
        x_rotation = msg.angular.x
        z_rotation = msg.angular.z
        if abs(x) <= CONTROLLER_DEADZONE and abs(y) <= CONTROLLER_DEADZONE:
            x = y = 0.0
        if abs(z_rotation) <= CONTROLLER_DEADZONE:
            z_rotation = 0.0
        if abs(z) <= CONTROLLER_DEADZONE:
            z = 0.0
        if abs(x_rotation) <= CONTROLLER_DEADZONE:
            x_rotation = 0.0

        ### Horizontal Motor Writing: translation in XY plus yaw
        self.set_thrusters_scaled({
            BACK_RIGHT: -ONEOVERROOTTWO * (x - y) + z_rotation * 0.75,
            FRONT_LEFT: ONEOVERROOTTWO * (x - y) + z_rotation * 0.75,
            FRONT_RIGHT: -ONEOVERROOTTWO * (-y - x) - z_rotation * 0.75,
            BACK_LEFT: ONEOVERROOTTWO * (-y - x) - z_rotation * 0.75,
        })

        ### Vertical Motor Writing: linear Z plus roll
        if z != 0.0 or x_rotation != 0.0:
            self.set_thrusters_scaled({
                MIDDLE_LEFT: -z + x_rotation,
                MIDDLE_RIGHT: -z - x_rotation,
            })
        # Depth Hover with timeout
        elif (self.get_clock().now() - self.last_stabilization_time).nanoseconds * 1e-9 < self.stabilization_timeout_sec:
            self.set_thruster(MIDDLE_LEFT, self.stabilization)
            self.set_thruster(MIDDLE_RIGHT, self.stabilization)
        else:
            self.set_thruster(MIDDLE_LEFT, 0.0)
            self.set_thruster(MIDDLE_RIGHT, 0.0)
    
        self.flush_thrusters()

    def stabilization_callback(self, msg: Twist):
        self.stabilization = msg.linear.z
        self.last_stabilization_time = self.get_clock().now()

    def close(self):
        if self.serial_conn.is_open:
            self.stop_thrusters()
            self.serial_conn.close()

def main(args=None):
    rclpy.init(args=args)

    drive_runner = DriveRunner()

    try:
        rclpy.spin(drive_runner)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Runs on Ctrl-C, launch shutdown and callback exceptions alike
        drive_runner.close()
        drive_runner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
