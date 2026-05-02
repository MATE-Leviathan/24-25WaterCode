"""
Author(s): Everett Tucker, Larry Zhao
Creation Date: 01/09/2024
Description: Gets controller input from the joy publisher and send a twist message
Subscribers: Joy
Publishers: Twist, Point
"""


import rclpy
import time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# Static global variables
LOW_SENSITIVITY = 0.65  # This is basically how much inputs are scaled when in sensitive mode
HIGH_SENSITIVITY = 1
A_BUTTON = 0
B_BUTTON = 1
X_BUTTON = 2
LEFT_BUMPER = 4
RIGHT_BUMPER = 5
DPAD_HORIZONTAL_AXIS = 6
SERVO_OPEN = 0.0
SERVO_CLOSED = 1.0
SERVO_STOP = 0.5
SMALL_CLAW_RATE_PER_SEC = 0.25
CLAW_ROTATE_HORIZONTAL = 0.0
CLAW_ROTATE_VERTICAL = 1.0
LOG_PERIOD_SEC = 0.5

# Global dynamic variables
controller_init = False
axes = []
buttons = []
sensitivity = 1
holding = False # if depth holding is on
old_press = 0
servo_20_position = SERVO_OPEN
servo_01_position = SERVO_STOP
left_trigger_axis = 2
right_trigger_axis = 5
dpad_horizontal_axis = DPAD_HORIZONTAL_AXIS
dpad_horizontal_value = 0.0


class ControllerSub(Node):

    def __init__(self):
        # Creating the subscriber
        super().__init__('controller_subscriber')
        global left_trigger_axis
        global right_trigger_axis
        global dpad_horizontal_axis

        self.declare_parameter('left_trigger_axis', left_trigger_axis)
        self.declare_parameter('right_trigger_axis', right_trigger_axis)
        self.declare_parameter('dpad_horizontal_axis', dpad_horizontal_axis)

        left_trigger_axis = int(self.get_parameter('left_trigger_axis').value)
        right_trigger_axis = int(self.get_parameter('right_trigger_axis').value)
        dpad_horizontal_axis = int(self.get_parameter('dpad_horizontal_axis').value)
        self.get_logger().info(
            'Controller axes: '
            f'left_trigger={left_trigger_axis}, '
            f'right_trigger={right_trigger_axis}, '
            f'dpad_horizontal={dpad_horizontal_axis}'
        )

        self.subscription = self.create_subscription(Joy, 'joy', self.listener_callback, 10)
        self.hold_pub = self.create_publisher(Bool, 'stabilization_toggle', 10) # Bool publisher to toggle depth hold
        self.last_claw_update_time = self.get_clock().now()

    def listener_callback(self, msg):
        """
        msg.buttons contains the button inputs.
        Each index below returns 1 if pressed and 0 if not
        A: 0
        B: 1
        X: 2
        Y: 3
        L Bump: 4
        R Bump: 5
        View Button (Small Left): 6
        Menu Button (Small Right): 7
        Power/Xbox Button: 8
        Left Joystick Press: 9
        Right Joystick Press: 10

        msg.axes contains the float inputs
        Each index below returns a float in [-1, 1]

        Left Stick Horizontal: 0 - [left, right] -> [1, -1]
        Left Stick Vertical: 1 - [up, down] -> [1, -1]
        Left Trigger: 2
        Right Stick Horizontal: 3 - [left, right] -> [1, -1]
        Right Stick Vertical: 4 - [up, down] -> [1, -1]
        Right Trigger: 5
        Pad Horizontal: 6 - left -> 1, right -> -1
        Pad Vertical: 7 - up -> 1, down -> -1
        """

        global controller_init, axes, buttons, sensitivity, old_press, holding
        global servo_20_position, servo_01_position, dpad_horizontal_value
        axes = msg.axes
        buttons = msg.buttons
        controller_init = True

        now = self.get_clock().now()
        elapsed = (now - self.last_claw_update_time).nanoseconds * 1e-9
        self.last_claw_update_time = now

        # Small claw on servo 20: hold A to open, hold B to close slowly.
        if self.button_pressed(A_BUTTON) and not self.button_pressed(B_BUTTON):
            servo_20_position = max(
                SERVO_OPEN,
                servo_20_position - (SMALL_CLAW_RATE_PER_SEC * elapsed),
            )
        elif self.button_pressed(B_BUTTON) and not self.button_pressed(A_BUTTON):
            servo_20_position = min(
                SERVO_CLOSED,
                servo_20_position + (SMALL_CLAW_RATE_PER_SEC * elapsed),
            )

        dpad_horizontal = self.axis_value(dpad_horizontal_axis)
        dpad_horizontal_value = dpad_horizontal
        if dpad_horizontal > 0.5:
            servo_01_position = CLAW_ROTATE_HORIZONTAL
        elif dpad_horizontal < -0.5:
            servo_01_position = CLAW_ROTATE_VERTICAL

        # Toggle depth holding with X button
        if self.button_pressed(X_BUTTON) and old_press == 0:
            old_press = 1
            holding = not holding
            self.get_logger().info(f"Depth hold toggled: {holding}")
            
            # Publish toggle state
            self.hold_pub.publish(Bool(data=holding))
            
        if not self.button_pressed(X_BUTTON):
            old_press = 0

    def button_pressed(self, index):
        return 0 <= index < len(buttons) and buttons[index] == 1

    def axis_value(self, index):
        if 0 <= index < len(axes):
            return axes[index]
        return 0.0

class TwistPub(Node):
    def __init__(self):
        # Creating the publisher
        super().__init__("twist_publisher")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,  # or small value like 5
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(Twist, 'twist', qos)
        timer_period = 0.02  # 50 Hz
        self.timer = self.create_timer(timer_period, self.publishTwist)
        self.trigger_axes_initialized = {}
        self.last_log_time = self.get_clock().now()

    def axis_value(self, index):
        if 0 <= index < len(axes):
            return axes[index]
        return 0.0

    def trigger_amount(self, value):
        return min(max((1.0 - value) / 2.0, 0.0), 1.0)

    def trigger_axis_value(self, index):
        value = self.axis_value(index)
        if abs(value) > 0.01:
            self.trigger_axes_initialized[index] = True
        elif not self.trigger_axes_initialized.get(index, False):
            return 1.0
        return value

    def should_log(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_log_time).nanoseconds * 1e-9
        if elapsed < LOG_PERIOD_SEC:
            return False
        self.last_log_time = now
        return True

    def publishTwist(self):
        global hold_depth
        # axes[0] left stick x
        # axes[1] left stick y 
        # Trigger axes rest at 1, pass through 0, and are fully pulled at -1.

        if controller_init:
            twist_message = Twist()

            # Linear Motion - (x, y, z), scaling inputs by sensitivity
            twist_message.linear.x = axes[1] * sensitivity
            twist_message.linear.y = axes[0] * sensitivity

            left_trigger_raw = self.axis_value(left_trigger_axis)
            right_trigger_raw = self.axis_value(right_trigger_axis)
            left_trigger = self.trigger_amount(
                self.trigger_axis_value(left_trigger_axis)
            )
            right_trigger = self.trigger_amount(
                self.trigger_axis_value(right_trigger_axis)
            )
            linear_z = left_trigger - right_trigger

            if abs(linear_z) > 0.08: # Deadzone
                twist_message.linear.z = linear_z
            else:
                twist_message.linear.z = 0.0

            # Angular Motion - Just yaw for now
            twist_message.angular.x = 0.0
            twist_message.angular.y = 0.0
            twist_message.angular.z = -axes[3] * 0.8 * sensitivity

            if self.should_log():
                self.get_logger().info(
                    'Controller twist: '
                    f'LT(axis {left_trigger_axis}) raw={left_trigger_raw:.2f} '
                    f'amt={left_trigger:.2f}, '
                    f'RT(axis {right_trigger_axis}) raw={right_trigger_raw:.2f} '
                    f'amt={right_trigger:.2f}, '
                    f'linear_z={twist_message.linear.z:.2f}, '
                    f'yaw={twist_message.angular.z:.2f}'
                )
            
            self.publisher.publish(twist_message)


class PointPub(Node):
    def __init__(self):
        super().__init__("point_publisher")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(Point, "aux_control", qos)
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.publishPoint)
        self.last_log_time = self.get_clock().now()

    def should_log(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_log_time).nanoseconds * 1e-9
        if elapsed < LOG_PERIOD_SEC:
            return False
        self.last_log_time = now
        return True


    def publishPoint(self):
        if controller_init:
            point_message = Point()

            left_bumper_pressed = self.button_pressed(LEFT_BUMPER)
            right_bumper_pressed = self.button_pressed(RIGHT_BUMPER)

            if right_bumper_pressed and not left_bumper_pressed:
                point_message.x = 1.0
            elif left_bumper_pressed and not right_bumper_pressed:
                point_message.x = -1.0
            else:
                point_message.x = 0.0

            point_message.y = servo_20_position
            point_message.z = servo_01_position

            if self.should_log():
                self.get_logger().info(
                    'Controller aux: '
                    f'linear_actuator={point_message.x:.1f}, '
                    f'claw={point_message.y:.1f}, '
                    f'claw_rotate={point_message.z:.1f}, '
                    f'dpad_axis={dpad_horizontal_axis}, '
                    f'dpad_raw={dpad_horizontal_value:.2f}'
                )

            self.publisher.publish(point_message)

    def button_pressed(self, index):
        return 0 <= index < len(buttons) and buttons[index] == 1


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    controller_sub = ControllerSub()
    twist_pub = TwistPub()
    point_pub = PointPub()

    executor.add_node(controller_sub)
    executor.add_node(twist_pub)
    executor.add_node(point_pub)

    # Launching the executor
    executor.spin()

    # Destroying Nodes
    controller_sub.destroy_node()
    twist_pub.destroy_node()
    point_pub.destroy_node()

    # Shutting down the program
    rclpy.shutdown()


if __name__ == '__main__':
    main()
