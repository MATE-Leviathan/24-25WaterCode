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


class ControllerSub(Node):

    def __init__(self):
        # Creating the subscriber
        super().__init__('controller_subscriber')
        global left_trigger_axis
        global right_trigger_axis

        self.declare_parameter('left_trigger_axis', left_trigger_axis)
        self.declare_parameter('right_trigger_axis', right_trigger_axis)

        left_trigger_axis = int(self.get_parameter('left_trigger_axis').value)
        right_trigger_axis = int(self.get_parameter('right_trigger_axis').value)

        self.subscription = self.create_subscription(Joy, 'joy', self.listener_callback, 10)
        self.hold_pub = self.create_publisher(Bool, 'stabilization_toggle', 10) # Bool publisher to toggle depth hold

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
        global servo_20_position, servo_01_position
        axes = msg.axes
        buttons = msg.buttons
        controller_init = True

        # Small claw on servo 20: A opens, B closes.
        if self.button_pressed(A_BUTTON):
            servo_20_position = SERVO_OPEN
        if self.button_pressed(B_BUTTON):
            servo_20_position = SERVO_CLOSED

        dpad_horizontal = self.axis_value(DPAD_HORIZONTAL_AXIS)
        if dpad_horizontal > 0.5:
            servo_01_position = SERVO_OPEN
        elif dpad_horizontal < -0.5:
            servo_01_position = SERVO_CLOSED
        else:
            servo_01_position = SERVO_STOP

        # Toggle depth holding with X button
        if self.button_pressed(X_BUTTON) and old_press == 0:
            print("X Button Pressed")
            old_press = 1
            holding = not holding
            print(f"Holding = {holding}")
            
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

            self.get_logger().info(
                f'Linear Z {linear_z} '
                f'raw triggers L={left_trigger_raw} R={right_trigger_raw}'
            )

            if abs(linear_z) > 0.08: # Deadzone
                twist_message.linear.z = linear_z
            else:
                twist_message.linear.z = 0.0

            # Angular Motion - Just yaw for now
            twist_message.angular.x = 0.0
            twist_message.angular.y = 0.0
            twist_message.angular.z = -axes[3] * 0.8 * sensitivity
            
            self.publisher.publish(twist_message)


class PointPub(Node):
    def __init__(self):
        super().__init__("point_publisher")
        self.publisher = self.create_publisher(Point, "aux_control", 10)
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.publishPoint)


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
