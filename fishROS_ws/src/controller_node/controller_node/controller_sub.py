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

JOY_TIMEOUT = 0.5  # seconds without /joy before sending a zero twist

# Global dynamic variables
controller_init = False
last_joy_time = 0.0
axes = []
buttons = []
sensitivity = 1
holding = False # if depth holding is on
old_press = 0
left_trigger_axis = 2
right_trigger_axis = 5


# Gamepads differ in how many axes and buttons they report, so read
# missing ones as released instead of raising IndexError in a callback
def axis_value(index):
    if 0 <= index < len(axes):
        return axes[index]
    return 0.0


def button_value(index):
    if 0 <= index < len(buttons):
        return buttons[index]
    return 0


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

        global controller_init, axes, buttons, sensitivity, old_press, holding, last_joy_time
        axes = msg.axes
        buttons = msg.buttons
        last_joy_time = time.monotonic()
        controller_init = True

        # Modifing sensitivity, A turns on low sensitivity mode, B turns it off
        if button_value(0) == 1:
            sensitivity = LOW_SENSITIVITY
        if button_value(1) == 1:
            sensitivity = HIGH_SENSITIVITY

        # Toggle depth holding with X button
        if button_value(2) == 1 and old_press == 0:
            print("X Button Pressed")
            old_press = 1
            holding = not holding
            print(f"Holding = {holding}")
            
            # Publish toggle state
            self.hold_pub.publish(Bool(data=holding))
            
        if button_value(2) == 0:
            old_press = 0

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
        self.joy_stale = False

    def trigger_amount(self, value):
        return min(max((1.0 - value) / 2.0, 0.0), 1.0)

    def trigger_axis_value(self, index):
        value = axis_value(index)
        if abs(value) > 0.01:
            self.trigger_axes_initialized[index] = True
        elif not self.trigger_axes_initialized.get(index, False):
            return 1.0
        return value

    def publishTwist(self):
        # axes[0] left stick x
        # axes[1] left stick y 
        # Trigger axes rest at 1, pass through 0, and are fully pulled at -1.

        if not controller_init:
            return

        # joy_linux repeats the last state at autorepeat_rate, so silence means
        # the gamepad or joy node is gone. Send zeros rather than replaying the
        # last stick position.
        if time.monotonic() - last_joy_time > JOY_TIMEOUT:
            if not self.joy_stale:
                self.get_logger().warn(f'No joy input for {JOY_TIMEOUT}s, sending zero twist')
                self.joy_stale = True
            self.publisher.publish(Twist())
            return
        if self.joy_stale:
            self.get_logger().info('Joy input back')
            self.joy_stale = False

        twist_message = Twist()

        # Linear Motion - (x, y, z), scaling inputs by sensitivity
        twist_message.linear.x = axis_value(1) * sensitivity
        twist_message.linear.y = axis_value(0) * sensitivity

        left_trigger_raw = axis_value(left_trigger_axis)
        right_trigger_raw = axis_value(right_trigger_axis)
        left_trigger = self.trigger_amount(
            self.trigger_axis_value(left_trigger_axis)
        )
        right_trigger = self.trigger_amount(
            self.trigger_axis_value(right_trigger_axis)
        )
        linear_z = left_trigger - right_trigger

        self.get_logger().debug(
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
        twist_message.angular.z = -axis_value(3) * 0.8 * sensitivity
        
        self.publisher.publish(twist_message)


class PointPub(Node):
    def __init__(self):
        super().__init__("point_publisher")
        self.publisher = self.create_publisher(Point, "claw", 10)
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.publishPoint)


    def publishPoint(self):
        if controller_init:
            point_message = Point()

            point_message.x = 0.0  # Not needed, left as 0
            point_message.y = float(button_value(4) - button_value(5))  # Rotate claw
            point_message.z = axis_value(7)  # Open/close claw

            self.publisher.publish(point_message)


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
