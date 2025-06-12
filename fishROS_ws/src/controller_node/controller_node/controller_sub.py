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

# Static global variables
LOW_SENSITIVITY = 0.5  # This is basically how much inputs are scaled when in sensitive mode
HIGH_SENSITIVITY = 1

# Global dynamic variables
controller_init = False
axes = []
buttons = []
sensitivity = 1
holding = False # if depth holding is on
old_press = 0


class ControllerSub(Node):

    def __init__(self):
        # Creating the subscriber
        super().__init__('controller_subscriber')
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
        Left Trigger: 2 - [pressed, not pressed] -> [-1, 1]
        Right Stick Horizontal: 3 - [left, right] -> [1, -1]
        Right Stick Vertical: 4 - [up, down] -> [1, -1]
        Right Trigger: 5 - [pressed, not pressed] -> [-1, 1]
        Pad Horizontal: 6 - left -> 1, right -> -1
        Pad Vertical: 7 - up -> 1, down -> -1
        """

        global controller_init, axes, buttons, sensitivity, old_press, holding, current_depth, hold_depth
        axes = msg.axes
        buttons = msg.buttons
        controller_init = True

        # Modifing sensitivity, A turns on low sensitivity mode, B turns it off
        if buttons[0] == 1:
            sensitivity = LOW_SENSITIVITY
        if buttons[1] == 1:
            sensitivity = HIGH_SENSITIVITY

        # Toggle depth holding with X button
        if buttons[2] == 1 and old_press == 0:
            print("X Button Pressed")
            old_press = 1
            holding = not holding
            print(f"Holding = {holding}")
            
            # Publish toggle state
            self.hold_pub.publish(Bool(data=holding))
            
        if buttons[2] == 0:
            old_press = 0

class TwistPub(Node):
    def __init__(self):
        # Creating the publisher
        super().__init__("twist_publisher")
        self.publisher = self.create_publisher(Twist, 'twist', 10)
        timer_period = 0.02  # 50 Hz
        self.timer = self.create_timer(timer_period, self.publishTwist)


    def publishTwist(self):
        global hold_depth

        if controller_init:
            twist_message = Twist()

            # Linear Motion - (x, y, z), scaling inputs by sensitivity
            twist_message.linear.x = axes[1] * sensitivity
            twist_message.linear.y = axes[0] * sensitivity

            if axes[2] < axes[5]:
                linear_z = (axes[2] - 1) / 2
            else:
                linear_z = -(axes[5] - 1) / 2

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
        self.publisher = self.create_publisher(Point, "claw", 10)
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.publishPoint)


    def publishPoint(self):
        if controller_init:
            point_message = Point()

            point_message.x = 0.0  # Not needed, left as 0
            point_message.y = float(buttons[4] - buttons[5]) # axes[7]  # Rotate claw
            point_message.z = axes[7]  # Open/close claw

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

