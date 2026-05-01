"""
Launch joystick control for Pico servos and the linear actuator.

Run:
  ros2 launch controller_node auxiliary_control_launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")

    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("baud", default_value="115200"),
            Node(
                package="joy_linux",
                executable="joy_linux_node",
            ),
            Node(
                package="controller_node",
                executable="auxiliary_control",
                parameters=[
                    {
                        "port": port,
                        "baud": ParameterValue(baud, value_type=int),
                    }
                ],
            ),
        ]
    )
