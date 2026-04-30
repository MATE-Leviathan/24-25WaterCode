""" 
Launch File to set up Topside nodes.
Launches the joystick and controller translator.

Run: colcon build
Then Run: ros2 launch controller_node topside_combined_launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            output='screen',
        ),
        Node(
            package='controller_node',
            executable='controller_node',
            output='screen',
        ),
    ])
