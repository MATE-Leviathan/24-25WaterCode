from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_node',
            namespace='',
            executable='controller_node'
            )])

