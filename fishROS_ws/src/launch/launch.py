from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_node',
            node_namespace='controller_node',
            node_executable='controller_sub.py',
            node_name='controller_node')])


