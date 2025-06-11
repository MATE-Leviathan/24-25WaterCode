from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ph_pub',
            node_namespace='ph_pub',
            node_executable='ph_pub.py',
            node_name='ph_pub')])
