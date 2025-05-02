from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pH_pub',
            node_namespace='pH_pub',
            node_executable='pH_pub.py',
            node_name='pH_pub')])
