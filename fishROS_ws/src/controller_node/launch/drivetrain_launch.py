from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_node',
            node_namespace='drivetrain_node',
            node_executable='twist_drivetrain.py',
            node_name='drivetrain_node')])
