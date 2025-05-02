from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_publisher',
            node_namespace='imu_pub',
            node_executable='imu_pub.py',
            node_name='imu_pub')])
