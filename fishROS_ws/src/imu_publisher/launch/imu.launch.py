from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_publisher',
            namespace='imu_pub',
            executable='imu_pub.py',
            name='imu_pub')])
