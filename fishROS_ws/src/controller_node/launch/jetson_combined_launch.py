""" 
Launch File to set up Jetson nodes.
Launches the cameras, depth sensor, drive runner, stabilization, and the pH sensor

Run: colcon build
Then Run: ros2 launch controller_node jetson_combined_launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fish_cam',
            executable='DWE_exploreHD_pub',
        ),
        Node(
            package='fish_cam',
            executable='DWE_exploreHD_pub2',
        ),
        Node(
            package='bar02_pub',
            executable='bar02_pub',
        ),
        Node(
            package='controller_node',
            executable='drivetrain_node',
        ),
        Node(
            package='stabilization_pub',
            executable='stabilization_pub',
        ),
        Node(
            package='ph_pub',
            executable='ph_pub',
        ),            
    ])