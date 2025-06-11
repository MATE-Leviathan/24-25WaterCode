""" 
Launch File to set up Jetson nodes.
Launches the cameras, depth sensor, drive runner, stabilization, the pH sensor, and the foxglove websocket

Run: colcon build
Then Run: ros2 launch controller_node jetson_combined_launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

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
        ExecuteProcess(cmd=["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"]), # I found having the bridge on the jetson was faster
    ])