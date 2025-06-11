""" 
Launch File to set up Topside nodes.
Launches the joystick, controller sub, and foxglove

Run: colcon build
Then Run: ros2 launch controller_node topside_combined_launch.py
Then open a connection using Foxglove Websocket
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy_linux',
            executable='joy_linux_node',
        ),
        Node(
            package='controller_node',
            executable='controller_node',
        ),

        #ExecuteProcess(cmd=["foxglove-studio"]),
        ExecuteProcess(cmd=["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"]),
    ])