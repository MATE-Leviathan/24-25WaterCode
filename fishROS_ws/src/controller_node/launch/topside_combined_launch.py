""" 
Launch File to set up Topside nodes.
Launches the joystick and controller translator.

Run: colcon build
Then Run: ros2 launch controller_node topside_combined_launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'jetson_ssh_target',
            default_value='ubuntu@jetson.local',
            description='SSH target used to scp captured images/videos topside.',
        ),
        DeclareLaunchArgument(
            'local_media_dir',
            default_value='~/fish_captures',
            description='Topside directory where copied captures are stored.',
        ),
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            output='screen',
        ),
        Node(
            package='controller_node',
            executable='controller_node',
            output='screen',
        ),
        Node(
            package='controller_node',
            executable='operator_dashboard',
            output='screen',
            parameters=[
                {
                    'jetson_ssh_target': LaunchConfiguration('jetson_ssh_target'),
                    'local_media_dir': LaunchConfiguration('local_media_dir'),
                }
            ],
        ),
    ])
