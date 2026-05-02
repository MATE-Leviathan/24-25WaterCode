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
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    left_trigger_axis = LaunchConfiguration('left_trigger_axis')
    right_trigger_axis = LaunchConfiguration('right_trigger_axis')
    dpad_horizontal_axis = LaunchConfiguration('dpad_horizontal_axis')

    return LaunchDescription([
        DeclareLaunchArgument(
            'jetson_ssh_target',
            default_value='ubuntu@10.49.2.100',
            description='SSH target used to scp captured images/videos topside.',
        ),
        DeclareLaunchArgument(
            'local_media_dir',
            default_value='~/fish_captures',
            description='Topside directory where copied captures are stored.',
        ),
        DeclareLaunchArgument(
            'left_trigger_axis',
            default_value='2',
            description='Joy axis index for the left trigger.',
        ),
        DeclareLaunchArgument(
            'right_trigger_axis',
            default_value='5',
            description='Joy axis index for the right trigger.',
        ),
        DeclareLaunchArgument(
            'dpad_horizontal_axis',
            default_value='6',
            description='Joy axis index for D-pad left/right claw rotation.',
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
            parameters=[
                {
                    'left_trigger_axis': ParameterValue(
                        left_trigger_axis,
                        value_type=int,
                    ),
                    'right_trigger_axis': ParameterValue(
                        right_trigger_axis,
                        value_type=int,
                    ),
                    'dpad_horizontal_axis': ParameterValue(
                        dpad_horizontal_axis,
                        value_type=int,
                    ),
                }
            ],
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
