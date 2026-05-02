"""
Launch File to set up Jetson nodes without thruster output.

Run: colcon build
Then Run: ros2 launch controller_node jetson_combined_launch.py
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    all_cameras_launch = PythonLaunchDescriptionSource(
        [
            get_package_share_directory("fish_cam"),
            "/launch/all_cameras.launch.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_thrusters",
                default_value="false",
                description="Set true only when you want joystick/controller input to drive thrusters.",
            ),
            DeclareLaunchArgument(
                "pico_port",
                default_value="/dev/ttyACM0",
                description="Serial device for the Pico that controls auxiliary outputs and thrusters.",
            ),
            DeclareLaunchArgument(
                "pico_baud",
                default_value="115200",
                description="Serial baud rate for the Pico controller.",
            ),
            IncludeLaunchDescription(all_cameras_launch),
            Node(
                package="controller_node",
                executable="drivetrain_node",
                output="screen",
                parameters=[
                    {
                        "enable_thrusters": ParameterValue(
                            LaunchConfiguration("enable_thrusters"),
                            value_type=bool,
                        ),
                        "port": LaunchConfiguration("pico_port"),
                        "baud": ParameterValue(
                            LaunchConfiguration("pico_baud"),
                            value_type=int,
                        ),
                    }
                ],
            ),
            Node(
                package="controller_node",
                executable="high_res_recorder",
                output="screen",
            ),
            ExecuteProcess(
                cmd=["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"],
                output="screen",
            ),
        ]
    )
