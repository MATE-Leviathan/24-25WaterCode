"""
Launch File to set up Jetson nodes.

Run: colcon build
Then Run: ros2 launch controller_node jetson_combined_launch.py
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
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
                default_value="true",
                description="Set false when you want auxiliary control without thruster output.",
            ),
            DeclareLaunchArgument(
                "enable_auxiliary",
                default_value="true",
                description="Set false to disable Pico auxiliary output.",
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
            DeclareLaunchArgument(
                "thruster_max_output",
                default_value="0.8",
                description="Maximum absolute thruster command.",
            ),
            DeclareLaunchArgument(
                "thruster_ramp_rate",
                default_value="0.5",
                description="Maximum thruster command change per second.",
            ),
            DeclareLaunchArgument(
                "enable_cameras",
                default_value="true",
                description="Set false to keep camera traffic off the control network.",
            ),
            DeclareLaunchArgument(
                "enable_foxglove",
                default_value="true",
                description="Set false to disable the Foxglove bridge.",
            ),
            DeclareLaunchArgument(
                "camera_width",
                default_value="320",
                description="Low-res camera stream width.",
            ),
            DeclareLaunchArgument(
                "camera_height",
                default_value="240",
                description="Low-res camera stream height.",
            ),
            DeclareLaunchArgument(
                "camera_fps",
                default_value="10.0",
                description="Low-res camera capture and publish FPS.",
            ),
            DeclareLaunchArgument(
                "jpeg_quality",
                default_value="40",
                description="Low-res camera JPEG quality.",
            ),
            IncludeLaunchDescription(
                all_cameras_launch,
                condition=IfCondition(LaunchConfiguration("enable_cameras")),
                launch_arguments={
                    "camera_width": LaunchConfiguration("camera_width"),
                    "camera_height": LaunchConfiguration("camera_height"),
                    "camera_fps": LaunchConfiguration("camera_fps"),
                    "jpeg_quality": LaunchConfiguration("jpeg_quality"),
                }.items(),
            ),
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
                        "enable_auxiliary": ParameterValue(
                            LaunchConfiguration("enable_auxiliary"),
                            value_type=bool,
                        ),
                        "port": LaunchConfiguration("pico_port"),
                        "baud": ParameterValue(
                            LaunchConfiguration("pico_baud"),
                            value_type=int,
                        ),
                        "thruster_max_output": ParameterValue(
                            LaunchConfiguration("thruster_max_output"),
                            value_type=float,
                        ),
                        "thruster_ramp_rate": ParameterValue(
                            LaunchConfiguration("thruster_ramp_rate"),
                            value_type=float,
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
                condition=IfCondition(LaunchConfiguration("enable_foxglove")),
                output="screen",
            ),
        ]
    )
