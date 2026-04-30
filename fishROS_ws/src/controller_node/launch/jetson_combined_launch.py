"""
Launch File to set up Jetson nodes.

Run: colcon build
Then Run: ros2 launch controller_node jetson_combined_launch.py
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    all_cameras_launch = PythonLaunchDescriptionSource(
        [
            get_package_share_directory("fish_cam"),
            "/launch/all_cameras.launch.py",
        ]
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(all_cameras_launch),
            Node(
                package="controller_node",
                executable="drivetrain_node",
                output="screen",
            ),
            ExecuteProcess(
                cmd=["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"],
                output="screen",
            ),
        ]
    )
