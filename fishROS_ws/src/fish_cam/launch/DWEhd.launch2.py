from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.SetParameter(name='video_device_id', value=4),
        Node(
            package='fish_cam',
            namespace='bottom_cam',
            executable='DWE_exploreHD_pub2.py',
            name='DWE_exploreHD_pub2',
            parameters=[
                {"video_device_id": 4}
            ]
        ),
    ])
