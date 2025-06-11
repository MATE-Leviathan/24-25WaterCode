
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Launch the example.launch.py launch file."""
    ''' 
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('fish_cam'), 'launch'),
            '/DWEhd.launch.py'])
    )
    '''

    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('controller_node'), 'launch'),
            '/joy.launch.py'])
    )

    '''
    drivetrain_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('controller_node'), 'launch'),
            '/drivetrain_launch.py'
        ])
    )'''


    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('controller_node'), 'launch'),
            '/controller_launch.py'
        ])
    )
    
    return LaunchDescription([
        joy_launch,
        #drivetrain_launch,
        controller_launch
        #imu_launch,
    ])