from datetime import datetime
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _ensure_bag_parent(context):
    bag_output = LaunchConfiguration('bag_output').perform(context)
    bag_output = os.path.expanduser(bag_output)
    bag_parent = os.path.dirname(bag_output)
    if bag_parent:
        os.makedirs(bag_parent, exist_ok=True)
    return [SetLaunchConfiguration('bag_output', bag_output)]


def generate_launch_description():
    default_bag_output = os.path.expanduser(
        f'~/rosbags/leviathan_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
    )

    serial_port = LaunchConfiguration('serial_port')
    serial_baud = LaunchConfiguration('serial_baud')
    serial_timeout = LaunchConfiguration('serial_timeout')
    camera_device = LaunchConfiguration('camera_device')
    ads1115_topic = LaunchConfiguration('ads1115_topic')
    ads1115_sample_period = LaunchConfiguration('ads1115_sample_period')
    ads1115_scl_pin = LaunchConfiguration('ads1115_scl_pin')
    ads1115_sda_pin = LaunchConfiguration('ads1115_sda_pin')
    ads1115_i2c_address = LaunchConfiguration('ads1115_i2c_address')
    ads1115_gain = LaunchConfiguration('ads1115_gain')
    tds_channel = LaunchConfiguration('tds_channel')
    turbidity_channel = LaunchConfiguration('turbidity_channel')
    ph_channel = LaunchConfiguration('ph_channel')
    bag_output = LaunchConfiguration('bag_output')
    enable_sonar = LaunchConfiguration('enable_sonar')
    enable_ads1115_water_quality = LaunchConfiguration(
        'enable_ads1115_water_quality'
    )
    enable_foxglove = LaunchConfiguration('enable_foxglove')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('serial_baud', default_value='115200'),
        DeclareLaunchArgument('serial_timeout', default_value='1.0'),
        DeclareLaunchArgument('camera_device', default_value='0'),
        DeclareLaunchArgument(
            'ads1115_topic',
            default_value='/sensors/water_quality/voltages',
        ),
        DeclareLaunchArgument('ads1115_sample_period', default_value='0.5'),
        DeclareLaunchArgument('ads1115_scl_pin', default_value='SCL'),
        DeclareLaunchArgument('ads1115_sda_pin', default_value='SDA'),
        DeclareLaunchArgument('ads1115_i2c_address', default_value='72'),
        DeclareLaunchArgument('ads1115_gain', default_value='1.0'),
        DeclareLaunchArgument('tds_channel', default_value='0'),
        DeclareLaunchArgument('turbidity_channel', default_value='1'),
        DeclareLaunchArgument('ph_channel', default_value='2'),
        DeclareLaunchArgument('bag_output', default_value=default_bag_output),
        DeclareLaunchArgument('enable_sonar', default_value='false'),
        DeclareLaunchArgument(
            'enable_ads1115_water_quality',
            default_value='true',
        ),
        DeclareLaunchArgument('enable_foxglove', default_value='false'),
        OpaqueFunction(function=_ensure_bag_parent),

        Node(
            package='sensor_bringup',
            executable='serial_json_publisher',
            name='serial_json_publisher',
            parameters=[{
                'port': serial_port,
                'baud': ParameterValue(serial_baud, value_type=int),
                'timeout': ParameterValue(serial_timeout, value_type=float),
                'topic': '/sensors/serial',
            }],
            output='screen',
        ),
        Node(
            package='fish_cam',
            executable='dual_stream_camera',
            name='dual_stream_camera',
            parameters=[{
                'video_device_id': ParameterValue(
                    camera_device,
                    value_type=int,
                ),
                'raw_topic': '/camera/image_raw',
                'low_res_topic': '/camera/image_low/compressed',
                'low_res_fps': 10.0,
                'low_res_width': 640,
                'low_res_height': 480,
            }],
            output='screen',
        ),
        Node(
            package='bar02_pub',
            executable='bar02_pub',
            output='screen',
        ),
        Node(
            package='sensor_bringup',
            executable='ads1115_water_quality_publisher',
            name='ads1115_water_quality_publisher',
            parameters=[{
                'topic': ads1115_topic,
                'sample_period': ParameterValue(
                    ads1115_sample_period,
                    value_type=float,
                ),
                'i2c_scl_pin': ads1115_scl_pin,
                'i2c_sda_pin': ads1115_sda_pin,
                'i2c_address': ParameterValue(
                    ads1115_i2c_address,
                    value_type=int,
                ),
                'gain': ParameterValue(ads1115_gain, value_type=float),
                'tds_channel': ParameterValue(tds_channel, value_type=int),
                'turbidity_channel': ParameterValue(
                    turbidity_channel,
                    value_type=int,
                ),
                'ph_channel': ParameterValue(ph_channel, value_type=int),
            }],
            condition=IfCondition(enable_ads1115_water_quality),
            output='screen',
        ),
        Node(
            package='stabilization_pub',
            executable='stabilization_pub',
            output='screen',
        ),
        Node(
            package='controller_node',
            executable='drivetrain_node',
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'launch', 'sonoptix_sonar', 'echo.launch.py'],
            condition=IfCondition(enable_sonar),
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'launch', 'foxglove_bridge',
                 'foxglove_bridge_launch.xml'],
            condition=IfCondition(enable_foxglove),
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o', bag_output],
            output='screen',
        ),
    ])
