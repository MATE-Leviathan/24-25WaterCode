from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_camera(context):
    camera_name = LaunchConfiguration("camera_name").perform(context)
    video_device_id = int(LaunchConfiguration("video_device_id").perform(context))
    camera_width = int(LaunchConfiguration("camera_width").perform(context))
    camera_height = int(LaunchConfiguration("camera_height").perform(context))
    camera_fps = float(LaunchConfiguration("camera_fps").perform(context))
    jpeg_quality = int(LaunchConfiguration("jpeg_quality").perform(context))
    high_res_selector_topic = LaunchConfiguration("high_res_selector_topic").perform(
        context
    )
    high_res_camera = LaunchConfiguration("high_res_camera").perform(context)
    high_res_topic = LaunchConfiguration("high_res_topic").perform(context)
    high_camera_width = int(LaunchConfiguration("high_camera_width").perform(context))
    high_camera_height = int(LaunchConfiguration("high_camera_height").perform(context))
    high_camera_fps = float(LaunchConfiguration("high_camera_fps").perform(context))
    high_jpeg_quality = int(LaunchConfiguration("high_jpeg_quality").perform(context))
    high_res_drain_frames = int(
        LaunchConfiguration("high_res_drain_frames").perform(context)
    )

    return [
        Node(
            package="fish_cam",
            executable="dual_stream_camera",
            namespace=camera_name,
            name="camera",
            output="screen",
            parameters=[
                {
                    "video_device_id": video_device_id,
                    "frame_id": camera_name,
                    "capture_fourcc": "MJPG",
                    "raw_topic": "image_raw",
                    "low_res_topic": "image_low/compressed",
                    "publish_raw": False,
                    "publish_low_res": True,
                    "capture_fps": camera_fps,
                    "low_res_fps": camera_fps,
                    "low_res_width": camera_width,
                    "low_res_height": camera_height,
                    "jpeg_quality": jpeg_quality,
                    "selected_high_res_camera": high_res_camera,
                    "high_res_selector_topic": high_res_selector_topic,
                    "high_res_topic": high_res_topic,
                    "high_res_width": high_camera_width,
                    "high_res_height": high_camera_height,
                    "high_res_fps": high_camera_fps,
                    "high_res_jpeg_quality": high_jpeg_quality,
                    "high_res_drain_frames": high_res_drain_frames,
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_name",
                default_value="test_camera",
                description="ROS namespace and frame ID for this camera.",
            ),
            DeclareLaunchArgument(
                "video_device_id",
                default_value="0",
                description="/dev/video ID to open.",
            ),
            DeclareLaunchArgument(
                "camera_width",
                default_value="320",
                description="Requested camera capture and compressed output width.",
            ),
            DeclareLaunchArgument(
                "camera_height",
                default_value="240",
                description="Requested camera capture and compressed output height.",
            ),
            DeclareLaunchArgument(
                "camera_fps",
                default_value="25.0",
                description="Requested capture and compressed output FPS.",
            ),
            DeclareLaunchArgument(
                "jpeg_quality",
                default_value="60",
                description="Compressed JPEG quality.",
            ),
            DeclareLaunchArgument(
                "high_res_selector_topic",
                default_value="/fish_cam/high_res_camera",
                description="std_msgs/String topic used to select high-res mode.",
            ),
            DeclareLaunchArgument(
                "high_res_camera",
                default_value="",
                description="Initial single high-res camera name or /dev/video ID.",
            ),
            DeclareLaunchArgument(
                "high_res_topic",
                default_value="image_high/compressed",
                description="Selected high-res compressed output topic.",
            ),
            DeclareLaunchArgument(
                "high_camera_width",
                default_value="1920",
                description="High-res selected camera compressed output width.",
            ),
            DeclareLaunchArgument(
                "high_camera_height",
                default_value="1080",
                description="High-res selected camera compressed output height.",
            ),
            DeclareLaunchArgument(
                "high_camera_fps",
                default_value="1.0",
                description="High-res selected camera compressed output FPS.",
            ),
            DeclareLaunchArgument(
                "high_jpeg_quality",
                default_value="85",
                description="High-res selected camera JPEG quality.",
            ),
            DeclareLaunchArgument(
                "high_res_drain_frames",
                default_value="0",
                description="Frames to discard after switching into high-res mode.",
            ),
            OpaqueFunction(function=_launch_camera),
        ]
    )
