from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_list(value):
    return [item.strip() for item in value.replace(";", ",").split(",") if item.strip()]


def _launch_cameras(context):
    camera_names = _parse_list(LaunchConfiguration("camera_names").perform(context))
    video_device_ids = _parse_list(LaunchConfiguration("video_device_ids").perform(context))
    executable = LaunchConfiguration("camera_executable").perform(context)
    camera_width = int(LaunchConfiguration("camera_width").perform(context))
    camera_height = int(LaunchConfiguration("camera_height").perform(context))
    camera_fps = float(LaunchConfiguration("camera_fps").perform(context))
    jpeg_quality = int(LaunchConfiguration("jpeg_quality").perform(context))
    high_res_camera = LaunchConfiguration("high_res_camera").perform(context)
    high_res_selector_topic = LaunchConfiguration("high_res_selector_topic").perform(
        context
    )
    high_camera_width = int(LaunchConfiguration("high_camera_width").perform(context))
    high_camera_height = int(LaunchConfiguration("high_camera_height").perform(context))
    high_camera_fps = float(LaunchConfiguration("high_camera_fps").perform(context))
    high_jpeg_quality = int(LaunchConfiguration("high_jpeg_quality").perform(context))

    if len(camera_names) != len(video_device_ids):
        raise RuntimeError(
            "camera_names and video_device_ids must have the same number of entries "
            f"(got {len(camera_names)} names and {len(video_device_ids)} devices)"
        )

    camera_nodes = []
    for camera_name, video_device_id in zip(camera_names, video_device_ids):
        camera_nodes.append(
            Node(
                package="fish_cam",
                executable=executable,
                namespace=camera_name,
                name=f"{camera_name}_camera",
                output="screen",
                parameters=[
                    {
                        "video_device_id": int(video_device_id),
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
                        "high_profile_width": high_camera_width,
                        "high_profile_height": high_camera_height,
                        "high_profile_fps": high_camera_fps,
                        "high_profile_jpeg_quality": high_jpeg_quality,
                    }
                ],
            )
        )

    return camera_nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_names",
                default_value="front,bottom,left,right",
                description="Comma-separated ROS namespaces/frame IDs for each camera.",
            ),
            DeclareLaunchArgument(
                "video_device_ids",
                default_value="0,4,8,12",
                description="Comma-separated /dev/video IDs, for example 0,4,8,12.",
            ),
            DeclareLaunchArgument(
                "camera_executable",
                default_value="dual_stream_camera",
                description="fish_cam camera executable to launch for every camera.",
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
                "high_res_camera",
                default_value="all",
                description="Initial high-res camera name or /dev/video ID.",
            ),
            DeclareLaunchArgument(
                "high_res_selector_topic",
                default_value="/fish_cam/high_res_camera",
                description="std_msgs/String topic used to select the high-res camera.",
            ),
            DeclareLaunchArgument(
                "high_camera_width",
                default_value="1280",
                description="High-res selected camera compressed output width.",
            ),
            DeclareLaunchArgument(
                "high_camera_height",
                default_value="720",
                description="High-res selected camera compressed output height.",
            ),
            DeclareLaunchArgument(
                "high_camera_fps",
                default_value="30.0",
                description="High-res selected camera compressed output FPS.",
            ),
            DeclareLaunchArgument(
                "high_jpeg_quality",
                default_value="75",
                description="High-res selected camera JPEG quality.",
            ),
            OpaqueFunction(function=_launch_cameras),
        ]
    )
