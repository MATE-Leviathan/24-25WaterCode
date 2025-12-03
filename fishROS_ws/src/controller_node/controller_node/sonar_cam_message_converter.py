"""
Author(s): Christopher Holley
Creation Date: 12/03/2025
Description: Converts ImagingSonar messages to OculusPing messages for sonar_camera_reconstruction
            and passes through odometry data
Subscribers:
    - /holoocean/auv0/ImagingSonar (holoocean_interfaces/msg/ImagingSonar)
    - /holoocean/auv0/DynamicsSensorOdom (nav_msgs/msg/Odometry)
Publishers:
    - /sonar_oculus_node/M750d/ping (sonar_oculus/msg/OculusPing)
    - /bruce/slam/localization/odom (nav_msgs/msg/Odometry)
"""

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge

from holoocean_interfaces.msg import ImagingSonar
from sonar_oculus.msg import OculusPing, OculusFire
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from std_msgs.msg import Header


class SonarMessageConverter(Node):
    """
    Converts ImagingSonar messages from HoloOcean to OculusPing messages
    compatible with the sonar_camera_reconstruction package.
    Also passes through odometry data.
    """

    def __init__(self):
        super().__init__('sonar_message_converter')

        # Declare parameters with defaults
        self.declare_parameter('input_topic', '/holoocean/auv0/ImagingSonar')
        self.declare_parameter('output_topic', '/sonar_oculus_node/M750d/ping')
        self.declare_parameter('odom_input_topic', '/holoocean/auv0/DynamicsSensorOdom')
        self.declare_parameter('odom_output_topic', '/bruce/slam/localization/odom')
        self.declare_parameter('azimuth_fov_deg', 120.0)  # Field of view in degrees
        self.declare_parameter('min_range', 1.0)  # Minimum range in meters
        self.declare_parameter('max_range', 40.0)  # Maximum range in meters
        self.declare_parameter('speed_of_sound', 1500.0)  # m/s
        self.declare_parameter('salinity', 35.0)  # ppt
        self.declare_parameter('compression_quality', 90)  # JPEG quality

        # Get parameter values
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        odom_input_topic = self.get_parameter('odom_input_topic').value
        odom_output_topic = self.get_parameter('odom_output_topic').value
        self.azimuth_fov = self.get_parameter('azimuth_fov_deg').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.speed_of_sound = self.get_parameter('speed_of_sound').value
        self.salinity = self.get_parameter('salinity').value
        self.compression_quality = self.get_parameter('compression_quality').value

        # Create sonar subscriber and publisher
        self.sonar_subscription = self.create_subscription(
            ImagingSonar,
            input_topic,
            self.convert_callback,
            10
        )

        self.sonar_publisher = self.create_publisher(
            OculusPing,
            output_topic,
            10
        )

        # Create odometry subscriber and publisher
        self.odom_subscription = self.create_subscription(
            Odometry,
            odom_input_topic,
            self.odom_callback,
            10
        )

        self.odom_publisher = self.create_publisher(
            Odometry,
            odom_output_topic,
            10
        )

        self.bridge = CvBridge()
        self.ping_id = 0

        self.get_logger().info(f'Sonar message converter initialized')
        self.get_logger().info(f'Sonar - Subscribing to: {input_topic}')
        self.get_logger().info(f'Sonar - Publishing to: {output_topic}')
        self.get_logger().info(f'Odom - Subscribing to: {odom_input_topic}')
        self.get_logger().info(f'Odom - Publishing to: {odom_output_topic}')

    def convert_callback(self, msg: ImagingSonar):
        print("Hello")
        """
        Callback that converts ImagingSonar to OculusPing message.

        Args:
            msg: ImagingSonar message from HoloOcean
        """
        try:
            # Create OculusPing message
            oculus_msg = OculusPing()

            # Copy header
            oculus_msg.header = msg.header

            # Create and populate OculusFire message
            fire_msg = OculusFire()
            fire_msg.header = msg.header
            fire_msg.mode = 0  # Default mode
            fire_msg.gamma = 127  # Default gamma (middle value)
            fire_msg.flags = 0
            fire_msg.range = self.max_range
            fire_msg.gain = 50.0  # Default gain percentage
            fire_msg.speed_of_sound = self.speed_of_sound
            fire_msg.salinity = self.salinity
            oculus_msg.fire_msg = fire_msg

            # Set ping metadata
            oculus_msg.ping_id = self.ping_id
            self.ping_id += 1
            oculus_msg.part_number = 0
            oculus_msg.start_time = msg.timestamp

            # Generate bearings array (in Oculus format: bearing * PI / 18000)
            # Convert azimuth FOV from degrees to the Oculus bearing format
            num_beams = msg.bins_azimuth
            half_fov_rad = np.deg2rad(self.azimuth_fov / 2.0)
            bearings_rad = np.linspace(-half_fov_rad, half_fov_rad, num_beams)
            # Convert to Oculus format: bearing_value = bearing_rad * 18000 / PI
            bearings_oculus = (bearings_rad * 18000.0 / np.pi).astype(np.int16)
            oculus_msg.bearings = bearings_oculus.tolist()

            # Set range information
            oculus_msg.num_ranges = msg.bins_range
            oculus_msg.range_resolution = (self.max_range - self.min_range) / msg.bins_range
            oculus_msg.num_beams = num_beams

            # Convert image data to compressed format
            # Reshape the flat array to 2D image (bins_azimuth x bins_range)
            image_data = np.array(msg.image, dtype=np.float32)

            # Check if reshape is needed
            if len(image_data) == msg.bins_azimuth * msg.bins_range:
                # Reshape: rows=num_ranges, cols=num_beams (transpose for proper orientation)
                image_2d = image_data.reshape(msg.bins_azimuth, msg.bins_range).T
            else:
                self.get_logger().warning(
                    f'Image data length {len(image_data)} does not match '
                    f'bins_azimuth ({msg.bins_azimuth}) * bins_range ({msg.bins_range})'
                )
                return

            # Normalize to 0-255 range for visualization
            if image_2d.max() > 0:
                image_normalized = ((image_2d - image_2d.min()) /
                                   (image_2d.max() - image_2d.min()) * 255.0)
            else:
                image_normalized = np.zeros_like(image_2d)

            image_uint8 = image_normalized.astype(np.uint8)

            # Convert to BGR for compression (grayscale to BGR)
            image_bgr = cv2.cvtColor(image_uint8, cv2.COLOR_GRAY2BGR)

            # Compress image to JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.compression_quality]
            _, compressed_data = cv2.imencode('.jpg', image_bgr, encode_param)

            # Create CompressedImage message
            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header
            compressed_msg.format = "jpeg"
            compressed_msg.data = compressed_data.tobytes()
            oculus_msg.ping = compressed_msg

            # Publish the converted message
            self.sonar_publisher.publish(oculus_msg)
            self.get_logger().info(
                f'Published OculusPing #{oculus_msg.ping_id} '
                f'({num_beams} beams x {msg.bins_range} ranges)'
            )

        except Exception as e:
            self.get_logger().error(f'Error converting message: {str(e)}')

    def odom_callback(self, msg: Odometry):
        """
        Callback that passes through odometry messages.

        Args:
            msg: Odometry message from HoloOcean
        """
        # Simply republish the odometry message
        self.odom_publisher.publish(msg)
        self.get_logger().debug('Published odometry message')


def main(args=None):
    rclpy.init(args=args)

    converter = SonarMessageConverter()

    try:
        rclpy.spin(converter)
    except KeyboardInterrupt:
        pass
    finally:
        converter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
