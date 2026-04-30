"""
Author(s): Christopher Holley, Everett Tucker
Creation Date: 09/09/2023
Description: Handles interfacing with the camera and publishing the images converted to ros messages
Publishers: Image
Subscribers: None
"""

import rclpy
# import imutils
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


VIDEO_DEVICE = 4 # /dev/videoX


class ExploreHDPub(Node):
    """
    The ExploreHDPub object represents the publisher node for the camera. It handles interfacing with the camera and publishing the images converted to ros messages

    Args:
        None

    Attributes:
        publisher: ROS publisher object, publishes images to the topic 'Image'
        cap: OpenCV VideoCapture object, used to read in images from the camera
        bridge: CvBridge object, used to convert OpenCV images to ros messages

    Special Cases:
        None
    """
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(Image, 'Image', 10)
        #self.get_logger().info(self.get_node_names_and_namespaces())
        self.declare_parameter('video_device_id', 4)

        VIDEO_DEVICE = self.get_parameter('video_device_id').get_parameter_value().integer_value
        print(f"Video device parameter is {VIDEO_DEVICE}")
        self.cap = cv2.VideoCapture(VIDEO_DEVICE)
        
        # Frame is normally 1920 x 1080
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920.0)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080.0)

        # FPS = 20 is very good, it decreses the window size at makes it a lot better
        self.cap.set(cv2.CAP_PROP_FPS, 20)

        self.bridge = CvBridge()

        if not self.cap.isOpened():
            print("Cannot open camera")
            exit()


    """
    Reads in the most recent image from the camera and publishes it to the topic 'Image' 
    NOTE: Will run forever until ros is shutdown

    Args:
        self - Must be a valid ExploreHDPub object

    Returns:
        None always  

    Raises:
        Does not raise but will log error if unable to read frame from camera
    """
    def publish_image(self):
        # essetnially while True but is ros shutdown safe
        while rclpy.ok():
            # Capture frame-by-frame
            ret, frame = self.cap.read()

            # Resizing

            # frame = imutils.resize(frame, width=1920, height=1080)
            # if frame is read correctly ret is True
            if not ret:
                self.get_logger().error("Unable to read frame from camera")
                break
            
            # publishes the image converted to a ros message to the topic 'Image'
            self.publisher.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            #self.get_logger().info("published frame")


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = ExploreHDPub()
    minimal_publisher.publish_image()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
   main()
