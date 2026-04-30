"""
Author(s): Christopher Holley
Creation Date: 09/09/2023
Description: Handles interfacing with the camera and publishing the images converted to ros messages
"""

import cv2


VIDEO_DEVICE = 0 # /dev/videoX


class ExploreHDPub():
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
        self.cap = cv2.VideoCapture(VIDEO_DEVICE)
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
        counter = 0
        # essetnially while True but is ros shutdown safe
        while True:
            # Capture frame-by-frame
            ret, frame = self.cap.read()
            # if frame is read correctly ret is True
            if not ret:
                self.get_logger().error("Unable to read frame from camera")
                break
            # show the image
            print(counter)
            counter += 1
            # cv2.imshow("hello", frame)
            # cv2.waitKey(1)
            # publishes the image converted to a ros message to the topic 'Image'


def main(args=None):
    minimal_publisher = ExploreHDPub()
    minimal_publisher.publish_image()


if __name__ == '__main__':
   main()
