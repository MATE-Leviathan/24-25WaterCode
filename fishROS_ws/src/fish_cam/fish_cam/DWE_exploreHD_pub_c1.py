import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

VIDEO_DEVICE = 4  # /dev/videoX

class ExploreHDPub(Node):
    def __init__(self):
        super().__init__('compressed_image_publisher')
        self.publisher = self.create_publisher(CompressedImage, 'Image/compressed', 10)
        self.declare_parameter('video_device_id', 4)

        VIDEO_DEVICE = self.get_parameter('video_device_id').get_parameter_value().integer_value
        print(f"Video device parameter is {VIDEO_DEVICE}")
        self.cap = cv2.VideoCapture(VIDEO_DEVICE)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 10)

        if not self.cap.isOpened():
            print("Cannot open camera")
            exit()

    def publish_image(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Unable to read frame from camera")
                break

            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tobytes()

            self.publisher.publish(msg)
            self.get_logger().info("Published compressed frame")

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = ExploreHDPub()
    minimal_publisher.publish_image()
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
