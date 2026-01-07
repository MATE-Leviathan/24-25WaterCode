"""
Author(s): Alex Vernon
Creation Date: 11/13/2025
Description: 
Subscribers: 
Publishers: 
"""

import threading
import rclpy
import time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from holoocean_interfaces.msg import ImagingSonar

import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import Float32
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge
bridge = CvBridge()

class SonarSub(Node):

    def __init__(self):
        # Creating the subscriber
        super().__init__('sonar_subscriber')
        self.subscription = self.create_subscription(ImagingSonar, '/holoocean/auv0/ImagingSonar', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Image, 'imagetopic', 10)

        # binsA = 512
        # binsR = 512
        # azi = 120
        # minR = 1
        # maxR = 40

        # plt.ion()
        # self.fig, self.ax = plt.subplots(subplot_kw=dict(projection='polar'), figsize=(8,5))
        
        # self.ax.set_theta_zero_location("N")
        # self.ax.set_thetamin(-azi/2)
        # self.ax.set_thetamax(azi/2)

        # self.theta = np.linspace(-azi/2, azi/2, binsA)*np.pi/180
        # self.r = np.linspace(minR, maxR, binsR)
        # self.T, self.R = np.meshgrid(self.theta, self.r)
        # self.z = np.zeros_like(self.T)

        # plt.grid(False)
        # self.plot = self.ax.pcolormesh(self.T, self.R, self.z, cmap='gray', shading='auto', vmin=0, vmax=1)
        # plt.tight_layout()
        # self.fig.canvas.draw()
        # self.fig.canvas.flush_events()

    # def img_pub(self):
    #     # Convert OpenCV image to ROS 2 Image message
        


    def listener_callback(self, msg):
        image = np.array(msg.image).reshape(512, 512)
        print(image)
        # cv.imshow("title", image)
        # cv.waitKey(0)
        ros_image_msg = bridge.cv2_to_imgmsg(image, "32FC1")
        ros_image_msg.header.stamp = self.get_clock().now().to_msg()
        ros_image_msg.header.frame_id = 'camera_frame' # Set an appropriate frame ID
        self.publisher_.publish(ros_image_msg)
        self.get_logger().info('Publishing image')
        # self.fig.canvas.draw()
        # self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    sonar_sub = SonarSub()

    executor.add_node(sonar_sub)

    # Launching the executor
    executor.spin()

    sonar_sub.destroy_node()

    # Shutting down the program
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
