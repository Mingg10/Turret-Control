#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImagePublisher(Node):
    def __init__(self, name):
        super().__init__(name)

        # Publishers
        self.pub_raw = self.create_publisher(Image, '/camera/image_raw', 10)
        self.pub_rect = self.create_publisher(Image, '/camera/image_rect', 10)

        # Subscribers
        self.sub_info = self.create_subscription(CameraInfo, '/camera/camera_info', self.info_callback, 10)

        # Timer to grab frames
        self.timer = self.create_timer(0.1, self.timer_callback)

        # OpenCV
        self.cap = cv2.VideoCapture(0)
        self.cv_bridge = CvBridge()

        # Rectification maps
        self.map1, self.map2 = None, None
        self.K, self.D, self.R, self.P, self.size = None, None, None, None, None

        self.get_logger().info("Camera node initialized, waiting for CameraInfo...")

    def info_callback(self, msg: CameraInfo):
        """Build rectification maps from CameraInfo (only once)."""
        self.K = np.array(msg.k).reshape(3, 3)
        self.D = np.array(msg.d)
        self.R = np.array(msg.r).reshape(3, 3)
        self.P = np.array(msg.p).reshape(3, 4)
        self.size = (msg.width, msg.height)

        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.K, self.D, self.R, self.P[:, :3], self.size, cv2.CV_16SC2
        )
        self.get_logger().info(f"Rectification maps built for {self.size[0]}x{self.size[1]} image")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to grab frame from camera")
            return

        # Publish raw image
        raw_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.pub_raw.publish(raw_msg)

        # Publish rectified image if calibration is available
        if self.map1 is not None and self.map2 is not None:
            rectified = cv2.remap(frame, self.map1, self.map2, cv2.INTER_LINEAR)
            rect_msg = self.cv_bridge.cv2_to_imgmsg(rectified, encoding='bgr8')
            rect_msg.header = raw_msg.header
            self.pub_rect.publish(rect_msg)

        self.get_logger().info("Published raw and rectified frames")

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher("topic_webcam_pub")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

