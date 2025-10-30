#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from tf_transformations import euler_from_quaternion

class TagPoseReader(Node):
    def __init__(self):
        super().__init__('tag_pose_reader')
        self.sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.cb,
            10)

    def cb(self, msg):
        for det in msg.detections:
            pose = det.pose.pose  
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            ori = pose.orientation
            roll, pitch, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
            self.get_logger().info(
                f"Tag {det.id}: dist = {z:.3f} m, yaw = {yaw:.3f} rad"
            )

def main(args=None):
    rclpy.init(args=args)
    node = TagPoseReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()