#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge


class FullFrameSlimeDetectionNode(Node):
    def __init__(self):
        super().__init__('full_frame_slime_detection_node')

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',  
            self.odom_callback,
            10)

        self.slime_odometry_publisher = self.create_publisher(Odometry, '/slime_odometry', 10)

        self.bridge = CvBridge()

        self.slime_color_lower = np.array([35, 100, 100])  
        self.slime_color_upper = np.array([85, 255, 255])  
        self.alert_threshold = 0.5   

        self.robot_odometry = None  

    def odom_callback(self, msg):
        
        self.robot_odometry = msg

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            slime_mask = cv2.inRange(hsv_image, self.slime_color_lower, self.slime_color_upper)

            total_pixels = slime_mask.size
            slime_pixels = cv2.countNonZero(slime_mask)
            slime_coverage = slime_pixels / total_pixels

            self.get_logger().info(f'Slime coverage: {slime_coverage * 100:.2f}%')

            if slime_coverage > self.alert_threshold:
                self.get_logger().warn('Slime detected across the frame!')
                if self.robot_odometry:
                    self.publish_world_odometry(self.robot_odometry)

            cv2.imshow('Slime Detection Mask', slime_mask)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')

    def publish_world_odometry(self, odom_msg):
        slime_odometry = Odometry()

       
        slime_odometry.header.stamp = self.get_clock().now().to_msg()
        slime_odometry.header.frame_id = 'odom'   

       
        slime_odometry.pose.pose.position = odom_msg.pose.pose.position
        slime_odometry.pose.pose.orientation = odom_msg.pose.pose.orientation

    
        self.slime_odometry_publisher.publish(slime_odometry)
        self.get_logger().info(f'Published world odometry: Position ({slime_odometry.pose.pose.position.x}, {slime_odometry.pose.pose.position.y})')

def main(args=None):
    rclpy.init(args=args)
    node = FullFrameSlimeDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

