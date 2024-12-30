#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import numpy as np
import struct
import cv2

class ObstacleOverrideNode(Node):
    def __init__(self):
        super().__init__('obstacle_override_node')

        # Subscribes to the PointCloud2 topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/points',
            self.pointcloud_callback,
            10
        )

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer to publish default forward velocity
        self.timer = self.create_timer(0.1, self.publish_default_velocity)

        self.obstacle_detected = False
        self.get_logger().info('Obstacle Override Node initialized')

    def pointcloud_callback(self, msg):
        # Extract point cloud data
        points = self.read_points(msg)
        if points is None:
            return

        # Visualize the point cloud as a depth image
        self.visualize_depth(points)

        # Analyze the point cloud to detect holes or bulges
        self.obstacle_detected = self.detect_hole_or_bulge(points)

        if self.obstacle_detected:
            self.override_obstacle()

    def read_points(self, msg):
        try:
            cloud_data = []
            point_step = msg.point_step
            row_step = msg.row_step
            data = msg.data

            for i in range(0, len(data), point_step):
                x, y, z = struct.unpack_from('fff', data, i)
                cloud_data.append((x, y, z))

            return np.array(cloud_data)
        except Exception as e:
            self.get_logger().error(f"Error reading points: {e}")
            return None

    def visualize_depth(self, points):
        try:
            # Convert point cloud data to a 2D depth image
            z_values = points[:, 2]
            z_min, z_max = np.min(z_values), np.max(z_values)
            depth_image = ((z_values - z_min) / (z_max - z_min) * 255).astype(np.uint8)

            # Ensure the depth image can be reshaped into a square
            side_length = int(np.sqrt(len(depth_image)))
            if side_length**2 != len(depth_image):
                self.get_logger().warn(f"Depth image cannot be reshaped into a square, truncating data.")
                depth_image = depth_image[:side_length**2]
            depth_image = depth_image.reshape(side_length, side_length)

            # Normalize and display the depth image
            depth_image = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
            cv2.imshow('Depth Camera View', depth_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error visualizing depth: {e}")

    def detect_hole_or_bulge(self, points):
        # Placeholder detection logic: Detect significant deviations in Z values
        z_values = points[:, 2]
        z_mean = np.mean(z_values)
        z_std = np.std(z_values)

        # Thresholds for anomalies (can be adjusted based on the application)
        hole_threshold = z_mean - 2 * z_std
        bulge_threshold = z_mean + 2 * z_std

        holes = z_values < hole_threshold
        bulges = z_values > bulge_threshold

        if np.any(holes):
            self.get_logger().info('Hole detected')
            return True
        elif np.any(bulges):
            self.get_logger().info('Bulge detected')
            return True
        else:
            return False

    def override_obstacle(self):
        # Publish velocity to help the robot override the obstacle
        twist = Twist()

        # Example: Move forward with a slight turn to avoid the obstacle
        twist.linear.x = -0.2  # Forward velocity
        twist.angular.z = 1.0  # Angular velocity to steer

        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info('Published override velocity command')

    def publish_default_velocity(self):
        if not self.obstacle_detected:
            twist = Twist()
            twist.linear.x = 0.2  # Default forward velocity
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleOverrideNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
