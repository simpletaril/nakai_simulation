#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
import math

class EdgeDetectionNode(Node):
    def __init__(self):  
        super().__init__('edge_detection_node')
    
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  
            self.image_callback,
            10
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',  
            self.odom_callback,
            10
        )
        self.slime_odom_subscription = self.create_subscription(
            Odometry,
            '/slime_odometry',
            self.slime_odom_callback,
            10
        )

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)


        self.timer_cw = self.create_timer(6.5, self.rotate_cw)



        self.bridge = CvBridge()
        self.edge_threshold = 0.29
        self.driving_forward = True

        self.positions = []  
        self.slime_positions = []  

        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Robot Path")
        self.ax.set_xlabel("X Position (m)")
        self.ax.set_ylabel("Y Position (m)")
        plt.ion()

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_frame(frame)

        if hasattr(self, 'frame_count'):
            self.frame_count += 1
        else:
            self.frame_count = 1

        if self.frame_count % 10 == 0:
            self.plot_green_histogram(frame)

        cv2.imshow("Camera Frame", frame)  
        cv2.waitKey(1)  

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.positions.append((position.x, position.y))
        self.update_map()

    def slime_odom_callback(self, msg):
        position = msg.pose.pose.position
        self.slime_positions.append((position.x, position.y))
        self.update_map()

    def rotate_cw(self):
        twist = Twist()
        twist.linear.x = 0.0  
        twist.angular.z = -0.6
        self.publisher.publish(twist)
        self.get_logger().info('rotate cw...')

        
    def update_map(self):
        self.ax.clear()

        if self.slime_positions:
            slime_x_vals, slime_y_vals = zip(*self.slime_positions)
            self.ax.scatter(slime_x_vals, slime_y_vals, color='g', label='Slime', s=50)

        if self.positions:
            x_vals, y_vals = zip(*self.positions)
            # self.ax.plot(x_vals, y_vals, marker='o', linestyle='-', color='b', label='Path')
            self.ax.scatter(x_vals, y_vals, color='b', label='Path', s=10)
        
        
        

        self.ax.set_title("Robot Path")
        self.ax.set_xlabel("X Position (m)")
        self.ax.set_ylabel("Y Position (m)")
        self.ax.legend()
        self.ax.grid()
        plt.pause(0.1)

    def process_frame(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        self.plot_green_histogram(frame)
        
        green_lower = np.array([40, 50, 50])  
        green_upper = np.array([80, 255, 255])
        gray_lower = np.array([0, 0, 50])    
        gray_upper = np.array([180, 50, 200])

        green_mask = cv2.inRange(hsv, green_lower, green_upper)
        gray_mask = cv2.inRange(hsv, gray_lower, gray_upper)

        green_pixels = np.sum(green_mask > 0)
        gray_pixels = np.sum(gray_mask > 0)
        total_pixels = frame.shape[0] * frame.shape[1]

        green_ratio = green_pixels / total_pixels
        gray_ratio = gray_pixels / total_pixels

      
        self.timer_cw
        if gray_ratio > self.edge_threshold:
            self.rotate()
        elif self.driving_forward:
            self.drive_forward()

    def plot_green_histogram(self, frame):
        """
        Plot the histogram for the green channel of the frame.
        This function focuses on the green color only.

        Args:
            frame: The input frame (image) from which the green channel is extracted.
        """
        green_channel = frame[:, :, 1]

        hist = cv2.calcHist([green_channel], [0], None, [256], [0, 256])
        hist = hist / hist.sum()  

        plt.figure('Green Color Histogram')
        plt.clf() 
        plt.plot(hist, color='g', label='Green Channel')

        plt.title('Normalized Green Color Histogram')
        plt.xlabel('Pixel Intensity')
        plt.ylabel('Normalized Frequency')
        plt.legend()
        plt.grid()
        plt.pause(0.1)  

    def rotate(self):
        twist = Twist()
        angular_speed = math.radians(65)  
        twist.angular.z = angular_speed  

        angle_in_degrees = 10
        angle_in_radians = math.radians(angle_in_degrees)

        duration = angle_in_radians / angular_speed  

        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(twist)
            time.sleep(0.1)  

        twist.angular.z = 0.0 
        self.publisher.publish(twist)

    def drive_forward(self):
        twist = Twist()
        twist.linear.x = 0.5  
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info('Driving forward...')


def main(args=None):
    rclpy.init(args=args)
    node = EdgeDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()




















# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# import matplotlib.pyplot as plt
# import time
# import math

# class EdgeDetectionNode(Node):
#     def __init__(self):  
#         super().__init__('edge_detection_node')
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera/image_raw',  
#             self.image_callback,
#             10
#         )
#         self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
#         self.odom_subscription = self.create_subscription(
#             Odometry,
#             '/odom',  
#             self.odom_callback,
#             10
#         )
#         self.bridge = CvBridge()
#         self.edge_threshold = 0.28
#         self.driving_forward = True

#         self.positions = []  # store robot positions for mapping
#         self.fig, self.ax = plt.subplots()
#         self.ax.set_title("Robot Path")
#         self.ax.set_xlabel("X Position (m)")
#         self.ax.set_ylabel("Y Position (m)")
#         plt.ion()

#     def image_callback(self, msg):
#         frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         self.process_frame(frame)

#         if hasattr(self, 'frame_count'):
#             self.frame_count += 1
#         else:
#             self.frame_count = 1

#         if self.frame_count % 10 == 0:
#             self.plot_green_histogram(frame)

#         cv2.imshow("Camera Frame", frame)  
#         cv2.waitKey(1)  

#     def odom_callback(self, msg):
#         position = msg.pose.pose.position
#         self.positions.append((position.x, position.y))
#         self.update_map()

#     def update_map(self):
#         x_vals, y_vals = zip(*self.positions) if self.positions else ([], [])
#         self.ax.clear()
#         self.ax.plot(x_vals, y_vals, marker='o', linestyle='-', color='b', label='Path')
#         self.ax.set_title("Robot Path")
#         self.ax.set_xlabel("X Position (m)")
#         self.ax.set_ylabel("Y Position (m)")
#         self.ax.legend()
#         self.ax.grid()
#         plt.pause(0.1)

#     def process_frame(self, frame):
#         hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
#         self.plot_green_histogram(frame)
        
#         green_lower = np.array([40, 50, 50])  
#         green_upper = np.array([80, 255, 255])
#         gray_lower = np.array([0, 0, 50])    
#         gray_upper = np.array([180, 50, 200])

#         green_mask = cv2.inRange(hsv, green_lower, green_upper)
#         gray_mask = cv2.inRange(hsv, gray_lower, gray_upper)

#         green_pixels = np.sum(green_mask > 0)
#         gray_pixels = np.sum(gray_mask > 0)
#         total_pixels = frame.shape[0] * frame.shape[1]

#         green_ratio = green_pixels / total_pixels
#         gray_ratio = gray_pixels / total_pixels

#         if gray_ratio > self.edge_threshold:
#             self.rotate()
#         elif self.driving_forward:
#             self.drive_forward()

#     def plot_green_histogram(self, frame):
#         """
#         Plot the histogram for the green channel of the frame.
#         This function focuses on the green color only.

#         Args:
#             frame: The input frame (image) from which the green channel is extracted.
#         """
#         green_channel = frame[:, :, 1]

       
#         hist = cv2.calcHist([green_channel], [0], None, [256], [0, 256])
#         hist = hist / hist.sum()  

#         plt.figure('Green Color Histogram')
#         plt.clf() 
#         plt.plot(hist, color='g', label='Green Channel')

#         plt.title('Normalized Green Color Histogram')
#         plt.xlabel('Pixel Intensity')
#         plt.ylabel('Normalized Frequency')
#         plt.legend()
#         plt.grid()
#         plt.pause(0.1)  

#     def rotate(self):
#         twist = Twist()
#         angular_speed = math.radians(45)  
#         twist.angular.z = angular_speed  

#         angle_in_degrees = 8
#         angle_in_radians = math.radians(angle_in_degrees)

#         duration = angle_in_radians / angular_speed  

#         start_time = time.time()
#         while time.time() - start_time < duration:
#             self.publisher.publish(twist)
#             time.sleep(0.1)  

#         twist.angular.z = 0.0 
#         self.publisher.publish(twist)

        

#     def drive_forward(self):
#         twist = Twist()
#         twist.linear.x = 0.5  
#         twist.angular.z = 0.0
#         self.publisher.publish(twist)
#         self.get_logger().info('Driving forward...')


# def main(args=None):
#     rclpy.init(args=args)
#     node = EdgeDetectionNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

#     cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()


    