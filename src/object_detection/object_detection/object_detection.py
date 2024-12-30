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
            '/camera_sensor/image_raw',  
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',  # Replace with the actual odometry topic if different
            self.odom_callback,
            10
        )
        self.bridge = CvBridge()
        self.edge_threshold = 0.35
        self.driving_forward = True
        self.turn_direction = 1  # Initialize turn direction (1 for CCW, -1 for CW)

        self.positions = []  # To store robot positions for mapping
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

    def update_map(self):
        x_vals, y_vals = zip(*self.positions) if self.positions else ([], [])
        self.ax.clear()
        self.ax.plot(x_vals, y_vals, marker='o', linestyle='-', color='b', label='Path')
        self.ax.set_title("Robot Path")
        self.ax.set_xlabel("X Position (m)")
        self.ax.set_ylabel("Y Position (m)")
        self.ax.legend()
        self.ax.grid()
        plt.pause(0.1)

    def process_frame(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        self.plot_green_histogram(frame)
        
        green_lower = np.array([40, 50, 50])  # Adjust HSV range for green
        green_upper = np.array([80, 255, 255])
        gray_lower = np.array([0, 0, 50])    # Adjust HSV range for gray
        gray_upper = np.array([180, 50, 200])

        green_mask = cv2.inRange(hsv, green_lower, green_upper)
        gray_mask = cv2.inRange(hsv, gray_lower, gray_upper)

        green_pixels = np.sum(green_mask > 0)
        gray_pixels = np.sum(gray_mask > 0)
        total_pixels = frame.shape[0] * frame.shape[1]

        green_ratio = green_pixels / total_pixels
        gray_ratio = gray_pixels / total_pixels

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
        # Extract the green channel (channel index 1 in BGR format)
        green_channel = frame[:, :, 1]

        # Calculate the histogram for the green channel
        hist = cv2.calcHist([green_channel], [0], None, [256], [0, 256])
        hist = hist / hist.sum()  # Normalize the histogram

        # Plot the histogram
        plt.figure('Green Color Histogram')
        plt.clf()  # Clear the figure for live updates
        plt.plot(hist, color='g', label='Green Channel')

        plt.title('Normalized Green Color Histogram')
        plt.xlabel('Pixel Intensity')
        plt.ylabel('Normalized Frequency')
        plt.legend()
        plt.grid()
        plt.pause(0.1)  # Pause to update the plot

    def rotate(self):
        twist = Twist()
        angular_speed = math.radians(45)  
        twist.angular.z = angular_speed * self.turn_direction

        duration = 460 / 45  # degrees/s = seconds (4 seconds for 180 degrees)

        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(twist)
            time.sleep(0.1)  

        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info(
            f'Rotation complete! Direction: {"CCW" if self.turn_direction == 1 else "CW"}')

        # Alternate the turn direction for the next U-turn
        self.turn_direction *= -1

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


























# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image

# import cv2 as cv
# from cv_bridge import CvBridge

# class CameraViewerNode(Node):
#     def __init__(self):
#         super().__init__('camera_viewer')
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera_sensor/image_raw',  # Replace with your image topic
#             self.listener_callback,
#             10
#         )
#         self.subscription  # prevent unused variable warning
#         self.bridge = CvBridge()

#     def listener_callback(self, data):
#         # Convert ROS Image message to OpenCV format
#         img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

#         # Display the color image
#         cv.imshow('Camera Feed', img)
#         cv.waitKey(1)

# def main(args=None):
#     rclpy.init(args=args)
#     camera_viewer_node = CameraViewerNode()
#     rclpy.spin(camera_viewer_node)
#     camera_viewer_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()









# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image

# import cv2 as cv
# from cv_bridge import CvBridge
# import numpy as np

# class CircleDetectionNode(Node):
#     def __init__(self):
#         super().__init__('image_subscriber')
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera_sensor/image_raw',  # Replace with your image topic
#             self.listener_callback,
#             10
#         )
#         self.subscription  # prevent unused variable warning
#         self.bridge = CvBridge()

#     def listener_callback(self, data):
#         # Convert ROS Image message to OpenCV format
#         img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

#         # Convert the image to grayscale
#         gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

#         # Apply median blur to reduce noise and improve circle detection
#         gray_img = cv.medianBlur(gray_img, 5)

#         # Convert the grayscale image back to BGR for displaying results
#         cimg = cv.cvtColor(gray_img, cv.COLOR_GRAY2BGR)

#         # Detect circles using HoughCircles
#         circles = cv.HoughCircles(
#             gray_img,
#             cv.HOUGH_GRADIENT,
#             dp=1,
#             minDist=2,
#             param1=50,
#             param2=30,
#             minRadius=0,
#             maxRadius=0
#         )

#         # If circles are detected, draw them
#         if circles is not None:
#             circles = np.uint16(np.around(circles))  # Round and convert to unsigned 16-bit integers
#             for i in circles[0, :]:
#                 # Draw the outer circle
#                 cv.circle(cimg, (i[0], i[1]), i[2], (0, 255, 0), 2)
#                 # Draw the center of the circle
#                 cv.circle(cimg, (i[0], i[1]), 2, (0, 0, 255), 3)

#         # Display the result
#         cv.imshow('Detected Circles', cimg)
#         cv.waitKey(1)

# def main(args=None):
#     rclpy.init(args=args)
#     circle_detection_node = CircleDetectionNode()
#     rclpy.spin(circle_detection_node)
#     circle_detection_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
















# # import rclpy
# # from rclpy.node import Node
# # from sensor_msgs.msg import Image

# # import cv2 as cv
# # from cv_bridge import CvBridge
# # import numpy as np

# # class FeatureDetectionNode(Node):
# #     def __init__(self):
# #         super().__init__('feature_detection_node')
# #         self.subscription = self.create_subscription(
# #             Image,
# #             '/camera_sensor/image_raw',  # Replace with your image topic
# #             self.listener_callback,
# #             10
# #         )
# #         self.bridge = CvBridge()

# #     def listener_callback(self, data):
# #         # Convert ROS Image message to OpenCV format
# #         img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

# #         # Convert the image to grayscale
# #         gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

# #         # Apply median blur to reduce noise
# #         gray_img = cv.medianBlur(gray_img, 5)

# #         # Convert the grayscale image back to BGR for displaying results
# #         cimg = cv.cvtColor(gray_img, cv.COLOR_GRAY2BGR)

# #         # Detect circles (hollows) using HoughCircles
# #         circles = cv.HoughCircles(
# #             gray_img,
# #             cv.HOUGH_GRADIENT,
# #             dp=1,
# #             minDist=20,
# #             param1=50,
# #             param2=30,
# #             minRadius=0,
# #             maxRadius=0
# #         )

# #         # If circles are detected, draw bounding boxes around them
# #         if circles is not None:
# #             circles = np.uint16(np.around(circles))  # Round and convert to unsigned 16-bit integers
# #             for i in circles[0, :]:
# #                 # Draw bounding box around the circle
# #                 x, y, r = i[0], i[1], i[2]
# #                 cv.rectangle(cimg, (x - r, y - r), (x + r, y + r), (0, 255, 0), 2)

# #         # Detect lines using HoughLinesP
# #         edges = cv.Canny(gray_img, 50, 150, apertureSize=3)
# #         lines = cv.HoughLinesP(edges, 1, np.pi / 180, threshold=100, minLineLength=50, maxLineGap=10)

# #         # Define the lower half of the image
# #         height, width = gray_img.shape
# #         lower_half_y = int(height / 2)  # Define the lower half starting from the middle

# #         # If lines are detected, draw bounding boxes around them if they are in the lower half
# #         if lines is not None:
# #             for line in lines:
# #                 x1, y1, x2, y2 = line[0]
# #                 # Check if both endpoints of the line are in the lower half of the image
# #                 if y1 > lower_half_y and y2 > lower_half_y:
# #                     # Calculate bounding box dimensions for the line
# #                     bb_x_min = min(x1, x2)
# #                     bb_y_min = min(y1, y2)
# #                     bb_x_max = max(x1, x2)
# #                     bb_y_max = max(y1, y2)
# #                     # Draw bounding box around the line
# #                     cv.rectangle(cimg, (bb_x_min, bb_y_min), (bb_x_max, bb_y_max), (255, 0, 0), 2)
# #                     # Alert if a line is detected in the lower half of the image
# #                     self.get_logger().info('ALERT: Line detected in the lower half of the image!')

# #         # Display the result
# #         cv.imshow('Detected Features', cimg)
# #         cv.waitKey(1)

# # def main(args=None):
# #     rclpy.init(args=args)
# #     feature_detection_node = FeatureDetectionNode()
# #     rclpy.spin(feature_detection_node)
# #     feature_detection_node.destroy_node()
# #     rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()















# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image

# import cv2 as cv
# from cv_bridge import CvBridge
# import numpy as np

# class ShapeDetectionNode(Node):
#     def __init__(self):
#         super().__init__('shape_detection_node')
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera_sensor/image_raw',  # Replace with your image topic
#             self.listener_callback,
#             10
#         )
#         self.bridge = CvBridge()

#     def listener_callback(self, data):
#         # Convert ROS Image message to OpenCV format
#         img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

#         # Convert the image to grayscale
#         gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

#         # Apply CLAHE (Contrast Limited Adaptive Histogram Equalization) to enhance contrast
#         clahe = cv.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
#         enhanced_img = clahe.apply(gray_img)

#         # Convert the grayscale image back to BGR for color display
#         cimg = cv.cvtColor(enhanced_img, cv.COLOR_GRAY2BGR)

#         # --- Circle Detection ---
#         circles = cv.HoughCircles(
#             enhanced_img,
#             cv.HOUGH_GRADIENT,
#             dp=1,
#             minDist=20,
#             param1=50,
#             param2=30,
#             minRadius=0,
#             maxRadius=0
#         )

#         # If circles are detected, draw them with green color
#         if circles is not None:
#             circles = np.uint16(np.around(circles))
#             for i in circles[0, :]:
#                 # Draw the outer circle
#                 cv.circle(cimg, (i[0], i[1]), i[2], (0, 255, 0), 2)
#                 # Draw the center of the circle
#                 cv.circle(cimg, (i[0], i[1]), 2, (0, 0, 255), 3)

#         # --- Rectangle Detection ---
#         # Detect edges using Canny edge detector
#         edges = cv.Canny(enhanced_img, 50, 150)

#         # Find contours in the edge-detected image
#         contours, _ = cv.findContours(edges, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

#         for contour in contours:
#             # Approximate the contour to reduce the number of points
#             approx = cv.approxPolyDP(contour, 0.02 * cv.arcLength(contour, True), True)
#             if len(approx) == 4:  # If the approximated contour has 4 points, it's likely a rectangle
#                 x, y, w, h = cv.boundingRect(approx)
#                 # Draw the bounding rectangle with blue color
#                 cv.rectangle(cimg, (x, y), (x + w, y + h), (255, 0, 0), 2)

#         # Display the result
#         cv.imshow('Detected Shapes', cimg)
#         cv.waitKey(1)

# def main(args=None):
#     rclpy.init(args=args)
#     shape_detection_node = ShapeDetectionNode()
#     rclpy.spin(shape_detection_node)
#     shape_detection_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()











# import rclpy
# from rclpy.node import Node
# import cv2
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image

# class FaceBlurNode(Node):
#     def __init__(self):
#         super().__init__('face_blur_node')

#         # Load the Haar Cascade for face detection
#         self.face_cascade = cv2.CascadeClassifier('/path/to/haarcascade_frontalface_default.xml')

#         # Initialize the video capture from the default camera (0)
#         self.cap = cv2.VideoCapture(0)
#         if not self.cap.isOpened():
#             self.get_logger().error("Error opening video stream")

#         # Create a publisher for the processed video stream
#         self.image_pub = self.create_publisher(Image, '/camera/blurred_face', 10)

#         # Timer to periodically capture and process frames
#         self.timer = self.create_timer(0.033, self.process_frame)  # 30 Hz

#         # Bridge between OpenCV and ROS Image messages
#         self.bridge = CvBridge()

#     def process_frame(self):
#         ret, frame = self.cap.read()  # Capture frame-by-frame
#         if not ret:
#             self.get_logger().error("Captured empty frame")
#             return

#         # Convert to grayscale for face detection
#         gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         cv2.equalizeHist(gray_frame, gray_frame)

#         # Detect faces in the frame
#         faces = self.face_cascade.detectMultiScale(gray_frame, 1.1, 4)

#         # Blur each detected face
#         for (x, y, w, h) in faces:
#             face_roi = frame[y:y+h, x:x+w]
#             # Apply Gaussian blur
#             face_roi = cv2.GaussianBlur(face_roi, (55, 55), 30)
#             frame[y:y+h, x:x+w] = face_roi  # Replace with the blurred face

#         # Convert the frame back to ROS2 Image and publish
#         image_message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
#         self.image_pub.publish(image_message)

# def main(args=None):
#     rclpy.init(args=args)
#     node = FaceBlurNode()
#     rclpy.spin(node)

#     # Clean up when done
#     node.cap.release()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
































