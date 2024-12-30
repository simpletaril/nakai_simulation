import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np

class HoleDetectionNode(Node):
    def __init__(self):
        super().__init__('hole_detection_node')
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',  
            self.image_callback,
            10)

        self.hole_odom_pub = self.create_publisher(Odometry, '/hole_odometry', 10)

        self.robot_odometry = None  
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',  
            self.odom_callback,
            10)

        self.last_bounding_box_height = 0  
        self.hole_detected = False  

        self.get_logger().info("Hole Detection Node has started.")

    def odom_callback(self, msg):
        
        self.robot_odometry = msg

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            blurred = cv2.GaussianBlur(gray, (5, 5), 0)

            _, thresh = cv2.threshold(blurred, 80, 255, cv2.THRESH_BINARY_INV)

            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            morphed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(morphed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            frame_height, frame_width = cv_image.shape[:2]
            for contour in contours:
                area = cv2.contourArea(contour)
                if 3000 < area < 50000:  # hole ranges
                    x, y, w, h = cv2.boundingRect(contour)


                    if y + h < frame_height * 0.42:
                        self.get_logger().info(f"Ignored bounding box in upper half: x={x}, y={y}, width={w}, height={h}")
                        continue


                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    self.get_logger().info(f"Hole detected at: x={x}, y={y}, width={w}, height={h}")

                    if w >= frame_width * 0.7 and h >= 95 or w >= frame_width * 0.9 :
                        self.hole_detected = True  

                        if self.robot_odometry:
                            self.publish_hole_odometry()

                    self.last_bounding_box_height = h

            cv2.imshow("Original Image", cv_image)
            cv2.imshow("Thresholded Image", morphed)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


    def publish_hole_odometry(self):
        if self.hole_detected:  
            hole_odom = Odometry()

            
            hole_odom.header.stamp = self.get_clock().now().to_msg()
            hole_odom.header.frame_id = 'odom'  
            hole_odom.pose.pose.position = self.robot_odometry.pose.pose.position

            self.hole_odom_pub.publish(hole_odom)
            self.get_logger().info(f"Published hole odometry: Position ({hole_odom.pose.pose.position.x}, {hole_odom.pose.pose.position.y})")
            self.hole_detected = False  

def main(args=None):
    rclpy.init(args=args)
    node = HoleDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



