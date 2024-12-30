import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class OrientationDetectorNode(Node):

    def __init__(self):
        super().__init__('orientation_detector_node')
        self.subscription = self.create_subscription(
            Imu,
            '/data/imu',
            self.imu_callback,
            10
        )
        self.subscription = self.create_subscription()
        self.subscription  # Prevent unused variable warning
        
        # Orientation detection parameters
        self.ORIENTATION_THRESHOLD_LOW = -10.0
        self.ORIENTATION_THRESHOLD_HIGH = -8.0
        self.OUT_OF_RANGE_LIMIT = 10  # Number of consecutive out-of-range readings required to trigger an alert

        # State variables
        self.out_of_range_count = 0

    def imu_callback(self, msg):
        # Extract the linear acceleration Z value from the IMU message
        z_value = msg.linear_acceleration.z

        # Check if the z-axis acceleration is outside the expected range for horizontal orientation
        if z_value < self.ORIENTATION_THRESHOLD_LOW or z_value > self.ORIENTATION_THRESHOLD_HIGH:
            self.out_of_range_count += 1
        else:
            self.out_of_range_count = 0

        # Alert if there are 10 consecutive out-of-range readings
        if self.out_of_range_count >= self.OUT_OF_RANGE_LIMIT:
            self.get_logger().warn(
                'Orientation Alert! Robot is not horizontal.'
            )
            # Reset the count to avoid continuous alerts
            self.out_of_range_count = 0

def main(args=None):
    rclpy.init(args=args)
    orientation_detector_node = OrientationDetectorNode()
    try:
        rclpy.spin(orientation_detector_node)
    except KeyboardInterrupt:
        pass
    finally:
        orientation_detector_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
