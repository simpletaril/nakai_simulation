import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

class ObstacleOverrideNode(Node):
    def __init__(self):
        super().__init__('obstacle_override_node')
        
        # Define a publisher for velocity commands (cmd_vel)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Define a subscriber to the range sensors
        self.range_sub = self.create_subscription(Range, '/range/raw', self.range_callback, 10)
        
        # Threshold distance to detect obstacles (in meters)
        self.obstacle_threshold = 0.6
        
        # Timer to publish velocity commands at regular intervals
        self.timer = self.create_timer(0.1, self.publish_velocity)
        
        # Initialize velocity command
        self.velocity_msg = Twist()
        self.obstacle_detected = False

    def range_callback(self, msg):
        # Check if any range sensor detects an object closer than the threshold
        if msg.range < self.obstacle_threshold:
            self.obstacle_detected = True
            self.get_logger().info('Obstacle detected! Overriding the object.')
        else:
            self.obstacle_detected = False

    def publish_velocity(self):
        # If an obstacle is detected, stop or override movement
        if self.obstacle_detected:
            # Example of stopping the robot
            self.velocity_msg.linear.x = -1.0
            self.velocity_msg.angular.z = -1.5 # Turn to avoid the object
        else:
            # Normal behavior (e.g., move forward)
            self.velocity_msg.linear.x = 0.25
            self.velocity_msg.angular.z = 0.0

        # Publish the velocity command
        self.cmd_vel_pub.publish(self.velocity_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleOverrideNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
