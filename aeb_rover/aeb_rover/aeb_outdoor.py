import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt16MultiArray
import math

class AEBRover(Node):
    def __init__(self):
        super().__init__('aeb_rover')

        # Subscribers
        self.subscription_rear = self.create_subscription(
            UInt16MultiArray, '/rover/obstacle_detector_rear/distance_mm', self.rear_callback, 5)
        self.subscription_front = self.create_subscription(
            UInt16MultiArray, '/rover/obstacle_detector_front/distance_mm', self.front_callback, 5)

        # Publishers
        self.publisher_aeb_triggered = self.create_publisher(Bool, '/aeb_triggered', 5)

        # Safety Threshold
        self.declare_parameter('distance_threshold', 50.0)
        self.DISTANCE_THRESHOLD = self.get_parameter('distance_threshold').value


        # Obstacle Flags
        self.front_obstacle_detected = False
        self.rear_obstacle_detected = False

    def rear_callback(self, msg):
        """Process rear sensor data."""
        self.rear_obstacle_detected = self.check_obstacle(msg, "rear")
        if self.rear_obstacle_detected:
            self.get_logger().warn("Obstacle detected at the rear! Stopping.", throttle_duration_sec=1.0)
        self.publish_control()

    def front_callback(self, msg):
        """Process front sensor data."""
        self.front_obstacle_detected = self.check_obstacle(msg, "front")
        if self.front_obstacle_detected:
            self.get_logger().warn("Obstacle detected at the front! Stopping.", throttle_duration_sec=1.0)
        self.publish_control()

    def check_obstacle(self, msg, sensor_position):
        """Processes ultrasonic data and detects obstacles."""
        if not msg.data:
            self.get_logger().warn(f"Received empty distance data from {sensor_position} sensor.", throttle_duration_sec=1.0)
            return False  
        
        # Ignore zero values and ensure valid distance readings
        valid_distances = [d for d in msg.data if isinstance(d, (int, float)) and d > 0]

        if not valid_distances:
            self.get_logger().warn(f"All {sensor_position} sensor values are invalid. Ignoring this message.", throttle_duration_sec=1.0)
            return False
        is_triggered = any(distance < self.DISTANCE_THRESHOLD for distance in valid_distances)

        return is_triggered

    def publish_control(self):
        """Publish if an obstacle is detected."""
        if self.front_obstacle_detected or self.rear_obstacle_detected:
            self.publisher_aeb_triggered.publish(Bool(data=True))
        else:
            self.publisher_aeb_triggered.publish(Bool(data=False))

        return
    
def main(args=None):
    rclpy.init(args=args)
    node = AEBRover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
