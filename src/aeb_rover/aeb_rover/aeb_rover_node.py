import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, UInt16MultiArray
from sensor_msgs.msg import Joy
import math

class AEBRover(Node):
    def __init__(self):
        super().__init__('aeb_rover')

        # Subscribers
        self.subscription_joystick = self.create_subscription(
            Joy, '/joy', self.joystick_callback, 75)
        self.subscription_rear = self.create_subscription(
            UInt16MultiArray, '/rover/obstacle_detector_rear/distance_mm', self.rear_callback, 5)
        self.subscription_front = self.create_subscription(
            UInt16MultiArray, '/rover/obstacle_detector_front/distance_mm', self.front_callback, 5)

        # Publishers
        self.publisher_steering = self.create_publisher(Float32, '/rover/steering', 75)
        self.publisher_throttle = self.create_publisher(Float32, '/rover/throttle', 75)

        # Control Variables
        self.steering = 0.0
        self.throttle = 0.0
        self.right_joy_x = 0.0
        self.right_joy_y = 0.0
        self.RT = 1.0

        # Safety Threshold
        self.DISTANCE_THRESHOLD = 50.0  # mm

        # Obstacle Flags
        self.front_obstacle_detected = False
        self.rear_obstacle_detected = False

        # Control Constants
        self.STEERING_JOY_RATIO = -45  # Maps joystick (1,-1) to (-45,45) degrees
        self.THROTTLE_Y_THRESHOLD = 0.5  # Dead zone for joystick movement
        self.THROTTLE_BASE_SPD = 20.0  # Base speed percentage

    def joystick_callback(self, msg):
        """Update control values from joystick input."""
        try:
            self.right_joy_x = msg.axes[3]  # Right stick X (1: left, -1: right)
            self.right_joy_y = msg.axes[4]  # Right stick Y (-1: down, 1: up)
            self.RT = msg.axes[5]  # RT trigger (-1: fully pressed, 1: unpressed)
            self.publish_control()
        except IndexError:
            self.get_logger().warn("Joystick input array out of bounds!")

    def rear_callback(self, msg):
        """Process rear sensor data."""
        self.rear_obstacle_detected = self.check_obstacle(msg, "rear")
        if self.rear_obstacle_detected:
            self.get_logger().warn("Obstacle detected at the rear! Stopping.")
        self.publish_control()

    def front_callback(self, msg):
        """Process front sensor data."""
        self.front_obstacle_detected = self.check_obstacle(msg, "front")
        if self.front_obstacle_detected:
            self.get_logger().warn("Obstacle detected at the front! Stopping.")
        self.publish_control()

    def check_obstacle(self, msg, sensor_position):
        """Processes ultrasonic data and detects obstacles."""
        if not msg.data:
            self.get_logger().warn(f"Received empty distance data from {sensor_position} sensor.")
            return False  


        # Ignore zero values and ensure valid distance readings
        valid_distances = [d for d in msg.data if isinstance(d, (int, float)) and d > 0]

        if not valid_distances:
            self.get_logger().warn(f"All {sensor_position} sensor values are invalid. Ignoring this message.")
            return False

        return any(distance < self.DISTANCE_THRESHOLD for distance in valid_distances)

    def publish_control(self):
        """Calculate and publish steering and throttle commands."""
        if self.front_obstacle_detected or self.rear_obstacle_detected:
            self.publisher_steering.publish(Float32(data=0.0))
            self.publisher_throttle.publish(Float32(data=0.0))
            return

        # Convert joystick input to steering angle (-45° to 45°)
        self.steering = self.STEERING_JOY_RATIO * self.right_joy_x

        # Convert RT to throttle percentage (linear mapping)
        RT_mapped = (-50/self.THROTTLE_BASE_SPD + 0.5)*self.RT + (50/self.THROTTLE_BASE_SPD + 0.5)  # default: Maps [1, -1] to [5, 1]

        if self.right_joy_y >= self.THROTTLE_Y_THRESHOLD:
            self.throttle = RT_mapped * self.THROTTLE_BASE_SPD # RT_mapped * 
        elif self.right_joy_y <= -self.THROTTLE_Y_THRESHOLD:
            self.throttle = -RT_mapped * self.THROTTLE_BASE_SPD # -RT_mapped * 
        else:
            self.throttle = 0.0

        # Publish commands
        self.publisher_steering.publish(Float32(data=self.steering))
        self.publisher_throttle.publish(Float32(data=self.throttle))

def main(args=None):
    rclpy.init(args=args)
    node = AEBRover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
