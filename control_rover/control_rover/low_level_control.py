import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import math
import numpy as np

class LowLevelCtrl(Node):
    def __init__(self):
        super().__init__('low_level_ctrl')

        # Subscribers
        self.subscription_joystick = self.create_subscription(
            Bool, '/aeb_triggered', self.aeb_callback, 5)
        self.subscription_ref_spd = self.create_subscription(
            Float32, '/ref_spd', self.ref_spd_callback, 5)  # TODO: check the topic name, frequency, and type
        # Publishers
        self.publisher_throttle = self.create_publisher(Float32, '/rover/throttle', 50)

        # State 
        self.aeb_triggered = Bool(data=False)
        self.ref_spd = 0.0

        # Control Variables
        self.throttle = 0.0
        

    def aeb_callback(self, msg):
        """Check if aeb is triggered."""
        self.aeb_triggered = msg.data
        if self.aeb_triggered:
            # self.get_logger().info("Throttle updated to: 0")
            self.publisher_throttle.publish(Float32(data=0.0))
        else:
            # self.get_logger().info(f"Throttle updated to: {self.throttle:.2f}")
            self.publisher_throttle.publish(Float32(data=self.throttle))


    def ref_spd_callback(self, msg):
        """Check the reference speed data, update the throttle control value."""
        self.ref_spd = msg.data
        self.throttle = self.spd_to_throttle(self.ref_spd)


    def spd_to_throttle(self, ref_spd):
        """Convert the reference speed to the throttle control value, use a open test mapping, this is NOT a closed-loop control."""
        if ref_spd < 1 or ref_spd > 3:
            self.get_logger().warn("The reference speed must be from the range of [1, 3].", throttle_duration_sec=1.0)
            return 0.0
        else:
            return (ref_spd + 0.62)/0.04 # this is calibrated from the data in the shared onedrive folder, linear fit for stable speed v.s. throttle input


def main(args=None):
    rclpy.init(args=args)
    node = LowLevelCtrl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()