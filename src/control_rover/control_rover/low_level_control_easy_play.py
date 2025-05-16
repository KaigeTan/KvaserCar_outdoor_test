import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
import math
import numpy as np
import time

class LowLevelCtrl(Node):
    def __init__(self):
        super().__init__('low_level_ctrl')

        # Subscribers
        self.subscription_joystick = self.create_subscription(
            Bool, '/aeb_triggered', self.aeb_callback, 5)
        self.subscription_ref_spd = self.create_subscription(
            Float32, '/ref_spd', self.ref_spd_callback, 5)
        self.subscription_curr_odom = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        # Publishers
        self.publisher_throttle = self.create_publisher(Float32, '/rover/throttle', 50)
        self.publisher_steering = self.create_publisher(Float32, '/rover/steering', 50)

        # add a timer to continuously publish throttle control data
        self.timer = self.create_timer(1.0 / 100.0, self.timer_callback)

        # Declare parameters, default value
        self.declare_parameter('is_steering_correction', 0)           # decide whether to use steering correction
        self.declare_parameter('is_throttle_correction', 0)           # decide whether to use throttle correction
        self.declare_parameter('distance_travel', 0.0)                  # the distance the vehicle will travel
        self.is_steering_correction = self.get_parameter('is_steering_correction').value # Read parameters
        self.is_throttle_correction = self.get_parameter('is_throttle_correction').value
        self.distance_total = self.get_parameter('distance_travel').value

        # State 
        self.aeb_triggered = Bool(data=False)
        self.ref_spd = 0.0
        self.ref_heading = 0.0
        self.distance_traveled = 0.0

        # Control Variables
        self.throttle = 0.0
        self.kp = 7.5 # Proportional gain
        self.ki = 1.0 # Integral gain
        self.output_limits = [0.0, 100.0] # (min, max) output
        self.integral = 0.0
        self.last_time = None
        self.vel = 0.0
        self.heading_angle = 0.0
        self.threshold_heading = math.pi/18 # 10 degre
        self.ctrl_steer_angle = 10.0 # 5 degree
        self.flag = 0 # judge if newly executed


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

    def odom_callback(self, msg):
        """Check the current longitudial velocity and vehicle heading"""
        self.vel = msg.twist.twist.linear.x
        self.distance_traveled = msg.pose.pose.position.x
        # Extract orientation quaternion
        orientation_q = msg.pose.pose.orientation
        r = R.from_quat([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        # Convert to Euler angles: here 'xyz' = roll, pitch, yaw
        # `degrees=True` returns angles in degrees; omit for radians
        _, _, self.heading_angle = r.as_euler('xyz')

    def timer_callback(self):
        """Publish motor commands at a fixed rate of 50Hz"""
        throttle_val = 0.0 if self.aeb_triggered else self.throttle
        self.publisher_throttle.publish(Float32(data=throttle_val))
        # self.get_logger().info("is_steering_correction {0}".format(self.is_steering_correction), throttle_duration_sec=1.0)
        if self.is_steering_correction:
            steer_val = self.cal_steering()
        else:
            steer_val = 0.0
        self.publisher_steering.publish(Float32(data=steer_val))

    def cal_throttle(self, ref_spd):
        """Calculate the throttle control input given a reference speed, use a feedforward mapping + PI controller"""
        # if the vehicle is still static in the beginning, only give the FF part
        if self.vel == 0 and self.flag == 0:
            # check feedforward output
            FF_out = (ref_spd + 0.62)/0.04 # this is calibrated from the data in the shared onedrive folder, linear fit for stable speed v.s. throttle input
            self.flag = 1
            return FF_out
        else:
            # check feedforward output
            FF_out = (ref_spd + 0.62)/0.04 # this is calibrated from the data in the shared onedrive folder, linear fit for stable speed v.s. throttle input
            if self.is_throttle_correction:
                curr_time = time.time()
                if self.last_time is None:
                    self.last_time = curr_time
                # check time difference
                dt = curr_time - self.last_time
                # check PI output
                err = ref_spd - self.vel
                self.integral = self.integral + err*dt
                PI_out = self.kp*err + self.ki*self.integral
                throttle_ctrl = PI_out + FF_out
            else:
                throttle_ctrl = FF_out
            throttle_ctrl = max(self.output_limits[0], min(self.output_limits[1], throttle_ctrl))
            return throttle_ctrl

    def spd_to_throttle(self, ref_spd):
        """Convert the reference speed to the throttle control value, use a open test mapping, this is NOT a closed-loop control."""
        if ref_spd < 1 or ref_spd > 3:
            self.get_logger().warn("The reference speed must be from the range of [1, 3].", throttle_duration_sec=1.0)
            return 0.0
        elif self.distance_traveled > self.distance_total:
            self.get_logger().info("Stop, since the vehicle has traveled {0} m".format(self.distance_traveled), throttle_duration_sec=1.0)
            self.get_logger().info("reference distance is {0} m".format(self.distance_total), throttle_duration_sec=1.0)
            return 0.0
        else:
            return self.cal_throttle(ref_spd)

    def cal_steering(self):
        """Steer back the vehicle heading if it deviates too much"""
        err = self.heading_angle - self.ref_heading
        # Apply correction only if deviation exceeds threshold
        if abs(err) < self.threshold_heading:
            return 0.0 # No correction needed
        else:
            self.get_logger().info("heading_angle {0}".format(err), throttle_duration_sec=1.0)
            # Clip to max steering range
            if err > 0:
                return self.ctrl_steer_angle
            else:
                return -self.ctrl_steer_angle


def main(args=None):
    rclpy.init(args=args)
    node = LowLevelCtrl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
