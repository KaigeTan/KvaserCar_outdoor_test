import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Float32
import math
import time

class VehicleState:
    FORWARD = 1
    REVERSE = -1
    STOPPED = 0  # New state for when velocity is very low

class RoverOdometryNode(Node):
    def __init__(self):
        super().__init__('rover_odometry_node')
        self.get_logger().info('RoverOdometryNode started!')

        # Declare parameters, default value
        self.declare_parameter('wheelbase', 0.55)           # wheelbase length
        self.declare_parameter('update_frequency', 20.0)    # update frequency for wheel odometry estimation
        self.declare_parameter('is_radio', 0)               # if controlled by radio, set 1; if controlled by ros topic, set 0.

        # Read parameters from the YAML file
        self.wheelbase = self.get_parameter('wheelbase').value
        self.update_frequency = self.get_parameter('update_frequency').value
        self.is_radio = self.get_parameter('is_radio').value
        

        self.get_logger().info(f"Wheelbase: {self.wheelbase}, Update Frequency: {self.update_frequency} Hz, Controlled by radio: {self.is_radio}")

        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Orientation (yaw)
        self.delta_radio = None  # Steering angle by radio, starts as None
        self.delta_ros = None  # Steering angle by ros, starts as None
        self.throttle_radio = None # Throttle percentage by radio, starts as None
        self.throttle_ros = None # Throttle percentage by ros, starts as None
        self.motion_state = VehicleState.STOPPED
        self.small_threshold = 0.01
        self.delay_timer = 0.0
        self.velocity_inertia_time = 0.2
        self.print_warn = 0
        self.print_info = 0
        

        # Subscribers
        self.create_subscription(Float32, '/rover/wheel_rear_left/speed_kph', self.rear_left_speed_callback, 10)
        self.create_subscription(Float32, '/rover/wheel_rear_right/speed_kph', self.rear_right_speed_callback, 10)
        self.create_subscription(Float32, '/rover/radio/steering', self.steering_radio_callback, 100)
        self.create_subscription(Float32, '/rover/steering', self.steering_ros_callback, 10)
        self.create_subscription(Float32, '/rover/radio/throttle', self.throttle_radio_callback, 100)
        self.create_subscription(Float32, '/rover/throttle', self.throttle_ros_callback, 10)
        
        

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odometry', 10)

        # Time tracking initialization
        self.last_time = self.get_clock().now()

        # Input data
        self.v_rear_left = 0.0
        self.v_rear_right = 0.0
        self.imu_yaw_rate = 0.0

    def rear_left_speed_callback(self, msg):
        self.v_rear_left = msg.data/3.6  # Convert kph to m/s

    def rear_right_speed_callback(self, msg):
        self.v_rear_right = msg.data/3.6  # Convert kph to m/s

    def steering_radio_callback(self, msg):
        self.delta_radio = msg.data/360*2*math.pi

    def steering_ros_callback(self, msg):
        self.delta_ros = msg.data/360*2*math.pi
    
    def throttle_radio_callback(self, msg):
        self.throttle_radio = msg.data

    def throttle_ros_callback(self, msg):
        self.throttle_ros = msg.data
    


    def compute_odometry(self):
        # Check if steering angle (delta) is set
        if self.is_radio:
            if self.delta_radio is None:
                if self.print_warn == 0:
                    self.get_logger().warn("Steering angle (delta) not set. Ensure /rover/radio/steering topic is publishing data. Skipping odometry computation.", throttle_duration_sec=1.0)
                    self.print_warn = 1
                return
            else:
                if self.print_info == 0:
                    self.get_logger().info(
                        "***********************Steering angle (delta) is set now.***********************"
                    )
                    self.print_info = 1
        else: # TODO: check this logic after the control function is developed
            if self.delta_ros is None:
                self.get_logger().warn("Steering angle (delta) not set. Ensure /rover/steering topic is publishing data. Skipping odometry computation.", throttle_duration_sec=1.0)
                return 


        # Current time and time difference
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds*1e-9
        # print(dt)

        # Compute forward velocity
        v_x_raw = (self.v_rear_left + self.v_rear_right)/2

        # Check the direction of the velocity, implemented by a state machine logic
        if self.is_radio:
            throttle = self.throttle_radio if self.throttle_radio is not None else 0
        else:
            throttle = self.throttle_ros if self.throttle_ros is not None else 0

        
        if throttle == 0:
            self.motion_state = VehicleState.STOPPED
        else:
            if self.motion_state == VehicleState.STOPPED:
                if throttle > 0:
                    self.motion_state = VehicleState.FORWARD
                elif throttle < 0:
                    self.motion_state = VehicleState.REVERSE

        if self.motion_state == VehicleState.FORWARD:
            v_x = v_x_raw
        elif self.motion_state == VehicleState.REVERSE:
            v_x = -v_x_raw
        else:
            v_x = 0.0


        # # Case 1: Throttle = 0 → Maintain previous state until velocity stops
        # if throttle == 0:
        #     if abs(v_x) < self.small_threshold:  # If nearly stopped, mark as STOPPED
        #         self.motion_state = VehicleState.STOPPED
        #     # Maintain previous motion direction until velocity stops
        #     v_x = v_x if self.motion_state == VehicleState.FORWARD else -v_x

        # # Case 2: Throttle is positive → Wants to move forward
        # elif throttle > 0:
        #     if self.motion_state == VehicleState.REVERSE and abs(v_x) > self.small_threshold:
        #         if self.delay_timer <= 0:
        #             self.delay_timer = self.velocity_inertia_time  # Start delay
        #     else:
        #         self.motion_state = VehicleState.FORWARD  # Allow forward motion

        # # Case 3: Throttle is negative → Wants to move backward
        # elif throttle < 0:
        #     if self.motion_state == VehicleState.FORWARD and abs(v_x) > self.small_threshold:
        #         if self.delay_timer <= 0:
        #             self.delay_timer = self.velocity_inertia_time  # Start delay
        #     else:
        #         self.motion_state = VehicleState.REVERSE  # Allow backward motion

        # # Apply delay mechanism to prevent abrupt switching
        # if self.delay_timer > 0:
        #     self.delay_timer -= dt  # Count down delay
        # else:
        #     # Only switch direction after delay expires
        #     v_x = v_x if self.motion_state == VehicleState.FORWARD else -v_x

        self.motion_state = VehicleState.STOPPED


        # integrate the check of v_x and correct here
        if self.is_radio:
            theta_dot = v_x/self.wheelbase*math.tan(self.delta_radio)
        else:
            theta_dot = v_x/self.wheelbase*math.tan(self.delta_ros)

        # Update position
        self.x += v_x*math.cos(self.theta)*dt
        self.y += v_x*math.sin(self.theta)*dt
        self.theta += theta_dot*dt
        # Normalize the angle to the range [-pi, pi]
        self.theta = (self.theta + math.pi)%(2*math.pi) - math.pi

        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Pose
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0  # Assuming planar motion
        odom_msg.pose.pose.orientation = Quaternion()
        qx, qy, qz, qw = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Twist
        odom_msg.twist.twist.linear.x = v_x
        odom_msg.twist.twist.angular.z = theta_dot

        # TODO: check the IMU covariance setting, Pose covariance (x, y, theta)
        odom_msg.pose.covariance = [float(x) for x in [
            0.01, 0, 0, 0, 0, 0,
            0, 0.01, 0, 0, 0, 0,
            0, 0, 0.1, 0, 0, 0,
            0, 0, 0, 0.1, 0, 0,
            0, 0, 0, 0, 0.1, 0,
            0, 0, 0, 0, 0, 0.1]]

        # TODO: check the IMU covariance setting, Twist covariance (vx, vy, vtheta)
        odom_msg.twist.covariance = [float(x) for x in [
            0.01, 0, 0, 0, 0, 0,
            0, 0.01, 0, 0, 0, 0,
            0, 0, 0.1, 0, 0, 0,
            0, 0, 0, 0.1, 0, 0,
            0, 0, 0, 0, 0.1, 0,
            0, 0, 0, 0, 0, 0.1]]

        self.odom_pub.publish(odom_msg)

        # Update time
        self.last_time = current_time

    def timer_callback(self):
        self.compute_odometry()

def main(args=None):
    rclpy.init(args=args)
    node = RoverOdometryNode()

    # Run odometry computation at the specified update frequency
    update_period = 1.0/node.update_frequency
    node.create_timer(update_period, node.timer_callback)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    
