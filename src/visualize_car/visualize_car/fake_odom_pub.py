import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
import math

class FakeOdomPublisher(Node):
    def __init__(self):
        super().__init__('fake_odom_publisher')

        self.blue_pub = self.create_publisher(Odometry, '/blue_car/odom', 10)
        self.red_pub = self.create_publisher(Odometry, '/red_car/odom', 10)

        self.timer = self.create_timer(0.1, self.publish_fake_odom)
        self.t = 0.0  # simulation time counter

    def publish_fake_odom(self):
        # Blue car moves left to right
        blue_odom = Odometry()
        blue_odom.header.frame_id = 'map'
        blue_odom.header.stamp = self.get_clock().now().to_msg()
        temp_x = 5.0 - 0.2*self.t
        blue_odom.pose.pose.position = Point(x=temp_x, y=1.0, z=0.0)
        blue_odom.pose.pose.orientation = Quaternion(w=1.0)

        # Red car moves top to bottom
        red_odom = Odometry()
        red_odom.header.frame_id = 'map'
        red_odom.header.stamp = self.get_clock().now().to_msg()
        temp_y = 3.0 - 0.1*self.t
        red_odom.pose.pose.position = Point(x=1.0, y=temp_y, z=0.0)
        red_odom.pose.pose.orientation = Quaternion(w=1.0)

        self.blue_pub.publish(blue_odom)
        self.red_pub.publish(red_odom)

        self.t += 0.1  # update simulated time

rclpy.init()
node = FakeOdomPublisher()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
