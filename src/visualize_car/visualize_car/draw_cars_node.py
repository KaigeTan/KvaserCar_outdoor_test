import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import math
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

class CarMarkerPublisher(Node):
    def __init__(self):
        super().__init__('CarMarker_publisher')
        self.publisher = self.create_publisher(Marker, 'car_marker', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # initialize the blue and red car odometry
        self.latest_blue_pose = None
        self.latest_red_pose = None
        

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.blue_odom_sub = self.create_subscription(
            Odometry,
            '/odometry/map/sim',
            self.update_blue_box,
            qos)

        self.red_odom_sub = self.create_subscription(
            Odometry,
            '/adv_emu_odometry',
            self.update_red_box,
            qos)
        

        # self.blue_odom_sub = self.create_subscription(
        #     Odometry,
        #     '/blue_car/odom', # TODO: check the blue car odom topic name
        #     self.update_blue_box,
        #     10)

        # self.red_odom_sub = self.create_subscription(
        #     Odometry,
        #     '/red_car/odom',
        #     self.update_red_box, # TODO: check the red car odom topic name
        #     10)


    # update the blue car position
    def update_blue_box(self, msg):
        self.latest_blue_pose = msg.pose.pose


    # update the blue car position
    def update_red_box(self, msg):
        self.latest_red_pose = msg.pose.pose


    # visualize and update the blue car box updated position
    def publish_blue_box(self):
        if self.latest_blue_pose is None:
            return

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "blue_car"
        marker.id = 10
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = self.latest_blue_pose
        marker.scale.x = 0.75
        marker.scale.y = 0.3
        marker.scale.z = 0.4
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        self.publisher.publish(marker)


    # visualize and update the red car box updated position
    def publish_red_box(self):
        if self.latest_red_pose is None:
            return

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "red_car"
        marker.id = 11
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = self.latest_red_pose
        marker.scale.x = 0.75
        marker.scale.y = 0.3
        marker.scale.z = 0.4
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.publisher.publish(marker)


    def timer_callback(self):
        self.publish_blue_box()
        self.publish_red_box()


rclpy.init()
node = CarMarkerPublisher()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
