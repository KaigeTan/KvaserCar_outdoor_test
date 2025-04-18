import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class StaticMarkerPublisher(Node):
    def __init__(self):
        super().__init__('StaticMarker_publisher')
        self.marker_publisher = self.create_publisher(Marker, 'intersection_marker', 10)
        self.camera_publisher = self.create_publisher(Marker, 'camera_marker', 10)
        self.blue_path_pub = self.create_publisher(Path, '/blue_car/path', 10)
        self.red_path_pub = self.create_publisher(Path, '/red_car/path', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)


    # draw and publish the box area for intersection
    def publish_fence(self):
        marker = Marker()
        marker.header.frame_id = "map"  # attached to the map coordinate
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "fence"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Fence properties
        marker.scale.x = 0.05  # line width
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Define square points (counterclockwise)
        fence_points = [
            (0.0, 0.0),
            (2.0, 0.0),
            (2.0, 2.0),
            (0.0, 2.0),
            (0.0, 0.0)  # Close the loop
        ]

        for x, y in fence_points:
            pt = Point()
            pt.x = x
            pt.y = y
            pt.z = 0.0
            marker.points.append(pt)

        self.marker_publisher.publish(marker)


    # draw and publish the camera as a cube
    def publish_camera(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "camera"
        marker.id = 2
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = -2.0
        marker.pose.position.y = -2.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 2.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.camera_publisher.publish(marker)


    # draw and publish the trajectory for the blue car
    def publish_traj_blue(self): 
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        for i in range(100):
            pose_stamped = PoseStamped()
            pose_stamped.header = path.header
            pose_stamped.pose.position.x = 5.0 - i * 0.1
            pose_stamped.pose.position.y = 1.0
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0  # no rotation
            path.poses.append(pose_stamped)

        self.blue_path_pub.publish(path)


    # draw and publish the trajectory for the blue car
    def publish_traj_red(self):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        for i in range(100):
            pose_stamped = PoseStamped()
            pose_stamped.header = path.header
            pose_stamped.pose.position.x = 1.0
            pose_stamped.pose.position.y = 3.0 - i * 0.05
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            path.poses.append(pose_stamped)

        self.red_path_pub.publish(path)
        
    
    def timer_callback(self):
        self.publish_fence()
        self.publish_traj_blue()
        self.publish_traj_red()
        self.publish_camera()



rclpy.init()
node = StaticMarkerPublisher()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
