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
        self.publisher = self.create_publisher(Marker, 'static_marker', 10)
        self.publisher = self.create_publisher(Marker, 'trajectory_blue', 10)
        self.publisher = self.create_publisher(Marker, 'trajectory_red', 10)


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

        self.publisher.publish(marker)

    # draw and publish the trajectory for the blue car
    def publish_traj_blue(self):
        marker = Marker()
        marker.header.frame_id = "map" # TODO: check the frame of it, it should linked to the car's odom
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trajectory_blue"
        marker.id = 2
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # sphere diameter
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        # Blue car dashed points along a path: from (5, 1) -> (-5, 1)
        for i in range(100):
            pt = Point()
            pt.x = 5.0 - i * 0.1
            pt.y = 1.0
            pt.z = 0.0
            marker.points.append(pt)

        self.publisher.publish(marker)     



    # draw and publish the trajectory for the blue car
    def publish_traj_red(self):
        marker = Marker()
        marker.header.frame_id = "map" # TODO: check the frame of it, it should linked to the car's odom
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trajectory_red"
        marker.id = 3
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # sphere diameter
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Blue car dashed points along a path: from (1, 3) -> (1, -2)
        for i in range(100):
            pt = Point()
            pt.x = 1.0
            pt.y = 3.0 - i * 0.05
            pt.z = 0.0
            marker.points.append(pt)

        self.publisher.publish(marker)
        


rclpy.init()
node = StaticMarkerPublisher()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
