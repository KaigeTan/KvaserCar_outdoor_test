#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class PubRefSpd(Node):
    def __init__(self):
        super().__init__('pub_ref_spd')

        # Declare parameters, default value
        self.declare_parameter('ref_spd', 0.0)           # reference speed, default is 1 m/s
        self.ref_spd = self.get_parameter('ref_spd').value # Read parameters

        # Subscriber and publisher
        self.pub = self.create_publisher(Float32, "/ref_spd", 2)

        # Publish the reference speed in a callback function, frequency 1Hz
        self.timer = self.create_timer(1.0, self.refspd_callback)

    def refspd_callback(self):
        msg = Float32()
        msg.data = float(self.ref_spd)
        self.pub.publish(msg)
        


def main():
    rclpy.init()
    node = PubRefSpd()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
