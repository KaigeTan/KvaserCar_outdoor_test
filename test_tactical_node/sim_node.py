#!/usr/bin/env python3
import socket
import json
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point
from std_msgs.msg import Float32  # for reference speed subscription

class Motion:
    def __init__(self, max_acc):
        """
        max_acc : float
            The maximum magnitude of acceleration or deceleration [m/s²].
        """
        self.v = 0.0
        self.d_total = 0.0
        self.d = 0.0
        self.acc = max_acc

    def get_displacement(self, ref_speed, delta_t):
        v_old = self.v
        if self.v < ref_speed:
            v_new = min(v_old + self.acc * delta_t, ref_speed)
        elif v_old > ref_speed:
            v_new = max(v_old - self.acc * delta_t, ref_speed)
        else:
            v_new = v_old

        self.d = 0.5 * (v_old + v_new) * delta_t
        self.v = v_new
        self.d_total += self.d
        return self.d, self.d_total


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q

class EgoAdvSimNode(Node):
    def __init__(self):
        super().__init__('ego_adv_sim')
        # Declare parameters
        self.declare_parameter('ego_start_x', 4.0)
        self.declare_parameter('ego_start_y', 0.0)
        self.declare_parameter('ego_end_x', -2.0)
        self.declare_parameter('ego_end_y', 0.0)
        self.declare_parameter('ego_ref_speed', 1.5)
        self.declare_parameter('ego_max_acc', 1.35)

        self.declare_parameter('adv_start_x', 0.0)
        self.declare_parameter('adv_start_y', -4.0)
        self.declare_parameter('adv_end_x', 0.0)
        self.declare_parameter('adv_end_y', 2.0)
        self.declare_parameter('adv_ref_speed', 1.25)
        self.declare_parameter('adv_max_acc', 1.0)

        self.declare_parameter('cr_point_1', [0.5, 0.5])
        self.declare_parameter('cr_point_2', [0.5, -0.5])
        self.declare_parameter('cr_point_3', [-0.5, -0.5])
        self.declare_parameter('cr_point_4', [-0.5, 0.5])

        # UDP & sim parameters
        self.declare_parameter('udp_target_ip', '127.0.0.1')
        self.declare_parameter('udp_target_port', 9999)
        self.declare_parameter('udp_delay', 0.05)
        self.declare_parameter('add_aoi', 0)
        self.declare_parameter('adv_queue_size', 1)
        self.declare_parameter('comm_fail_start', 0)
        self.declare_parameter('comm_fail_duration', 0)
        self.declare_parameter('sim_step', 0.1)
        self.declare_parameter('comm_step', 0.001)

        # Read parameters
        ego_start = (self.get_parameter('ego_start_x').value,
                     self.get_parameter('ego_start_y').value)
        ego_end = (self.get_parameter('ego_end_x').value,
                   self.get_parameter('ego_end_y').value)
        adv_start = (self.get_parameter('adv_start_x').value,
                     self.get_parameter('adv_start_y').value)
        adv_end = (self.get_parameter('adv_end_x').value,
                   self.get_parameter('adv_end_y').value)
        self.ego_ref_speed = self.get_parameter('ego_ref_speed').value
        ego_max_acc = self.get_parameter('ego_max_acc').value
        self.adv_ref_speed = self.get_parameter('adv_ref_speed').value
        adv_max_acc = self.get_parameter('adv_max_acc').value

        # Motion objects
        self.ego_motion = Motion(ego_max_acc)
        self.adv_motion = Motion(adv_max_acc)

        # Direction vectors and total distance
        def setup_motion(start, end, motion):
            dx, dy = end[0]-start[0], end[1]-start[1]
            dist = math.hypot(dx, dy)
            return (dx/dist, dy/dist), dist

        self.ego_dir, self.ego_path_dist = setup_motion(ego_start, ego_end, self.ego_motion)
        self.adv_dir, self.adv_path_dist = setup_motion(adv_start, adv_end, self.adv_motion)
        self.ego_start, self.ego_end = ego_start, ego_end
        self.adv_start, self.adv_end = adv_start, adv_end
        self.ego_yaw = math.atan2(self.ego_dir[1], self.ego_dir[0])
        self.adv_yaw = math.atan2(self.adv_dir[1], self.adv_dir[0])

        # QoS and publishers/subscribers
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                         depth=2,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        self.ego_pub = self.create_publisher(Odometry, '/odometry/map/sim', 1)
        self.adv_pub = self.create_publisher(Odometry, '/adv_emu_odometry', qos)
        self.ref_spd_sub = self.create_subscription(
            Float32, '/ref_spd', self._ref_spd_callback, qos)

        # UDP setup
        self.udp_ip = self.get_parameter('udp_target_ip').value
        self.udp_port = self.get_parameter('udp_target_port').value
        self._udp_delay_ns = int(self.get_parameter('udp_delay').value * 1e9)
        self.adv_queue_size = self.get_parameter('adv_queue_size').value
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.adv_queue = deque()
        self._msg_id = 0

        # Communication failure window
        self.start_ns = self.get_clock().now().nanoseconds
        cfs = self.get_parameter('comm_fail_start').value
        cfd = self.get_parameter('comm_fail_duration').value
        self._comm_fail_start = self.start_ns + int(cfs*1e9)
        self._comm_fail_end = self._comm_fail_start + int(cfd*1e9)
        self.add_aoi = self.get_parameter('add_aoi').value * 1e6

        # Path publishers for plotter
        self.coord_pub = {}
        for name, coord in [('ego_start', ego_start),
                            ('ego_end', ego_end),
                            ('adv_start', adv_start),
                            ('adv_end', adv_end)]:
            pub = self.create_publisher(Point, f'/{name}', qos)
            msg = Point(x=coord[0], y=coord[1], z=0.0)
            pub.publish(msg)

        # Timers
        self.sim_timer = self.create_timer(self.get_parameter('sim_step').value,
                                           self._simulate)
        self.comm_timer = self.create_timer(self.get_parameter('comm_step').value,
                                            self._send_udp)

        self.get_logger().info('Simulation started')

    def _ref_spd_callback(self, msg: Float32):
        self.ego_ref_speed = msg.data

    def _simulate(self):
        # Check completion
        if (self.ego_motion.d_total >= self.ego_path_dist or
            self.adv_motion.d_total >= self.adv_path_dist):
            self.get_logger().info('Reached end of paths; stopping simulation')
            self.sim_timer.cancel()
            return

        # Ego
        d_e, _ = self.ego_motion.get_displacement(self.ego_ref_speed,
                                                  self.sim_timer.timer_period_ns/1e9)
        ex = self.ego_start[0] + self.ego_dir[0] * self.ego_motion.d_total
        ey = self.ego_start[1] + self.ego_dir[1] * self.ego_motion.d_total
        ego_msg = Odometry()
        ego_msg.header.stamp = self.get_clock().now().to_msg()
        ego_msg.header.frame_id = 'map'
        ego_msg.child_frame_id = 'ego_base_link'
        ego_msg.pose.pose.position.x = ex
        ego_msg.pose.pose.position.y = ey
        ego_msg.pose.pose.orientation = quaternion_from_yaw(self.ego_yaw)
        ego_msg.twist.twist.linear.x = self.ego_motion.v
        self.ego_pub.publish(ego_msg)

        # Adv
        d_a, _ = self.adv_motion.get_displacement(self.adv_ref_speed,
                                                  self.sim_timer.timer_period_ns/1e9)
        ax = self.adv_start[0] + self.adv_dir[0] * self.adv_motion.d_total
        ay = self.adv_start[1] + self.adv_dir[1] * self.adv_motion.d_total
        adv_msg = Odometry()
        adv_msg.header.stamp = self.get_clock().now().to_msg()
        adv_msg.header.frame_id = 'map'
        adv_msg.child_frame_id = 'adv_base_link'
        adv_msg.pose.pose.position.x = ax
        adv_msg.pose.pose.position.y = ay
        adv_msg.pose.pose.orientation = quaternion_from_yaw(self.adv_yaw)
        adv_msg.twist.twist.linear.x = self.adv_motion.v
        self.adv_pub.publish(adv_msg)

        # Queue UDP payload
        if len(self.adv_queue) < self.adv_queue_size:
            now = self.get_clock().now().nanoseconds
            payload = {
                'id': self._msg_id,
                't_stamp': now,
                'front_x': ax,
                'front_y': ay,
                'vel': self._add_vel_noise(self.adv_motion.v)
            }
            self.adv_queue.append(payload)
            self._msg_id += 1

    def _send_udp(self):
        now_ns = self.get_clock().now().nanoseconds
        # Stop both timers when both done
        if (self.ego_motion.d_total >= self.ego_path_dist and
            self.adv_motion.d_total >= self.adv_path_dist):
            self.get_logger().info('Paths done; stopping UDP sends')
            self.comm_timer.cancel()
            return

        # Suppress during comm failure
        if self._comm_fail_start <= now_ns < self._comm_fail_end:
            print(f"{(now_ns- self.start_ns)/1e9} COMM FAILURE")
            return

        # Send if delay elapsed
        if self.adv_queue and (now_ns - self.adv_queue[0]['t_stamp'] >= self._udp_delay_ns):
            info = self.adv_queue.popleft()
            info["t_stamp"] -= self.add_aoi 
            print(f"{(now_ns - self.start_ns)/1e9} SENDING {(info['t_stamp']- self.start_ns)/1e9} diff {(now_ns -info['t_stamp'])/1e9}")
            packet = json.dumps(info).encode('utf-8')
            self.udp_sock.sendto(packet, (self.udp_ip, self.udp_port))

    def _add_vel_noise(self, value: float) -> float:
        return value - 0.2

    def destroy_node(self):
        self.get_logger().info('Shutting down, closing UDP socket')
        self.udp_sock.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EgoAdvSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
