#!/usr/bin/env python3
import socket
import json
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
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
        # parameters for ego (start/end positions overridden directly)
        self.declare_parameter('ego_start_x', 5.0)
        self.declare_parameter('ego_start_y', 0.0)
        self.declare_parameter('ego_end_x', -3.0)
        self.declare_parameter('ego_end_y', 0.0)
        self.declare_parameter('ego_ref_speed', 1.0)
        self.declare_parameter('ego_max_acc', 2.0)
        # parameters for adv (start/end positions overridden directly)
        self.declare_parameter('adv_start_x', 0.0)
        self.declare_parameter('adv_start_y', -10.0)
        self.declare_parameter('adv_end_x', 0.0)
        self.declare_parameter('adv_end_y', 10.0)
        self.declare_parameter('adv_ref_speed', 1.0)
        self.declare_parameter('adv_max_acc', 2.0)
        # UDP parameters
        self.declare_parameter('udp_target_ip', '127.0.0.1')
        self.declare_parameter('udp_target_port', 9999)
        # transmission delay T [s]
        self.declare_parameter('udp_delay', 0.1)
        # simulation step
        self.declare_parameter('sim_step', 0.02)

        # read back parameters
        ex, ey = (self.get_parameter('ego_end_x').value,
                  self.get_parameter('ego_end_y').value)
        sx, sy = (self.get_parameter('ego_start_x').value,
                  self.get_parameter('ego_start_y').value)
        # initial reference speed, will be updated by subscriber
        self.ego_ref_speed = self.get_parameter('ego_ref_speed').value
        self.ego_motion = Motion(self.get_parameter('ego_max_acc').value)
        self.ego_dir = ((ex - sx), (ey - sy))
        ego_dist = math.hypot(*self.ego_dir)
        self.ego_dir = (self.ego_dir[0] / ego_dist, self.ego_dir[1] / ego_dist)
        self.ego_start = (sx, sy)
        self.ego_yaw = math.atan2(self.ego_dir[1], self.ego_dir[0])

        ax, ay = (self.get_parameter('adv_start_x').value,
                  self.get_parameter('adv_start_y').value)
        ex2, ey2 = (self.get_parameter('adv_end_x').value,
                    self.get_parameter('adv_end_y').value)
        self.adv_ref_speed = self.get_parameter('adv_ref_speed').value
        self.adv_motion = Motion(self.get_parameter('adv_max_acc').value)
        self.adv_dir = ((ex2 - ax), (ey2 - ay))
        adv_dist = math.hypot(*self.adv_dir)
        self.adv_dir = (self.adv_dir[0] / adv_dist, self.adv_dir[1] / adv_dist)
        self.adv_start = (ax, ay)
        self.adv_yaw = math.atan2(self.adv_dir[1], self.adv_dir[0])

        # publishers and QoS
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        self.ego_pub = self.create_publisher(Odometry, '/odometry/map/sim', qos)
        self.adv_pub = self.create_publisher(Odometry, '/adv_emu_odometry', qos)

        # subscriber for reference speed
        self.ref_spd_sub = self.create_subscription(
            Float32, '/ref_spd', self._ref_spd_callback, qos
        )

        # UDP socket & queue
        self.udp_ip = self.get_parameter('udp_target_ip').value
        self.udp_port = self.get_parameter('udp_target_port').value
        self.udp_delay = self.get_parameter('udp_delay').value
        # nanosecond delay for comparison
        self._udp_delay_ns = int(self.udp_delay * 1e9)
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # queue stores dicts with 't_stamp' in ns and message id
        self.adv_queue = deque()
        self._msg_id = 0  # initialize message ID counter

        # timers for simulation and UDP
        self.sim_step = self.get_parameter('sim_step').value
        self.create_timer(self.sim_step, self._simulate)
        self.create_timer(self.sim_step, self._send_udp)
        self.get_logger().info(
            'Ego-Adv simulation started with UDP delay %.3f s' % self.udp_delay
        )

    def _ref_spd_callback(self, msg: Float32):
        """
        Update the ego reference speed based on incoming topic message.
        """
        self.ego_ref_speed = msg.data

    def _simulate(self):
        # record timestamp for when info is generated
        now = self.get_clock().now().nanoseconds
        # ego motion and publish
        self._publish_ego()
        # adv motion & enqueue
        d_a, _ = self.adv_motion.get_displacement(
            self.adv_ref_speed, self.sim_step
        )
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
        # enqueue with message id
        self.adv_queue.append({
            "id": self._msg_id,
            "t_stamp": now,
            "front_x": ax,
            "front_y": ay,
            "vel": self.adv_motion.v
        })
        self._msg_id += 1

    def _publish_ego(self):
        # advance ego using dynamic ref_spd and publish odometry
        d_e, _ = self.ego_motion.get_displacement(
            self.ego_ref_speed, self.sim_step
        )
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

    def _send_udp(self):
        """
        Send any queued adversary info whose age >= udp_delay.
        """
        if not self.adv_queue:
            return
        now = self.get_clock().now().nanoseconds
        while self.adv_queue and (
            now - self.adv_queue[0]["t_stamp"] >= self._udp_delay_ns
        ):
            info = self.adv_queue.popleft()
            packet = json.dumps(info).encode('utf-8')
            self.udp_sock.sendto(packet, (self.udp_ip, self.udp_port))

    def _add_noise(self, value: float) -> float:
        # stub: add uncertainty, edit later
        return value

    def destroy_node(self):
        # close UDP socket and stop
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
