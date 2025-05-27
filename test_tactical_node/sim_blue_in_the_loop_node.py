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


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class Motion:
    def __init__(self, max_acc: float):
        self.v = 0.0
        self.d_total = 0.0
        self.acc = max_acc

    def get_displacement(self, ref_speed: float, delta_t: float):
        v_old = self.v
        if self.v < ref_speed:
            v_new = min(v_old + self.acc * delta_t, ref_speed)
        elif v_old > ref_speed:
            v_new = max(v_old - self.acc * delta_t, ref_speed)
        else:
            v_new = v_old

        d = 0.5 * (v_old + v_new) * delta_t
        self.v = v_new
        self.d_total += d
        return d, self.d_total


class SimBlueInTheLoop(Node):
    def __init__(self):
        super().__init__('sim_blue_in_the_loop')

        # --- Startup prompt & TCP handshake ---
        ans = input("start in the loop sim (y) ")
        if ans.strip().lower() == 'y':
            blue_ip = self.declare_parameter('BLUE_IP', '127.0.0.1').value
            blue_port = self.declare_parameter('BLUE_PORT', 9999).value
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((blue_ip, blue_port))
                sock.sendall(b'start')
                sock.close()
                self.get_logger().info(f"Sent 'start' to {blue_ip}:{blue_port}")
            except Exception as e:
                self.get_logger().error(f"Failed to send start: {e}")

        # --- Declare simulation parameters ---
        sx = self.declare_parameter('adv_start_x', 0.0).value
        sy = self.declare_parameter('adv_start_y', -5.0).value
        ex = self.declare_parameter('adv_end_x', 0.0).value
        ey = self.declare_parameter('adv_end_y', 5.0).value
        self.adv_ref_speed = self.declare_parameter('adv_ref_speed', 2.5).value
        adv_max_acc = self.declare_parameter('adv_max_acc', 2.5).value

        # --- UDP & communication parameters ---
        self.udp_ip = self.declare_parameter('udp_target_ip', '127.0.0.1').value
        self.udp_port = self.declare_parameter('udp_target_port', 9999).value
        udp_delay = self.declare_parameter('udp_delay', 0.05).value
        self._udp_delay_ns = int(udp_delay * 1e9)
        self.add_aoi = self.declare_parameter('add_aoi', 0).value * 1e6
        self.adv_queue_size = self.declare_parameter('adv_queue_size', 1).value
        cfs = self.declare_parameter('comm_fail_start', 0.02).value
        cfd = self.declare_parameter('comm_fail_duration', 1.5).value

        sim_step = self.declare_parameter('sim_step', 0.03).value
        comm_step = self.declare_parameter('comm_step', 0.001).value

        # --- Setup adversary motion ---
        self.adv_motion = Motion(adv_max_acc)
        dx, dy = ex - sx, ey - sy
        dist = math.hypot(dx, dy)
        self.adv_dir = (dx/dist, dy/dist)
        self.adv_path_dist = dist
        self.adv_start = (sx, sy)
        self.adv_yaw = math.atan2(dy, dx)

        # --- Publishers ---
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        topic = self.declare_parameter('ADV_ODOM_TOPIC', '/adv_emu_odometry').value
        self.adv_pub = self.create_publisher(Odometry, topic, qos)

        # --- UDP socket & queue ---
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.adv_queue = deque()
        self._msg_id = 0
        self.start_ns = self.get_clock().now().nanoseconds
        self._comm_fail_start = self.start_ns + int(cfs * 1e9)
        self._comm_fail_end = self._comm_fail_start + int(cfd * 1e9)

        # --- Timers ---
        self.sim_timer = self.create_timer(sim_step, self._simulate)
        self.comm_timer = self.create_timer(comm_step, self._send_udp)
        self.get_logger().info('In-the-loop sim: publishing and UDP sending enabled')

    def _simulate(self):
        # Stop simulation when done
        if self.adv_motion.d_total >= self.adv_path_dist:
            self.get_logger().info('Adv reached goal; stopping sim timer')
            self.sim_timer.cancel()
            return

        # Compute next adv pose
        _, _ = self.adv_motion.get_displacement(
            self.adv_ref_speed,
            self.sim_timer.timer_period_ns / 1e9
        )
        ax = self.adv_start[0] + self.adv_dir[0] * self.adv_motion.d_total
        ay = self.adv_start[1] + self.adv_dir[1] * self.adv_motion.d_total

        # Publish odometry
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'adv_base_link'
        msg.pose.pose.position.x = ax
        msg.pose.pose.position.y = ay
        msg.pose.pose.orientation = quaternion_from_yaw(self.adv_yaw)
        msg.twist.twist.linear.x = self.adv_motion.v
        self.adv_pub.publish(msg)

        # Queue UDP payload
        if len(self.adv_queue) < self.adv_queue_size:
            now = self.get_clock().now().nanoseconds
            payload = {
                'id': self._msg_id,
                't_stamp': now,
                'front_x': ax,
                'front_y': ay,
                'vel': self.adv_motion.v
            }
            self.adv_queue.append(payload)
            self._msg_id += 1

    def _send_udp(self):
        now_ns = self.get_clock().now().nanoseconds
        # Stop UDP when done
        if self.adv_motion.d_total >= self.adv_path_dist:
            self.get_logger().info('Adv done; stopping UDP timer')
            self.comm_timer.cancel()
            return

        # Suppress during comm failure
        if self._comm_fail_start <= now_ns < self._comm_fail_end:
            print(f"{(now_ns - self.start_ns)/1e9} COMM FAILURE")
            return

        # Send packet if delay elapsed
        if self.adv_queue and (now_ns - self.adv_queue[0]['t_stamp'] >= self._udp_delay_ns):
            info = self.adv_queue.popleft()
            info['t_stamp'] -= self.add_aoi
            print(f"{(now_ns - self.start_ns)/1e9} SENDING {(info['t_stamp'] - self.start_ns)/1e9} diff {(now_ns - info['t_stamp'])/1e9}")
            packet = json.dumps(info).encode('utf-8')
            self.udp_sock.sendto(packet, (self.udp_ip, self.udp_port))

    def destroy_node(self):
        self.get_logger().info('Shutting down, closing UDP socket')
        self.udp_sock.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimBlueInTheLoop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
