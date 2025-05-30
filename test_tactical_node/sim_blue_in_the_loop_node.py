#!/usr/bin/env python3
import socket
import json
import math
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from nav_msgs.msg import Odometry
from tactical_msgs.msg import LogEntry  # replace with actual package name
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation


def quaternion_from_yaw(yaw: float):
    q = Odometry().pose.pose.orientation.__class__()
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
        if v_old < ref_speed:
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

        # --- startup prompt & TCP handshake ---
        ans = input("start in the loop sim (y) ")
        if ans.strip().lower() == 'y':
            blue_ip = self.declare_parameter('BLUE_IP', '192.168.1.202').value
            blue_port = self.declare_parameter('BLUE_PORT', 9999).value
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((blue_ip, 9000))
                start_id = "test_id"
                message = {"cmd": "start", "start_id": start_id}
                message_str = json.dumps(message)
                sock.sendall(message_str.encode('utf-8'))
                sock.close()
                self.get_logger().info(f"Sent 'start' to {blue_ip}:{9000}")
            except Exception as e:
                self.get_logger().error(f"Failed to send start: {e}")

        # --- simulation parameters ---
        sx = self.declare_parameter('adv_start_x', 0.0).value
        sy = self.declare_parameter('adv_start_y', -5.0).value
        ex = self.declare_parameter('adv_end_x', 0.0).value
        ey = self.declare_parameter('adv_end_y', 5.0).value
        self.adv_ref_speed = self.declare_parameter('adv_ref_speed', 2.5).value
        adv_max_acc = self.declare_parameter('adv_max_acc', 2.5).value

        # --- UDP parameters ---
        self.udp_ip = self.declare_parameter('udp_target_ip', '192.168.1.202').value
        self.udp_port = self.declare_parameter('udp_target_port', 9999).value
        udp_delay = self.declare_parameter('udp_delay', 0.05).value
        self._udp_delay_ns = int(udp_delay * 1e9)
        self.adv_queue_size = self.declare_parameter('adv_queue_size', 1).value
        cfs = self.declare_parameter('comm_fail_start', 0.02).value
        cfd = self.declare_parameter('comm_fail_duration', 1.5).value

        sim_step = self.declare_parameter('sim_step', 0.03).value
        comm_step = self.declare_parameter('comm_step', 0.001).value

        # --- motion setup ---
        self.adv_motion = Motion(adv_max_acc)
        dx, dy = ex - sx, ey - sy
        dist = math.hypot(dx, dy)
        self.adv_dir = (dx / dist, dy / dist)
        self.adv_path_dist = dist
        self.adv_start = (sx, sy)
        self.adv_yaw = math.atan2(dy, dx)

        # --- data buffers for plotting ---
        self.ego_buf = deque(maxlen=1000)
        self.adv_buf = deque(maxlen=1000)
        self.tac_front = None

        # --- static path endpoints ---
        self.ego_path_start = (5.0, 0.0)
        self.ego_path_end = (-5.0, 0.0)
        self.adv_path_start = (0.0, -5.0)
        self.adv_path_end = (0.0, 5.0)

        # --- subscriptions (only input needed) ---
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                         depth=2,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, '/odometry/map', self.ego_cb, qos)
        self.create_subscription(LogEntry, '/tactical_log', self.tac_cb, qos)

        # --- UDP socket & queue ---
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.adv_queue = deque()
        self._msg_id = 0
        self.start_ns = self.get_clock().now().nanoseconds
        self._comm_fail_start = self.start_ns + int(cfs * 1e9)
        self._comm_fail_end = self._comm_fail_start + int(cfd * 1e9)

        # --- plotting setup ---
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_title('Live Trajectories')
        self.ax.grid(True)
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)

        self.ego_traj_line, = self.ax.plot([], [], 'b-', label='Ego traj')
        self.adv_traj_line, = self.ax.plot([], [], 'r-', label='Adv traj')
        self.ego_path_line, = self.ax.plot([], [], 'b--', label='Ego path')
        self.adv_path_line, = self.ax.plot([], [], 'r--', label='Adv path')

        length, width = 0.72, 0.515
        self.ego_patch = Rectangle((-length/2, -width/2), length, width, fc='blue', alpha=0.6)
        self.adv_patch = Rectangle((-length/2, -width/2), length, width, fc='red', alpha=0.6)
        self.tac_patch = Rectangle((-length, -width/2), length, width,
                                   fc='none', ec='green', ls=':', lw=2, alpha=0.8)
        for p in (self.ego_patch, self.adv_patch, self.tac_patch):
            self.ax.add_patch(p)
        self.ax.legend()

        # --- timers ---
        self.sim_timer = self.create_timer(sim_step, self._simulate)
        self.comm_timer = self.create_timer(comm_step, self._send_udp)

        # --- start ROS spinning and animation ---
        threading.Thread(target=rclpy.spin, args=(self,), daemon=True).start()
        self.anim = FuncAnimation(self.fig, self.update, init_func=self._init_anim,
                                  interval=50, blit=True)
        try:
            plt.show()
        except KeyboardInterrupt:
            self.get_logger().info('Keyboard interrupt received; shutting down')
            self.destroy_node()
            rclpy.shutdown()

    def _init_anim(self):
        for line in (self.ego_traj_line, self.adv_traj_line,
                     self.ego_path_line, self.adv_path_line):
            line.set_data([], [])
        for patch in (self.ego_patch, self.adv_patch, self.tac_patch):
            patch.set_transform(self.ax.transData)
        return (self.ego_path_line, self.adv_path_line,
                self.ego_traj_line, self.adv_traj_line,
                self.ego_patch, self.adv_patch, self.tac_patch)

    def _simulate(self):
        if self.adv_motion.d_total >= self.adv_path_dist:
            self.get_logger().info('Adv reached goal; stopping sim')
            self.sim_timer.cancel()
            return
        self.adv_motion.get_displacement(self.adv_ref_speed,
                                         self.sim_timer.timer_period_ns/1e9)
        ax = self.adv_start[0] + self.adv_dir[0]*self.adv_motion.d_total
        ay = self.adv_start[1] + self.adv_dir[1]*self.adv_motion.d_total
        yaw = self.adv_yaw
        self.adv_buf.append((ax, ay, yaw))
        if len(self.adv_queue) < self.adv_queue_size:
            now_ns = self.get_clock().now().nanoseconds
            payload = {'id': self._msg_id,
                       't_stamp': now_ns,
                       'front_x': ax,
                       'front_y': ay,
                       'vel': self.adv_motion.v}
            self.adv_queue.append(payload)
            self._msg_id += 1

    def _send_udp(self):
        now_ns = self.get_clock().now().nanoseconds
        if self.adv_motion.d_total >= self.adv_path_dist:
            self.get_logger().info('Stopping UDP')
            self.comm_timer.cancel()
            return
        if self._comm_fail_start <= now_ns < self._comm_fail_end:
            return
        if (self.adv_queue and
            now_ns - self.adv_queue[0]['t_stamp'] >= self._udp_delay_ns):
            info = self.adv_queue.popleft()
            packet = json.dumps(info).encode('utf-8')
            self.udp_sock.sendto(packet, (self.udp_ip, self.udp_port))

    def ego_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z), 1 - 2*(q.z*q.z))
        self.ego_buf.append((x, y, yaw))

    def tac_cb(self, msg: LogEntry):
        self.tac_front = (msg.target_front_coord_x,
                          msg.target_front_coord_y)

    def update(self, frame):
        artists = []
        xs = [self.ego_path_start[0], self.ego_path_end[0]]
        ys = [self.ego_path_start[1], self.ego_path_end[1]]
        self.ego_path_line.set_data(xs, ys)
        artists.append(self.ego_path_line)
        xs2 = [self.adv_path_start[0], self.adv_path_end[0]]
        ys2 = [self.adv_path_start[1], self.adv_path_end[1]]
        self.adv_path_line.set_data(xs2, ys2)
        artists.append(self.adv_path_line)

        if self.ego_buf:
            xs, ys, yaws = zip(*self.ego_buf)
            self.ego_traj_line.set_data(xs, ys)
            xe, ye, yeaw = self.ego_buf[-1]
            trans = (plt.matplotlib.transforms.Affine2D()
                     .rotate_around(0, 0, yeaw)
                     .translate(xe, ye)
                     + self.ax.transData)
            self.ego_patch.set_transform(trans)
            artists.extend([self.ego_traj_line, self.ego_patch])

        if self.adv_buf:
            xs2, ys2, yaws2 = zip(*self.adv_buf)
            self.adv_traj_line.set_data(xs2, ys2)
            xa, ya, yaw = self.adv_buf[-1]
            trans2 = (plt.matplotlib.transforms.Affine2D()
                      .rotate_around(0, 0, yaw)
                      .translate(xa, ya)
                      + self.ax.transData)
            self.adv_patch.set_transform(trans2)
            artists.extend([self.adv_traj_line, self.adv_patch])

        if self.tac_front and self.adv_buf:
            xf, yf = self.tac_front
            _, _, adv_yaw = self.adv_buf[-1]
            trans3 = (plt.matplotlib.transforms.Affine2D()
                      .rotate_around(0, 0, adv_yaw)
                      .translate(xf, yf)
                      + self.ax.transData)
            self.tac_patch.set_transform(trans3)
            artists.append(self.tac_patch)

        return artists

    def destroy_node(self):
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
