#!/usr/bin/env python3
import threading
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from nav_msgs.msg import Odometry
from tactical_msgs.msg import LogEntry  # replace with actual package name
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation


class PlotNode(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')

        # buffers for received poses
        self.ego_buf = deque(maxlen=1000)
        self.adv_buf = deque(maxlen=1000)

        # latest tactical log target front coords
        self.tac_front = None  # (x, y)

        # static path endpoints for plotting
        # these are the start and end positions (x, y) for each vehicle
        self.ego_path_start = (5.0, 0.0)
        self.ego_path_end   = (-5.0, 0.0)
        self.adv_path_start = (0.0, -5.0)
        self.adv_path_end   = (0.0, 5.0)

        # common QoS
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # subscriptions
        self.create_subscription(
            Odometry,
            '/odometry/map',
            self.ego_cb,
            qos
        )
        self.create_subscription(
            Odometry,
            '/adv_emu_odometry',
            self.adv_cb,
            qos
        )
        self.create_subscription(
            LogEntry,
            '/tactical_log',
            self.tac_cb,
            qos
        )

        # matplotlib setup
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_xlabel('X position [m]')
        self.ax.set_ylabel('Y position [m]')
        self.ax.set_title('Live Trajectories')
        self.ax.grid(True)
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)

        # trajectory lines
        self.ego_traj_line, = self.ax.plot([], [], 'b-', label='Ego trajectory')
        self.adv_traj_line, = self.ax.plot([], [], 'r-', label='Adv trajectory')
        # planned path lines
        self.ego_path_line, = self.ax.plot([], [], 'b--', label='Ego path')
        self.adv_path_line, = self.ax.plot([], [], 'r--', label='Adv path')

        # vehicle shape dimensions
        self.length = 0.720
        self.width = 0.515

        # patches for ego and adv vehicles
        self.ego_patch = Rectangle(
            (-self.length / 2, -self.width / 2),
            self.length,
            self.width,
            fc='blue',
            alpha=0.6
        )
        self.adv_patch = Rectangle(
            (-self.length / 2, -self.width / 2),
            self.length,
            self.width,
            fc='red',
            alpha=0.6
        )
        # tactical adversary patch: transparent fill, dotted border
        self.tac_patch = Rectangle(
            (-self.length, -self.width / 2),
            self.length,
            self.width,
            fc='none',
            ec='green',
            ls=':',
            lw=2,
            alpha=0.8
        )

        for patch in (self.ego_patch, self.adv_patch, self.tac_patch):
            self.ax.add_patch(patch)

        self.ax.legend()

        # start ROS spinning in background
        threading.Thread(target=rclpy.spin, args=(self,), daemon=True).start()

        # start animation
        self.anim = FuncAnimation(
            self.fig,
            self.update,
            init_func=self._init_anim,
            interval=50,
            blit=True,
            cache_frame_data=False
        )
        plt.show()

    def _init_anim(self):
        # clear all lines
        self.ego_traj_line.set_data([], [])
        self.adv_traj_line.set_data([], [])
        self.ego_path_line.set_data([], [])
        self.adv_path_line.set_data([], [])
        # reset patches off-screen
        for patch in (self.ego_patch, self.adv_patch, self.tac_patch):
            patch.set_transform(self.ax.transData)
        return (
            self.ego_path_line,
            self.adv_path_line,
            self.ego_traj_line,
            self.adv_traj_line,
            self.ego_patch,
            self.adv_patch,
            self.tac_patch
        )

    # Odometry callbacks
    def ego_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z), 1 - 2 * (q.z * q.z))
        self.ego_buf.append((x, y, yaw))

    def adv_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z), 1 - 2 * (q.z * q.z))
        self.adv_buf.append((x, y, yaw))

    def tac_cb(self, msg: LogEntry):
        # target_front_coord fields
        self.tac_front = (
            msg.target_front_coord_x,
            msg.target_front_coord_y
        )

    def update(self, frame):
        artists = []
        # plot planned paths (static)
        xs = [self.ego_path_start[0], self.ego_path_end[0]]
        ys = [self.ego_path_start[1], self.ego_path_end[1]]
        self.ego_path_line.set_data(xs, ys)
        artists.append(self.ego_path_line)

        xs2 = [self.adv_path_start[0], self.adv_path_end[0]]
        ys2 = [self.adv_path_start[1], self.adv_path_end[1]]
        self.adv_path_line.set_data(xs2, ys2)
        artists.append(self.adv_path_line)

        # update ego trajectory and patch
        if self.ego_buf:
            xs, ys, yaws = zip(*self.ego_buf)
            self.ego_traj_line.set_data(xs, ys)
            xe, ye, yeaw = self.ego_buf[-1]
            trans = (
                plt.matplotlib.transforms.Affine2D()
                .rotate_around(0, 0, yeaw)
                .translate(xe, ye)
                + self.ax.transData
            )
            self.ego_patch.set_transform(trans)
            artists += [self.ego_traj_line, self.ego_patch]

        # update adversary trajectory and patch
        if self.adv_buf:
            xs2, ys2, yaws2 = zip(*self.adv_buf)
            self.adv_traj_line.set_data(xs2, ys2)
            xa, ya, yaw = self.adv_buf[-1]
            trans2 = (
                plt.matplotlib.transforms.Affine2D()
                .rotate_around(0, 0, yaw)
                .translate(xa, ya)
                + self.ax.transData
            )
            self.adv_patch.set_transform(trans2)
            artists += [self.adv_traj_line, self.adv_patch]

        # update tactical patch at front coord
        if self.tac_front and self.adv_buf:
            xf, yf = self.tac_front
            _, _, adv_yaw = self.adv_buf[-1]
            trans3 = (
                plt.matplotlib.transforms.Affine2D()
                .rotate_around(0, 0, adv_yaw)
                .translate(xf, yf)
                + self.ax.transData
            )
            self.tac_patch.set_transform(trans3)
            self.tac_patch.set_linestyle((0, (1, 1)))
            artists.append(self.tac_patch)

        return artists


def main(args=None):
    rclpy.init(args=args)
    node = PlotNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
