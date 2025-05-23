#!/usr/bin/env python3
import threading
import math
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from tactical_msgs.msg import LogEntry  # replace with your actual package path

class CVPlotNode(Node):
    def __init__(self):
        super().__init__('trajectory_plotter_cv2')

        # buffers for poses
        self.ego_buf = []
        self.adv_buf = []
        self.tac_front = None

        # path endpoints
        self.ego_start_pt = None
        self.ego_end_pt = None
        self.adv_start_pt = None
        self.adv_end_pt = None

        # ROS subscribers
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.create_subscription(Odometry, '/odometry/map/sim', self.ego_cb, qos)
        self.create_subscription(Odometry, '/adv_emu_odometry', self.adv_cb, qos)
        self.create_subscription(Point, '/ego_start', self.ego_start_cb, qos)
        self.create_subscription(Point, '/ego_end', self.ego_end_cb, qos)
        self.create_subscription(Point, '/adv_start', self.adv_start_cb, qos)
        self.create_subscription(Point, '/adv_end', self.adv_end_cb, qos)
        self.create_subscription(LogEntry, '/tactical_log', self.tac_cb, qos)

        # window setup
        self.win_name = 'Live Trajectories CV2'
        cv2.namedWindow(self.win_name, cv2.WINDOW_NORMAL)

        # start ROS spin in background
        threading.Thread(target=rclpy.spin, args=(self,), daemon=True).start()

        # start the drawing loop
        self.run()

    def ego_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z), 1-2*(q.z*q.z))
        self.ego_buf.append((x, y, yaw))

    def adv_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z), 1-2*(q.z*q.z))
        self.adv_buf.append((x, y, yaw))

    def tac_cb(self, msg: LogEntry):
        self.tac_front = (msg.target_front_coord_x, msg.target_front_coord_y)

    # path endpoint callbacks
    def ego_start_cb(self, msg: Point):
        self.ego_start_pt = (msg.x, msg.y)
    def ego_end_cb(self, msg: Point):
        self.ego_end_pt = (msg.x, msg.y)
    def adv_start_cb(self, msg: Point):
        self.adv_start_pt = (msg.x, msg.y)
    def adv_end_cb(self, msg: Point):
        self.adv_end_pt = (msg.x, msg.y)

    def world_to_pixel(self, x, y, img_w, img_h, scale=20):
        px = int(img_w/2 + x*scale)
        py = int(img_h/2 - y*scale)
        return px, py

    def draw_car(self, img, center, yaw, length, width, color, fill=True, dotted=False):
        cx, cy = center
        rect = ((cx, cy), (length*20, width*20), -math.degrees(yaw))
        box = cv2.boxPoints(rect)
        pts = np.int0(box)
        if fill:
            cv2.fillConvexPoly(img, pts, color)
        else:
            if dotted:
                for i in range(len(pts)):
                    p1 = tuple(pts[i])
                    p2 = tuple(pts[(i+1)%len(pts)])
                    cv2.line(img, p1, p2, color, 1, lineType=cv2.LINE_8)
            else:
                cv2.polylines(img, [pts], True, color, 2)

    def run(self):
        length = 0.720  # m
        width = 0.515   # m
        img_w, img_h = 800, 600
        while rclpy.ok():
            img = np.zeros((img_h, img_w, 3), dtype=np.uint8)

            # draw planned paths if endpoints available
            if self.ego_start_pt and self.ego_end_pt:
                p1 = self.world_to_pixel(*self.ego_start_pt, img_w, img_h)
                p2 = self.world_to_pixel(*self.ego_end_pt, img_w, img_h)
                cv2.line(img, p1, p2, (255,200,0), 1, lineType=cv2.LINE_AA)
            if self.adv_start_pt and self.adv_end_pt:
                p1 = self.world_to_pixel(*self.adv_start_pt, img_w, img_h)
                p2 = self.world_to_pixel(*self.adv_end_pt, img_w, img_h)
                cv2.line(img, p1, p2, (0,0,255), 1, lineType=cv2.LINE_AA)

            # draw trajectories
            if len(self.ego_buf) > 1:
                pts = [self.world_to_pixel(x,y,img_w,img_h) for x,y,_ in self.ego_buf]
                cv2.polylines(img, [np.array(pts)], False, (255,200,0), 2)
            if len(self.adv_buf) > 1:
                pts = [self.world_to_pixel(x,y,img_w,img_h) for x,y,_ in self.adv_buf]
                cv2.polylines(img, [np.array(pts)], False, (0,0,255), 2)

            # draw latest ego
            if self.ego_buf:
                x,y,yaw = self.ego_buf[-1]
                p = self.world_to_pixel(x,y,img_w,img_h)
                self.draw_car(img, p, yaw, length, width, (255,200,0), fill=True)

            # draw latest adv
            if self.adv_buf:
                x,y,yaw = self.adv_buf[-1]
                p = self.world_to_pixel(x,y,img_w,img_h)
                self.draw_car(img, p, yaw, length, width, (0,0,255), fill=True)

            # draw tactical dotted car
            if self.tac_front and self.adv_buf:
                xf,yf = self.tac_front
                yaw = self.adv_buf[-1][2]
                p = self.world_to_pixel(xf, yf, img_w, img_h)
                self.draw_car(img, p, yaw, length, width, (0,255,0), fill=False, dotted=True)

            cv2.imshow(self.win_name, img)
            if cv2.waitKey(30) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = CVPlotNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
