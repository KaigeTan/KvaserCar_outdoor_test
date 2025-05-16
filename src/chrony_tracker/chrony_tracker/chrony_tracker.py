#!/usr/bin/env python3
import re
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class ChronyTrackerNode(Node):
    def __init__(self):
        super().__init__('chrony_tracker')
        # publishers
        self.pub_system = self.create_publisher(Float64, 'chrony/system_time_offset', 10)
        self.pub_last   = self.create_publisher(Float64, 'chrony/last_offset',        10)
        self.pub_rms    = self.create_publisher(Float64, 'chrony/rms_offset',         10)

        # run every second
        self.timer = self.create_timer(1.0, self.update_tracking)

        # regexes
        self.rx_system = re.compile(r'^System time\s*:\s*([+-]?\d+\.\d+)\s*seconds\s*(slow|fast)', re.MULTILINE)
        self.rx_last   = re.compile(r'^Last offset\s*:\s*([+-]?\d+\.\d+)\s*seconds',             re.MULTILINE)
        self.rx_rms    = re.compile(r'^RMS offset\s*:\s*([+-]?\d+\.\d+)\s*seconds',              re.MULTILINE)

    def update_tracking(self):
        try:
            out = subprocess.check_output(['chronyc', 'tracking'], stderr=subprocess.STDOUT)
            txt = out.decode('utf-8')
            #self.get_logger().info(f"chronyc message : {txt}")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"chronyc failed: {e.output.decode().strip()}")
            return

        # parse
        sys_val = self.parse_system(txt)
        last_val = self.parse_last(txt)
        rms_val = self.parse_rms(txt)

        if sys_val is not None:
            msg = Float64(data=sys_val)
            self.pub_system.publish(msg)
        if last_val is not None:
            msg = Float64(data=last_val)
            self.pub_last.publish(msg)
        if rms_val is not None:
            msg = Float64(data=rms_val)
            self.pub_rms.publish(msg)

    def parse_system(self, text):
        m = self.rx_system.search(text)
        if not m:
            return None
        val = float(m.group(1))
        # if “slow” → system is behind NTP, so negative offset
        if m.group(2) == 'slow':
            return -val
        else:
            return +val

    def parse_last(self, text):
        m = self.rx_last.search(text)
        return float(m.group(1)) if m else None

    def parse_rms(self, text):
        m = self.rx_rms.search(text)
        return float(m.group(1)) if m else None

def main(args=None):
    rclpy.init(args=args)
    node = ChronyTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
