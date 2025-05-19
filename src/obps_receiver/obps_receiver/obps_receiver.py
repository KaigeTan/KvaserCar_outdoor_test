#!/usr/bin/env python3
import socket
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String

TOPIC_NAME = "/obps"

class UdpListenerNode(Node):
    def __init__(self):
        super().__init__('udp_listener')

        # Declare and read parameters
        self.declare_parameter('port', 9999)
        self.declare_parameter('buffer_size', 2048)
        self.declare_parameter('topic', 'udp_topic')

        port = self.get_parameter('port').value
        buf_size = self.get_parameter('buffer_size').value

        # Create publisher
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,            # only keep up to `depth` messages
            depth=1,                                       # queue size = 1
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # drop rather than retry old data
        )
        self.pub = self.create_publisher(String, TOPIC_NAME, qos)

        # Set up UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', port))
        self.sock.settimeout(1.0)  # so we can cleanly shut down

        # Start background thread to listen
        self.keep_listening = True
        self._listen_thread = threading.Thread(target=self._listen_loop,
                                               args=(buf_size,),
                                               daemon=True)
        self._listen_thread.start()

        self.get_logger().info(
            f'OBPS Receiver started on port {port}, publishing to "{TOPIC_NAME}"'
        )

    def _listen_loop(self, buf_size):
        while self.keep_listening:
            try:
                data, addr = self.sock.recvfrom(buf_size)
                text = data.decode('utf-8', errors='replace')
                self.pub.publish(String(data=text))
                self.get_logger().debug(f'[{addr}] {text!r}')
            except socket.timeout:
                # just loop again
                continue
            except OSError:
                # socket was closed; break out
                break
            except Exception as e:
                self.get_logger().error(f'UDP recv error: {e}')

        self.get_logger().info('Listener thread exiting')

    def destroy_node(self):
        # 1) signal the thread to stop
        self.get_logger().info('Shutting down: closing UDP socket…')
        self.keep_listening = False

        # 2) close socket — this will unblock recvfrom() with an OSError
        try:
            self.sock.close()
        except Exception:
            pass

        # 3) wait for thread to exit cleanly
        self._listen_thread.join(timeout=2.0)

        # 4) finally destroy the ROS node
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UdpListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
