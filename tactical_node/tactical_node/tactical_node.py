import queue
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math

import tactical_node.critical_region as cr
from tactical_node.comm_test import CommInterfaceTest
from tactical_node.logs import ExpLog
from tactical_node.tactical_behaviour import TacticalBehavior
import tactical_node.parameters as parameters
from tactical_node.ego_pose import EgoPose


# topics name
ROS_TOPIC_AEB = "/aeb_triggered"
ROS_TOPIC_ODOM = "/odometry/filtered"
ROS_TOPIC_REF_VEL = "/ref_spd"

# TODO set proper times for callbacks
ROS_AEB_TIMER = 10
ROS_ODOM_TIMER = 10

# TODO set proper time for main timer
ROS_TACTICAL_LOGIC_TIMER = 1/10

class TacticalNode(Node):

    def __init__(self):
        super().__init__('tactical_node')
        self.run = True
        self.q_pose = queue.SimpleQueue()
        self.q_aeb = queue.SimpleQueue()
        self.q_msg = queue.SimpleQueue()

        self.sub_aeb = self.create_subscription(
            Bool,
            ROS_TOPIC_AEB,
            self.list_aeb_callback,
            10)

        self.sub_odom = self.create_subscription(
            Odometry,
            ROS_TOPIC_ODOM,
            self.list_odom_callback,
            10)

        self.logic_timer = self.create_timer(ROS_TACTICAL_LOGIC_TIMER, self.logic_callback)

        self.pub_vel = self.create_publisher(Float32, ROS_TOPIC_REF_VEL, 2)

        critical_region_poly = cr.create_cr_polygon([parameters.CR_POINT_1,
                                                                  parameters.CR_POINT_2,
                                                                  parameters.CR_POINT_3,
                                                                  parameters.CR_POINT_4])

        target_path = cr.create_path(parameters.ADV_PATH_START, parameters.ADV_PATH_END)
        target_cr = cr.CriticalRegion(target_path, critical_region_poly)
        target_cr.compute_critical_points()

        ego_path = cr.create_path(parameters.EGO_PATH_START, parameters.EGO_PATH_END)
        ego_cr = cr.CriticalRegion(ego_path, critical_region_poly)
        ego_cr.compute_critical_points()

        # communication interface
        self.comm = CommInterfaceTest()
        self.comm.start()

        self.behaviour = TacticalBehavior(ego_reference_speed=parameters.EGO_REFERENCE_SPEED,
                                     ego_critical_region=ego_cr,
                                     target_critical_region=target_cr)

    def list_aeb_callback(self, msg):
        self.get_logger().info('AEB message is: "%s"' % msg.data)
        self.q_aeb.put(msg.data)

    def list_odom_callback(self, msg):
        self.get_logger().info('ODOM message is: "%s"' % msg.data)
        #TODO create EgoPose and queue it
        #parameters.EGO_LENGTH
        # Ectract central point of the vehicle
        center_x = msg.pose.pose.position.x
        center_y = msg.pose.pose.position.y
        # Extract orientation quaternion
        orientation_q = msg.pose.pose.orientation
        _, _, body_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        # Calculate the front and rear point of the vehicle, considering the orientiation
        front_x = center_x + parameters.EGO_LENGTH/2*math.cos(body_yaw)
        front_y = center_y + parameters.EGO_LENGTH/2*math.sin(body_yaw)
        rear_x = center_x - parameters.EGO_LENGTH/2*math.cos(body_yaw)
        rear_y = center_y - parameters.EGO_LENGTH/2*math.sin(body_yaw)
        # Extract vehicle velocity, vy is 0, since no lateral motion
        vel_x = msg.twist.twist.linear.x
        vel_y = 0
        ego_pose = EgoPose(front_x, front_y, rear_x, rear_y, vel_x, vel_y)
        self.q_pose.put(ego_pose)


    def logic_callback(self):
        cause, run = self.check_termination()
        if not run:
            self.run = False
            self.get_logger().info("stopping the car, exp terminated with cause={0}".format(cause))
            self.pub_ref_speed(0)
            exp_log = ExpLog(self.behaviour.data_log, cause)
            exp_log.write_to_file()

        if self.run:
            msg = self.comm.get_latest_message()
            try:
                ego_pose = self.q_pose.get_nowait()
            except queue.Empty:
                ego_pose = None
            action = self.behaviour.decision(msg, ego_pose)
            speed = self.behaviour.action_to_speed(action)
            self.pub_ref_speed(speed)
            self.behaviour.log()

    def check_termination(self):
        try:
            aeb = self.q_pose.get_nowait()
        except queue.Empty:
            aeb = None
        if aeb is not None:
            if aeb is True:
                return "AEB", False

        if self.behaviour.ego_d_front > self.behaviour.ego_prediction.cr.cr_path.length - 3:
            return "PASSED", False

        return None, True

    def pub_ref_speed(self, vel):
        self.pub_vel.publish(Float32(data=vel))

    def destroy_node(self):
        self.comm.stop()


def main(args=None):
    rclpy.init(args=args)

    tactical_node = TacticalNode()

    rclpy.spin(tactical_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tactical_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()