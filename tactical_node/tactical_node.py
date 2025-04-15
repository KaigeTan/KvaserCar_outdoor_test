import queue
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

import critical_region
from test.comm_test import CommInterfaceTest
from logs import ExpLog
from tactical_behaviour import TacticalBehavior
import parameters
from ego_pose import EgoPose

# TODO put the correct topics name
ROS_TOPIC_AEB = "/aeb_triggered"
ROS_TOPIC_ODOM = ""
ROS_TOPIC_REF_VEL = "/ref_spd"

# TODO set proper times for callbacks
ROS_AEB_TIMER = 10
ROS_ODOM_TIMER = 10

# TODO set proper time for main timer
ROS_TACTICAL_LOGIC_TIMER = 10

class TacticalNode(Node):

    def __init__(self):
        super().__init__('tactical_node')
        self.run = True
        self.q_pose = queue.SimpleQueue()
        self.q_aeb = queue.SimpleQueue()
        self.q_msg = queue.SimpleQueue()

        self.sub_odom = self.create_subscription(
            String,
            ROS_TOPIC_AEB,
            self.list_aeb_callback,
            10)

        self.sub_AEB = self.create_subscription(
            String,
            ROS_TOPIC_ODOM,
            self.list_odom_callback,
            10)

        self.logic_timer = self.create_timer(ROS_TACTICAL_LOGIC_TIMER, self.logic_callback)

        # TODO put the correct format
        self.pub_vel = self.create_publisher(Float, ROS_TOPIC_REF_VEL, 2)

        critical_region_poly = critical_region.create_cr_polygon([parameters.CR_POINT_1,
                                                                  parameters.CR_POINT_2,
                                                                  parameters.CR_POINT_3,
                                                                  parameters.CR_POINT_4])

        target_path = critical_region.create_path(parameters.ADV_PATH_START, parameters.ADV_PATH_END)
        target_cr = critical_region.CriticalRegion(target_path, critical_region_poly)
        target_cr.compute_critical_points()

        ego_path = critical_region.create_path(parameters.EGO_PATH_START, parameters.EGO_PATH_END)
        ego_cr = critical_region.CriticalRegion(ego_path, critical_region_poly)
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
        ego_pose = EgoPose(msg.data.x, msg.data.y, 0, 0, 0,0)
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
                return "AEB", True

        if self.behaviour.ego_d_front > len(self.behaviour.ego_prediction.cr.path) - 3:
            return "PASSED", True

        return None, False

    def pub_ref_speed(self, vel):
        msg = Float32()
        msg.data = vel
        self.pub_vel.publish(vel)

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