import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
import math

import tactical_node.critical_region as cr
from tactical_node.comm_test import CommInterfaceTest
from tactical_node.logs import ExpLog
from tactical_node.tactical_behaviour import TacticalBehavior
import tactical_node.parameters as parameters
from tactical_node.ego_pose import EgoPose
from tactical_node.shared_object import SharedObj


# topics name
ROS_TOPIC_AEB = "/aeb_triggered"
ROS_TOPIC_ODOM = "/odometry/map"
ROS_TOPIC_REF_VEL = "/ref_spd"

# TODO set proper time for main timer
ROS_TACTICAL_LOGIC_TIMER = 1/10

class TacticalNode(Node):

    def __init__(self):
        super().__init__('tactical_node')
        self.run = True
        self.pose = SharedObj()
        self.aeb = SharedObj()

        self.sub_aeb = self.create_subscription(
            Bool,
            ROS_TOPIC_AEB,
            self.list_aeb_callback,
            2)

        self.sub_odom = self.create_subscription(
            Odometry,
            ROS_TOPIC_ODOM,
            self.list_odom_callback,
            2)

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
        # self.get_logger().info('AEB message is: "%s"' % msg.data)
        self.aeb.set_data(msg.data)

    def list_odom_callback(self, msg):
        #self.get_logger().info('ODOM message is: "%s"' % msg.data)
        #TODO create EgoPose and queue it
        #parameters.EGO_LENGTH
        # Extract central point of the vehicle
        center_x = msg.pose.pose.position.x
        center_y = msg.pose.pose.position.y
        # Extract orientation quaternion
        orientation_q = msg.pose.pose.orientation
        r = R.from_quat([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        # Convert to Euler angles: here 'xyz' = roll, pitch, yaw
        # `degrees=True` returns angles in degrees; omit for radians
        _, _, body_yaw = r.as_euler('xyz')
        # Calculate the front and rear point of the vehicle, considering the orientation
        front_x = center_x + parameters.EGO_LENGTH/2*math.cos(body_yaw)
        front_y = center_y + parameters.EGO_WIDTH/2*math.sin(body_yaw)
        rear_x = center_x - parameters.EGO_LENGTH/2*math.cos(body_yaw)
        rear_y = center_y - parameters.EGO_WIDTH/2*math.sin(body_yaw)
        # Extract vehicle velocity, vy is 0, since no lateral motion
        vel_x = msg.twist.twist.linear.x
        vel_y = 0
        ego_pose = EgoPose(front_x, front_y, rear_x, rear_y, vel_x, vel_y)
        self.pose.set_data(ego_pose)


    def logic_callback(self):
        cause, end = self.is_exp_ended()
        if end:
            self.get_logger().info("stopping the car, exp terminated with cause={0}".format(cause))
            self.pub_ref_speed(0.0)
            if self.run:
                exp_log = ExpLog(self.behaviour.data_log, cause)
                exp_log.write_to_file()
            self.run = False

        if self.run:
            msg = self.comm.get_latest_message()
            ego_pose = self.pose.get_data()
            action = self.behaviour.decision(msg, ego_pose)
            speed = self.behaviour.action_to_speed(action)
            self.pub_ref_speed(speed)
            self.behaviour.log()

    def is_exp_ended(self):
        aeb = self.aeb.get_data()            
        if aeb is not None:
            #self.get_logger().info("AEB IS {0}".format(aeb), throttle_duration_sec=1.0)
            if aeb is True:
                self.get_logger().warn("AEB IS TRUE", throttle_duration_sec=1.0)
                return "AEB", True
        
        self.get_logger().info("ego_front_d {0}".format(self.behaviour.ego_d_front), throttle_duration_sec=1.0)
        self.get_logger().info("path_length {0}".format(self.behaviour.ego_prediction.cr.cr_path.length), throttle_duration_sec=1.0)
        if self.behaviour.ego_d_front > self.behaviour.ego_prediction.cr.cr_path.length - 1:
            return "PASSED", True

        return None, False

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
