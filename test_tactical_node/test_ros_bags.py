import math
import time
from scipy.spatial.transform import Rotation as R

import test_tactical_node.extract_ros_bags as extract
import tactical_node.critical_region as cr
from tactical_node.comm_test import CommInterfaceTest
from tactical_node.logs import ExpLog
from tactical_node.tactical_behaviour import TacticalBehavior
import tactical_node.parameters as parameters
from tactical_node.ego_pose import EgoPose
from tactical_node.comm_msg import ComMsg



def get_ego_pose(msg):
    # Extract central point of the vehicle
    center_x, center_y, vel_fw, orientation_q = msg
    if center_x is None:
        return None
    # Extract orientation quaternion
    r = R.from_quat(orientation_q)
    # Convert to Euler angles: here 'xyz' = roll, pitch, yaw
    # `degrees=True` returns angles in degrees; omit for radians
    _, _, body_yaw = r.as_euler('xyz', degrees=True)
    # Calculate the front and rear point of the vehicle, considering the orientation
    front_x = center_x + parameters.EGO_LENGTH/2*math.cos(body_yaw)
    front_y = center_y + parameters.EGO_WIDTH/2*math.sin(body_yaw)
    rear_x = center_x - parameters.EGO_LENGTH/2*math.cos(body_yaw)
    rear_y = center_y - parameters.EGO_WIDTH/2*math.sin(body_yaw)
    # Extract vehicle velocity, vy is 0, since no lateral motion
    vel_x = vel_fw
    vel_y = 0
    ego_pose = EgoPose(front_x, front_y, rear_x, rear_y, vel_x, vel_y)
    print("pos_x ",front_x)
    return ego_pose



def is_exp_ended(aeb_msg, behaviour):
    if aeb_msg is not None:
        if aeb_msg is True:
            print("AEB IS TRUE")
            return "AEB", True

    if behaviour.ego_d_front > behaviour.ego_prediction.cr.cr_path.length - 1:
        print("ego_d_front ", behaviour.ego_d_front)
        return "PASSED", True

    return None, False

def pub_ref_speed(val):
    pass

def get_obps_msg():
    return ComMsg(0,
                    time.perf_counter_ns() // (1000 * 1000),
                    (0.5 , 0),
                    (0.5, 0 - parameters.ADV_LENGTH),
                        0)

def main():

    #read all messages from the rosbag
    bag_path_1 = "/home/gianfi/Documents/KvaserCar_outdoor_test/recorded_rosbag/rosbag2_1970_01_01-02_30_13"
    bag_path_2 = "/home/gianfi/Documents/KvaserCar_outdoor_test/recorded_rosbag/rosbag2_1970_01_01-02_31_15"
    bag_path_3 = "/home/gianfi/Documents/KvaserCar_outdoor_test/recorded_rosbag/rosbag2_1970_01_01-02_31_50"
    ros_msgs = extract.get_messages(bag_path_1)
                    
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

    target_cr.plot_regions("target")
    ego_cr.plot_regions("ego")

    # communication interface
    comm = CommInterfaceTest()
    comm.start()
            
    behaviour = TacticalBehavior(ego_reference_speed=parameters.EGO_REFERENCE_SPEED,
                                     ego_critical_region=ego_cr,
                                     target_critical_region=target_cr)
    
    run = True

    for i in range(len(ros_msgs["t"])):

        cause, end = is_exp_ended(ros_msgs["aeb_trg"][i], behaviour)
        if end:
            print("stopping the car, exp terminated with cause={0}".format(cause))
            pub_ref_speed(0.0)
            if run:
                exp_log = ExpLog(behaviour.data_log, cause)
                #exp_log.write_to_file()
                break

        msg = None
        msg = get_obps_msg()
        ego_pose = get_ego_pose(ros_msgs["odom"][i])

        action = behaviour.decision(msg, ego_pose)
        speed = behaviour.action_to_speed(action)
        pub_ref_speed(speed)
        behaviour.log()



if __name__ == '__main__':
    main()

