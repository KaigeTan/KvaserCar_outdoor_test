import time

import shapely

import src.tactical_node.tactical_node.critical_region as critical_region
import test_params as parameters
from comm_test import CommInterfaceTest
from ego_pose import EgoPose
from logs import ExpLog
from tactical_behaviour import TacticalBehavior
from tests_tactical_node.motion import Motion


def is_ended(b: TacticalBehavior, front_ego, front_target):
    if b.ego_d_front > b.ego_prediction.cr.cr_path.length - 3:
        return "PASSED", True

    if shapely.distance(front_target, front_ego) < 0.1:
        return "CRASH", True

    return None, False


def pub_ego_ref_speed(vel):
    print("pub speed: {0}".format(vel))


if __name__ == '__main__':

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

    #target_cr.plot_regions("TARGET")
    #ego_cr.plot_regions("EGO")

    # communication interface
    comm = CommInterfaceTest()
    comm.start()

    behaviour = TacticalBehavior(ego_reference_speed=parameters.EGO_REFERENCE_SPEED,
                                 ego_critical_region=ego_cr,
                                 target_critical_region=target_cr)

    ego_motion = Motion(max_acc=parameters.EGO_MAX_ACC)
    adv_motion = Motion(max_acc=parameters.ADV_MAX_ACC)

    print((time.perf_counter_ns() // (1000 * 1000)) - 50)
    print(time.perf_counter_ns() // (1000 * 1000))

    total_dist_ego = 0
    total_dist_target = 0
    end = False
    cause = "Unknown"
    ego_ref = parameters.EGO_REFERENCE_SPEED
    log_debug = {"target_front_d":list(), "ego_front_d":list(), "ego_v":list(), "target_v":list()}
    while 1:
        time.sleep(0.01)
        if end:
            print("stopping the car, exp terminated with cause={0}".format(cause))
            pub_ego_ref_speed(0.0)
            exp_log = ExpLog(behaviour.data_log, log_debug, cause)
            exp_log.write_to_file()
            break

        _, total_dist_target = adv_motion.get_displacement(parameters.ADV_REFERENCE_SPEED, 0.01)
        front_target = target_path.interpolate(total_dist_target)
        log_debug["target_front_d"].append(total_dist_target)
        log_debug["target_v"].append(adv_motion.v)

        _, total_dist_ego = ego_motion.get_displacement(ego_ref, 0.01)
        front_ego = ego_path.interpolate(total_dist_ego)
        log_debug["ego_front_d"].append(total_dist_ego)
        log_debug["ego_v"].append(ego_motion.v)

        msg = comm.get_test_message(adv_motion.v, (front_target.x, front_target.y))
        #msg = None
        ego_pose = EgoPose(front_ego.x,
                           front_ego.y,
                           front_ego.x - parameters.EGO_LENGTH,
                           front_ego.y - parameters.EGO_LENGTH,
                           ego_motion.v,
                           0)

        action = behaviour.decision(msg, ego_pose)
        ego_ref = behaviour.action_to_speed(action)
        pub_ego_ref_speed(ego_ref)
        behaviour.log()
        cause, end = is_ended(behaviour, front_ego, front_target)

    comm.stop()
