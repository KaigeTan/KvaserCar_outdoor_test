import time

import tactical_node.critical_region as critical_region
import tactical_node.parameters as parameters
from comm_test import CommInterfaceTest
from tactical_node.ego_pose import EgoPose
from tactical_node.logs import ExpLog
from tactical_node.tactical_behaviour import TacticalBehavior



def check_termination(b: TacticalBehavior):
    if b.ego_d_front > len(b.ego_prediction.cr.path) - 3:
        return "PASSED", True

    return None, False

def pub_ego_ref_speed(vel):
    pass

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

  target_cr.plot_regions("TARGET")
  ego_cr.plot_regions("EGO")

  # communication interface
  comm = CommInterfaceTest()
  comm.connect()

  behaviour = TacticalBehavior(ego_reference_speed=parameters.EGO_REFERENCE_SPEED,
                               ego_critical_region=ego_cr,
                               target_critical_region=target_cr)

  run = True
  cause = "Unknown"
  for i in range(50):
      time.sleep(0.1)
      if run:
          msg = comm.get_latest_message()
          ego_pose = EgoPose(0,0, 0,0 ,0,0)
          action = behaviour.decision(msg, ego_pose)
          speed = behaviour.action_to_speed(action)
          pub_ego_ref_speed(speed)
          behaviour.log()
          cause, run = check_termination(behaviour)
      else:
          print("stopping the car, exp terminated with cause={0}".format(cause))
          pub_ego_ref_speed(0)
          exp_log = ExpLog(behaviour.data_log, cause)
          exp_log.write_to_file()


  print(behaviour.data_log)
  comm.disconnect()