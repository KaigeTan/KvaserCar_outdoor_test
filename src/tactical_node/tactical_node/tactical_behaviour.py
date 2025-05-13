import time
from enum import IntEnum

import shapely

from tactical_node.critical_region import CriticalRegion
from tactical_node.ego_pose import EgoPose
from tactical_node.ego_prediction import EgoPrediction
from tactical_node.kalman_filter import KalmanFilter
from tactical_node.target_prediction import TargetPrediction
import tactical_node.parameters as parameters
from  tactical_node.comm_msg import ComMsg



def get_time() -> int:
    return time.time_ns()


class TacticalAction(IntEnum):
    BREAKING = 0
    CONTINUE = 1


class TacticalBehavior:
    def __init__(self,
                 ego_reference_speed: float,
                 ego_critical_region: CriticalRegion,
                 target_critical_region: CriticalRegion):

        self.target_prediction = TargetPrediction(0, target_critical_region)
        self.ego_prediction = EgoPrediction(ego_reference_speed,
                                            parameters.EGO_MAX_DEC,
                                            parameters.EGO_LENGTH,
                                            ego_critical_region)
        self.aoi = -1
        self.msg = None
        self.ego_ttcr = -1
        self.target_ttcr = -1
        self.ego_d_to_cr = -1
        self.ego_d_front = -1
        self.target_d_to_cr = -1
        self.ego_pos = CriticalRegion.Position.UNKNOWN
        self.ego_pred_go_pos = CriticalRegion.Position.UNKNOWN
        self.ego_pred_break_pos = CriticalRegion.Position.UNKNOWN
        self.target_pred_pos = CriticalRegion.Position.UNKNOWN
        self.reference_speed = ego_reference_speed
        self.ego_tactical_speed = self.reference_speed
        self.ego_action = TacticalAction.CONTINUE
        self.target_acc = -1
        self.kalman_f = KalmanFilter(1)
        self.decision_time = -1
        self.data_log = {"decision_time": list(),
                         "aoi": list(),
                         "ego_tactical_speed": list(),
                         "ego_front_d": list(),
                         "ego_pos": list(),
                         "ego_pred_go_pos": list(),
                         "ego_d_to_cr": list(),
                         "ego_ttcr": list(),
                         "target_acc": list(),
                         "target_ttcr": list(),
                         "target_d_to_cr": list(),
                         "target_pos": list(),
                         "target_d_front": list()}

    def _get_target_acc(self, aoi, velocity):

        kalman_acc = self.kalman_f.predict_acc(aoi, velocity)
        if kalman_acc > parameters.ADV_MAX_ACC:
            print("kalman acc is higher: {0}, assumption acc: {1}".format(kalman_acc, parameters.ADV_MAX_ACC))
            return kalman_acc

        return parameters.ADV_MAX_ACC

    def decision(self, msg: ComMsg, ego_pose: EgoPose) -> TacticalAction:
        """


        :param msg:
        :param ego_pose:
        :return:
        """
        self.ego_action = TacticalAction.BREAKING
        if ego_pose is None:
            return self.ego_action

        self.ego_action = TacticalAction.CONTINUE
        ego_front_p = shapely.Point((ego_pose.front_x, ego_pose.front_y))
        shapely.prepare(ego_front_p)
        ego_vel = ego_pose.vel_fw
        ego_acc = ego_pose.acc_fw        
        
        self.msg = msg
        self.ego_d_front, self.ego_d_to_cr, self.ego_ttcr = self.ego_prediction.get_dist_and_time_to_cr(ego_vel, ego_front_p)
        if self.msg is None:
            return self.ego_action

        self.decision_time = get_time()
        self.aoi = self.decision_time - self.msg.time_stamp
        self.aoi = self.aoi * 0.001 #transform from milli!
        target_length = self.msg.length if self.msg.length is not None else parameters.ADV_LENGTH
        target_front = shapely.Point(self.msg.front[0], self.msg.front[1])
        shapely.prepare(target_front)
        self.target_acc = self._get_target_acc(self.aoi, self.msg.velocity)
        self.target_pred_pos, self.target_ttcr = self.target_prediction.estimate_time_to_cr(
            self.aoi,
            target_front,
            target_length,
            self.msg.velocity,
            self.target_acc)

        self.ego_action = TacticalAction.CONTINUE

        if self.target_pred_pos == CriticalRegion.Position.BEFORE_CR:

            pred_go_ttcr, pred_break_ttcr = self.ego_prediction.get_predicted_positions(ego_vel,
                                                                                        ego_acc,
                                                                                        self.target_ttcr,
                                                                                        ego_front_p)
            if pred_go_ttcr == CriticalRegion.Position.AFTER_CR:
                self.ego_action = TacticalAction.CONTINUE
            else:
                t_to_leave_cr = self.target_prediction.get_time_to_leave_cr(self.msg.velocity, self.target_acc)
                pred_go_leave_ttcr, _ = self.ego_prediction.get_predicted_positions(ego_vel,
                                                                                    ego_acc,
                                                                                    t_to_leave_cr,
                                                                                    ego_front_p)
                self.ego_pos = pred_go_leave_ttcr

                if pred_go_leave_ttcr == CriticalRegion.Position.INSIDE_CR:
                    self.ego_action = TacticalAction.BREAKING
                elif pred_go_leave_ttcr == CriticalRegion.Position.BEFORE_CR:
                    self.ego_action = TacticalAction.CONTINUE

        elif self.target_pred_pos == CriticalRegion.Position.AFTER_CR:
            self.ego_action = TacticalAction.CONTINUE

        elif self.target_pred_pos == CriticalRegion.Position.INSIDE_CR:
            _, _, self.ego_pos = self.ego_prediction.get_current_pos(ego_front_p)
            if self.ego_pos == CriticalRegion.Position.AFTER_CR:
                self.ego_action = TacticalAction.CONTINUE
            else:
                t_to_leave_cr = self.target_prediction.get_time_to_leave_cr(self.msg.velocity, self.target_acc)
                pred_go_leave_ttcr, _ = self.ego_prediction.get_predicted_positions(ego_vel,
                                                                                    ego_acc,
                                                                                    t_to_leave_cr,
                                                                                    ego_front_p)
                if pred_go_leave_ttcr != CriticalRegion.Position.BEFORE_CR:
                    self.ego_pos = pred_go_leave_ttcr
                    self.ego_action = TacticalAction.BREAKING

        self.ego_d_front, self.ego_d_to_cr, self.ego_ttcr = self.ego_prediction.get_dist_and_time_to_cr(ego_vel, ego_front_p)

        self.target_d_to_cr = self.target_prediction.dist_to_cr

        return self.ego_action

    def action_to_speed(self, action: TacticalAction):
        if action == TacticalAction.CONTINUE:
            self.ego_tactical_speed = self.reference_speed
        else:
            self.ego_tactical_speed = 0.0

        return self.ego_tactical_speed

    def log(self):
        self.data_log["decision_time"].append(self.decision_time)
        self.data_log["ego_tactical_speed"].append(self.ego_tactical_speed)
        self.data_log["ego_front_d"].append(self.ego_d_front)
        self.data_log["ego_pos"].append(self.ego_pos)
        self.data_log["ego_pred_go_pos"].append(self.ego_pred_go_pos)
        self.data_log["ego_d_to_cr"].append(self.ego_d_to_cr)
        self.data_log["ego_ttcr"].append(self.ego_ttcr)
        self.data_log["aoi"].append(self.aoi)

        self.data_log["target_acc"].append(self.target_acc)
        self.data_log["target_ttcr"].append(self.target_ttcr)
        self.data_log["target_d_to_cr"].append(self.target_d_to_cr)
        self.data_log["target_pos"].append(self.target_pred_pos)
        self.data_log["target_d_front"].append(self.target_prediction.d_front)
