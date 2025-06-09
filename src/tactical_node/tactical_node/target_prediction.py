
import shapely
import math
from tactical_node.critical_region import CriticalRegion

class TargetPrediction:

    NO_TIME_TO_CR = -1

    def __init__(self,
                 target_id,
                 critical_region: CriticalRegion,
                 max_speed
                 ):

        self.id = target_id
        self.cr = critical_region
        self.target_pos = CriticalRegion.Position.UNKNOWN
        self.d_front = -1
        self.d_rear = -1
        self.dist_to_cr = -1
        self.max_speed = max_speed
        self.v_delta = 0.1

    def at_max_speed(self, target_vel)-> bool:
        return math.fabs(target_vel - self.max_speed) < self.v_delta
           

    def project_to_path(self, front: shapely.Point, target_length: float, displacement: float):
        # project the detected front on the critical path and find its distance
        d_front = shapely.line_locate_point(self.cr.cr_path, front) + displacement

        # find the rear point on the critical path
        d_rear = d_front - target_length
        d_rear = d_rear if d_rear > 0 else -1
        return d_front, d_rear
    

    def get_predicted_aoi_displacement(self, delta_time: float, current_vel:float, target_acc:float):
        if self.at_max_speed():
            return self.max_speed * delta_time
        else:
            t_x = (self.max_speed - current_vel) / target_acc
            if delta_time - t_x >= 0:
                d_1 = current_vel * t_x + 0.5 * target_acc * t_x ** 2
                d_2 = self.max_speed * (delta_time - t_x)
                d = d_1 + d_2
                return d
            else:
                d = current_vel * delta_time + 0.5 * target_acc * delta_time ** 2
                return d        



    def get_cr_relative_position(self,
                                 aoi:float,
                                 target_vel:float,
                                 target_acc:float,
                                 target_length:float,
                                 front: shapely.Point):
        displacement = self.get_predicted_aoi_displacement(aoi, target_vel, target_acc)

        self.d_front, self.d_rear = self.project_to_path(front, target_length, displacement)

        self.dist_to_cr = max(0, self.cr.cn_orig_d - self.d_front)

        if self.d_front < self.cr.cn_orig_d:
            self.target_pos = CriticalRegion.Position.BEFORE_CR
            return self.target_pos

        cond_1 = self.cr.cn_orig_d <= self.d_front <= self.cr.cf_orig_d
        cond_2 = self.cr.cn_orig_d <= self.d_rear <= self.cr.cf_orig_d

        if cond_1 or cond_2:
            self.target_pos = CriticalRegion.Position.INSIDE_CR
            return self.target_pos

        if self.d_rear > self.cr.cf_orig_d:
            self.target_pos = CriticalRegion.Position.AFTER_CR
            return self.target_pos


    def estimate_time_to_cr(self,
                            aoi: float,
                            front: shapely.Point,
                            target_len: float,
                            target_vel:float,
                            target_acc:float):
        """
        # Compensate for the AoI by calculating the displacement that the target makes
        # in delta_t = (current_t - aoi)
        # Consider that the target is at aoi_comp_pos = (observed position + compensated displacement)
        # From that position (aoi_comp_pos) calculate the target time to critical region (t_t_cr_a)

        :param target_acc:
        :param target_vel:
        :param target_len:
        :param front:
        :param aoi:
        :return:
        """
        target_time = -1
        relative_pos = self.get_cr_relative_position(aoi, target_vel, target_acc, target_len, front)

        if relative_pos == CriticalRegion.Position.UNKNOWN or relative_pos == CriticalRegion.Position.AFTER_CR:
            return self.target_pos, target_time

        if relative_pos == CriticalRegion.Position.BEFORE_CR:
            # get target time to critical region
            # discriminant V*v - 4*0.5*a*(-distance)
            # we are before the CN!
            distance = math.fabs(self.cr.cn_orig_d - self.d_front)
            if self.at_max_speed():
                target_time = distance / target_vel
            else:            
                discriminant = target_vel ** 2 + 2 * target_acc * (-distance)
                target_time = (-target_vel + math.sqrt(discriminant)) / target_acc
            if target_time < 0:
                print("ERROR get_target_time_to_cr for target with id: {0}!!!".format(self.id))

        return self.target_pos, target_time
    


    def get_time_to_leave_cr(self, target_pos, target_vel, target_acc) -> float:

        target_time = self.NO_TIME_TO_CR

        if target_pos == CriticalRegion.Position.BEFORE_CR:
            distance = math.fabs(self.d_front - self.cr.cn_orig_d)
            if self.at_max_speed():
                target_time = distance / target_vel
            else:
                discriminant = target_vel ** 2 + 2 * target_acc * (-distance)
                target_time = (-target_vel + math.sqrt(discriminant)) / target_acc

        elif target_pos == CriticalRegion.Position.INSIDE_CR:
            distance = math.fabs(self.cr.cf_orig_d - self.d_rear)
            if self.at_max_speed():
                target_time = distance / target_vel
            else:
                discriminant = target_vel ** 2 + 2 * target_acc * (-distance)
                target_time = (-target_vel + math.sqrt(discriminant)) / target_acc

        return target_time
    
    def get_coords_of_projected_front(self):
        front_point = self.cr.cr_path.interpolate(self.d_front)
        return front_point.x, front_point.y




