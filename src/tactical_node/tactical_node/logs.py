import json
import os
from datetime import datetime

import tactical_node.parameters as parameters



class ExpLog:
    def __init__(self, path_ros_bag):
        self.log_dir = self.create_log_dir()
        self.log_file_name = self.create_log_file_name()
        self.register_file = os.path.join(self.log_dir, "blue_car_records")
        self.path_ros_bag = path_ros_bag
        self.done = False


    def create_log_dir(self):
        # Base folder to store rosbag files
        bag_base_dir = os.path.expanduser(parameters.EXP_LOG_PATH)
        # Generate date-based subfolder (e.g., 0513)
        date_str = datetime.now().strftime('%m%d')
        log_subdir = os.path.join(bag_base_dir, date_str)
        # Ensure directory exists
        os.makedirs(log_subdir, exist_ok=True)

        return log_subdir
    
    def create_log_file_name(self):
        now = datetime.now()
        name = (
            f"{now.day:02d}"
            f"{now.hour:02d}"
            f"{now.minute:02d}"
            f"{now.second:02d}"
            f"{now.microsecond:06d}"
        )
        file_name = os.path.join(self.log_dir, name + ".json")

        return file_name


    def write_to_file(self, start_id, start_time, ini_params, vehicle_log, termination_cause):
        if not self.done:
            data = {"start_id":start_id,
                    "start_time":start_time,
                    "term_cause":termination_cause,
                    "ini_params": ini_params,
                    "ego": vehicle_log}

            with open(self.log_file_name, 'w') as f:
                json.dump(data, f)

            with open(self.register_file, 'a') as f:
                line = f"[start_id:{start_id}, ros_bag:{self.path_ros_bag}, tactical_log:{self.log_file_name}]\n"
                f.write(line)
            
            print(f"Saved, {start_id} {self.path_ros_bag} {self.log_file_name}")

        self.done = True
