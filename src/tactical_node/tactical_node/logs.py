import json
import os
from datetime import datetime

import tactical_node.parameters as parameters



class ExpLog:
    def __init__(self, vehicle_log, termination_cause):
        self.vehicle_log = vehicle_log
        self.term_cause = termination_cause


    def write_to_file(self):
        now = datetime.now()

        # Base folder to store rosbag files
        bag_base_dir = os.path.expanduser('~/KvaserCar_outdoor_test/tactical_data')

        # Generate date-based subfolder (e.g., 0513)
        date_str = datetime.now().strftime('%m%d')
        bag_subdir = os.path.join(bag_base_dir, date_str)

        # Ensure directory exists
        os.makedirs(bag_subdir, exist_ok=True)

        name = (
            f"{now.day:02d}"
            f"{now.hour:02d}"
            f"{now.minute:02d}"
            f"{now.second:02d}"
            f"{now.microsecond:06d}"
        )

        file_name = os.path.join(bag_subdir, name + ".json")

        data = {"exp_name":name,
                "term_cause":self.term_cause,
                "ego": self.vehicle_log}

        with open(file_name, 'w') as f:
            json.dump(data, f)