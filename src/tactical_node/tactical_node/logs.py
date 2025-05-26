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
        
        os.makedirs(parameters.EXP_LOG_PATH, exist_ok=True)
        # Create dated subdirectory name
        date_str = now.strftime('%d_%m_%y')
        dated_dir = os.path.join(parameters.EXP_LOG_PATH, date_str)

        # Create the dated subdirectory
        os.makedirs(dated_dir, exist_ok=True)

        name = (
            f"{now.day:02d}"
            f"{now.hour:02d}"
            f"{now.minute:02d}"
            f"{now.second:02d}"
            f"{now.microsecond:06d}"
        )

        file_name = os.path.join(dated_dir, name + ".json")

        data = {"exp_name":name,
                "term_cause":self.term_cause,
                "ego": self.vehicle_log}

        with open(file_name, 'w') as f:
            json.dump(data, f)