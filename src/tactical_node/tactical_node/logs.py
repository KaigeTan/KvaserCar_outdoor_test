import json
import os
import time
import datetime

import tactical_node.parameters as parameters



class ExpLog:
    def __init__(self, vehicle_log, termination_cause):
        self.vehicle_log = vehicle_log
        self.term_cause = termination_cause


    def write_to_file(self):
        now = datetime.now()
        ns = now.microsecond
        
        os.makedirs(parameters.EXP_LOG_PATH, exist_ok=True)
        # Create dated subdirectory name
        date_str = now.strftime('%d_%m_%y')
        dated_dir = os.path.join(parameters.EXP_LOG_PATH, date_str)

        # Create the dated subdirectory
        os.makedirs(dated_dir, exist_ok=True)

        file_name = "{0}.json".format(ns)
        file_name = os.path.join(os.path.expanduser(parameters.EXP_LOG_PATH), file_name)

        data = {"exp_name":ns,
                "term_cause":self.term_cause,
                "ego": self.vehicle_log}

        with open(file_name, 'w') as f:
            json.dump(data, f)