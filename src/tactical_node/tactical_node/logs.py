import json
import os
import time

import tactical_node.parameters as parameters



class ExpLog:
    def __init__(self, vehicle_log, termination_cause):
        self.vehicle_log = vehicle_log
        self.term_cause = termination_cause


    def write_to_file(self):
        timestamp = time.time()
        file_name = "{0}.json".format(timestamp)
        os.path.join(parameters.EXP_LOG_PATH, file_name)
        data = {"exp_name":timestamp,
                "term_cause":self.term_cause,
                "ego": self.vehicle_log}

        with open(file_name, 'w') as f:
            json.dump(data, f)