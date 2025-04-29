# FILE LOGGING
# TODO FILL WITH AN ABSOLUTE PATH
EXP_LOG_PATH = "~/KvaserCar_outdoor_test/data"


# CRITICAL REGION POINTS
#  critical region
#    (P4)-----(P1)
#     |         |
#     |         |
#    (P3)-----(P2)
#
CR_POINT_1 = (1, 1)
CR_POINT_2 = (1, -1)
CR_POINT_3 = (-1, -1)
CR_POINT_4 = (-1, 1)
#CR_POINT_1 = (4, 0.5)
#CR_POINT_2 = (4, -0.5)
#CR_POINT_3 = (3, -0.5)
#CR_POINT_4 = (3, 0.5)

# ADVERSARY VEHICLE PATH POINTS
ADV_PATH_START = (0, -10)
ADV_PATH_END = (0, 10)

# EGO VEHICLE PATH POINTS
EGO_PATH_START = (9.007, 0)
EGO_PATH_END = (-5, 0)#(8, 0)

# ADVERSARY PARAMETERS
ADV_REFERENCE_SPEED = 3.0 #[m/s]
ADV_MAX_ACC = 2.5 #[m/s]
ADV_LENGTH = 0.720 #[m]
ADV_WIDTH = 0.515 #[m]

# EGO PARAMETERS
EGO_REFERENCE_SPEED = 1.0 #[m/s]
EGO_MAX_ACC = 2.5 # [m/s]
EGO_MAX_DEC = 2.5 # [m/s]
EGO_LENGTH = 0.720 #[m]
EGO_WIDTH = 0.515 #[m]
