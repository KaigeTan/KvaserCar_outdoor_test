std_msgs/Header header

# timestamps
int64 decision_time
int64 aoi

# ego state
float64 ego_tactical_speed
float64 ego_front_d
int32   ego_pos
int32   ego_pred_go_pos
float64 ego_d_to_cr
float64 ego_ttcr

# target state
float64 target_acc
float64 target_ttcr
float64 target_d_to_cr
int32   target_pos
float64 target_d_front

float64 kalman_acc_val