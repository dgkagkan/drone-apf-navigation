uint8 WAITING=0
uint8 ACCEPTED=1
uint8 EXECUTING=2
uint8 SUCCEEDED=3
uint8 FAILED=4
uint8 CANCELED=5
uint8 REJECTED=6

uint64 target_id
string drone_id
geometry_msgs/PoseStamped target
geometry_msgs/Point current_position
float64 cost
float64 distance_remaining_m
uint8 state
string message
