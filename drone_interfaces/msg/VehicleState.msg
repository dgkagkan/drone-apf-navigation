uint8 MODE_MULTICOPTER=0
uint8 MODE_FIXED_WING=1
uint8 MODE_TRANSITION=2
uint8 MODE_UNKNOWN=255

std_msgs/Header header
bool position_valid
bool attitude_valid
bool armed
bool offboard
uint8 vehicle_mode
geometry_msgs/Point position_enu
geometry_msgs/Vector3 velocity_enu
geometry_msgs/Quaternion attitude_body_to_ned
float64 heading_ned_rad
