uint8 MODE_MULTICOPTER=0
uint8 MODE_FIXED_WING=1

uint8 SOURCE_NONE=0
uint8 SOURCE_MANUAL=1
uint8 SOURCE_AUTONOMOUS=2
uint8 SOURCE_SUPERVISOR=3

std_msgs/Header header
uint8 vehicle_mode
uint8 source
bool active
bool manual_override
geometry_msgs/Vector3 velocity_enu
float64 yaw_rate_ned
bool hold_altitude
float64 target_altitude_m
