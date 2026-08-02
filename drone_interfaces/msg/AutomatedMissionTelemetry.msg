std_msgs/Header header

string state
string result

bool armed
bool offboard
bool fixed_wing
bool avoidance_active

float64 elapsed_time
float64 goal_distance
float64 east
float64 north
float64 altitude
float64 roll_rad
float64 pitch_rad
float64 path_length
float64 max_cross_track_error

float64 nearest_path_obstacle_distance
float64 nearest_lidar_distance
float64 minimum_lidar_distance
uint32 avoidance_activations

geometry_msgs/Vector3 attractive_force_enu
geometry_msgs/Vector3 repulsive_force_enu
geometry_msgs/Vector3 safe_command_enu
