uint8 EXECUTE=0
uint8 CANCEL=1

std_msgs/Header header
uint64 command_id
uint64 mission_id
uint32 revision
uint8 command
SwarmRoute[] routes
string[] target_drone_ids
