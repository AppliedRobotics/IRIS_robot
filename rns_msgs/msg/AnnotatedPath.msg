std_msgs/Header header

# Path request id. 
int32 id
# Actual path
rns_msgs/Waypoint[] waypoints
# Should mover take care of waypoint directions
bool useDirections
# If path is reversed
bool reversed