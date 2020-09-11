# Message describes joint parameterss

# Joint channel ID
int32 channel
# Regulator parameters
float32 P
float32 I
float32 D
float32 rate

# Joint limits
float32 max_velocity
float32 lower_bound
float32 upper_bound
bool limit_position
