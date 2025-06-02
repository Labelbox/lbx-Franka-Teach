# Velocity command for robot control
# Contains normalized velocities for robot motion

# Header for timestamp
std_msgs/Header header

# Velocity values: [linear_x, linear_y, linear_z, angular_x, angular_y, angular_z, gripper]
# Linear velocities are normalized [-1, 1] representing direction
# Angular velocities are normalized [-1, 1] representing rotation direction
# Gripper velocity: positive closes, negative opens
float64[] velocities

# Optional: raw trigger value from VR controller [0, 1]
# Used by FrankaController for direct gripper control
float64 trigger_value 