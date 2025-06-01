# VR Controller State Message
# Contains current state of VR controller for system monitoring

std_msgs/Header header

# Controller identification
string controller_id  # "r" for right, "l" for left

# Button states
bool grip_pressed
bool joystick_pressed
bool a_button  # A on right, X on left
bool b_button  # B on right, Y on left

# Continuous values
float32 trigger_value  # 0.0 to 1.0

# Pose data (optional - for advanced monitoring)
geometry_msgs/Pose controller_pose 