# System Status Message
# Overall system state and status information

std_msgs/Header header

# System state
string system_state  # idle, calibrating, teleop, recording, emergency_stop

# Status flags
bool recording_active
bool teleoperation_enabled
bool controller_connected

# Calibration info
string calibration_mode  # "", "forward", "origin"

# VR calibration data (4x4 transformation matrix as flat array)
float64[16] vr_calibration_matrix
bool vr_calibration_valid

# Performance metrics (optional)
float32 control_frequency  # Actual Hz
float32 ik_success_rate    # 0-100%
int32 commands_sent        # Total commands sent 