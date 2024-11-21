import numpy as np

# Network constants
HOST = "localhost"
CAM_PORT = 10005
FISHEYE_CAM_PORT = 10010
VR_CONTROLLER_STATE_PORT = 8889
STATE_PORT = 8900
CONTROL_PORT = 8901
RESKIN_STREAM_PORT = 12005


STATE_TOPIC = "state"
CONTROL_TOPIC = "control"

# VR constants
VR_TCP_HOST = "10.19.128.106"
VR_TCP_PORT = 5555
VR_CONTROLLER_TOPIC = b"oculus_controller"

# Robot constants
GRIPPER_OPEN = -1
GRIPPER_CLOSE = 1
H_R_V = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
H_R_V_star = np.array([[-1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
x_min, x_max = 0.2, 0.75
y_min, y_max = -0.4, 0.4
z_min, z_max = 0.05, 0.7  # 232, 550
ROBOT_WORKSPACE_MIN = np.array([x_min, y_min, z_min])
ROBOT_WORKSPACE_MAX = np.array([x_max, y_max, z_max])

TRANSLATIONAL_POSE_VELOCITY_SCALE = 5
ROTATIONAL_POSE_VELOCITY_SCALE = 0.75
ROTATION_VELOCITY_LIMIT = 0.5
TRANSLATION_VELOCITY_LIMIT = 1

# Frequencies
VR_FREQ = 90
STATE_FREQ = 100
CAM_FPS = 30
DEPTH_PORT_OFFSET = 1000
