import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/labelbox/projects/moveit/lbx-Franka-Teach/ros2_moveit_franka/install/ros2_moveit_franka'
