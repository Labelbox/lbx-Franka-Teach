# LBX Robotics Workspace

This document describes the overall `lbx_robotics` ROS2 workspace, how to build it, and how to launch its main functionalities.

## Workspace Structure

Refer to `directory_structure.md` for the detailed layout.

## Building the Workspace

1.  Navigate to the `lbx-Franka-Teach/lbx_robotics/` directory.
2.  Source your ROS2 Humble environment: `source /opt/ros/humble/setup.bash`
3.  Build the workspace: `colcon build --symlink-install`

## Launching

Source the workspace setup file: `source install/setup.bash`

### Core Teleoperation

`ros2 launch lbx_franka_control franka_teleop.launch.py`

### Individual Components

(Details to be added as packages are implemented)
