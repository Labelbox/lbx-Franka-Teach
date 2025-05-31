# LBX Robotics Architecture

This document outlines the high-level architecture of the `lbx_robotics` ROS2 workspace and how the different packages and nodes interact.

## Core Data Flow (Teleoperation Example)

1.  **`lbx_input_oculus` (`oculus_node`)**:

    - Reads raw data from the Oculus Quest controller.
    - Processes and publishes standardized VR input data (poses, button states) to the `/vr_input` topic (`lbx_interfaces/msg/VRInput`).

2.  **`lbx_vision_realsense` (`realsense_node`)** (and other future vision packages):

    - Manages RealSense camera(s).
    - Publishes raw color/depth images and camera info to topics like `/camera/color/image_raw`, `/camera/depth/image_raw`, etc.

3.  **`lbx_franka_control` (`teleop_node`)**:

    - Subscribes to `/vr_input`.
    - Subscribes to `/joint_states` (or relevant robot state topic from MoveIt).
    - Interprets VR commands and current robot state to determine desired robot actions.
    - This node contains the core teleoperation logic, coordinate transformations, and safety limits.
    - Publishes robot commands, potentially as `lbx_interfaces/msg/RobotCommand` to `/robot_command` or directly sends goals to the MoveIt2 interface.

4.  **`lbx_franka_control` (`moveit_interface_node`)**:

    - If `/robot_command` is used, this node subscribes to it.
    - Interfaces with the MoveIt2 framework (`move_group` node, planning services, action servers).
    - Handles motion planning, execution, and collision checking for the Franka FR3 arm.
    - Publishes robot state information (e.g., joint states if not directly from hardware drivers through `ros2_control`).

5.  **`lbx_franka_description` / `franka_fr3_moveit_config`**:

    - Provides the robot URDF (`robot_description` topic) and MoveIt configuration, which are used by `robot_state_publisher` and MoveIt nodes.

6.  **`ros2_control` / Franka Hardware Interface** (typically part of `franka_ros2` or `franka_hardware`):

    - Low-level hardware drivers that communicate directly with the Franka robot.
    - Provide joint states and execute trajectories.

7.  **`lbx_data_recorder` (`mcap_recorder_node`)**:
    - Subscribes to a configurable list of topics (e.g., `/vr_input`, `/robot_state`, image topics, `/robot_command`, `/tf`, etc.).
    - Records the selected data into MCAP files for offline analysis and training.

## Topic and Service Overview (Examples)

- `/vr_input` (`lbx_interfaces/msg/VRInput`): Published by `oculus_node`.
- `/camera/color/image_raw` (`sensor_msgs/msg/Image`): Published by `realsense_node`.
- `/camera/depth/image_raw` (`sensor_msgs/msg/Image`): Published by `realsense_node`.
- `/robot_command` (`lbx_interfaces/msg/RobotCommand`): Published by `teleop_node`, subscribed by `moveit_interface_node` (or `teleop_node` directly sends MoveIt goals).
- `/joint_states` (`sensor_msgs/msg/JointState`): Published by `robot_state_publisher` or hardware interface.
- `/tf`, `/tf_static` (`tf2_msgs/msg/TFMessage`): For coordinate transforms.
- MoveIt Action Servers (e.g., `/move_action`, `/execute_trajectory`): Used by `moveit_interface_node`.

(More details to be added as packages are fleshed out)
