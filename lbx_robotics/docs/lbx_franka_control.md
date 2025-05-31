# Franka Control Package (lbx_franka_control)

Documentation for the Franka robot control and teleoperation ROS2 package.

## Nodes

- `teleop_node`
  - **Subscribes**: `/vr_input`
  - **Publishes**: `/robot_command` or MoveIt goals
- `moveit_interface_node`
  - Manages MoveIt2 interaction.
- `system_health_node`
  - Monitors system health.

## Launch Files

- `franka_teleop.launch.py`
- `franka_moveit_bringup.launch.py`
