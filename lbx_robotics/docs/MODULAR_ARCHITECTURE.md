# Modular Architecture for Franka VR Control System

## Overview

The system has been refactored into a modular architecture following ROS2 best practices. Each node has a single responsibility and communicates with others through well-defined interfaces. The sophisticated control logic from `franka_controller.py` is preserved within the `robot_control_node`.

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        System Architecture                               │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐ │
│  │ system_         │     │ robot_control_   │     │ vr_teleop_node  │ │
│  │ orchestrator    │────▶│ node             │◀────│                 │ │
│  │                 │     │                  │     │                 │ │
│  │ - State Machine │     │ - FrankaController│     │ - VR Processing │ │
│  │ - Coordination  │     │ - MoveIt IK/FK   │     │ - Calibration   │ │
│  │ - Health Monitor│     │ - Trajectory Exec│     │ - Velocity Calc │ │
│  └────────┬────────┘     └──────────────────┘     └─────────────────┘ │
│           │                                                              │
│           │ Services                                                     │
│           │                                                              │
│  ┌────────▼────────┐     ┌──────────────────┐     ┌─────────────────┐ │
│  │ ui_node         │     │ oculus_node      │     │ data_recorder   │ │
│  │                 │     │                  │     │                 │ │
│  │ - User Commands │     │ - VR Input       │     │ - MCAP Recording│ │
│  │ - Status Display│     │ - Joy Messages   │     │ - Data Logging  │ │
│  └─────────────────┘     └──────────────────┘     └─────────────────┘ │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

## Node Descriptions

### 1. **system_orchestrator**
- **Purpose**: Lightweight coordinator that manages system state
- **Responsibilities**:
  - System state machine (initializing, idle, calibrating, teleoperation)
  - Service server for high-level commands
  - Health monitoring and diagnostics
  - Startup sequencing
- **Key Services**:
  - `/system/initialize` - Initialize system and reset robot to home
  - `/system/start_calibration` - Start VR calibration
  - `/system/start_teleoperation` - Enable teleoperation
  - `/system/stop` - Stop all operations
  - `/system/reset` - Reset system

### 2. **robot_control_node**
- **Purpose**: Sophisticated robot control using FrankaController
- **Responsibilities**:
  - Uses FrankaController internally for all control logic
  - MoveIt IK/FK computation with collision checking
  - Trajectory execution with velocity profiles
  - Pose smoothing with adaptive algorithms
  - Gripper control via Franka actions
  - Emergency stop capability
- **Key Services**:
  - `/robot/reset_to_home` - Move robot to home position
  - `/robot/enable_control` - Enable/disable velocity control
  - `/robot/emergency_stop` - Emergency halt
- **Key Topics**:
  - Subscribes: `/robot/velocity_command`, `/joint_states`
  - Publishes: `/robot/ready`, `/robot/current_pose`

### 3. **vr_teleop_node**
- **Purpose**: Process VR input and generate robot commands
- **Responsibilities**:
  - VR pose to robot frame transformation
  - Calibration management (translation, rotation, scale)
  - Velocity command generation (normalized [-1, 1])
  - Trigger to gripper mapping
- **Key Services**:
  - `/vr/calibrate` - Perform VR calibration
  - `/vr/enable_teleoperation` - Enable/disable teleoperation
  - `/vr/save_calibration` - Save calibration to file
- **Key Topics**:
  - Subscribes: `/vr/controller_pose`, `/vr/controller_joy`, `/robot/current_pose`
  - Publishes: `/robot/velocity_command`, `/vr/target_pose`

### 4. **ui_node** (Optional)
- **Purpose**: Simple terminal UI for user interaction
- **Responsibilities**:
  - Display system status
  - Accept user commands
  - Call services on other nodes
- **Features**:
  - Real-time status display
  - Single-key commands
  - Service call feedback

## Key Implementation Details

### FrankaController Integration
The `robot_control_node` uses the battle-tested `FrankaController` class internally, preserving:
- Sophisticated velocity-to-position conversion
- Adaptive pose smoothing with SLERP
- IK computation with collision checking
- Optimized trajectory execution
- Gripper control with proper Franka actions
- Diagnostic monitoring

### Velocity Command Flow
1. VR pose changes detected by `vr_teleop_node`
2. Normalized velocity commands computed ([-1, 1] range)
3. `VelocityCommand` message sent with trigger value
4. `robot_control_node` receives command
5. `FrankaController.execute_command()` processes:
   - Velocity to position target conversion
   - Workspace bounds checking
   - Pose smoothing
   - IK computation
   - Trajectory execution
   - Gripper control based on trigger

### Calibration System
- Translation offset between VR and robot frames
- Rotation offset as quaternion
- Scale factor for position mapping
- Persistent storage in JSON file

## Message Definitions

### VelocityCommand.msg
```
std_msgs/Header header
float64[] velocities  # [lin_x, lin_y, lin_z, ang_x, ang_y, ang_z, gripper]
float64 trigger_value # Raw trigger value [0, 1] for gripper control
```

### VelocityControl.action
```
# Goal
---
# Result
bool success
string message
---
# Feedback
float64[] current_velocities
```

## Launch Configuration

```bash
# Launch with new modular architecture
ros2 launch lbx_launch system_bringup.launch.py

# Launch with UI (default)
ros2 launch lbx_launch system_bringup.launch.py enable_ui:=true

# Launch without UI (headless)
ros2 launch lbx_launch system_bringup.launch.py enable_ui:=false

# Launch with cameras
ros2 launch lbx_launch system_bringup.launch.py enable_cameras:=true
```

## Benefits Over Previous Architecture

1. **Preserved Sophistication**: All complex control logic from `franka_controller.py` retained
2. **Clear Separation**: Each node has single, well-defined purpose
3. **No Duplication**: FrankaController used as internal component, not duplicated
4. **Better Testing**: Each node can be tested independently
5. **Improved Reliability**: Node failures don't crash entire system
6. **Easier Maintenance**: Clear interfaces between components

## Migration Notes

- Legacy `system_manager` and `main_system` nodes removed
- All robot control logic consolidated in `robot_control_node` using `FrankaController`
- VR processing separated into dedicated `vr_teleop_node`
- System orchestration simplified into lightweight `system_orchestrator`
- UI functionality moved to optional `ui_node` 