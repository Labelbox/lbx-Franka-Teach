# LBX Franka Control

VR-based teleoperation control for Franka robots using MoveIt 2, preserving the exact DROID VRPolicy control pipeline.

## Overview

This package implements a complete VR teleoperation system for Franka robots, faithfully reproducing the control pipeline from `oculus_vr_server_moveit.py`. The system provides:

- **DROID-exact VR-to-robot transformations** - All coordinate transforms preserved exactly
- **High-performance asynchronous control** - 60Hz VR processing with 45Hz robot commands
- **MoveIt 2 integration** - IK solving, collision avoidance, trajectory planning
- **Intuitive calibration** - Forward direction and origin calibration via VR gestures
- **Data recording** - MCAP recording with optional camera integration
- **System orchestration** - Unified control of all components

## Architecture

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│  Oculus Quest   │────▶│  lbx_input_oculus│────▶│  System Manager │
│  (VR Controller)│ USB │  (VR Reader Node)│ ROS │  (Orchestrator) │
└─────────────────┘     └──────────────────┘     └────────┬────────┘
                                                           │
                        ┌──────────────────────────────────┼──────────────────────────────┐
                        │                                  │                              │
                        ▼                                  ▼                              ▼
              ┌─────────────────┐            ┌──────────────────────┐         ┌──────────────────┐
              │ Franka Controller│            │   Data Recorder      │         │ Camera Manager   │
              │ (MoveIt Control) │            │   (MCAP Writer)      │         │ (RealSense/ZED)  │
              └────────┬─────────┘            └──────────────────────┘         └──────────────────┘
                       │
                       ▼
              ┌─────────────────┐
              │  Franka Robot   │
              │  (via MoveIt)   │
              └─────────────────┘
```

## Key Components

### 1. System Manager (`system_manager.py`)

The main orchestrator that:

- Subscribes to VR input from `lbx_input_oculus`
- Manages robot control state machine
- Handles calibration sequences
- Coordinates data recording
- Publishes system status

### 2. Franka Controller (`franka_controller.py`)

Handles all robot control operations:

- MoveIt service communication (IK, FK, planning scene)
- Trajectory execution with velocity limiting
- Gripper control via Franka actions
- Pose smoothing and filtering
- Emergency stop functionality

### 3. Configuration (`franka_vr_control_config.yaml`)

All control parameters in one place:

- DROID VRPolicy parameters (gains, deltas, velocities)
- MoveIt settings (timeouts, tolerances)
- Coordinate transformations
- Smoothing and filtering parameters

## VR Control Pipeline

The exact pipeline from VR input to robot motion:

```
1. VR Data Capture (60Hz)
   └─> Raw poses from Oculus Reader

2. Coordinate Transform
   └─> Apply calibrated transformation: [X,Y,Z] → [-Y,X,Z]

3. Velocity Calculation
   └─> Position/rotation differences × gains (pos=5, rot=2)

4. Velocity Limiting
   └─> Clip to [-1, 1] normalized range

5. Position Delta Conversion
   └─> Scale by max_delta parameters (0.075m linear, 0.15rad angular)

6. Target Pose Calculation
   └─> Add deltas to current robot position/orientation

7. MoveIt IK Solver (45Hz)
   └─> Convert Cartesian target to joint positions via MoveIt service

8. Trajectory Execution
   └─> Send joint commands to robot controller
```

## Controls

### VR Button Mapping

| Button                   | Action                                   |
| ------------------------ | ---------------------------------------- |
| **Grip** (hold)          | Enable teleoperation                     |
| **Grip** (release)       | Pause teleoperation + recalibrate origin |
| **Trigger** (pull)       | Close gripper                            |
| **Trigger** (release)    | Open gripper                             |
| **A/X**                  | Toggle recording start/stop              |
| **B/Y**                  | Mark recording as successful             |
| **Joystick** (hold+move) | Calibrate forward direction              |

### Calibration

#### Forward Direction Calibration

1. Hold joystick button
2. Move controller in desired forward direction (>3mm)
3. Release joystick button
4. System aligns VR forward with robot +X axis

#### Origin Calibration

- Automatic on grip press/release
- Aligns current VR pose with current robot pose

## Usage

### Basic Launch

```bash
# Launch complete system with default settings
ros2 launch lbx_robotics system_bringup.launch.py

# With specific robot IP
ros2 launch lbx_robotics system_bringup.launch.py robot_ip:=192.168.1.100

# Enable camera recording
ros2 launch lbx_robotics system_bringup.launch.py enable_cameras:=true camera_config:=/path/to/cameras.yaml

# Use left controller
ros2 launch lbx_robotics system_bringup.launch.py use_left_controller:=true
```

### Performance Mode

```bash
# Enable performance mode (120Hz VR, higher gains)
ros2 launch lbx_robotics system_bringup.launch.py performance_mode:=true
```

### Monitoring

```bash
# Monitor system status
ros2 topic echo /system_status

# Monitor VR controller state
ros2 topic echo /vr_control_state

# View robot in RViz (auto-launched)
# RViz opens automatically showing robot state
```

## Services

The system provides ROS services for external control:

```bash
# Start recording
ros2 service call /start_recording std_srvs/srv/Empty

# Stop recording
ros2 service call /stop_recording std_srvs/srv/Empty

# Reset robot to home
ros2 service call /reset_robot std_srvs/srv/Empty

# Emergency stop
ros2 service call /emergency_stop std_srvs/srv/Empty
```

## Configuration

### Key Parameters in `franka_vr_control_config.yaml`:

```yaml
vr_control:
  # Control gains (DROID-exact)
  pos_action_gain: 5.0 # Position control responsiveness
  rot_action_gain: 2.0 # Rotation control responsiveness

  # Velocity-to-position conversion
  max_lin_delta: 0.075 # Max linear motion per timestep (m)
  max_rot_delta: 0.15 # Max angular motion per timestep (rad)

  # Smoothing (for stable motion)
  pose_smoothing_alpha: 0.35 # 0=max smoothing, 1=no smoothing
  adaptive_smoothing: true # Adjust based on motion speed

  # Command rate
  control_hz: 60 # VR processing rate
  min_command_interval: 0.022 # 45Hz robot commands
```

## Safety Features

1. **Workspace Bounds** - Robot motion limited to safe workspace
2. **Velocity Limiting** - Joint velocities capped at safe limits
3. **Collision Avoidance** - MoveIt collision checking enabled
4. **Emergency Stop** - Immediate halt via service or Ctrl+C
5. **Graceful Shutdown** - Proper cleanup on exit

## Troubleshooting

### Common Issues

1. **"IK service not available"**

   - Ensure MoveIt is running: `ros2 launch lbx_franka_moveit franka_moveit.launch.py`

2. **"VR controller not detected"**

   - Check USB connection or network IP
   - Verify Oculus Reader permissions: `adb devices`

3. **Robot not moving smoothly**

   - Adjust `pose_smoothing_alpha` (lower = smoother)
   - Check `min_command_interval` timing
   - Verify network latency to robot

4. **Calibration issues**
   - Ensure >3mm movement for forward calibration
   - Try different `coord_transform` values if needed

### Debug Mode

Run system manager alone for testing:

```bash
ros2 run lbx_franka_control system_manager --ros-args -p config_file:=/path/to/config.yaml
```

## Development

### Adding New Features

1. **New VR gestures**: Modify `_handle_calibration()` in `system_manager.py`
2. **Control parameters**: Add to `franka_vr_control_config.yaml`
3. **Robot capabilities**: Extend `FrankaController` class
4. **System states**: Update state machine in `SystemManager`

### Testing

```bash
# Run unit tests
colcon test --packages-select lbx_franka_control

# Integration test with simulation
ros2 launch lbx_robotics system_bringup.launch.py use_sim:=true
```

## Performance Optimization

For lowest latency teleoperation:

1. **Wired Ethernet** to robot (not WiFi)
2. **USB connection** for VR (not network)
3. **Performance mode** enabled
4. **Minimal background processes**
5. **Real-time kernel** (optional)

Expected latencies:

- VR input → System Manager: <20ms
- System Manager → MoveIt IK: <10ms
- MoveIt → Robot motion: <30ms
- Total: ~60ms end-to-end

## Credits

This implementation preserves the exact control pipeline from the original `oculus_vr_server_moveit.py`, ensuring compatibility with DROID-trained policies and maintaining the intuitive VR control feel.
