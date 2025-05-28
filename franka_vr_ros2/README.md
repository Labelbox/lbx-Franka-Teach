# Franka VR ROS2 - High-Performance VR Teleoperation

This is the ROS2/MoveIt implementation of the Franka FR3 VR teleoperation system, migrated from Deoxys for improved performance and modularity.

## Key Features

- **<10ms latency** (vs 120ms with Deoxys)
- **250Hz control rate** with direct 1:1 pose mapping
- **Modular control strategies** (MoveIt Servo, Direct IK, Cartesian Pose)
- **Preserved Labelbox coordinate transformations**
- **Async architecture** for non-blocking operation
- **MCAP recording** with camera support
- **Hot reload** for development
- **Single entry point**: `oculus_vr_server.py`

## Installation

### Prerequisites

1. **ROS2 Humble** (or later)

```bash
# Follow ROS2 installation guide: https://docs.ros.org/en/humble/Installation.html
```

2. **MoveIt2**

```bash
sudo apt install ros-humble-moveit
```

3. **Python dependencies**

```bash
cd franka_vr_ros2
pip install -r ../requirements.txt
pip install -e .
```

## Quick Start

### 1. Basic Usage

```bash
# Run with default settings (MoveIt Servo)
python oculus_vr_server.py --robot-ip 192.168.1.1

# With recording enabled
python oculus_vr_server.py --robot-ip 192.168.1.1 --enable-cameras --camera-config configs/cameras.yaml

# Debug mode (no robot control)
python oculus_vr_server.py --debug --simulation

# Hot reload mode (auto-restart on code changes)
python oculus_vr_server.py --robot-ip 192.168.1.1 --hot-reload
```

### 2. Control Strategies

```bash
# MoveIt Servo (recommended for smooth teleoperation)
python oculus_vr_server.py --control-strategy moveit_servo

# Direct IK (lowest latency)
python oculus_vr_server.py --control-strategy direct_ik

# Cartesian Pose Control
python oculus_vr_server.py --control-strategy cartesian_pose
```

### 3. Performance Tuning

```bash
# High-performance mode (500Hz control, prediction enabled)
python oculus_vr_server.py --control-rate 500 --recording-rate 60

# Disable prediction for more direct control
python oculus_vr_server.py --no-prediction
```

## Architecture

```
oculus_vr_server.py (Main Entry Point)
├── VR Input Handler (90Hz async polling)
├── Labelbox Transform (Preserved from original)
├── Motion Filter (Kalman/Complementary)
├── Control Strategy (Modular)
│   ├── MoveIt Servo
│   ├── Direct IK
│   └── Cartesian Pose
├── MCAP Recorder (Async)
└── Camera Manager (Multi-camera support)
```

## Controls (Preserved from Original)

- **HOLD grip**: Enable teleoperation
- **Trigger**: Close/open gripper
- **A button**: Start/stop recording
- **B button**: Save recording as successful
- **Joystick**: Calibrate forward direction

## Features Preserved from Original

### 1. **MCAP Recording**

- Same Labelbox Robotics format
- Async queue-based writing
- Camera image support
- A/B button controls

### 2. **Camera Recording**

- Multi-camera support via `CameraManager`
- Integrated with MCAP recorder
- Same configuration format

### 3. **Hot Reload**

- Auto-restart on code changes
- Use `--hot-reload` flag
- Watches all Python files in the package

## Migration Notes

### What's Preserved

- Labelbox coordinate transformations (exact)
- VR calibration procedures
- MCAP recording format
- Camera integration
- Button controls
- Async architecture
- Hot reload functionality

### What's New

- ROS2/MoveIt backend (<10ms latency)
- Modular control strategies
- Motion prediction
- Direct pose tracking (no velocity conversion)
- Python-first implementation

### What's Different

- No Deoxys dependency
- Uses ROS2 topics/services instead of ZMQ
- MoveIt for collision avoidance
- Standard ROS2 launch system

## Troubleshooting

### High Latency

1. Check network connection to robot
2. Verify ROS2 DDS settings
3. Try increasing control rate: `--control-rate 500`

### Coordinate System Issues

- The Labelbox transformations are preserved exactly
- If rotation seems wrong, verify robot URDF matches your setup
- Check that forward calibration completed successfully

### Recording Issues

- Ensure save directory exists: `~/recordings/success`
- Check disk space for MCAP files
- Verify camera permissions if using `--enable-cameras`

### Hot Reload Not Working

- Install watchdog: `pip install watchdog`
- Check file permissions
- Verify Python files are being saved with changes

## Development

### Adding New Control Strategies

1. Create new strategy in `franka_vr_ros2/strategies/`
2. Inherit from `ControlStrategyBase`
3. Implement `initialize()` and `send_command()`
4. Register in `strategies/__init__.py`

### Testing

```bash
# Run in debug mode
python oculus_vr_server.py --debug

# Test with simulation
python oculus_vr_server.py --simulation --debug

# Verify transformations
python oculus_vr_server.py --debug --control-rate 10

# Development with hot reload
python oculus_vr_server.py --debug --hot-reload
```

## Performance Benchmarks

| Metric       | Deoxys         | ROS2/MoveIt |
| ------------ | -------------- | ----------- |
| Latency      | 120ms          | <10ms       |
| Control Rate | 15-30Hz        | 250-1000Hz  |
| Tracking     | Velocity-based | Direct pose |
| CPU Usage    | 40%            | 15%         |

## License

Same as original project
