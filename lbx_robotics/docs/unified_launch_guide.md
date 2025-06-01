# Unified Launch Script Guide

The `unified_launch.sh` script combines all launch capabilities into a single, comprehensive launcher for the LBX Robotics system. It is located in the `lbx_robotics` directory.

## Key Features

### Process Management

- **Comprehensive Process Killing**: Automatically stops all existing ROS2 and robot processes before starting
- **Graceful Shutdown**: Clean shutdown with Ctrl+C
- **Emergency Stop**: Immediate process termination with `--emergency-stop`

### Build Management

- **Optional Build**: Build only when needed with `--build` flag
- **Clean Build**: Fresh build with `--clean-build` flag
- **No Automatic Build**: Unlike the old scripts, building is now explicit
- **Smart Clean**: Only cleans build files when `--clean-build` is specified

### Unified Arguments

All relevant arguments from different scripts are now in one place:

- Robot configuration (IP, fake hardware)
- VR control options (controller selection, network mode)
- Data and sensor options (cameras, recording)
- Visualization (RViz)
- Development features (hot reload, log levels)

## Quick Start

```bash
# Navigate to lbx_robotics directory
cd lbx_robotics

# Run with existing build
./unified_launch.sh

# Build and run
./unified_launch.sh --build

# Clean build for testing
./unified_launch.sh --clean-build --fake-hardware

# Run with cameras and no recording
./unified_launch.sh --cameras --no-recording

# Network VR mode
./unified_launch.sh --network-vr 192.168.1.50

# Custom robot IP with left controller
./unified_launch.sh --robot-ip 192.168.1.100 --left-controller
```

## Migration from Old Scripts

| Old Command                                 | New Command                           |
| ------------------------------------------- | ------------------------------------- |
| `bash run.sh`                               | `./unified_launch.sh --build`         |
| `bash run.sh --skip-build`                  | `./unified_launch.sh`                 |
| `bash run_teleoperation.sh --cameras`       | `./unified_launch.sh --cameras`       |
| `bash run_teleoperation.sh --network-vr IP` | `./unified_launch.sh --network-vr IP` |

## All Options

```bash
./unified_launch.sh --help
```

### Build Options

- `--build` - Perform colcon build
- `--clean-build` - Clean workspace then build (removes build/, install/, log/)

### Robot Options

- `--robot-ip <IP>` - Robot IP address (default: 192.168.1.59)
- `--fake-hardware` - Use fake hardware for testing
- `--no-fake-hardware` - Use real hardware (default)

### Visualization Options

- `--rviz` - Enable RViz (default)
- `--no-rviz` - Disable RViz

### VR Control Options

- `--left-controller` - Use left VR controller
- `--right-controller` - Use right VR controller (default)
- `--network-vr <IP>` - Use network VR mode with specified IP

### Data & Sensors Options

- `--cameras` - Enable camera system
- `--no-cameras` - Disable cameras (default)
- `--recording` - Enable data recording (default)
- `--no-recording` - Disable data recording

### Development Options

- `--hot-reload` - Enable hot reload for VR server
- `--log-level <LEVEL>` - Set log level (DEBUG, INFO, WARN, ERROR)

### System Control

- `--shutdown` - Gracefully shutdown running system
- `--emergency-stop` - Emergency stop all processes
- `--help` - Show help message

## Benefits

1. **Single Entry Point**: No need to remember multiple scripts or locations
2. **Explicit Build Control**: Build only when you need it, saving time during development
3. **Comprehensive Process Management**: Ensures clean system state before each run
4. **Unified Configuration**: All options in one place with clear organization
5. **Better Error Handling**: Inherited from the robust Franka scripts
6. **Smart Build Management**: Only cleans build artifacts when explicitly requested

## Notes

- The script automatically detects and uses the correct ROS2 distribution
- It creates necessary directories (like recordings) automatically
- Process killing is comprehensive but safe, with proper ordering
- The workspace path is automatically determined relative to the script location
- Build cleaning is now opt-in with `--clean-build` flag to preserve incremental builds
