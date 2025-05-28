# Franka-Teach

A comprehensive teleoperation and data collection system for Franka robots, featuring VR control via Meta Quest, MCAP data recording, and simulation support.

## Features

- **VR Teleoperation**: Full 6DOF control using Meta Quest/Oculus controllers
- **MCAP Recording**: DROID-compatible data format for robot learning
- **Multi-Camera Support**: Intel RealSense and ZED camera integration
- **Simulation Mode**: Test without hardware using FR3 robot simulation
- **Real-time Control**: 15-30Hz control loop with safety features
- **High-Performance Async Architecture**: 40Hz data recording with 6.6Hz robot control
- **Hot Reload**: Automatic server restart on code changes
- **Performance Mode**: Optimized for high-frequency control and recording
- **Data Verification**: Automatic validation of recorded trajectories

## Table of Contents

- [System Requirements](#system-requirements)
- [Installation](#installation)
  - [NUC Setup](#nuc-setup)
  - [Lambda Machine Setup](#lambda-machine-setup)
- [Quick Start](#quick-start)
- [VR Teleoperation](#vr-teleoperation)
- [Data Recording](#data-recording)
- [Camera Setup](#camera-setup)
- [Simulation Mode](#simulation-mode)
- [Troubleshooting](#troubleshooting)
- [Additional Documentation](#additional-documentation)

## System Requirements

### Hardware
- Franka Emika Robot (FR3 or Panda)
- NUC computer with real-time kernel
- Lambda workstation or similar
- Meta Quest 2/3 or Oculus headset
- Intel RealSense or ZED cameras (optional)

### Software
- Ubuntu 22.04 (NUC) / Ubuntu 20.04+ (Lambda)
- CUDA-capable GPU (for Lambda machine)
- Python 3.10+
- Real-time kernel (NUC only)

## Installation

### NUC Setup

1. **Install Ubuntu 22.04 with real-time kernel**
   ```bash
   # Follow instructions at:
   # https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel
   ```

2. **Configure network settings**
   - Set static IP for robot network interface
   - Ensure NUC can communicate with robot at 172.16.0.4 (right) and 172.16.1.4 (left)

### Lambda Machine Setup

1. **Clone and setup deoxys_control**
   ```bash
   git clone git@github.com:NYU-robot-learning/deoxys_control.git
   cd deoxys_control/deoxys
   
   # Create conda environment
   mamba create -n "franka_teach" python=3.10
   conda activate franka_teach
   
   # Install deoxys (select version 0.13.3 for libfranka when prompted)
   ./InstallPackage
   make -j build_deoxys=1
   pip install -U -r requirements.txt
   ```

2. **Clone and setup Franka-Teach**
   ```bash
   git clone <your-franka-teach-repo>
   cd Franka-Teach
   pip install -r requirements.txt
   ```

3. **Install ReSkin sensor library (optional)**
   ```bash
   git clone git@github.com:NYU-robot-learning/reskin_sensor.git
   cd reskin_sensor
   pip install -e .
   ```

4. **Setup Oculus Reader for VR control**
   ```bash
   cd oculus_reader_app
   # Follow instructions in oculus_reader_app/README.md
   ```

### Network Proxy Setup

1. **Install FoxyProxy extension** in Chrome/Firefox

2. **Configure SSH host** in `~/.ssh/config`:
   ```
   Host nuc
       HostName 10.19.248.70
       User robot-lab
       LogLevel ERROR
       DynamicForward 1337
   ```

3. **Configure FoxyProxy** to use SOCKS5 proxy on localhost:1337

## Quick Start

### 1. Connect to Robot

```bash
# SSH into NUC
ssh nuc

# Access Franka Desk interface via browser (with proxy enabled)
# Right robot: http://172.16.0.4/desk
# Left robot: http://172.16.1.4/desk
# Username: GRAIL
# Password: grail1234
```

### 2. Enable Robot

In Franka Desk interface:
1. Click "Unlock joints" (open brakes)
2. Enable FCI mode
3. If gripper issues: Settings → End Effector → Power cycle → Re-initialize

### 3. Start Deoxys Control (on NUC)

```bash
# Terminal 1: Start arm control
cd /home/robot-lab/work/deoxys_control/deoxys
./auto_scripts/auto_arm.sh config/franka_right.yml  # or franka_left.yml

# Terminal 2: Start gripper control (optional)
./auto_scripts/auto_gripper.sh config/franka_right.yml
```

### 4. Start Servers (on Lambda)

```bash
# Terminal 1: Start Franka server
cd /path/to/Franka-Teach
python franka_server.py
```

### 5. Start VR Teleoperation with Recording

```bash
# Terminal 3: Start Oculus VR server with MCAP recording
python oculus_vr_server.py

# Or with specific options:
python oculus_vr_server.py --left-controller  # Use left controller
python oculus_vr_server.py --simulation       # Test in simulation
python oculus_vr_server.py --debug           # Debug mode (no robot control)
python oculus_vr_server.py --no-recording    # Disable MCAP recording
```

## VR Teleoperation

### Controller Setup

1. **Connect Quest headset** via USB or WiFi
2. **Test connection**: `python test_oculus_reader.py`
3. **Calibrate forward direction**: Hold joystick + move controller forward

### Control Scheme

#### Right Controller (default)
- **Hold Grip**: Enable robot movement
- **Release Grip**: Pause movement (robot stays in place)
- **Hold Trigger**: Close gripper
- **Release Trigger**: Open gripper
- **Joystick Press**: Reset controller orientation

#### Recording Controls (when enabled)
- **A Button**: Start new recording / Reset current recording
- **B Button**: Mark recording as successful and save

See [oculus_control_readme.md](oculus_control_readme.md) for detailed VR control instructions.

## Data Recording

The system supports MCAP data recording in Labelbox Robotics format:
- Press **A button** to start/reset recording
- Press **B button** to mark recording as successful and save
- Recordings are saved to `~/recordings/success/` or `~/recordings/failure/`
- See [MCAP_RECORDING_README.md](MCAP_RECORDING_README.md) for details

### Visualizing Robot in Foxglove Studio

The MCAP recordings include the robot URDF model and joint states for visualization:
- See [FOXGLOVE_ROBOT_VISUALIZATION.md](FOXGLOVE_ROBOT_VISUALIZATION.md) for setup instructions
- Test with `python3 test_foxglove_robot.py` to create a sample file

## Camera Setup

### Configure Cameras

Edit `

## Simulation Mode

### Simulation Setup

1. **Test in simulation**: `python oculus_vr_server.py --simulation`
2. **Test without hardware**: Use FR3 robot simulation

## Troubleshooting

### Low Recording Frequency
- Ensure performance mode is enabled (default with `run_server.sh`)
- Check CPU usage and reduce other processes
- Verify in console output: should show ~39-40Hz

### Robot Communication Slow
- The ~149ms latency is hardware limited
- System automatically uses predictive control
- Check network/USB connection quality

### VR Controller Not Detected
- Ensure Oculus is connected via USB or network
- Check with `adb devices` for USB connection
- For network: use `--ip <quest-ip-address>`

## Additional Documentation

### Architecture Overview

The system uses a sophisticated asynchronous architecture to achieve high-frequency data recording (40Hz) while managing slower robot communication (6.6Hz):

```
VR Thread (50Hz) → Control Thread (40Hz) → Recording Thread (40Hz)
                          ↓
                   Robot Comm Thread → Robot Hardware (6.6Hz)
```

**Key Benefits:**
- 6x higher recording rate than traditional synchronous approaches
- Non-blocking operation ensures smooth teleoperation
- Predictive control maintains responsiveness
- Thread-safe data handling with minimal lock contention

For detailed architecture documentation, see [ASYNC_ARCHITECTURE_README.md](ASYNC_ARCHITECTURE_README.md).

### Performance Optimization

The system includes several performance optimizations:

#### Performance Mode (Default)
- Control frequency: 40Hz (2x base rate)
- Position gain: 10.0 (100% higher)
- Rotation gain: 3.0 (50% higher)
- Optimized for tight tracking

#### Async Features
- **Decoupled threads** for VR, control, recording, and robot I/O
- **Predictive control** when robot feedback is delayed
- **Non-blocking queues** for data flow
- **Lock-free design** where possible

See [ASYNC_ARCHITECTURE_README.md](ASYNC_ARCHITECTURE_README.md) for detailed performance documentation.

### Hot Reload

Enable automatic server restart on code changes:

```bash
./run_server.sh --hot-reload
```

This monitors Python files and configs, restarting the server when changes are detected. Perfect for rapid development and testing.

See [HOT_RELOAD_README.md](HOT_RELOAD_README.md) for details.

### Data Format

Recordings are saved in MCAP format with the following structure:

- `/robot_state`: Joint positions, cartesian pose, gripper state (40Hz)
- `/action`: Velocity commands sent to robot (40Hz)
- `/vr_controller`: Raw VR controller data (40Hz)
- `/tf`: Transform tree for visualization
- `/joint_states`: ROS-compatible joint states

### Documentation

- [Async Architecture & Performance](ASYNC_ARCHITECTURE_README.md)
- [Hot Reload Feature](HOT_RELOAD_README.md)
- [MCAP Recording Format](MCAP_RECORDING_README.md)
- [Foxglove Visualization](FOXGLOVE_ROBOT_VISUALIZATION.md)
- [VR Robot Mapping](VR_ROBOT_MAPPING.md)

### Contributing

1. Fork the repository
2. Create a feature branch
3. Make changes and test with hot reload
4. Submit a pull request

### License

This project is licensed under the MIT License - see the LICENSE file for details.

### Acknowledgments

- Based on the DROID VRPolicy implementation
- Uses Deoxys control framework for Franka robots
- MCAP format for efficient data storage

## Camera Integration

The system supports multiple camera types with automatic discovery and configuration:

### Supported Cameras
- Intel RealSense (D405, D415, D435, etc.)
- ZED cameras (with SDK installed)

### Quick Start
```bash
# Auto-discover and use all connected cameras
./run_server.sh --auto-discover-cameras

# Or use specific camera configuration
./run_server.sh --camera-config configs/cameras_intel.yaml
```

### Camera Discovery
```bash
# Discover all connected cameras
python discover_cameras.py

# List cameras with simple output
python list_cameras.py
```

### Camera Testing
```bash
# Test camera functionality
python test_cameras.py
```

See [CAMERA_INTEGRATION_README.md](CAMERA_INTEGRATION_README.md) for detailed camera setup instructions.