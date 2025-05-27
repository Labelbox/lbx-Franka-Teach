# Franka-Teach

A comprehensive teleoperation and data collection system for Franka robots, featuring VR control via Meta Quest, MCAP data recording, and simulation support.

## Features

- **VR Teleoperation**: Full 6DOF control using Meta Quest/Oculus controllers
- **MCAP Recording**: DROID-compatible data format for robot learning
- **Multi-Camera Support**: Intel RealSense and ZED camera integration
- **Simulation Mode**: Test without hardware using FR3 robot simulation
- **Real-time Control**: 15-30Hz control loop with safety features

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

# Terminal 2: Start camera server (optional)
python camera_server.py --config camera_config_example.json
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