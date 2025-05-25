# Oculus VR Control System

Control the Franka robot arm using Meta Quest (Oculus Quest 2) VR controllers with ergonomic relative motion control.

## Prerequisites

1. **Meta Quest 2 or Quest Pro** with developer mode enabled
2. **USB-C cable** connected between Quest and computer
3. **Robot** connected and powered on
4. **Network** configured (PC: 192.168.1.122, WiFi: 192.168.1.54)

## Quick Start

### 1. Test Oculus Connection (Debug Mode)

First, verify the Oculus Reader is working without controlling the robot:

```bash
# Test with USB connection (recommended)
python3 oculus_vr_server.py --debug

# Test with WiFi connection
python3 oculus_vr_server.py --debug --ip 192.168.1.XXX
```

In debug mode, you should see:
- Controller positions updating as you move them
- Button states changing when pressed
- Trigger values (0.0 to 1.0) when squeezed
- Grip state showing "Robot Active" or "Robot Idle"

### 2. Start Robot Control System

Once debug mode works, start the full system:

#### Terminal 1: Start Deoxys (Robot Interface)
```bash
python3 run_deoxys_correct.py
```

#### Terminal 2: Start Franka Server (Robot Control)
```bash
python3 franka_server.py
```

#### Terminal 3: Start Oculus VR Server
```bash
# For USB connection (recommended)
python3 oculus_vr_server.py

# For WiFi connection
python3 oculus_vr_server.py --ip 192.168.1.XXX

# To use left controller instead of right
python3 oculus_vr_server.py --left-controller
```

#### Terminal 4: Start Teleop (Main Control)
```bash
python3 teleop.py teleop_mode=robot
```

## VR Controller Usage (Ergonomic Mode)

### Control Scheme
The system uses an ergonomic relative motion control scheme:

1. **Grip to Enable**: Robot only moves when grip button is held
2. **Relative Motion**: When grip is pressed, current controller position becomes "home"
3. **Natural Movement**: Move controller relative to grip point to control robot

### Right Controller (Default)
- **Press and Hold Right Grip**: Enable robot control (sets home position)
- **Move Controller While Gripping**: Control robot position (relative to grip point)
- **Hold Right Trigger**: Close gripper (release to open - default is open)
- **A Button**: Mark trajectory as success and stop recording
- **B Button**: Mark trajectory as failure and stop recording
- **Right Joystick Click**: Reset controller orientation

### Left Controller (with --left-controller flag)
- **Press and Hold Left Grip**: Enable robot control (sets home position)
- **Move Controller While Gripping**: Control robot position (relative to grip point)
- **Hold Left Trigger**: Close gripper (release to open - default is open)
- **X Button**: Mark trajectory as success and stop recording
- **Y Button**: Mark trajectory as failure and stop recording
- **Left Joystick Click**: Reset controller orientation

## Ergonomic Workflow

1. **Put on VR headset** and pick up the controller
2. **Start all services** in the order shown above
3. **Position controller** comfortably where you want to start
4. **Press and hold grip button** - this sets the home position
5. **Move controller** relative to grip point to control robot
6. **Release grip** to stop robot movement (robot stays in place)
7. **Reposition controller** comfortably and grip again to continue from new home
8. **Hold trigger** to close gripper, release to open (gripper is open by default)

This relative motion approach prevents arm fatigue and allows comfortable operation from any position.

## Troubleshooting

### Oculus Not Detected

1. Check USB connection:
```bash
adb devices
```

2. If no devices shown:
   - Put on headset and accept "Allow USB Debugging" prompt
   - Check USB cable is data-capable (not charge-only)

3. Restart ADB:
```bash
adb kill-server
adb start-server
```

### Debug Mode Shows No Data

1. Verify APK is installed:
```bash
python3 oculus_reader_app/oculus_reader/reader.py
```

2. Check Quest tracking:
   - Ensure adequate lighting
   - Clear play area of reflective surfaces
   - Restart Quest if tracking is lost

### Robot Not Moving When Grip Pressed

1. Check debug output shows "Grip: PRESSED (Robot Active)"
2. Verify robot services are all running
3. Ensure teleop is in robot mode
4. Check that robot is not in error state

### Performance Issues

- Use **USB connection** for lowest latency (~15-20ms)
- WiFi adds ~10ms latency
- Ensure Quest controllers have fresh batteries
- Close unnecessary applications on computer

## Command Line Options

### oculus_vr_server.py

```bash
# Debug mode - print data without controlling robot
python3 oculus_vr_server.py --debug

# Use left controller instead of right
python3 oculus_vr_server.py --left-controller

# Connect via WiFi instead of USB
python3 oculus_vr_server.py --ip 192.168.1.100

# Combine options
python3 oculus_vr_server.py --debug --left-controller --ip 192.168.1.100
```

## Safety Notes

1. **Always test in debug mode first** before controlling the robot
2. **Robot only moves when grip is held** - release to stop immediately
3. **Start with small movements** until comfortable with relative control
4. **Ensure clear workspace** around robot
5. **Emergency stop** accessible at all times

## Technical Details

- **Update Rate**: 20 Hz (50ms)
- **Control Mode**: Relative motion from grip point
- **Coordinate System**: Quest tracking space mapped to robot workspace
- **Gripper Control**: Momentary trigger (hold=closed, release=open, default=open)
- **Communication**: ZMQ PUB/SUB on tcp://192.168.1.54:5555
- **Safety**: Robot only active when grip button held

## Comparison with Mouse Control

| Feature | Oculus VR | Mouse Control |
|---------|-----------|---------------|
| DOF | 6 (position + rotation) | 2 (X/Y only) |
| Gripper | Analog trigger | Not available |
| Control Mode | Relative motion from grip | Absolute position |
| Ergonomics | Excellent (any position) | Limited (desk only) |
| Safety | Grip-to-enable | Click-to-enable |
| Intuitiveness | High (natural hand movement) | Medium |
| Setup | Requires headset | Simple |
| Latency | 15-30ms | <10ms |
| Workspace | Full 3D | 2D plane | 