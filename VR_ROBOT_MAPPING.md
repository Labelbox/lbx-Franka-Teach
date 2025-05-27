# VR-to-Robot Control Mapping Documentation

## Overview
This document describes the complete pipeline for mapping Oculus VR controller data to Franka robot commands in the lbx-Franka-Teach system.

## VR Controller Coordinate System (Calibrated)
Based on calibration results:
- **Roll**: Around Y-axis
- **Pitch**: Around X-axis  
- **Yaw**: Around Z-axis

## Control Pipeline

### 1. VR Data Capture (50Hz)
- Raw 4x4 pose matrices from Oculus Reader
- Button states (grip, trigger, joystick, A/B/X/Y)
- No transformations applied at this stage

### 2. Coordinate Transformation
```python
transformed_mat = global_to_env_mat @ vr_to_global_mat @ rot_mat
```
- `global_to_env_mat`: Position reordering (default: [-3, -1, 2, 4])
- `vr_to_global_mat`: Forward direction calibration
- Rotation axis transform: [X, Y, Z] → [-Y, X, Z] (roll inverted for ergonomics)

### 3. Velocity Calculation
```python
pos_action = (target_pos_offset - robot_pos_offset) * pos_action_gain
rot_action = euler_angles_from_quaternion_diff * rot_action_gain
```
- Position gain: 5.0
- Rotation gain: 2.0
- Gripper gain: 3.0

### 4. Velocity Limiting
```python
if norm > max_vel:
    vel = vel * max_vel / norm
```
- Max linear velocity: 1.0
- Max angular velocity: 1.0
- Max gripper velocity: 1.0

### 5. Delta Conversion (DROID-style)
```python
pos_delta = lin_vel * max_lin_delta  # 0.075m
rot_delta = rot_vel * max_rot_delta  # 0.15 rad
```

### 6. Position Target Calculation
```python
target_pos = current_pos + pos_delta
target_quat = pre_calculated_quaternion  # Direct from VR relative rotation
```

### 7. Robot Command (15Hz)
- Send to Deoxys: position (3D) + quaternion (4D) + gripper state
- Workspace bounds: clipped to safety limits
- Deoxys handles inverse kinematics internally

## Key Implementation Details

### Quaternion Handling
Unlike DROID's Polymetis (euler angles), Deoxys expects quaternions directly:
```python
# Calculate target quaternion from VR relative rotation
vr_relative_rot = neutral_rot.inv() * current_rot
target_rot = robot_origin_rot * vr_relative_rot
target_quat = target_rot.as_quat()
```

### Neutral Pose Calibration
- Stores controller orientation during calibration
- All rotations calculated relative to neutral
- Prevents false rotation readings

### Motion Mapping
- VR Roll → Robot Roll (INVERTED for ergonomics)
- VR Pitch → Robot Pitch
- VR Yaw → Robot Yaw

The inversion of roll makes the teleoperation more intuitive - rolling the controller left causes the robot to roll left from the operator's perspective.

## Control Parameters (DROID v1.1 Compatible)
```python
control_hz = 15
pos_action_gain = 5.0
rot_action_gain = 2.0
gripper_action_gain = 3.0
max_lin_delta = 0.075  # meters per cycle
max_rot_delta = 0.15   # radians per cycle
```

## Usage
```bash
# Standard operation
python oculus_vr_server.py

# Debug mode (no robot control)
python oculus_vr_server.py --debug

# Custom coordinate transform
python oculus_vr_server.py --coord-transform -3 -1 2 4
```

## Calibration Process
1. Hold joystick button
2. Move controller forward (>3mm)
3. Release joystick
4. System stores neutral pose and forward direction

## Safety Features
- Workspace bounds checking
- Velocity limiting
- Graceful error handling
- Emergency stop (Ctrl+C) 