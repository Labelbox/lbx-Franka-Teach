# Foxglove Robot Visualization Guide

This guide explains how to visualize the Franka FR3 robot in Foxglove Studio when viewing MCAP recordings from the Labelbox Franka-Teach system.

## Quick Start

1. **Open your MCAP file in Foxglove Studio**
   - Drag and drop the MCAP file into Foxglove Studio (web or desktop)
   - Or use File → Open and select your MCAP file

2. **Add a 3D Panel**
   - Click the "+" button in the layout
   - Select "3D" from the panel list

3. **Load the Robot Model**
   - In the 3D panel settings (gear icon), scroll to "Custom Layers"
   - Click "Add URDF"
   - Configure the URDF source:
     - **Recommended**: Select Source: "Attachment" and choose `robot_description`
     - **Alternative**: Select Source: "Topic" and choose `/robot_description`

4. **Enable Robot Visualization**
   - In the 3D panel settings, under "Topics"
   - Enable `/joint_states` to see the robot's joint positions
   - Enable `/tf` to see transform frames
   - Enable `/robot_state` to see additional robot data

5. **Set Display Frame**
   - In the 3D panel settings, under "Frame"
   - Set Display frame to "world" or "base"

## What's Included in the MCAP Files

Our MCAP recordings include:

- **Accurate FR3 URDF Model**: The actual Franka FR3 robot model with proper meshes and joint limits
- **Joint States** (`/joint_states`): Real-time joint positions for all 7 DOF + gripper
- **Robot State** (`/robot_state`): Complete robot state including cartesian position
- **Transform Frames** (`/tf`): Full transform tree (world → base → fr3_link0)
- **VR Controller Data** (`/vr_controller`): Oculus controller poses and button states
- **Actions** (`/action`): Commanded velocity actions

## Key Improvements

### Accurate Robot Model
The system now uses the official Franka FR3 URDF from the `franka_description` package, which includes:
- Accurate visual meshes (DAE files)
- Proper collision geometry (STL files)
- Correct joint limits and dynamics
- Realistic inertial properties

### Complete Transform Tree
The transform tree now includes:
- `world` → `base` → `fr3_link0` → ... → `fr3_link7` → `fr3_hand`
- All transforms are published continuously during recording
- Proper timestamps to avoid "undefined to bigint" errors

## Troubleshooting

### Transform Errors

If you see errors like:
- **"frame world not found. no coordinate frames found"**
- **"tf cant convert undefined to bigint"**

These have been fixed in the latest version. The issues were:
1. Transform messages were using the wrong schema format
2. Initial transforms weren't being written at recording start
3. Timestamps could be undefined in some cases

If you still see these errors with older recordings, try creating a new recording with the updated code.

### Robot Model Not Visible

1. **Check URDF Loading**:
   - Ensure you've added a URDF layer in Custom Layers
   - Try both "Topic" and "Attachment" source options
   - Check the console for any URDF parsing errors

2. **Check Joint States**:
   - Verify `/joint_states` topic is enabled and has data
   - Use a Raw Messages panel to inspect the joint state messages

3. **Check Transforms**:
   - Enable transform visualization in the 3D panel
   - Ensure the display frame is set correctly (usually "world" or "fr3_link0")

### Robot Appears Static

- Ensure playback is running (not paused)
- Check that `/joint_states` topic is enabled
- Verify the time range slider shows the full recording

### Robot in Wrong Position

- Check the display frame setting in the 3D panel
- Try different follow modes (Fixed, Position, Pose)
- Ensure transforms are being published correctly

## Advanced Configuration

### 3D Panel Settings

- **Display Frame**: Set to "world" for global view or "fr3_link0" for robot-centric view
- **Follow Mode**: 
  - "Fixed": Camera stays in place
  - "Position": Camera follows robot position
  - "Pose": Camera follows robot position and orientation

### URDF Layer Settings

- **Frame Prefix**: Leave empty unless using namespaced robots
- **Display Mode**: "Auto" (shows visual geometry)
- **Color**: Fallback color if URDF doesn't specify colors
- **Show Outlines**: Enable to see wireframe edges
- **Show Axis**: Enable to see coordinate frames for each link

## Creating Test Files

Use the provided test script to create sample MCAP files:

```bash
python3 test_foxglove_robot.py
```

This creates a minimal MCAP file with:
- Oscillating joint positions
- Opening/closing gripper
- All required topics for visualization

## Technical Details

### URDF Structure

The embedded URDF describes the Franka FR3 robot with:
- 7 revolute joints (fr3_joint1 through fr3_joint7)
- 2 prismatic joints for gripper fingers
- Simplified geometry (cylinders and boxes)
- Joint limits matching the real robot

### Message Formats

All messages use JSON encoding for compatibility:
- Joint states: ROS2 `sensor_msgs/msg/JointState` format
- Transforms: ROS2 `geometry_msgs/msg/TransformStamped` format
- Robot description: ROS2 `std_msgs/msg/String` format

### Known Limitations

1. The URDF uses simplified geometry, not the actual CAD models
2. Collision geometry is not included
3. Some visual details (cables, logos) are omitted
4. Material properties are basic (white/grey colors only)

For production use, consider loading the official Franka URDF with full mesh files. 