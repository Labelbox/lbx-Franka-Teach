# MCAP Data Recording for Franka-Teach

This system records teleoperation data in MCAP format compatible with Labelbox Robotics data pipeline.

## Features

- Records robot states (including joint positions), actions, and VR controller data
- Supports multiple camera streams (Intel RealSense/ZED)
- Compatible with Foxglove Studio visualization
- Automatic success/failure categorization
- Thread-safe recording with queues
- Robot visualization support with joint states

## Recording Controls

When using the Oculus VR server with recording enabled:

- **A button**: Start/reset recording
- **B button**: Mark recording as successful and save

## Data Format

The MCAP files contain the following data streams:

### Robot State (`/robot_state`)
- Joint positions (7 DOF)
- Joint velocities
- Joint efforts
- Cartesian position (6 DOF: x, y, z, roll, pitch, yaw)
- Cartesian velocity
- Gripper position (0-1)
- Gripper velocity

### Joint States (`/joint_states`)
- ROS2-style sensor_msgs/msg/JointState messages for robot visualization
- Includes all 7 arm joints + 2 finger joints
- Compatible with Foxglove Studio 3D visualization
- Uses JSON encoding for compatibility

### Action (`/action`)
- 7-DOF velocity commands sent to the robot
- Format: [linear_vel_x, linear_vel_y, linear_vel_z, angular_vel_x, angular_vel_y, angular_vel_z, gripper_vel]

### VR Controller (`/vr_controller`)
- Controller poses (4x4 transformation matrices)
- Button states
- Movement enabled flag
- Success/failure flags

### Camera Images (`/camera/{id}/compressed`)
- JPEG compressed images from configured cameras
- Foxglove-compatible CompressedImage format

## File Organization

Recordings are saved in:
- `~/recordings/success/` - Successful demonstrations
- `~/recordings/failure/` - Failed/incomplete demonstrations

Filename format: `trajectory_YYYYMMDD_HHMMSS_XXmYYs.mcap`
- Timestamp when recording started
- Duration of the recording

## Usage

### Basic Recording
```bash
# Start VR server with recording enabled (default)
python oculus_vr_server.py

# Start VR server without recording
python oculus_vr_server.py --no-recording

# Start VR server with automatic data verification
python oculus_vr_server.py --verify-data
```

### With Camera Recording

To include camera data in your MCAP recordings:

```bash
# Auto-discover and use all connected cameras
python oculus_vr_server.py --enable-cameras --auto-discover-cameras

# Or use specific camera configuration
python oculus_vr_server.py --enable-cameras --camera-config configs/cameras_intel.yaml
```

The camera data will be automatically included in the MCAP file with RGB and depth streams.

### Data Verification

The `--verify-data` flag enables automatic verification after each successful recording. The verifier checks:

- **File integrity**: Proper MCAP format and readability
- **Data completeness**: All required data streams present
- **Joint data**: Validates 7-DOF joint positions are recorded
- **Message frequencies**: Ensures data is recorded at expected rates
- **Data ranges**: Checks for reasonable values in actions and states
- **Visualization readiness**: Confirms joint states for Foxglove Studio

Example verification output:
```
🔍 Verifying MCAP file: trajectory_20240115_143022_02m15s.mcap
============================================================

📁 File Information:
   Size: 45.23 MB
   Duration: 02m15s

📊 Data Streams:

   /robot_state:
      Schema: labelbox_robotics.RobotState
      Messages: 2025
      Frequency: 15.0 Hz (±0.1)
      Has joint data: ✅
      Joint data valid: ✅
      Gripper range: [0.000, 1.000]

   /action:
      Schema: labelbox_robotics.Action
      Messages: 2025
      Frequency: 15.0 Hz (±0.1)
      Linear velocity range: [-0.523, 0.612]
      Angular velocity range: [-0.234, 0.189]

   /joint_states:
      Schema: sensor_msgs/JointState
      Messages: 2025
      Frequency: 15.0 Hz (±0.1)

📋 Summary:
   Total messages: 6075
   Data streams present: 3/3
   Joint data: ✅ Valid
   Visualization ready: ✅

✅ VERIFICATION PASSED
```

### Manual Verification

You can also verify existing MCAP files manually:

```python
from frankateach.mcap_verifier import MCAPVerifier

# Verify a specific file
verifier = MCAPVerifier("~/recordings/success/trajectory_20240115_143022_02m15s.mcap")
results = verifier.verify(verbose=True)

# Check results programmatically
if results["summary"]["is_valid"]:
    print("Recording is valid!")
else:
    print(f"Issues found: {results['summary']['error_count']} errors, {results['summary']['warning_count']} warnings")
```

### Testing Joint Data Recording
```bash
# Verify that joint data is being recorded correctly
python test_joint_recording.py
```

## Viewing Recordings

### Using Foxglove Studio

1. Download [Foxglove Studio](https://foxglove.dev/download)
2. Open the MCAP file
3. Add panels:
   - 3D panel to visualize robot motion (requires URDF)
   - Plot panels for joint positions
   - Image panels for camera streams
   - Raw Messages panel to inspect data

#### Visualizing the Robot in 3D

The MCAP files now include:
- Robot URDF model embedded as an attachment
- Joint states published to `/joint_states` topic
- Base transform to `/base_transform` topic

To visualize the robot:

1. **Open the MCAP file** in Foxglove Studio
2. **Add a 3D panel** (click + → 3D)
3. **Configure the 3D panel**:
   - In the panel settings, you should see the robot model automatically loaded
   - If not, check the "Attachments" section in Foxglove for `robot_description`
   - The robot should appear and move according to the recorded joint states

4. **Troubleshooting visualization**:
   - Ensure `/joint_states` topic is visible in the data source panel
   - Check that joint names match: `fr3_joint1` through `fr3_joint7`
   - Verify the URDF is loaded (check Attachments panel)
   - Try adjusting the 3D view camera position

### Programmatic Access
```python
from frankateach.mcap_data_reader import MCAPDataReader

reader = MCAPDataReader("trajectory_20240115_143022_02m15s.mcap")
for timestep in reader:
    robot_state = timestep["observation"]["robot_state"]
    action = timestep["action"]
    # Process data...
```

## Schema Format

All robot data uses the `labelbox_robotics` schema namespace:
- `labelbox_robotics.RobotState`
- `labelbox_robotics.Action`
- `labelbox_robotics.VRController`

Camera images use standard Foxglove schemas:
- `foxglove.CompressedImage`

## Troubleshooting

### No Joint Data in Recording
- Ensure the robot server is updated to include joint positions
- Run `test_joint_recording.py` to verify joint data availability
- Check that `FrankaState` message includes `joint_positions` field

### Cannot Visualize Robot in Foxglove
- Joint state messages are published to `/joint_states` topic
- Ensure your URDF is loaded in Foxglove Studio
- Check that joint names match: `fr3_joint1` through `fr3_joint7`

### Recording Not Starting
- Check that the MCAP recorder initialized successfully
- Verify write permissions to `~/recordings/` directory
- Look for error messages in the console

## Implementation Details

The recording system consists of:

1. **MCAPDataRecorder** (`frankateach/mcap_data_recorder.py`)
   - Manages MCAP file writing
   - Handles data queuing and threading
   - Provides success/failure categorization

2. **VR Server Integration** (`oculus_vr_server.py`)
   - Captures robot state at control frequency
   - Records VR controller data
   - Manages recording lifecycle with button controls

3. **Robot Server** (`frankateach/franka_server.py`)
   - Provides joint position data via `FrankaState` message
   - Includes 7-DOF joint positions from Deoxys interface

## Future Improvements

- [ ] Add support for force/torque data
- [ ] Include robot URDF directly in MCAP metadata
- [ ] Support for multiple robot configurations
- [ ] Real-time streaming to Foxglove Studio
- [ ] Automatic data validation and quality checks 