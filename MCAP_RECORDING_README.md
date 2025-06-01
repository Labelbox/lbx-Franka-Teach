# MCAP Data Recording in LBX Robotics

This document describes the MCAP data recording system used in the LBX Robotics workspace.

## Overview

The `lbx_data_recorder` ROS 2 package provides a node (`mcap_recorder_node`) for high-performance, asynchronous data recording into MCAP files. It is designed to subscribe to various topics published by other nodes in the system (e.g., robot state, VR controller input, camera imagery) and save them in a structured format suitable for offline analysis, training, and visualization with tools like Foxglove Studio.

## Key Features

- **Asynchronous Writing:** Uses a dedicated writer thread and a message queue to prevent I/O operations from blocking ROS 2 subscriber callbacks, ensuring high-frequency data capture.
- **Configurable Topics:** The specific topics to record and their message types are defined in a YAML configuration file (`recorder_config.yaml`).
- **Customizable Schemas:** While it can leverage `mcap_ros2.ROS2Writer` for standard ROS 2 messages (recommended), it also supports custom JSON schemas for specific data structures (e.g., a combined robot/controller state for training datasets). The current implementation uses the base `mcap.Writer` for more control over custom JSON schemas similar to the original `frankateach` system.
- **Start/Stop Control:** (Future/TODO) Recording can be started and stopped via ROS 2 services.
- **Automatic File Naming:** Generates timestamped MCAP files, categorizing them into `success/` or `failure/` subdirectories based on service call parameters.
- **Diagnostic Publishing:** Publishes its status (recording state, queue size, file path) via ROS 2 diagnostics on the `/diagnostics` topic.
- **Robot URDF:** Saves the robot URDF as an attachment and publishes it on `/robot_description` for visualization.
- **Static Transforms:** Saves static transforms from `/tf_static`.

## Node: `mcap_recorder_node`

### Parameters

- **`base_directory`** (string, default: `~/recordings`): The root directory where recordings will be saved.
- **`default_robot_name`** (string, default: `franka_fr3`): Robot name used in metadata.
- **`auto_start_recording`** (bool, default: `false`): Whether to automatically start recording when the node launches.
- **`auto_start_filename_prefix`** (string, default: `auto_trajectory`): Prefix for auto-started recordings.
- **`topics_to_record`** (string_array, default: `[]`): A list of topic names to subscribe to and record.
- **`topic_types_map`** (string_array, default: `[]`): A list of strings mapping topic names to their full ROS 2 message types, e.g., `"/joint_states:sensor_msgs.msg.JointState"`. **This is crucial for correct deserialization and schema handling.**
- **`recording_frequency_hz`** (double, default: `20.0`): The frequency at which buffered messages are bundled and queued for writing to the MCAP file.
- **`image_jpeg_quality`** (int, default: `85`): JPEG quality for compressing `sensor_msgs/Image` messages if written as `foxglove.CompressedImage`.
- **`mcap_queue_size`** (int, default: `1000`): The maximum number of message bundles to hold in the write queue.
- **`diagnostics_publish_rate_hz`** (double, default: `0.2`): Rate at which diagnostic information is published (e.g., 0.2 Hz = every 5 seconds).
- **`mcap_write_chunk_size_kb`** (int, default: `1024`): MCAP file chunk size in kilobytes. Larger chunks can improve write performance but increase memory usage during writing.

### Subscribed Topics

- Dynamically subscribes to topics listed in the `topics_to_record` parameter.
- `/tf` and `/tf_static` for transformations.

### Published Topics

- `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`): Publishes node status and recording statistics.

### Services (TODO)

- `~/start_recording` (Custom Srv: `StartRecording`): Starts a new recording session.
  - Request: `string filename_prefix`, `string metadata_json` (optional JSON string for custom metadata)
  - Response: `bool success`, `string filepath`, `string message`
- `~/stop_recording` (Custom Srv: `StopRecording`): Stops the current recording.
  - Request: `bool mark_as_success`
  - Response: `bool success`, `string filepath`, `string message`

## Configuration (`recorder_config.yaml`)

An example configuration is provided in the `config/` directory of the `lbx_data_recorder` package. Key fields to customize:

- `base_directory`
- `topics_to_record`: List all topics you need.
- `topic_types_map`: **Crucial for correct operation.** Provide the full ROS 2 message type for each topic in `topics_to_record`.

Example `topic_types_map` entry:

```yaml
topic_types_map:
  [
    "/joint_states:sensor_msgs.msg.JointState",
    "/vision_camera_node/cam_realsense_hand/color/image_raw:sensor_msgs.msg.Image",
  ]
```

## Usage

1.  **Configure:** Modify `config/recorder_config.yaml` to specify the desired topics and their types, base directory, etc.
2.  **Launch:**
    ```bash
    ros2 launch lbx_data_recorder recorder.launch.py
    ```
    Or include it in your main system launch file (see `lbx_robotics/launch/system_bringup.launch.py` for an example).
3.  **Control Recording (Manual/Service - when implemented):**
    - (Future) Use ROS 2 service calls to `~/start_recording` and `~/stop_recording`.
    - For now, if `auto_start_recording` is `true`, it will begin on launch. Otherwise, recording must be triggered by a future service implementation.
4.  **Output:** MCAP files will be saved to `<base_directory>/success/` or `<base_directory>/failure/`.

## MCAP File Format & Schemas

The recorder aims to maintain compatibility with the existing "Labelbox Robotics Timestep" format by writing messages with custom JSON schemas for combined robot state, actions, and VR controller data. Standard ROS 2 messages like `sensor_msgs/Image` or `sensor_msgs/CameraInfo` can be recorded using their native CDR encoding if `mcap_ros2.ROS2Writer` is used, or transformed to a JSON representation (e.g., `foxglove.CompressedImage`) if the base `mcap.Writer` is used with custom JSON schemas.

The current implementation uses the base `mcap.Writer` and defines JSON schemas for:

- `labelbox_robotics.RobotState`
- `labelbox_robotics.Action`
- `labelbox_robotics.VRController`
- `foxglove.CompressedImage` (for `sensor_msgs/Image`)
- `foxglove.CameraCalibration` (for `sensor_msgs/CameraInfo`)
- `tf2_msgs/TFMessage` (for `/tf` and `/tf_static`)
- `std_msgs/String` (for `/robot_description`)
- `sensor_msgs/JointState` (for `/joint_states`)

**Important:** The node's `_serialize_to_custom_json_labelbox` method contains placeholder logic for converting ROS messages to these custom JSON structures. This method **must be completed by the developer** to ensure data is stored in the precise desired format.

## Verification

The `lbx_data_recorder` package also includes a verifier script:

```bash
ros2 run lbx_data_recorder verify_mcap <path_to_mcap_file.mcap>
```

This script checks for basic integrity, message counts, and frequencies.

## Dependencies

- `mcap_ros2_support` (for the `mcap` Python library)
- `python3-opencv` (if encoding images to JPEG within the recorder)
- Other standard ROS 2 packages (`rclpy`, `sensor_msgs`, etc.)
- Any custom message packages if used.

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

```bash
# Create camera configuration file (see camera_config_example.json)
# Start camera server
python camera_server.py --config camera_config.json

# Start VR server with camera configuration
python oculus_vr_server.py --camera-config camera_config.json

# With both camera recording and verification
python oculus_vr_server.py --camera-config camera_config.json --verify-data
```

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
