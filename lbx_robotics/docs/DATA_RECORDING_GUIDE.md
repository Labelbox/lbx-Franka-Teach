# Data Recording Guide

## Overview

The Labelbox Robotics teleoperation system includes a comprehensive data recording system that captures all teleoperation data in MCAP format. The recorder ensures complete data capture including VR states, robot states with torques, camera data (RGB and depth), and calibration information.

## Recorded Data

### Core Data Streams

1. **VR Controller Data**

   - Controller poses (60Hz)
   - Button states and trigger values
   - VR calibration transformation matrix (stored once per recording)

2. **Robot Data**

   - Joint positions, velocities, and **torques** (1000Hz)
   - End-effector poses
   - Gripper states
   - Robot status and diagnostics

3. **Intel RealSense Camera Data**

   - **RGB images** from all configured cameras
   - **Depth images** (raw and aligned to color)
   - **Point clouds** (3D data)
   - Camera calibration info (intrinsics for both RGB and depth)
   - Camera transforms via TF

4. **System Data**
   - System status messages
   - Diagnostics
   - Timestamps and synchronization data

## Recording Process

### Starting a Recording

Recordings can be started in two ways:

1. **VR Controller**: Press the A/X button
2. **ROS Service**: Call `/start_recording` service

```bash
# Start via command line
ros2 service call /start_recording std_srvs/srv/Trigger
```

### During Recording

The system captures:

- All sensor data at native rates
- VR calibration matrix (automatically stored when available)
- Transforms for all frames including cameras
- High-frequency robot torque data
- Both RGB and depth streams from Intel RealSense cameras

### Stopping a Recording

1. **Normal Stop**: Press A/X button again
2. **Mark as Successful**: Press B/Y button (moves to `success/` folder)
3. **Service Call**:
   ```bash
   ros2 service call /stop_recording std_srvs/srv/Trigger
   ```

## Data Verification

When `--verify-data` flag is used, the system automatically verifies recordings after completion.

### Verification Checks

The system verifies presence of:

- ✓ VR data (poses and states)
- ✓ Robot data (joint states)
- ✓ VR calibration matrix
- ✓ Robot torques
- ✓ RGB images
- ✓ Depth images
- ✓ Point clouds (if enabled)
- ✓ Camera transforms

### Verification Output

After recording stops, you'll see:

```
📊 Recording Verification Results
──────────────────────────────────────────────────
📁 File: teleoperation_20240115_143022.mcap
📏 Size: 124.5 MB

Data Completeness:
   ✓ VR Data: ✅
   ✓ Robot Data: ✅
   ✓ VR Calibration: ✅
   ✓ Robot Torques: ✅

Camera Data:
   ✓ RGB Images: ✅
   ✓ Depth Images: ✅
   ✓ Point Clouds: ✅
   ✓ Camera Transforms: ✅

Recording Statistics:
   ⏱️  Duration: 45.2 seconds
   📦 Total Messages: 52,340
   📑 Total Topics: 18

✅ All essential data recorded successfully!
```

## File Organization

```
~/lbx_recordings/
├── teleoperation_20240115_143022.mcap    # Active recordings
├── teleoperation_20240115_144156.mcap
└── success/                               # Successful recordings
    ├── teleoperation_20240115_141234.mcap
    └── teleoperation_20240115_142856.mcap
```

## MCAP File Structure

The MCAP files contain:

```
/vr_pose                    # VR controller pose
/vr_state                   # VR controller state
/joint_states               # Robot joint data with torques
/tf                         # Dynamic transforms
/tf_static                  # Static transforms
/cameras/wrist_camera/color/image_raw        # RGB images
/cameras/wrist_camera/depth/image_rect_raw   # Depth images
/cameras/wrist_camera/aligned_depth_to_color/image_raw  # Aligned depth
/cameras/wrist_camera/depth/color/points     # Point clouds
/cameras/wrist_camera/color/camera_info      # RGB camera calibration
/cameras/wrist_camera/depth/camera_info      # Depth camera calibration
/system_status              # System state with calibration
/diagnostics                # System health
/vr_calibration            # VR transformation matrix (metadata)
```

## Accessing Recorded Data

### Using MCAP CLI

```bash
# View file info
mcap info ~/lbx_recordings/teleoperation_20240115_143022.mcap

# List topics
mcap list topics ~/lbx_recordings/teleoperation_20240115_143022.mcap

# Extract specific topic
mcap filter ~/lbx_recordings/teleoperation_20240115_143022.mcap \
  --topic /joint_states \
  --output joint_data.mcap

# Extract RGB images
mcap filter ~/lbx_recordings/teleoperation_20240115_143022.mcap \
  --topic "/cameras/wrist_camera/color/image_raw" \
  --output rgb_images.mcap

# Extract depth images
mcap filter ~/lbx_recordings/teleoperation_20240115_143022.mcap \
  --topic "/cameras/wrist_camera/depth/image_rect_raw" \
  --output depth_images.mcap
```

### Python Access

```python
from mcap.reader import make_reader
import json
import numpy as np

# Read MCAP file
with open("teleoperation_20240115_143022.mcap", "rb") as f:
    reader = make_reader(f)

    # Find VR calibration
    for schema, channel, message in reader.iter_messages(topics=["/vr_calibration"]):
        calibration_data = json.loads(message.data)
        vr_transform = calibration_data['vr_to_global_transform']
        print(f"VR Calibration Matrix: {vr_transform}")
        break

    # Check for torques in joint states
    for schema, channel, message in reader.iter_messages(topics=["/joint_states"]):
        if hasattr(message, 'effort') and len(message.effort) > 0:
            print(f"Torques available: {message.effort}")
        break

    # Access RGB and depth images
    for schema, channel, message in reader.iter_messages():
        if "color/image_raw" in channel.topic:
            # RGB image in message.data
            print(f"RGB image from {channel.topic}")
        elif "depth/image_rect_raw" in channel.topic:
            # Depth image in message.data
            print(f"Depth image from {channel.topic}")
```

### Foxglove Studio

MCAP files can be directly opened in Foxglove Studio for visualization:

1. Open Foxglove Studio
2. File → Open → Select MCAP file
3. View synchronized playback of all data streams
4. RGB and depth images display in separate panels
5. Point clouds render in 3D view

## Performance Considerations

### Queue Management

- Default queue size: 1000 messages
- Dropped messages logged as warnings
- Monitor `/recording_status` for queue health

### Storage Requirements

Typical data rates:

- VR only: ~5 MB/minute
- With robot data: ~20 MB/minute
- With RGB cameras: ~100-150 MB/minute
- With RGB + depth + point clouds: ~200-300 MB/minute

### Optimization Tips

1. **Reduce Camera FPS** if storage is limited
2. **Disable point clouds** if not needed (saves ~50% camera bandwidth)
3. **Use aligned depth only** instead of both raw and aligned
4. **Monitor queue size** during recording
5. **Use SSD storage** for best performance

## Troubleshooting

### Recording Won't Start

- Check if recorder node is running: `ros2 node list | grep recorder`
- Verify services available: `ros2 service list | grep recording`
- Check disk space: `df -h ~/lbx_recordings`

### Missing Data in Verification

- **No VR Calibration**: Ensure calibration completed before recording
- **No Torques**: Check if robot is publishing effort values
- **No RGB Images**: Verify RealSense camera is configured for color
- **No Depth Images**: Check if depth stream is enabled
- **No Point Clouds**: May be disabled for performance
- **No Transforms**: Check TF tree is complete

### Performance Issues

- High CPU usage: Disable point cloud generation
- Queue overflow: Increase `queue_size` parameter or reduce camera FPS
- Slow disk writes: Use faster storage (NVMe SSD recommended)

## Advanced Usage

### Custom Recording Directory

```bash
ros2 launch lbx_data_recorder recorder.launch.py \
  output_dir:=/path/to/recordings
```

### Batch Verification

```bash
# Verify all recordings in a directory
for file in ~/lbx_recordings/*.mcap; do
  echo "Verifying $file..."
  python3 verify_mcap.py "$file"
done
```

### Integration with Training Pipeline

The MCAP format is compatible with common ML frameworks:

```python
# Convert to HDF5 for training
from mcap_to_hdf5 import convert_recording

convert_recording(
    "teleoperation_20240115_143022.mcap",
    "training_data.hdf5",
    include_rgb=True,
    include_depth=True,
    include_point_clouds=False,  # Omit for smaller files
    downsample_images=2  # Reduce image size by factor of 2
)
```

## Intel RealSense Specific Notes

### Camera Topics Structure

Each RealSense camera publishes:

- `/cameras/<name>/color/image_raw` - RGB image (typically 1920x1080)
- `/cameras/<name>/depth/image_rect_raw` - Raw depth image (typically 640x480)
- `/cameras/<name>/aligned_depth_to_color/image_raw` - Depth aligned to RGB frame
- `/cameras/<name>/depth/color/points` - Colored point cloud
- `/cameras/<name>/color/camera_info` - RGB camera intrinsics
- `/cameras/<name>/depth/camera_info` - Depth camera intrinsics

### Depth Data Format

- Depth images use 16-bit unsigned integers
- Values represent distance in millimeters
- 0 indicates no depth reading
- Maximum range typically 10 meters (10000mm)

## Best Practices

1. **Always verify critical recordings** with `--verify-data`
2. **Mark successful demonstrations** immediately (B/Y button)
3. **Monitor disk space** before long recording sessions
4. **Test recording setup** before important demonstrations
5. **Keep recordings organized** by task/date
6. **Consider disabling point clouds** for longer recordings
7. **Use aligned depth** for RGB-D learning applications
