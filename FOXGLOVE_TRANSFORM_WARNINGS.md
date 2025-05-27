# Foxglove Transform Warnings Explained

When viewing MCAP recordings in Foxglove Studio, you may see warnings like:

```
Frame "fr3_link1" is not provided by the data source. The default URDF transform will be used.
Frame "fr3_link2" is not provided by the data source. The default URDF transform will be used.
...
Frame "fr3_hand" is not provided by the data source. The default URDF transform will be used.
```

## What These Warnings Mean

These warnings are **normal and expected**. They indicate that:

1. **Foxglove is looking for explicit transform messages** for each link in the robot URDF
2. **We only publish base transforms** (world → base → fr3_link0)
3. **Foxglove automatically computes the remaining transforms** using:
   - The URDF model (joint definitions)
   - The joint state messages (joint positions)

## Why We Don't Publish All Transforms

Publishing transforms for every robot link would:
- Increase MCAP file size significantly
- Duplicate information already in the URDF and joint states
- Add unnecessary computational overhead

Instead, we follow the standard ROS approach:
- Publish static transforms for the base
- Publish joint states with current joint positions
- Let the visualization tool compute forward kinematics

## The Robot Still Visualizes Correctly

Despite these warnings:
- ✅ The robot model appears correctly in Foxglove
- ✅ All links are in their proper positions
- ✅ The robot moves according to the recorded joint states
- ✅ The gripper opens and closes properly

## If You See "can't convert undefined to BigInt" Error

This error might occur if:
1. Timestamps are not properly formatted as integers
2. There's a mismatch in timestamp formats

We've added safeguards to ensure all timestamps are integers, but if you still see this error:
- Check that your recording software is up to date
- Try re-recording the data
- Report the issue with the specific MCAP file

## Summary

The transform warnings are informational only and don't affect the visualization. Your robot should appear and move correctly in Foxglove Studio despite these warnings. 

# Foxglove Transform Warnings - Troubleshooting Guide

## ⚠️ "Frame X is not provided by the data source" - This is NORMAL!

When you see warnings like:
```
Frame "fr3_link1" is not provided by the data source. The default URDF transform will be used.
Frame "fr3_link2" is not provided by the data source. The default URDF transform will be used.
...
Frame "fr3_hand" is not provided by the data source. The default URDF transform will be used.
```

**These are NOT errors!** They are informational messages telling you that Foxglove is doing exactly what it should:

1. ✅ Using the URDF model to understand how robot links connect
2. ✅ Using joint state messages to get current joint angles  
3. ✅ Computing forward kinematics to position each link
4. ✅ The robot will visualize correctly despite these warnings

### Why We Don't Publish All Transforms

Publishing transforms for every robot link would:
- 🚫 Duplicate information already in the URDF
- 🚫 Increase file size unnecessarily
- 🚫 Add computational overhead
- 🚫 Be redundant since joint states already provide the needed data

### What We DO Publish

We follow the standard ROS approach:
- ✅ Static transforms: world → base → fr3_link0
- ✅ Joint states with current joint positions
- ✅ The URDF model (embedded in the MCAP file)

Foxglove automatically computes all other transforms using forward kinematics.

## Common Errors

### 1. "frame world not found. no coordinate frames found"
This error occurs when Foxglove can't find the transform tree. Make sure:
- Transform messages are being published to `/tf` topic
- The root frame is named "world"
- Transforms are published before other data

### 2. "tf cant convert undefined to bigint"
This error occurs when there's a timestamp format mismatch. Foxglove expects consistent timestamp formats across all messages.

**Root Cause**: Mixing ROS1 (`sec`/`nsec`) and ROS2 (`sec`/`nanosec`) timestamp formats in the same MCAP file.

**Solution**: Use consistent timestamp format throughout. We standardized on ROS2 format:
```json
{
  "timestamp": {
    "sec": 1234567890,
    "nanosec": 123456789  // Note: "nanosec" not "nsec"
  }
}
```

## Fixed Issues

### Timestamp Format Consistency (FIXED)
- **Problem**: We were using ROS1 format (`nsec`) for custom schemas and ROS2 format (`nanosec`) for standard ROS messages
- **Solution**: Updated all schemas and write methods to use ROS2 format (`nanosec`) consistently
- **Files Changed**: 
  - `frankateach/mcap_data_recorder.py` - Updated all schema definitions and write methods
  - Changed: `labelbox_robotics.RobotState`, `labelbox_robotics.Action`, `labelbox_robotics.VRController`, `foxglove.CompressedImage`

### Transform Publishing (FIXED)
- **Problem**: Transforms weren't being published at recording start
- **Solution**: Added `_write_initial_transforms()` method that publishes world→base→fr3_link0 transform chain at recording start
- **Implementation**: Transforms are published to both `/tf` and `/tf_static` topics

### URDF Embedding (FIXED)
- **Problem**: Robot model wasn't visible in Foxglove
- **Solution**: 
  - Created `fr3_embedded.urdf` with primitive shapes (no external meshes)
  - URDF is embedded as MCAP attachment
  - Also published on `/robot_description` topic for compatibility

## Verification Steps

1. Start recording with VR server:
   ```bash
   python oculus_vr_server.py --verify-data
   ```

2. Press A to start recording, move robot, press B to save

3. Open MCAP file in Foxglove Studio:
   ```bash
   foxglove-studio ~/recordings/success/trajectory_*.mcap
   ```

4. Check:
   - No transform warnings in console
   - Robot model is visible in 3D panel
   - Joint states update properly
   - Transform tree shows world→base→fr3_link0

## Technical Details

### MCAP Timestamp Format
- MCAP internally uses uint64 nanoseconds for `log_time` and `publish_time`
- Message data can use any format, but consistency is key
- Foxglove's JSON parser expects consistent format across all messages

### ROS2 Timestamp Format
```json
{
  "header": {
    "stamp": {
      "sec": 1234567890,
      "nanosec": 123456789
    },
    "frame_id": "world"
  }
}
```

### Transform Message Structure
```json
{
  "transforms": [
    {
      "header": {
        "stamp": {"sec": 1234567890, "nanosec": 123456789},
        "frame_id": "world"
      },
      "child_frame_id": "base",
      "transform": {
        "translation": {"x": 0, "y": 0, "z": 0},
        "rotation": {"x": 0, "y": 0, "z": 0, "w": 1}
      }
    }
  ]
}
``` 