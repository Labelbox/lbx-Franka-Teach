# Hand-Mounted Camera Configuration Update

## Changes Made

### 1. Camera Resolution Update
- Updated both RealSense cameras from 640×480 to **1280×720** (HD resolution)
- Depth images are stored at **full 1280×720 resolution** (no decimation)
- Higher resolution provides better image quality for manipulation tasks

### 2. Hand-Mounted Camera Support
- **realsense_1** is now configured as a hand-mounted camera
- Parent frame changed from `base` to `fr3_hand`
- Camera moves with the robot end-effector for close-up views

### 3. Dynamic Transform Support
Added support for dynamic camera transforms in MCAP recording:
- Static cameras (mounted on base) publish to `/tf_static` once
- Hand-mounted cameras publish to `/tf` with every robot state update
- Ensures camera position is always synchronized with robot motion

### 4. Original Resolution Storage
The MCAP files now store complete camera calibration data:
- **Color camera intrinsics**: Full 1280×720 resolution parameters
- **Depth camera intrinsics**: Full 1280×720 resolution parameters (no decimation)
- **Decimation factor**: Set to 1 (no decimation)
- Separate calibration topics for color and depth:
  - `/camera/{id}/camera_info` - RGB calibration
  - `/camera/{id}/depth/camera_info` - Depth calibration

## Configuration Files Updated

### `configs/cameras_intel.yaml`
```yaml
width: 1280
height: 720
decimation_filter: 1  # No decimation - store original resolution
```

### `configs/camera_transforms.yaml`
```yaml
realsense_0:
  parent_frame: "base"  # Fixed camera
  translation:
    x: -0.3  # 30cm left of robot
    y: 0.2   # 20cm forward
    z: 0.5   # 50cm up

realsense_1:
  parent_frame: "fr3_hand"  # Hand-mounted camera
  translation:
    x: 0.0   # Centered on hand
    y: 0.0   # No lateral offset
    z: 0.1   # 10cm forward from mounting
```

## Benefits

1. **Better Manipulation Views**: Hand-mounted camera provides close-up views of grasping and manipulation
2. **Higher Quality Images**: 1280×720 resolution captures more detail
3. **Synchronized Motion**: Camera transforms update with robot motion in real-time
4. **Flexible Configuration**: Easy to change camera mounting via YAML config
5. **Complete Calibration Data**: Both RGB and depth at original resolution for accurate 3D reconstruction
6. **Maximum Data Fidelity**: No decimation means no loss of depth information

## Resolution Details

With the current settings:
- **RGB Images**: 1280×720 (original resolution)
- **Depth Images**: 1280×720 (original resolution, no decimation)
- **Decimation Factor**: 1 (no reduction)

Storing full resolution depth data:
- Preserves all depth information for training and analysis
- Enables high-quality 3D reconstruction
- Larger file sizes but maximum data fidelity
- Post-processing decimation can be applied later if needed

## Testing Results

- Both cameras tested successfully at 1280×720
- Actual FPS: ~20fps (may be lower with full resolution depth)
- Depth coverage warnings are normal for close-range D405 cameras

## Usage

No changes needed to the server startup:
```bash
./run_server.sh
```

The hand-mounted camera will automatically move with the robot during teleoperation and its position will be correctly recorded in the MCAP files for playback in Foxglove Studio. 