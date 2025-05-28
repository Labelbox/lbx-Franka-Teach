# Foxglove Compatibility Fixes for Camera Data

## Issues Fixed

### 1. Distortion Model Compatibility
**Problem**: RealSense cameras report distortion model as "distortion.inverse_brown_conrady" which Foxglove doesn't recognize.

**Solution**: Added conversion in `frankateach/mcap_data_recorder.py` to map all brown_conrady variants to the standard "plumb_bob" model:
```python
if model == 'distortion.brown_conrady':
    model = 'plumb_bob'
elif model == 'distortion.inverse_brown_conrady':
    model = 'plumb_bob'
elif 'brown_conrady' in str(model).lower():
    model = 'plumb_bob'
```

### 2. Missing Camera Transforms
**Problem**: Foxglove reported "Missing transform from frame <camera_realsense_0> to frame <world>"

**Solution**: 
- Created `configs/camera_transforms.yaml` to define camera positions relative to robot base
- Added `_write_camera_transforms()` method to write static transforms for all camera frames
- Transforms are published to `/tf_static` topic at recording start

### 3. Depth Image Encoding
**Problem**: "Error decoding image: offset is outside the bounds of the DataView"

**Solution**: Fixed depth image encoding in `_write_depth_image_mcap()`:
- Removed PNG compression, using raw bytes instead
- Properly convert float32 depth to uint16 before encoding
- Use native byte order with `tobytes()`
- Correctly calculate dimensions after decimation filter

### 4. Browser Compatibility Warning
**Note**: The warning about unsupported browser is a Foxglove limitation. Use Chrome v111+ for best compatibility.

## Camera Transform Configuration

Camera positions are defined in `configs/camera_transforms.yaml`:
```yaml
camera_transforms:
  realsense_0:
    translation:
      x: -0.3  # 30cm left of robot
      y: 0.2   # 20cm forward
      z: 0.5   # 50cm up
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
```

## Verification

Use `test_mcap_foxglove.py` to check MCAP files for Foxglove compatibility:
```bash
python3 test_mcap_foxglove.py [mcap_file]
```

## Important Notes

1. **Depth Resolution**: With decimation_filter=1 (no decimation), depth images are:
   - 1280×720 for 1280×720 RGB (full HD resolution)
   - 640×480 for 640×480 RGB (if using lower resolution)
2. **Distortion Model**: All RealSense distortion models are mapped to "plumb_bob" for compatibility
3. **Frame IDs**: Camera frames use format "camera_{camera_id}" and "camera_{camera_id}_depth"
4. **Transforms**: 
   - Static camera transforms are written to `/tf_static` at recording start
   - Hand-mounted cameras have dynamic transforms published to `/tf` with robot state updates
5. **Hand-Mounted Cameras**: Cameras with `parent_frame: "fr3_hand"` move with the robot end-effector
6. **Calibration Data**: Separate calibration messages for RGB and depth cameras:
   - `/camera/{id}/camera_info` - RGB camera calibration (original resolution)
   - `/camera/{id}/depth/camera_info` - Depth camera calibration (original resolution)

## Future Improvements

1. Support for custom distortion models with proper coefficient mapping
2. Dynamic camera transform updates during recording
3. Point cloud generation from depth+RGB+calibration data
4. Automatic browser detection and warning in Foxglove 