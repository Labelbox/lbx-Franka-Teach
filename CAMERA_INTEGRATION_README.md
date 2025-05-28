# Camera Integration for Labelbox Franka Teach System

This document describes the camera integration with the Oculus VR server for recording teleoperation demonstrations with visual data.

## Overview

The camera system supports professional-grade depth cameras and integrates seamlessly with the async architecture of the Oculus VR server:

- **Intel RealSense** cameras (RGB + Depth)
- **ZED** stereo cameras (RGB + Depth)

All camera data is recorded directly into MCAP files alongside robot and VR controller data.

## Features

- 🎥 **Multi-camera support** - Record from multiple cameras simultaneously
- 🔄 **Asynchronous capture** - Non-blocking camera threads for minimal latency
- 📊 **Depth recording** - Native depth data from RealSense/ZED cameras
- 🗜️ **Efficient compression** - JPEG for RGB, PNG for depth
- ⚡ **High performance** - Queue-based architecture prevents frame drops
- 🔧 **Modular design** - Easy to add new camera types
- 🧪 **Built-in testing** - Verify camera functionality before use

## Quick Start

### 1. Install Dependencies

```bash
# Intel RealSense SDK (for RealSense cameras)
pip install pyrealsense2==2.55.1.6486

# OpenCV (for image processing)
pip install opencv-python>=4.5.0

# PyYAML (for configuration files)
pip install pyyaml

# For ZED cameras:
# 1. Install ZED SDK 5.0 from https://www.stereolabs.com/developers/release
# 2. Run: python3 /usr/local/zed/get_python_api.py
```

### 2. Configure Cameras

The system uses **manufacturer-specific configuration files**:

#### Automatic Configuration Generation

```bash
# Discover cameras and generate configs
python3 discover_cameras.py

# This creates:
#   - configs/cameras_intel.yaml    (Intel RealSense cameras)
#   - configs/cameras_zed.yaml      (ZED cameras)
```

#### Manual Configuration

Edit the appropriate manufacturer config file:

**Intel RealSense** (`configs/cameras_intel.yaml`):
```yaml
manufacturer: Intel RealSense
cameras:
  realsense_0:
    enabled: true
    type: realsense
    serial_number: "218622277093"
    port_offset: 0
    config:
      width: 640
      height: 480
      fps: 30
      enable_depth: true
    description: "Intel RealSense D405"
    usb_port_info: "2-3.3"  # Automatically detected
```

**ZED** (`configs/cameras_zed.yaml`):
```yaml
manufacturer: ZED
cameras:
  zed_0:
    enabled: true
    type: zed
    serial_number: "12345678"
    port_offset: 0
    config:
      resolution: HD720
      fps: 30
      enable_depth: true
      depth_mode: ULTRA
    description: "ZED 2i"
```

### 3. Test Cameras

**Always test cameras before running the main server:**

```bash
# Test Intel RealSense cameras
python3 test_cameras.py --config configs/cameras_intel.yaml

# Test ZED cameras
python3 test_cameras.py --config configs/cameras_zed.yaml

# Test with verbose output
python3 test_cameras.py --config configs/cameras_intel.yaml --verbose
```

The test will:
- ✅ Connect to each enabled camera
- ✅ Capture RGB and depth frames
- ✅ Measure actual FPS vs target
- ✅ Check depth data quality
- ✅ Report any issues or warnings

Example output:
```
🔍 Starting camera tests...
============================================================

📷 Testing realsense_0 (realsense)...
Connected to Intel RealSense D405 (FW: 5.12.14.100)
   ✅ PASS realsense_0 (realsense) - 640x480 @ 29.8fps

============================================================
📊 Camera Test Summary:
   Total cameras tested: 1
   Passed: 1
   Failed: 0

✅ All camera tests PASSED!
```

### 4. Run with Cameras

The server automatically detects the appropriate camera configuration:

```bash
# Automatic detection (checks for Intel, then ZED configs)
./run_server.sh

# With hot reload support
./run_server.sh --hot-reload

# Or explicitly specify a manufacturer config
python3 oculus_vr_server.py --enable-cameras --camera-config configs/cameras_intel.yaml
```

## Camera Testing

### Automatic Testing on Server Start

When cameras are enabled, the Oculus VR server automatically runs camera tests during initialization:

```
🔍 Testing camera functionality...
[Test results shown here]

❌ Camera tests failed!
   Some cameras are not functioning properly.
   Continue anyway? (y/N): 
```

If tests fail, you can:
- Press `N` to exit and debug the issue
- Press `Y` to continue with available cameras

### Manual Testing

Run camera tests independently:

```bash
# Basic test
python3 test_cameras.py --config configs/cameras_intel.yaml

# Verbose test (shows more details)
python3 test_cameras.py --config configs/cameras_intel.yaml --verbose
```

### Common Test Failures and Solutions

| Error | Cause | Solution |
|-------|-------|----------|
| "RealSense SDK not available" | pyrealsense2 not installed | `pip install pyrealsense2==2.55.1.6486` |
| "ZED SDK not available" | pyzed not installed | Install ZED SDK and Python API |
| "Failed to open camera" | Camera not connected or in use | Check USB connection, close other apps |
| "Low depth coverage" | Poor lighting or surface | Improve lighting, avoid reflective surfaces |
| "Low FPS" | USB bandwidth or CPU load | Use USB 3.0, reduce resolution/fps |
| "No serial number" | Missing config | Add serial number to YAML config |

## Camera Configuration

### Configuration File Structure

The camera configuration file has two main sections:

1. **`cameras`** - Individual camera definitions
2. **`camera_settings`** - Global settings

### Intel RealSense Configuration

```yaml
cameras:
  realsense_example:
    enabled: true
    type: realsense
    serial_number: "123456789"  # Find using rs-enumerate-devices
    port_offset: 0              # Unique offset for ZMQ ports
    config:
      width: 640
      height: 480
      fps: 30
      enable_depth: true
      depth_processing_preset: 1  # 1=High Accuracy, 2=High Density, 3=Medium
      align_depth_to_color: true  # Align depth to RGB frame
      decimation_filter: 2        # Downsample depth by factor
      spatial_filter: true        # Edge-preserving spatial filter
      temporal_filter: true       # Temporal consistency filter
    description: "Example RealSense camera"
```

### ZED Configuration

```yaml
cameras:
  zed_example:
    enabled: true
    type: zed
    serial_number: "12345678"
    port_offset: 0
    config:
      resolution: HD720         # HD720, HD1080, HD2K, VGA
      fps: 30
      enable_depth: true
      depth_mode: ULTRA        # NONE, PERFORMANCE, QUALITY, ULTRA, NEURAL
      depth_minimum_distance: 0.3
      confidence_threshold: 100
      texture_confidence_threshold: 100
    description: "Example ZED camera"
```

### Automatic Camera Discovery

The system can automatically discover all connected cameras:

```bash
# Run the discovery tool
python discover_cameras.py

# Or use the module directly
python -m frankateach.camera_utils
```

This will:
1. Find all Intel RealSense cameras with serial numbers
2. Detect ZED cameras via SDK
3. Show USB port assignments (e.g., "1-2.3")
4. Generate configuration files automatically

### Finding Camera Serial Numbers

For Intel RealSense cameras:
```bash
# List all connected RealSense devices
rs-enumerate-devices

# Or use Python
python -c "import pyrealsense2 as rs; print([d.get_info(rs.camera_info.serial_number) for d in rs.context().devices])"
```

For ZED cameras:
```bash
# Use ZED SDK tools
ZED_Explorer
# Or check in the ZED SDK application
```

## Architecture

### Async Data Flow

```
Camera Hardware → Camera Manager → Frame Queues → MCAP Recorder → MCAP File
       ↓               ↓                ↓              ↓
   [30-60 FPS]    [Per-camera     [Non-blocking]  [Compressed
                   threads]         queues]         & timestamped]
```

### Key Components

1. **`CameraManager`** (`frankateach/camera_manager.py`)
   - Manages multiple cameras
   - Spawns capture threads
   - Provides frame queues

2. **`MCAPDataRecorder`** (`frankateach/mcap_data_recorder.py`)
   - Consumes frames from queues
   - Compresses and encodes data
   - Writes to MCAP file

3. **Camera Implementations**
   - `RealsenseCamera` - Intel RealSense support
   - `ZEDCamera` - ZED stereo camera support

## MCAP Data Format

Camera data is stored in MCAP files with the following topics:

- `/camera/{camera_id}/compressed` - JPEG compressed RGB images
- `/camera/{camera_id}/depth` - PNG compressed depth images (16-bit)

### RGB Image Message
```json
{
  "timestamp": {"sec": 1234567890, "nanosec": 123456789},
  "frame_id": "camera_realsense_hand",
  "data": "base64_encoded_jpeg_data",
  "format": "jpeg"
}
```

### Depth Image Message
```json
{
  "timestamp": {"sec": 1234567890, "nanosec": 123456789},
  "frame_id": "camera_realsense_hand_depth",
  "data": "base64_encoded_png_data",
  "encoding": "16UC1",
  "scale": 0.001  // Scale to convert to meters
}
```

## Performance Considerations

### Recommended Settings

For optimal performance:

```yaml
camera_settings:
  jpeg_quality: 90        # Balance quality vs file size
  enable_threading: true  # Always use threading
  buffer_size: 5         # Small buffer to prevent memory buildup
```

### Camera Resolution Trade-offs

| Resolution | FPS | Use Case |
|------------|-----|----------|
| 640x480    | 30  | Default - good balance |
| 1280x720   | 15  | Higher quality, slower |
| 320x240    | 60  | Fast motion capture |

### Depth Processing

Depth processing can be CPU intensive. Options to improve performance:

1. **Decimation** - Reduce depth resolution
2. **Disable filters** - Turn off spatial/temporal filters
3. **Lower FPS** - Reduce depth stream FPS

## Troubleshooting

### Common Issues

1. **Camera not detected**
   ```
   ⚠️  Failed to initialize camera_id: No device connected
   ```
   - Check USB connection
   - Verify serial number in config
   - Run `rs-enumerate-devices` or ZED tools

2. **Frame drops**
   ```
   ⚠️  MCAP queue full, dropping frame
   ```
   - Reduce camera FPS
   - Lower resolution
   - Check CPU usage

3. **Permission errors**
   ```
   ⚠️  Failed to open camera: Permission denied
   ```
   - Add user to `video` group: `sudo usermod -a -G video $USER`
   - Logout and login again

### Debug Mode

Run with debug output:
```bash
python oculus_vr_server.py --debug --enable-cameras --camera-config configs/cameras_intel.yaml
```

## Hot Reload Support

The camera system fully supports hot reload mode:

```bash
# Run with hot reload and cameras
./run_server.sh --hot-reload

# The server will automatically:
# 1. Detect Intel or ZED camera configs
# 2. Pass camera arguments to hot reload mode
# 3. Restart with cameras when code changes
```

## Adding New Camera Types

To add support for a new camera type:

1. Create a new camera class in `frankateach/camera_manager.py`:
```python
class MyNewCamera:
    def __init__(self, camera_id: str, config: Dict):
        # Initialize camera
        pass
        
    def capture_frame(self) -> Optional[CameraFrame]:
        # Return CameraFrame with color, depth, intrinsics
        pass
        
    def stop(self):
        # Cleanup
        pass
```

2. Register in `CameraManager._init_camera()`:
```python
elif cam_type == 'mynewcamera':
    return MyNewCamera(camera_id, config)
```

3. Add configuration example

## Integration with Training

The recorded MCAP files contain synchronized:
- Robot states (joint positions, cartesian pose)
- VR controller data (poses, buttons)
- Camera images (RGB + depth)
- Actions (velocity commands)

This data can be used for:
- Imitation learning
- Visual servoing
- Dataset creation
- Replay and analysis

## Future Enhancements

- [ ] Multi-camera synchronization
- [ ] Camera calibration tools
- [ ] ROI (Region of Interest) recording
- [ ] Real-time visualization
- [ ] Automatic exposure/gain control

## Camera Mounting Configurations

The system supports both fixed and robot-mounted cameras:

### Fixed Cameras
Cameras mounted on tables or stands use `parent_frame: "base"` and have static transforms published once at recording start.

### Hand-Mounted Cameras
Cameras mounted on the robot hand use `parent_frame: "fr3_hand"` and have dynamic transforms that update with robot motion. This allows the camera to move with the end-effector for close-up manipulation views.

Example configuration in `configs/camera_transforms.yaml`:
```yaml
realsense_1:
  parent_frame: "fr3_hand"  # Mounted on robot hand
  translation:
    x: 0.0   # Centered on hand
    y: 0.0   # No lateral offset
    z: 0.1   # 10cm forward from mounting point
```

## Camera Resolution

The default resolution is now **1280×720** (HD) for better image quality. With decimation_filter=2, depth images will be 640×360. 