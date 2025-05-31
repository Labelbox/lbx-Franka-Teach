# RealSense Vision Package (lbx_vision_realsense)

Documentation for the Intel RealSense camera ROS2 package.

## Nodes

- `realsense_node`
  - **Publishes**:
    - `/camera/color/image_raw` (`sensor_msgs/msg/Image`)
    - `/camera/color/camera_info` (`sensor_msgs/msg/CameraInfo`)
    - `/camera/depth/image_raw` (`sensor_msgs/msg/Image`)
    - `/camera/depth/camera_info` (`sensor_msgs/msg/CameraInfo`)

## Launch Files

- `realsense.launch.py`

## Configuration

- `src/lbx_vision_realsense/config/realsense_cameras.yaml`
