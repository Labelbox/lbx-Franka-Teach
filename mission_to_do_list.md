# Mission To-Do List

## Completed ✅

[x] Convert VR server to async

- ✅ Achieved 40Hz recording rate (6x improvement from 6.6Hz)
- ✅ Implemented 5-thread architecture with non-blocking operation
- ✅ Added predictive control for smooth teleoperation
- ✅ Created comprehensive documentation (ASYNC_ARCHITECTURE_README.md)
- ✅ Added hot reload feature for development
- ✅ Performance mode with doubled frequency and increased gains
- ✅ Data verification after recording

[x] Hardware Improvements

- ✅ Snug the Franka FR3 hand to the joint (adjusted URDF with xyz_ee offset and rotation)
  - Final configuration: 110mm closer (offset: -0.110m) with -45° rotation (45° CCW)
  - Created adjustment tools: apply_hand_offset.py, patch_mcap_urdf.py, test_hand_rotation.py
  - Documented adjustment methods in HAND_ADJUSTMENT_TOOLS.md
  - MCAP recordings now use updated URDF from robot_urdf_models/fr3_franka_hand_snug.urdf

[x] Multi-Camera System

- ✅ Set up dual camera system
- ✅ Integrated and configured two cameras for multi-angle capture
- ✅ Added camera synchronization with robot data

[x] Migrate to MoveIt/ROS2 for Low-Latency Control

- ✅ Created modular ROS2/MoveIt implementation in franka_vr_ros2/
- ✅ Preserved exact Labelbox coordinate transformations
- ✅ Single entry point: oculus_vr_server.py (launches everything)
- ✅ Modular control strategies: MoveIt Servo, Direct IK, Cartesian Pose
- ✅ Async architecture with <10ms latency target (vs 120ms with Deoxys)
- ✅ Preserved all features: MCAP recording, camera capture, VR calibration
- ✅ Python-first implementation with C++ only where necessary
- ✅ Updated requirements.txt with ROS2 dependencies
- ✅ Created comprehensive technical documentation

## In Progress 🚧

[ ] Audio Recording

- [ ] Add microphone audio recording
- [ ] Continuously record mic audio during sessions
- [ ] Store audio data in the MCAP file with proper timestamps

## Future Enhancements 🔮

[ ] Performance Optimizations

- [ ] Investigate UDP communication to reduce robot latency
- [ ] Implement shared memory for local robot communication
- [ ] Add GPU acceleration for complex transformations
- [ ] Adaptive frequency based on system load

[ ] Data Collection Features

- [ ] Add automatic data quality metrics
- [ ] Implement trajectory smoothing/filtering options
- [ ] Add support for multiple robot configurations
- [ ] Create data collection dashboard/UI

[ ] VR Control Enhancements

- [ ] Add haptic feedback support
- [ ] Implement gesture recognition for special commands
- [ ] Add VR visualization of robot state
- [ ] Support for bimanual control (two robots)

## Notes 📝

### ROS2/MoveIt Migration Complete! 🎉

The new implementation is in `franka_vr_ros2/` with:

- **250Hz control rate** (vs 15-30Hz with Deoxys)
- **<10ms latency** target (vs 120ms)
- **Modular architecture** for easy strategy swapping
- **Full feature preservation** from original system

To use the new system:

```bash
cd franka_vr_ros2
python oculus_vr_server.py --robot-ip 192.168.1.1
```
