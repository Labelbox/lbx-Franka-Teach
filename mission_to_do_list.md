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

[x] Unified Launch System

- ✅ Created unified_launch.sh combining all launch scripts
- ✅ Made build optional with --build and --clean-build flags
- ✅ Integrated comprehensive process killing from robust Franka scripts
- ✅ Combined arguments from teleoperation, build, and control scripts
- ✅ Added graceful shutdown and emergency stop capabilities
- ✅ Created documentation in unified_launch_guide.md
- ✅ Moved script to lbx_robotics directory for better organization
- ✅ Fixed clean build to be opt-in only (preserves incremental builds)
- ✅ Moved documentation to lbx_robotics/docs/unified_launch_guide.md
- ✅ Updated all paths and references to new locations

## In Progress 🚧

[ ] Migrate to MoveIt/ROS2 for Low-Latency Control

- [ ] Replace Deoxys with ROS2-based control system (target: <25ms latency vs current 120ms)
- [ ] Implement control pipeline: VR System → ROS tf2 → Target Pose Generation → IK Solver → MoveIt! → ROS 2 Control → FCI → FR3
- [ ] Set up ROS2 environment and MoveIt configuration for Franka FR3
- [ ] Implement high-frequency controller using ROS 2 Control
- [ ] Create coordinate transformation system using tf2
- [ ] Integrate inverse kinematics solver
- [ ] Configure MoveIt for motion planning and collision avoidance
- [ ] Benchmark and optimize latency at each pipeline stage

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
