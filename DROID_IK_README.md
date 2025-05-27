# DROID MuJoCo-based IK Solver Integration

This document describes how to use DROID's MuJoCo-based inverse kinematics solver with the Oculus VR Server, bypassing Deoxys' internal IK.

## Overview

By default, the Oculus VR Server sends cartesian position and quaternion targets to Deoxys, which then uses its internal IK solver to compute joint positions. With the `--use-droid-ik` flag, you can instead use DROID's MuJoCo-based IK solver, which:

- Uses physics simulation for more robust IK solutions
- Handles singularities better than simple Jacobian methods
- Provides the exact same IK behavior as DROID's teleoperation system
- Sends joint position commands directly to the robot

## Architecture Comparison

### Default (Deoxys IK):
```
VR Controller → Cartesian Velocities → Position/Quaternion Targets → Deoxys IK → Joint Commands → Robot
```

### With DROID IK:
```
VR Controller → Cartesian Velocities → DROID MuJoCo IK → Joint Velocities → Joint Targets → Robot
```

## Installation

1. First, ensure you have the DROID repository cloned:
```bash
cd /path/to/your/workspace
git clone https://github.com/your-org/lbx-droid-franka-robots.git
```

2. Install the DROID package:
```bash
pip install -e lbx-droid-franka-robots/
```

3. Test the installation:
```bash
python test_droid_ik.py
```

## Usage

To use DROID's IK solver, simply add the `--use-droid-ik` flag:

```bash
# Standard mode (Deoxys IK)
python oculus_vr_server.py

# With DROID IK solver
python oculus_vr_server.py --use-droid-ik

# Debug mode with DROID IK
python oculus_vr_server.py --debug --use-droid-ik
```

## Technical Details

### DROID IK Solver Parameters

The DROID IK solver uses these exact parameters from the original implementation:

- **Max joint delta**: 0.2 rad/cycle (per joint)
- **Max linear delta**: 0.075 m/cycle
- **Max rotation delta**: 0.15 rad/cycle
- **Control frequency**: 15 Hz
- **Nullspace gain**: 0.025
- **Regularization weight**: 0.01

### Joint Control Implementation

When using DROID IK, the system:

1. Converts VR cartesian velocities to joint velocities using MuJoCo physics
2. Applies velocity-to-delta conversion with proper scaling
3. Sends joint position targets to Deoxys using `JOINT_POSITION` controller
4. Returns joint state feedback for closed-loop control

### Modified Components

1. **oculus_vr_server.py**:
   - Added `--use-droid-ik` flag
   - Imports and initializes DROID's `RobotIKSolver`
   - Converts cartesian velocities to joint commands
   - Tracks joint positions and velocities

2. **frankateach/messages.py**:
   - Added `joint_positions` and `joint_velocities` to `FrankaState`
   - Added `joint_positions` and `control_mode` to `FrankaAction`

3. **frankateach/franka_server.py**:
   - Added `joint_move()` method for joint control
   - Handles `JOINT_POSITION` control mode
   - Returns joint state in feedback

4. **frankateach/configs/joint-position-controller.yml**:
   - Configuration for Deoxys joint position controller

## Benefits

1. **Better Singularity Handling**: MuJoCo's physics-based IK is more robust near singularities
2. **Exact DROID Behavior**: Matches the IK behavior of DROID's original implementation
3. **Joint-Level Control**: Direct control over joint positions can be safer in some scenarios
4. **Debugging**: Easier to debug joint-level issues

## Limitations

1. **Performance**: MuJoCo IK solver may add slight computational overhead
2. **Dependencies**: Requires DROID package and its dependencies (MuJoCo, dm_control)
3. **Calibration**: Joint control may require different calibration than cartesian control

## Troubleshooting

### Import Error
If you see "DROID IK solver not available", ensure:
- The lbx-droid-franka-robots package is installed
- All dependencies (mujoco, dm_control, dm_robotics) are installed

### Joint Limits
If the robot hits joint limits, the IK solver will try to find alternative solutions using nullspace optimization.

### Performance Issues
If control feels sluggish:
- Ensure you're running at 15Hz control rate
- Check CPU usage - MuJoCo simulation can be CPU intensive
- Consider using cartesian control for less computational overhead

## Example Output

When running with `--use-droid-ik --debug`, you'll see:

```
🔧 Initializing DROID MuJoCo-based IK solver...
✅ DROID IK solver initialized
   Will send joint commands instead of cartesian commands

📊 Debug Data [0042]:
   🟢 TELEOPERATION: ACTIVE (Joint Control)
   Cartesian Vel: [0.123, -0.045, 0.012, 0.023, -0.015, 0.008]
   Joint Vel: [0.045, -0.023, 0.012, -0.034, 0.018, 0.025, -0.011]
   Joint Delta: [0.003, -0.002, 0.001, -0.003, 0.001, 0.002, -0.001]
   Target Joints: [0.092, -0.200, -0.019, -2.476, -0.012, 2.306, 0.847]
   Gripper: 🟢 OPEN (trigger: 0.000)
```

## Future Improvements

1. **Hybrid Mode**: Switch between IK solvers based on robot configuration
2. **Custom Constraints**: Add task-specific joint constraints
3. **Performance Optimization**: GPU acceleration for MuJoCo simulation
4. **Visualization**: Real-time visualization of IK solutions 