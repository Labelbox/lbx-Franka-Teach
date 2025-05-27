# FR3 Robot Simulation Module

This module provides a complete simulation environment for the Franka FR3 robot arm, allowing you to test and develop teleoperation control without requiring physical hardware.

## Features

- **Accurate FR3 kinematics**: Based on official DH parameters and specifications
- **PyBullet physics simulation**: Realistic 3D visualization with physics engine
- **Socket interface compatibility**: Mimics the real robot server interface exactly
- **VR teleoperation support**: Fully integrated with the Oculus VR server
- **Joint limits and workspace bounds**: Enforces realistic robot constraints
- **Gripper simulation**: Simulates gripper open/close states

## Components

### 1. `fr3_robot_model.py`

The core robot model implementing:

- Forward kinematics using DH parameters
- Joint limit checking and enforcement
- Jacobian calculation for inverse kinematics
- FR3-specific parameters (joint limits, torques, etc.)

### 2. `fr3_pybullet_visualizer.py`

PyBullet-based 3D visualization system featuring:

- Real-time robot pose rendering with physics
- End-effector trajectory tracking
- Workspace boundary visualization
- Target marker display
- Camera image capture support

### 3. `fr3_sim_controller.py`

Simulated robot controller providing:

- Position and orientation control
- Simplified inverse kinematics
- Smooth motion with PD control
- Thread-safe state management
- Trajectory recording capabilities

### 4. `fr3_sim_server.py`

Network server that:

- Provides the same ZMQ socket interface as the real robot
- Handles FrankaAction commands
- Publishes FrankaState messages
- Supports all standard robot operations (reset, move, gripper control)

## Usage

### Running with VR Teleoperation

To use the simulated robot with VR control:

```bash
# Start the Oculus VR server in simulation mode
python oculus_vr_server.py --simulation

# Optional: disable visualization for headless operation
python oculus_vr_server.py --simulation --debug
```

### Standalone Simulation Server

To run just the simulation server:

```bash
# With visualization
python -m simulation.fr3_sim_server

# Without visualization (headless)
python -m simulation.fr3_sim_server --no-viz
```

### Testing the Simulation

Run the test suite to verify functionality:

```bash
cd simulation
python test_simulation.py
```

This will test:

- Forward kinematics calculations
- PyBullet 3D visualization
- Trajectory visualization
- Controller functionality

### Programmatic Usage

```python
from simulation.fr3_robot_model import FR3RobotModel
from simulation.fr3_pybullet_visualizer import FR3PyBulletVisualizer
from simulation.fr3_sim_controller import FR3SimController

# Create robot model
robot = FR3RobotModel()

# Calculate forward kinematics
pos, quat = robot.forward_kinematics(robot.rest_pose)

# Create PyBullet visualizer
viz = FR3PyBulletVisualizer(robot)
viz.update_robot_pose(robot.rest_pose)

# Create controller
controller = FR3SimController(visualize=True)
controller.start()
controller.set_target_pose(target_pos, target_quat, gripper_state)
```

## Robot Specifications

The simulation uses accurate FR3 specifications:

- **Degrees of Freedom**: 7
- **Joint Limits**: Enforced based on FR3 documentation
- **Workspace**:
  - X: 0.2 to 0.75 m
  - Y: -0.4 to 0.4 m
  - Z: 0.05 to 0.7 m
- **DH Parameters**: Based on modified DH convention
- **Home Position**: Configured for optimal workspace reach

## PyBullet Visualization Features

When visualization is enabled:

- Real-time 3D rendering with physics engine
- Robot model loaded from URDF (Panda model used as FR3 proxy)
- Green trajectory lines show end-effector path
- Gray wireframe shows workspace boundaries
- Red sphere indicates target position
- Gripper fingers animate open/close
- Camera viewpoint can be adjusted interactively

## Network Interface

The simulation server provides:

- **Control Port**: 8901 (REQ/REP for commands)
- **State Port**: 8900 (PUB/SUB for state updates)
- **Message Format**: Pickled FrankaAction/FrankaState objects
- **Update Rate**: 100Hz state publishing

## Dependencies

The simulation requires:

- `numpy`: Numerical computations
- `scipy`: Rotation mathematics
- `pybullet`: Physics simulation and visualization

Install with:

```bash
pip install numpy scipy pybullet
```

## Limitations

- Simplified inverse kinematics (uses Jacobian pseudo-inverse)
- No collision detection between links
- Gripper is binary (open/close) rather than continuous
- Uses Panda URDF as FR3 model (very similar kinematics)

## Future Enhancements

Potential improvements:

- Custom FR3 URDF model
- Full inverse kinematics solver
- Collision detection
- Force/torque simulation
- Multiple robot support
- Integration with camera simulation
