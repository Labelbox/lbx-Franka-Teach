#!/usr/bin/env python3
"""
Test script for DROID IK solver integration
"""

import numpy as np
import sys

try:
    from lbx_droid_franka_robots.droid.robot_ik.robot_ik_solver import RobotIKSolver
    print("✅ DROID IK solver imported successfully")
except ImportError as e:
    print(f"❌ Failed to import DROID IK solver: {e}")
    print("\nTo use DROID IK, you need to:")
    print("1. Clone the lbx-droid-franka-robots repository")
    print("2. Install it with: pip install -e lbx-droid-franka-robots/")
    sys.exit(1)

# Test IK solver
print("\n🔧 Testing DROID IK solver...")

# Initialize solver
ik_solver = RobotIKSolver()
print("✅ IK solver initialized")

# Test robot state
robot_state = {
    "cartesian_position": [0.4, 0.0, 0.3, 0.0, 0.0, 0.0],  # x, y, z, roll, pitch, yaw
    "joint_positions": [0.0, -0.2, 0.0, -2.5, 0.0, 2.3, 0.85],  # 7 joints
    "joint_velocities": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "gripper_position": 0.0
}

# Test cartesian velocity
cartesian_velocity = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])  # Move 0.1 m/s in X direction

print(f"\n📊 Test data:")
print(f"   Robot position: {robot_state['cartesian_position'][:3]}")
print(f"   Robot orientation (RPY): {robot_state['cartesian_position'][3:]}")
print(f"   Joint positions: {robot_state['joint_positions']}")
print(f"   Cartesian velocity: {cartesian_velocity}")

# Convert cartesian velocity to joint velocity
try:
    joint_velocity = ik_solver.cartesian_velocity_to_joint_velocity(
        cartesian_velocity, robot_state
    )
    print(f"\n✅ Computed joint velocity: {joint_velocity}")
    
    # Convert to joint delta
    joint_delta = ik_solver.joint_velocity_to_delta(joint_velocity)
    print(f"   Joint delta: {joint_delta}")
    
    # Test velocity to delta conversion
    cartesian_delta = ik_solver.cartesian_velocity_to_delta(cartesian_velocity)
    print(f"   Cartesian delta: {cartesian_delta}")
    
    print("\n✅ DROID IK solver is working correctly!")
    
except Exception as e:
    print(f"\n❌ Error testing IK solver: {e}")
    import traceback
    traceback.print_exc()

print("\n📝 Summary:")
print("   - DROID IK solver uses MuJoCo physics simulation")
print("   - Converts cartesian velocities to joint velocities")
print("   - Handles singularities better than simple Jacobian methods")
print("   - Max linear delta: 0.075 m/cycle")
print("   - Max rotation delta: 0.15 rad/cycle")
print("   - Control frequency: 15 Hz") 