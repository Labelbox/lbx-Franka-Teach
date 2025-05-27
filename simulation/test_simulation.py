#!/usr/bin/env python3
"""
Test script for FR3 robot simulation with PyBullet
Demonstrates basic functionality without VR controller
"""

import sys
import os
# Add the parent directory to the path so we can import the simulation modules
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import time
from simulation.fr3_robot_model import FR3RobotModel
from simulation.fr3_pybullet_visualizer import FR3PyBulletVisualizer
from simulation.fr3_sim_controller import FR3SimController


def test_forward_kinematics():
    """Test forward kinematics calculation"""
    print("🧪 Testing Forward Kinematics...")
    
    robot = FR3RobotModel()
    
    # Test with home position
    pos, quat = robot.forward_kinematics(robot.rest_pose)
    print(f"   Home position: {pos}")
    print(f"   Home quaternion: {quat}")
    
    # Test with zero angles
    zero_angles = np.zeros(7)
    pos, quat = robot.forward_kinematics(zero_angles)
    print(f"   Zero angles position: {pos}")
    print(f"   Zero angles quaternion: {quat}")
    
    print("✅ Forward kinematics test complete\n")


def test_pybullet_visualization():
    """Test PyBullet robot visualization"""
    print("🧪 Testing PyBullet Visualization...")
    
    robot = FR3RobotModel()
    viz = FR3PyBulletVisualizer(robot)
    
    try:
        # Show robot in different poses
        poses = [
            robot.rest_pose,
            np.zeros(7),
            np.array([0.5, 0.5, 0.5, -1.5, 0.5, 1.5, 0.5]),
            np.array([-0.5, -0.5, -0.5, -2.0, -0.5, 2.0, -0.5]),
        ]
        
        print("   Showing robot in different poses...")
        for i, pose in enumerate(poses):
            print(f"   Pose {i+1}/4")
            viz.update_robot_pose(pose, gripper_state=0.0 if i < 2 else 1.0)
            time.sleep(1.5)
        
        print("✅ PyBullet visualization test complete\n")
    finally:
        # Always close the visualizer
        viz.close()


def test_trajectory_visualization():
    """Test trajectory visualization in PyBullet"""
    print("🧪 Testing Trajectory Visualization...")
    
    robot = FR3RobotModel()
    viz = FR3PyBulletVisualizer(robot)
    
    try:
        # Create a smooth trajectory
        t = np.linspace(0, 2*np.pi, 100)
        
        print("   Animating smooth trajectory...")
        for i in range(len(t)):
            # Create sinusoidal joint motion
            joints = robot.rest_pose.copy()
            joints[0] += 0.5 * np.sin(t[i])
            joints[1] += 0.3 * np.sin(2*t[i])
            joints[3] += 0.4 * np.sin(t[i] + np.pi/4)
            joints[5] += 0.3 * np.sin(2*t[i] + np.pi/2)
            
            # Ensure joint limits
            joints = robot.clip_joint_angles(joints)
            
            # Update visualization
            viz.update_robot_pose(joints, show_trajectory=True)
            time.sleep(0.03)  # ~30 FPS
        
        print("✅ Trajectory visualization test complete\n")
    finally:
        # Always close the visualizer
        viz.close()


def test_sim_controller():
    """Test simulated controller with PyBullet"""
    print("🧪 Testing Simulated Controller with PyBullet...")
    
    controller = FR3SimController(visualize=True)
    controller.start()
    
    try:
        print("   Moving to different positions...")
        
        # Test position 1
        target_pos = np.array([0.5, 0.1, 0.4])
        target_quat = np.array([0, 0, 0, 1])  # Identity quaternion
        controller.set_target_pose(target_pos, target_quat, 0.0)
        print(f"   Target 1: pos={target_pos}")
        time.sleep(3)
        
        # Test position 2 with rotation
        target_pos = np.array([0.4, -0.2, 0.5])
        target_quat = np.array([0, 0, 0.7071, 0.7071])  # 90 deg rotation around Z
        controller.set_target_pose(target_pos, target_quat, 1.0)  # Close gripper
        print(f"   Target 2: pos={target_pos}, gripper closed")
        time.sleep(3)
        
        # Test position 3
        target_pos = np.array([0.6, 0.0, 0.3])
        target_quat = np.array([0, 0, 0, 1])
        controller.set_target_pose(target_pos, target_quat, 0.0)  # Open gripper
        print(f"   Target 3: pos={target_pos}, gripper open")
        time.sleep(3)
        
        # Return to home
        print("   Returning to home...")
        controller.reset_to_home()
        time.sleep(3)
        
        print("✅ Simulated controller test complete\n")
    finally:
        # Always stop the controller
        controller.stop()


def test_all_in_one_window():
    """Run all visualization tests in a single PyBullet window"""
    print("\n🤖 Running All Tests in Single Window\n")
    
    robot = FR3RobotModel()
    viz = FR3PyBulletVisualizer(robot)
    
    try:
        # Test 1: Show different poses
        print("📍 Test 1: Different poses")
        poses = [
            robot.rest_pose,
            np.zeros(7),
            np.array([0.5, 0.5, 0.5, -1.5, 0.5, 1.5, 0.5]),
        ]
        
        for i, pose in enumerate(poses):
            print(f"   Pose {i+1}/3")
            viz.update_robot_pose(pose, gripper_state=0.0 if i < 2 else 1.0)
            time.sleep(1.0)
        
        # Test 2: Trajectory
        print("\n📍 Test 2: Smooth trajectory")
        viz.clear_trajectory()
        t = np.linspace(0, 2*np.pi, 50)
        
        for i in range(len(t)):
            joints = robot.rest_pose.copy()
            joints[0] += 0.3 * np.sin(t[i])
            joints[1] += 0.2 * np.sin(2*t[i])
            viz.update_robot_pose(joints, show_trajectory=True)
            time.sleep(0.05)
        
        print("\n✅ All visualization tests complete!")
        time.sleep(2)
        
    finally:
        viz.close()


def main():
    """Run all tests"""
    print("\n🤖 FR3 Robot Simulation Test Suite (PyBullet)\n")
    
    # Ask user which mode to run
    print("Choose test mode:")
    print("1. Run all tests separately (multiple windows)")
    print("2. Run visualization tests in single window")
    print("3. Run only kinematics test (no visualization)")
    
    choice = input("\nEnter choice (1/2/3) [default=2]: ").strip() or "2"
    
    if choice == "1":
        # Run all tests separately
        test_forward_kinematics()
        test_pybullet_visualization()
        test_trajectory_visualization()
        test_sim_controller()
    elif choice == "2":
        # Run kinematics test first
        test_forward_kinematics()
        # Then run all visualization tests in one window
        test_all_in_one_window()
    elif choice == "3":
        # Only kinematics
        test_forward_kinematics()
    else:
        print("Invalid choice, running default option 2")
        test_forward_kinematics()
        test_all_in_one_window()
    
    print("\n✅ All tests complete!")


if __name__ == "__main__":
    main() 