#!/usr/bin/env python3
"""Test script to verify MoveIt environment"""

import sys
import os

print("=== Python Environment Test ===")
print(f"Python executable: {sys.executable}")
print(f"Python version: {sys.version}")
print()

print("Environment variables:")
for var in ['PYTHONPATH', 'LD_LIBRARY_PATH', 'ROS_DISTRO', 'ROS_VERSION']:
    value = os.environ.get(var, 'NOT SET')
    print(f"  {var}: {value}")
print()

print("Python path:")
for i, p in enumerate(sys.path):
    print(f"  {i}: {p}")
print()

print("Testing moveit_commander import:")
try:
    import moveit_commander
    print("✓ moveit_commander import successful")
    print(f"  Location: {moveit_commander.__file__}")
    
    # Test basic functionality
    print("Testing moveit_commander.roscpp_initialize...")
    moveit_commander.roscpp_initialize(sys.argv)
    print("✓ roscpp_initialize successful")
    
    print("Testing RobotCommander...")
    robot = moveit_commander.RobotCommander()
    print("✓ RobotCommander created successfully")
    
    moveit_commander.roscpp_shutdown()
    print("✓ All MoveIt tests passed")
    
except ImportError as e:
    print(f"✗ moveit_commander import failed: {e}")
except Exception as e:
    print(f"✗ MoveIt functionality test failed: {e}")

print("\nSearching for moveit_commander in likely locations:")
likely_paths = [
    "/opt/ros/humble/lib/python3.10/site-packages",
    "/opt/ros/humble/local/lib/python3.10/dist-packages", 
    "/home/labelbox/franka_ros2_ws/install/lib/python3.10/site-packages",
    "/home/labelbox/franka_ros2_ws/install/local/lib/python3.10/dist-packages",
]

for path in likely_paths:
    moveit_path = os.path.join(path, "moveit_commander")
    if os.path.exists(moveit_path):
        print(f"✓ Found moveit_commander at: {moveit_path}")
    else:
        print(f"✗ Not found at: {moveit_path}") 