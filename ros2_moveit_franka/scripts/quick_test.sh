#!/bin/bash
# Quick test script for ros2_moveit_franka package

set -e  # Exit on any error

echo "🤖 ROS 2 MoveIt Franka FR3 Quick Test Script"
echo "============================================="

# Check if we're in a ROS 2 environment
if [[ -z "$ROS_DISTRO" ]]; then
    echo "❌ Error: ROS 2 environment not sourced!"
    echo "   Please run: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "✅ ROS 2 $ROS_DISTRO environment detected"

# Check if franka packages are available
if ! ros2 pkg list | grep -q "franka_fr3_moveit_config"; then
    echo "❌ Error: Franka ROS 2 packages not found!"
    echo "   Please install franka_ros2 following the README instructions"
    exit 1
fi

echo "✅ Franka ROS 2 packages found"

# Build the package
echo "🔨 Building ros2_moveit_franka package..."
cd ..  # Go up to workspace root

if ! colcon build --packages-select ros2_moveit_franka; then
    echo "❌ Build failed!"
    exit 1
fi

echo "✅ Package built successfully"

# Source the workspace
source install/setup.bash

echo "📋 Package information:"
echo "   Package: $(ros2 pkg prefix ros2_moveit_franka)"
echo "   Executables:"
ros2 pkg executables ros2_moveit_franka

echo ""
echo "🚀 Ready to run! Use one of these commands:"
echo ""
echo "   # Simulation mode (safe testing):"
echo "   ros2 launch ros2_moveit_franka franka_demo.launch.py use_fake_hardware:=true"
echo ""
echo "   # Real robot mode (ensure robot is ready!):"
echo "   ros2 launch ros2_moveit_franka franka_demo.launch.py robot_ip:=192.168.1.59"
echo ""
echo "   # Manual execution:"
echo "   ros2 run ros2_moveit_franka simple_arm_control"
echo ""

read -p "Do you want to run the simulation test now? (y/N): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "🎮 Starting simulation test..."
    ros2 launch ros2_moveit_franka franka_demo.launch.py use_fake_hardware:=true
fi 