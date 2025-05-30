#!/bin/bash
# Setup script for Franka ROS 2 with necessary fixes

set -e  # Exit on error

echo "Setting up Franka ROS 2 workspace..."

# Check if workspace already exists
if [ -d "$HOME/franka_ros2_ws" ]; then
    echo "Franka ROS 2 workspace already exists at ~/franka_ros2_ws"
    read -p "Do you want to update it? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Skipping Franka ROS 2 setup"
        exit 0
    fi
else
    # Create workspace
    mkdir -p ~/franka_ros2_ws/src
fi

cd ~/franka_ros2_ws

# Clone or update franka_ros2
if [ -d "src/franka_ros2" ]; then
    echo "Updating franka_ros2..."
    cd src
    git pull
    cd ..
else
    echo "Cloning franka_ros2..."
    git clone https://github.com/frankaemika/franka_ros2.git src
fi

# Import dependencies
echo "Importing dependencies..."
vcs import src < src/franka.repos --recursive --skip-existing

# Install ROS dependencies
echo "Installing ROS dependencies..."
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -y

# Apply the version parameter fix
echo "Applying version parameter fix..."
XACRO_FILE="src/franka_description/robots/common/franka_arm.ros2_control.xacro"
if [ -f "$XACRO_FILE" ]; then
    # Check if version parameter already exists
    if ! grep -q '<param name="version">' "$XACRO_FILE"; then
        echo "Adding version parameter to URDF..."
        # Add version parameter after arm_prefix parameter
        sed -i '/<param name="prefix">\${arm_prefix}<\/param>/a\          <param name="version">0.1.0</param>' "$XACRO_FILE"
        echo "Version parameter added successfully"
    else
        echo "Version parameter already exists"
    fi
else
    echo "Warning: Could not find $XACRO_FILE"
fi

# Build the workspace
echo "Building Franka ROS 2 workspace..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-skip franka_ign_ros2_control franka_gazebo

echo "Setup complete! Don't forget to source the workspace:"
echo "source ~/franka_ros2_ws/install/setup.bash" 