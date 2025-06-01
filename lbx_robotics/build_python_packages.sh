#!/bin/bash
# Workaround script to build Python packages that have --editable issues

echo "Building Python packages with workaround for --editable issue..."
echo ""

# Source ROS2 if not already sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Sourcing ROS2 Humble..."
    source /opt/ros/humble/setup.bash
fi

# First, try building without symlink-install
echo "Step 1: Building without symlink-install (this should work)..."
colcon build --packages-select lbx_input_oculus lbx_vision_camera lbx_launch

if [ $? -eq 0 ]; then
    echo "✓ Build successful without symlink-install"
    
    # Now try with symlink-install
    echo ""
    echo "Step 2: Attempting build with symlink-install..."
    colcon build --symlink-install --packages-select lbx_input_oculus lbx_vision_camera lbx_launch
    
    if [ $? -eq 0 ]; then
        echo "✓ Build successful with symlink-install"
    else
        echo "⚠️  Symlink install failed, but packages are built and usable"
        echo "You can still use the packages, just without editable installs"
    fi
else
    echo "❌ Build failed. Running diagnostic..."
    echo ""
    
    # Check Python syntax
    echo "Checking Python syntax in packages..."
    for pkg in lbx_input_oculus lbx_vision_camera lbx_launch; do
        echo "Checking $pkg..."
        find src/$pkg -name "*.py" -exec python3 -m py_compile {} \; 2>&1 | grep -E "SyntaxError|Error"
    done
    
    echo ""
    echo "Try running ./fix_python_editable.sh for a more comprehensive fix"
fi

echo ""
echo "To use the packages after building:"
echo "  source install/setup.bash"
echo "  ros2 run <package_name> <node_name>" 