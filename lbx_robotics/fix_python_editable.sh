#!/bin/bash
# Script to diagnose and fix the --editable option error in ROS2 Python packages

echo "=== Diagnosing Python Package Build Issues ==="
echo ""

# 1. Check colcon versions
echo "1. Checking colcon package versions:"
echo "-----------------------------------"
python3 -m pip list | grep -E "colcon-|setuptools" | sort
echo ""

# 2. Check for problematic versions
echo "2. Checking for known problematic versions:"
echo "-----------------------------------------"
COLCON_PYTHON_VERSION=$(python3 -m pip show colcon-python-setup-py 2>/dev/null | grep Version | awk '{print $2}')
if [ -z "$COLCON_PYTHON_VERSION" ]; then
    echo "❌ colcon-python-setup-py is not installed!"
else
    echo "colcon-python-setup-py version: $COLCON_PYTHON_VERSION"
    # Version 0.2.7 and below have issues with --editable
    if python3 -c "from packaging import version; exit(0 if version.parse('$COLCON_PYTHON_VERSION') <= version.parse('0.2.7') else 1)" 2>/dev/null; then
        echo "❌ This version has known issues with --editable option"
    else
        echo "✓ Version should support --editable"
    fi
fi
echo ""

# 3. Check setuptools version
echo "3. Checking setuptools version:"
echo "-----------------------------"
SETUPTOOLS_VERSION=$(python3 -m pip show setuptools | grep Version | awk '{print $2}')
echo "setuptools version: $SETUPTOOLS_VERSION"
# Setuptools 58.2.0+ is recommended
if python3 -c "from packaging import version; exit(0 if version.parse('$SETUPTOOLS_VERSION') >= version.parse('58.2.0') else 1)" 2>/dev/null; then
    echo "✓ setuptools version is sufficient"
else
    echo "⚠️  Consider upgrading setuptools to 58.2.0+"
fi
echo ""

# 4. Apply fixes
echo "4. Applying fixes:"
echo "-----------------"

# Fix 1: Upgrade all colcon packages
echo "Upgrading colcon packages..."
python3 -m pip install --user --upgrade \
    colcon-common-extensions \
    colcon-core \
    colcon-python-setup-py \
    colcon-ros \
    setuptools \
    wheel \
    packaging

# Fix 2: Clear any cached build artifacts
echo ""
echo "Clearing build artifacts..."
if [ -d "build" ] || [ -d "install" ] || [ -d "log" ]; then
    echo "Found existing build directories. Cleaning..."
    rm -rf build install log
    echo "✓ Build directories cleaned"
fi

# Fix 3: Environment check
echo ""
echo "5. Environment verification:"
echo "--------------------------"
# Check if we're in a ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠️  ROS2 environment not sourced. Run: source /opt/ros/humble/setup.bash"
else
    echo "✓ ROS2 $ROS_DISTRO environment is active"
fi

# Check PATH for colcon
if command -v colcon &> /dev/null; then
    echo "✓ colcon is in PATH: $(which colcon)"
else
    echo "❌ colcon not found in PATH!"
fi

echo ""
echo "=== SOLUTIONS ==="
echo ""
echo "If you're still getting --editable errors after running this script:"
echo ""
echo "1. Try building without --symlink-install first:"
echo "   colcon build --packages-select lbx_input_oculus lbx_vision_camera"
echo ""
echo "2. If that works, the issue is specific to editable installs. Try:"
echo "   colcon build --symlink-install --packages-select lbx_input_oculus lbx_vision_camera --cmake-args -DCMAKE_BUILD_TYPE=Release"
echo ""
echo "3. As a last resort, you can disable editable installs for specific packages by adding"
echo "   this to their setup.cfg (create the file if it doesn't exist):"
echo "   [develop]"
echo "   no-deps=1"
echo ""
echo "4. Check if there are any Python syntax errors in the packages:"
echo "   python3 -m py_compile src/lbx_input_oculus/lbx_input_oculus/*.py"
echo "   python3 -m py_compile src/lbx_vision_camera/lbx_vision_camera/*.py" 