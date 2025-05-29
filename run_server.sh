#!/bin/bash

# Oculus VR Server with MoveIt Integration
# Automatically builds, configures, and starts the VR teleoperation system
#
# Usage:
#   ./run_server.sh                    # Real robot (with readiness check)
#   ./run_server.sh --simulation       # Force simulation mode
#   ./run_server.sh --hot-reload       # Hot reload development mode
#   ./run_server.sh --enable-cameras   # Enable camera recording
#   ./run_server.sh --debug            # Debug mode (no robot control)
#
# The script will:
# 1. Setup ROS2 environment and build packages
# 2. Check robot readiness (for real hardware)
# 3. Start MoveIt system (with fallback to simulation)
# 4. Launch VR server with appropriate settings

# Kill any existing oculus server processes
echo "🛑 Stopping any existing Oculus servers..."
./kill_oculus_server.sh

# Kill any existing ROS2/MoveIt processes
echo "🛑 Stopping any existing ROS2/MoveIt processes..."
pkill -f "ros2 launch"
pkill -f "moveit"
pkill -f "rviz"

# Wait a moment for processes to fully terminate
sleep 2

# Function to check if a process is running
is_process_running() {
    pgrep -f "$1" > /dev/null
}

# Function to check robot connectivity and readiness
check_robot_readiness() {
    echo "🔍 Checking robot readiness..."
    
    # Check if robot IP is reachable
    if ping -c 1 -W 3 192.168.1.59 >/dev/null 2>&1; then
        echo "✅ Robot IP 192.168.1.59 is reachable"
    else
        echo "❌ Robot IP 192.168.1.59 is not reachable!"
        echo "   Please check:"
        echo "   - Robot is powered on"
        echo "   - Network connection is working"
        echo "   - Robot IP is correct (192.168.1.59)"
        return 1
    fi
    
    # Check if robot is in the correct mode
    echo "⚠️  IMPORTANT: Make sure the robot is ready:"
    echo "   1. Robot should be in 'Programming Mode' (not Execution Mode)"
    echo "   2. E-stop should be released (if any)"
    echo "   3. Robot should show 'External Control' capability"
    echo "   4. No safety violations or error states"
    
    read -p "   Is the robot ready for external control? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "❌ Please prepare the robot first, then try again"
        return 1
    fi
    
    return 0
}

# Parse arguments early to check for simulation mode
USE_SIMULATION=false
for arg in "$@"; do
    case $arg in
        --simulation|--sim)
            USE_SIMULATION=true
            ;;
    esac
done

# Check and setup ROS2 environment
echo "🔧 Setting up ROS2 environment..."

# Source ROS2 base
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ Sourced ROS2 Humble"
else
    echo "❌ ROS2 Humble not found! Please install ROS2 Humble first."
    exit 1
fi

# Source Franka ROS2 workspace if available
if [ -f "$HOME/franka_ros2_ws/install/setup.bash" ]; then
    source $HOME/franka_ros2_ws/install/setup.bash
    echo "✅ Sourced Franka ROS2 workspace"
else
    echo "⚠️  Franka ROS2 workspace not found at ~/franka_ros2_ws/"
    echo "   Setting up Franka ROS2 workspace..."
    
    # Check if setup script exists
    if [ -f "ros2_moveit_franka/scripts/setup_franka_ros2.sh" ]; then
        echo "🔧 Running Franka ROS2 setup script..."
        cd ros2_moveit_franka
        ./scripts/setup_franka_ros2.sh
        cd ..
        source $HOME/franka_ros2_ws/install/setup.bash
        echo "✅ Franka ROS2 workspace setup complete"
    else
        echo "❌ Please install Franka ROS2 workspace first:"
        echo "   cd ros2_moveit_franka && ./scripts/setup_franka_ros2.sh"
        exit 1
    fi
fi

# Build ros2_moveit_franka package if needed
echo "🔨 Building ros2_moveit_franka package..."
if [ -d "ros2_moveit_franka" ]; then
    cd ros2_moveit_franka
    
    # Install dependencies
    echo "📦 Installing dependencies..."
    rosdep install --from-paths . --ignore-src --rosdistro humble -y
    
    # Build the package
    echo "🔨 Building package..."
    colcon build --packages-select ros2_moveit_franka --symlink-install
    
    if [ $? -eq 0 ]; then
        echo "✅ Package built successfully"
        source install/setup.bash
        echo "✅ Package sourced successfully"
    else
        echo "❌ Package build failed!"
        exit 1
    fi
    
    cd ..
else
    echo "⚠️  ros2_moveit_franka directory not found, skipping package build"
fi

# Start MoveIt system
echo "🤖 Starting MoveIt system for FR3 robot..."

# Check if we should use simulation or real robot
if [ "$USE_SIMULATION" = true ]; then
    echo "🎮 Using simulation mode (fake hardware)"
    ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=192.168.1.59 use_fake_hardware:=true &
    MOVEIT_PID=$!
else
    # Check robot readiness for real hardware
    if check_robot_readiness; then
        echo "🤖 Starting MoveIt with real robot hardware..."
        ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=192.168.1.59 use_fake_hardware:=false &
        MOVEIT_PID=$!
    else
        echo "⚠️  Robot not ready. Do you want to use simulation mode instead?"
        read -p "   Use simulation? (Y/n): " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Nn]$ ]]; then
            echo "❌ Exiting. Please prepare the robot and try again."
            exit 1
        else
            echo "🎮 Falling back to simulation mode..."
            ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=192.168.1.59 use_fake_hardware:=true &
            MOVEIT_PID=$!
            USE_SIMULATION=true
        fi
    fi
fi

# Wait for MoveIt to start
echo "⏳ Waiting for MoveIt to initialize..."
sleep 10  # Increased wait time for full initialization

# Check if MoveIt is still running and handle crashes
startup_attempts=0
max_attempts=3

while [ $startup_attempts -lt $max_attempts ]; do
    if kill -0 $MOVEIT_PID 2>/dev/null; then
        echo "✅ MoveIt system started successfully"
        break
    else
        startup_attempts=$((startup_attempts + 1))
        echo "❌ MoveIt startup failed (attempt $startup_attempts/$max_attempts)"
        
        if [ $startup_attempts -lt $max_attempts ]; then
            if [ "$USE_SIMULATION" = false ]; then
                echo "⚠️  This is often due to robot communication issues."
                echo "   Do you want to try simulation mode instead?"
                read -p "   Use simulation? (Y/n): " -n 1 -r
                echo
                if [[ ! $REPLY =~ ^[Nn]$ ]]; then
                    echo "🎮 Retrying with simulation mode..."
                    ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=192.168.1.59 use_fake_hardware:=true &
                    MOVEIT_PID=$!
                    USE_SIMULATION=true
                    sleep 10
                    continue
                fi
            fi
            
            echo "🔄 Retrying MoveIt startup..."
            sleep 5
        else
            echo "❌ MoveIt failed to start after $max_attempts attempts!"
            echo ""
            echo "🔧 TROUBLESHOOTING STEPS:"
            echo "1. Check robot status:"
            echo "   - Robot must be in 'Programming Mode'"
            echo "   - E-stop must be released"
            echo "   - Check robot web interface at http://192.168.1.59"
            echo ""
            echo "2. Try manual launch:"
            echo "   ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=192.168.1.59 use_fake_hardware:=true"
            echo ""
            echo "3. Common issues:"
            echo "   - 'communication_constraints_violation': Robot not in correct mode"
            echo "   - 'Connection refused': Robot not reachable or FCI disabled"
            echo "   - 'Motion aborted by reflex': Safety system active"
            exit 1
        fi
    fi
done

# Parse command line arguments for camera options
SKIP_CAMERA_CHECK=true  # Default to skipping camera check
CAMERA_CONFIG=""
CAMERA_ARGS=""

for arg in "$@"; do
    case $arg in
        --enable-cameras)
            SKIP_CAMERA_CHECK=false
            ;;
        --check-cameras)
            SKIP_CAMERA_CHECK=false
            ;;
        --camera-config)
            SKIP_CAMERA_CHECK=false
            ;;
        --camera-config=*)
            SKIP_CAMERA_CHECK=false
            ;;
    esac
done

# Check for camera configs only if explicitly requested
if [ "$SKIP_CAMERA_CHECK" = false ]; then
    echo "📷 Checking for camera configurations..."
    
    # Check for Intel RealSense config
    if [ -f "configs/cameras_intel.yaml" ]; then
        CAMERA_CONFIG="configs/cameras_intel.yaml"
        echo "📷 Found Intel RealSense camera configuration"
    # Check for ZED config
    elif [ -f "configs/cameras_zed.yaml" ]; then
        CAMERA_CONFIG="configs/cameras_zed.yaml"
        echo "📷 Found ZED camera configuration"
    fi
    
    # Set camera arguments if config found
    if [ -n "$CAMERA_CONFIG" ]; then
        CAMERA_ARGS="--enable-cameras --camera-config $CAMERA_CONFIG"
    fi
else
    echo "📷 Cameras disabled by default"
    echo "   Use --enable-cameras or --check-cameras to enable camera support"
    # Explicitly clear camera variables to ensure no cameras
    CAMERA_CONFIG=""
    CAMERA_ARGS=""
fi

# Check if hot reload is requested
HOT_RELOAD=false
for arg in "$@"; do
    if [ "$arg" = "--hot-reload" ] || [ "$arg" = "-h" ]; then
        HOT_RELOAD=true
        break
    fi
done

# Cleanup function
cleanup() {
    echo "🛑 Cleaning up..."
    if [ -n "$MOVEIT_PID" ] && kill -0 $MOVEIT_PID 2>/dev/null; then
        echo "   Stopping MoveIt system..."
        kill $MOVEIT_PID
        wait $MOVEIT_PID 2>/dev/null
    fi
    pkill -f "ros2 launch"
    pkill -f "moveit"
    exit 0
}

# Set trap for cleanup on exit
trap cleanup EXIT INT TERM

if [ "$HOT_RELOAD" = true ]; then
    # Run in hot reload mode - only use cameras if explicitly requested
    if [ "$SKIP_CAMERA_CHECK" = false ] && [ -n "$CAMERA_CONFIG" ]; then
        echo "🔥 Starting async Oculus VR server in HOT RELOAD mode with cameras..."
        if [ "$USE_SIMULATION" = true ]; then
            python3 oculus_vr_server.py --hot-reload --performance --simulation $CAMERA_ARGS "$@"
        else
            python3 oculus_vr_server.py --hot-reload --performance $CAMERA_ARGS "$@"
        fi
    else
        echo "🔥 Starting async Oculus VR server in HOT RELOAD mode (no cameras)..."
        if [ "$USE_SIMULATION" = true ]; then
            python3 oculus_vr_server.py --hot-reload --performance --simulation "$@"
        else
            python3 oculus_vr_server.py --hot-reload --performance "$@"
        fi
    fi
else
    # Normal mode - only use cameras if explicitly requested
    if [ "$SKIP_CAMERA_CHECK" = false ] && [ -n "$CAMERA_CONFIG" ]; then
        echo "🚀 Starting async Oculus VR server with performance mode, data verification, and cameras..."
        if [ "$USE_SIMULATION" = true ]; then
            python3 oculus_vr_server.py --performance --verify-data --simulation $CAMERA_ARGS "$@"
        else
            python3 oculus_vr_server.py --performance --verify-data $CAMERA_ARGS "$@"
        fi
    else
        # Run normally without cameras
        if [ "$USE_SIMULATION" = true ]; then
            echo "🚀 Starting async Oculus VR server with performance mode, data verification (simulation mode)..."
            echo "🎮 Connecting to simulated MoveIt system..."
            python3 oculus_vr_server.py --performance --verify-data --simulation "$@"
        else
            echo "🚀 Starting async Oculus VR server with performance mode, data verification (no cameras)..."
            echo "🤖 Connecting to MoveIt system for robot control..."
            python3 oculus_vr_server.py --performance --verify-data "$@"
        fi
    fi
fi 