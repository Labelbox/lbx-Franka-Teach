#!/bin/bash
# Unified LBX Robotics Launch Script
# Combines teleoperation, build management, and robust process handling

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Default parameters
ROBOT_IP="192.168.1.59"
USE_FAKE_HARDWARE="false"
ENABLE_RVIZ="true"
ENABLE_CAMERAS="false"
ENABLE_RECORDING="true"
USE_RIGHT_CONTROLLER="true"
VR_MODE="usb"
VR_IP=""
HOT_RELOAD="false"
LOG_LEVEL="INFO"
PERFORM_BUILD="false"
CLEAN_BUILD="false"

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"  # Script is now inside lbx_robotics

# Helper functions
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Help function
show_help() {
    echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
    echo -e "${CYAN}            Unified LBX Robotics Launch Script                  ${NC}"
    echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo -e "${BLUE}Build Options:${NC}"
    echo "  --build                   Perform colcon build"
    echo "  --clean-build             Clean workspace then build"
    echo ""
    echo -e "${BLUE}Robot Options:${NC}"
    echo "  --robot-ip <IP>           Robot IP address (default: $ROBOT_IP)"
    echo "  --fake-hardware           Use fake hardware for testing"
    echo "  --no-fake-hardware        Use real hardware (default)"
    echo ""
    echo -e "${BLUE}Visualization Options:${NC}"
    echo "  --rviz                    Enable RViz (default)"
    echo "  --no-rviz                 Disable RViz"
    echo ""
    echo -e "${BLUE}VR Control Options:${NC}"
    echo "  --left-controller         Use left VR controller"
    echo "  --right-controller        Use right VR controller (default)"
    echo "  --network-vr <IP>         Use network VR mode with specified IP"
    echo ""
    echo -e "${BLUE}Data & Sensors Options:${NC}"
    echo "  --cameras                 Enable camera system"
    echo "  --no-cameras              Disable cameras (default)"
    echo "  --recording               Enable data recording (default)"
    echo "  --no-recording            Disable data recording"
    echo ""
    echo -e "${BLUE}Development Options:${NC}"
    echo "  --hot-reload              Enable hot reload for VR server"
    echo "  --log-level <LEVEL>       Set log level (DEBUG, INFO, WARN, ERROR)"
    echo ""
    echo -e "${BLUE}System Control:${NC}"
    echo "  --shutdown                Gracefully shutdown running system"
    echo "  --emergency-stop          Emergency stop all processes"
    echo "  --help                    Show this help message"
    echo ""
    echo -e "${CYAN}Examples:${NC}"
    echo "  $0 --build                           # Build then run with defaults"
    echo "  $0 --clean-build --fake-hardware    # Clean build for testing"
    echo "  $0 --cameras --no-recording          # Run with cameras, no recording"
    echo "  $0 --network-vr 192.168.1.50        # Network VR mode"
    echo ""
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --build)
            PERFORM_BUILD="true"
            shift
            ;;
        --clean-build)
            PERFORM_BUILD="true"
            CLEAN_BUILD="true"
            shift
            ;;
        --robot-ip)
            ROBOT_IP="$2"
            shift 2
            ;;
        --fake-hardware)
            USE_FAKE_HARDWARE="true"
            shift
            ;;
        --no-fake-hardware)
            USE_FAKE_HARDWARE="false"
            shift
            ;;
        --rviz)
            ENABLE_RVIZ="true"
            shift
            ;;
        --no-rviz)
            ENABLE_RVIZ="false"
            shift
            ;;
        --cameras)
            ENABLE_CAMERAS="true"
            shift
            ;;
        --no-cameras)
            ENABLE_CAMERAS="false"
            shift
            ;;
        --recording)
            ENABLE_RECORDING="true"
            shift
            ;;
        --no-recording)
            ENABLE_RECORDING="false"
            shift
            ;;
        --left-controller)
            USE_RIGHT_CONTROLLER="false"
            shift
            ;;
        --right-controller)
            USE_RIGHT_CONTROLLER="true"
            shift
            ;;
        --network-vr)
            VR_MODE="network"
            VR_IP="$2"
            shift 2
            ;;
        --hot-reload)
            HOT_RELOAD="true"
            shift
            ;;
        --log-level)
            LOG_LEVEL="$2"
            shift 2
            ;;
        --shutdown)
            print_info "Graceful shutdown requested"
            graceful_shutdown
            exit 0
            ;;
        --emergency-stop)
            print_error "Emergency stop requested"
            emergency_stop
            exit 0
            ;;
        --help)
            show_help
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# Function to check ROS2 environment
check_ros2_environment() {
    print_info "Checking ROS2 environment..."
    if ! command -v ros2 &> /dev/null; then
        print_error "ROS2 not found. Please source your ROS2 environment."
        echo "Example: source /opt/ros/humble/setup.bash"
        return 1
    fi
    print_success "ROS2 environment found ($ROS_DISTRO)"
    return 0
}

# Function to kill existing ROS and robot processes
kill_existing_processes() {
    print_warning "Stopping existing ROS and robot processes..."
    
    # Stop ROS2 daemon
    print_info "Stopping ROS2 daemon..."
    ros2 daemon stop 2>/dev/null && echo "  ✓ ROS2 daemon stopped" || echo "  ✓ ROS2 daemon was not running"
    
    # Kill VR and teleoperation processes
    print_info "Stopping VR and teleoperation processes..."
    pkill -f "oculus_vr_server" 2>/dev/null && echo "  ✓ Stopped oculus_vr_server" || echo "  ✓ No oculus_vr_server found"
    pkill -f "vr_control_interface" 2>/dev/null && echo "  ✓ Stopped vr_control_interface" || echo "  ✓ No vr_control_interface found"
    pkill -f "system_manager" 2>/dev/null && echo "  ✓ Stopped system_manager" || echo "  ✓ No system_manager found"
    
    # Kill camera processes
    print_info "Stopping camera processes..."
    pkill -f "camera_server" 2>/dev/null && echo "  ✓ Stopped camera_server" || echo "  ✓ No camera_server found"
    pkill -f "realsense" 2>/dev/null && echo "  ✓ Stopped realsense processes" || echo "  ✓ No realsense processes found"
    
    # Kill data recording processes
    print_info "Stopping data recording processes..."
    pkill -f "data_recorder" 2>/dev/null && echo "  ✓ Stopped data_recorder" || echo "  ✓ No data_recorder found"
    pkill -f "mcap" 2>/dev/null && echo "  ✓ Stopped mcap processes" || echo "  ✓ No mcap processes found"
    
    # Kill MoveIt processes
    print_info "Stopping MoveIt processes..."
    pkill -f "moveit" 2>/dev/null && echo "  ✓ Stopped moveit processes" || echo "  ✓ No moveit processes found"
    pkill -f "move_group" 2>/dev/null && echo "  ✓ Stopped move_group" || echo "  ✓ No move_group found"
    
    # Kill robot control processes
    print_info "Stopping robot control processes..."
    pkill -f "robot_state_publisher" 2>/dev/null && echo "  ✓ Stopped robot_state_publisher" || echo "  ✓ No robot_state_publisher found"
    pkill -f "joint_state_publisher" 2>/dev/null && echo "  ✓ Stopped joint_state_publisher" || echo "  ✓ No joint_state_publisher found"
    pkill -f "controller_manager" 2>/dev/null && echo "  ✓ Stopped controller_manager" || echo "  ✓ No controller_manager found"
    pkill -f "spawner" 2>/dev/null && echo "  ✓ Stopped spawner processes" || echo "  ✓ No spawner processes found"
    
    # Kill Franka specific processes
    print_info "Stopping Franka processes..."
    pkill -f "franka_hardware" 2>/dev/null && echo "  ✓ Stopped franka_hardware" || echo "  ✓ No franka_hardware found"
    pkill -f "franka_gripper" 2>/dev/null && echo "  ✓ Stopped franka_gripper" || echo "  ✓ No franka_gripper found"
    pkill -f "franka_control" 2>/dev/null && echo "  ✓ Stopped franka_control" || echo "  ✓ No franka_control found"
    
    # Kill visualization
    print_info "Stopping visualization..."
    pkill -f "rviz2" 2>/dev/null && echo "  ✓ Stopped rviz2" || echo "  ✓ No rviz2 found"
    pkill -f "foxglove" 2>/dev/null && echo "  ✓ Stopped foxglove" || echo "  ✓ No foxglove found"
    
    # Kill any remaining ROS processes
    print_info "Stopping remaining ROS processes..."
    pkill -f "ros2 run" 2>/dev/null && echo "  ✓ Stopped ros2 run processes" || echo "  ✓ No ros2 run processes found"
    pkill -f "ros2 launch" 2>/dev/null && echo "  ✓ Stopped ros2 launch processes" || echo "  ✓ No ros2 launch processes found"
    
    # Wait for processes to terminate
    print_info "Waiting for processes to terminate..."
    sleep 3
    
    # Force kill any stubborn processes
    print_info "Force killing any remaining processes..."
    pkill -9 -f "moveit" 2>/dev/null || true
    pkill -9 -f "franka" 2>/dev/null || true
    pkill -9 -f "rviz2" 2>/dev/null || true
    pkill -9 -f "oculus_vr_server" 2>/dev/null || true
    pkill -9 -f "camera_server" 2>/dev/null || true
    
    print_success "All existing processes terminated"
}

# Function to perform build
perform_build() {
    if [ "$PERFORM_BUILD" != "true" ]; then
        return 0
    fi
    
    print_info "Performing build..."
    cd "$WORKSPACE_DIR"
    
    # Check if colcon is available
    if ! command -v colcon &> /dev/null; then
        print_error "colcon is not installed or not in PATH."
        print_info "Please run the setup script first:"
        print_info "  cd $WORKSPACE_DIR"
        print_info "  ./setup_environment.sh"
        print_info ""
        print_info "If colcon was installed via pip, ensure ~/.local/bin is in your PATH:"
        print_info "  export PATH=\"\$HOME/.local/bin:\$PATH\""
        return 1
    fi
    
    # Clean only if explicitly requested with --clean-build
    if [ "$CLEAN_BUILD" = "true" ]; then
        print_info "Cleaning old build files (build/, install/, log/)..."
        rm -rf build/ install/ log/ 2>/dev/null || true
    fi
    
    # Source ROS2 environment for build
    print_info "Sourcing ROS2 environment for build (source /opt/ros/$ROS_DISTRO/setup.bash)..."
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    
    # Build the workspace using system-level libraries
    print_info "Building workspace with colcon build..."
    
    # Basic CMake arguments, relying on system/ROS paths
    CMAKE_ARGS="-DCMAKE_BUILD_TYPE=Release"
    
    # Add hints for system libraries if they were installed to /usr/local
    CMAKE_ARGS="$CMAKE_ARGS -DCMAKE_PREFIX_PATH=/usr/local;/opt/ros/$ROS_DISTRO"
    CMAKE_ARGS="$CMAKE_ARGS -DCMAKE_FIND_ROOT_PATH=/usr/local;/opt/ros/$ROS_DISTRO;/usr"

    print_info "Final CMake arguments: $CMAKE_ARGS"
    
    # Use colcon build with proper arguments
    # Note: --symlink-install allows for faster development by symlinking Python files
    # The --editable option issue has been fixed in colcon-core >= 0.8.3
    colcon build --symlink-install --cmake-args $CMAKE_ARGS 2>&1 | tee build.log
    BUILD_RESULT=${PIPESTATUS[0]}  # Get colcon's exit code, not tee's
    
    if [ $BUILD_RESULT -eq 0 ]; then
        print_success "Build completed successfully"
        # Check for any build warnings
        if grep -q "warning:" build.log; then
            print_warning "Build completed with warnings. Check build.log for details."
        fi
        rm -f build.log
    else
        print_error "Build failed. Check the output above for errors."
        print_info "Common issues:"
        print_info "  - Missing dependencies: Run 'rosdep install --from-paths src --ignore-src -r -y'"
        print_info "  - Outdated colcon: Run 'pip3 install --upgrade colcon-common-extensions'"
        print_info "  - CMake issues: Check CMake version >= 3.22"
        return 1
    fi
    
    return 0
}

# Function to source workspace
source_workspace() {
    print_info "Sourcing workspace..."
    
    # Source ROS2 base
    if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
        source "/opt/ros/$ROS_DISTRO/setup.bash"
    fi
    
    # Source workspace
    if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
        source "$WORKSPACE_DIR/install/setup.bash"
        print_success "Workspace sourced"
        return 0
    else
        print_error "Failed to source workspace. Build might be missing."
        echo "Run with --build flag to build the workspace"
        return 1
    fi
}

# Function to check robot connectivity
check_robot_connectivity() {
    if [ "$USE_FAKE_HARDWARE" = "false" ]; then
        print_info "Checking robot connectivity to $ROBOT_IP..."
        if ping -c 1 -W 2 "$ROBOT_IP" > /dev/null 2>&1; then
            print_success "Robot is reachable"
        else
            print_warning "Robot at $ROBOT_IP is not reachable"
            echo "Continuing anyway... (use --fake-hardware for testing)"
        fi
    else
        print_info "Using fake hardware - skipping connectivity check"
    fi
}

# Function to create directories
create_directories() {
    # Create recordings directory if needed
    if [ "$ENABLE_RECORDING" = "true" ]; then
        RECORDING_DIR="$HOME/lbx_recordings"
        if [ ! -d "$RECORDING_DIR" ]; then
            print_info "Creating recordings directory: $RECORDING_DIR"
            mkdir -p "$RECORDING_DIR"
        fi
    fi
}

# Function for graceful shutdown
graceful_shutdown() {
    echo ""
    print_info "Initiating graceful shutdown..."
    
    # Stop robot motion first
    print_info "Stopping robot motion..."
    timeout 3 ros2 service call /fr3_arm_controller/stop std_srvs/srv/Trigger 2>/dev/null || true
    
    # Stop VR control
    print_info "Stopping VR control..."
    pkill -SIGTERM -f "oculus_vr_server" 2>/dev/null || true
    pkill -SIGTERM -f "vr_control_interface" 2>/dev/null || true
    
    # Stop other components gracefully
    print_info "Stopping system components..."
    pkill -SIGTERM -f "system_manager" 2>/dev/null || true
    pkill -SIGTERM -f "data_recorder" 2>/dev/null || true
    pkill -SIGTERM -f "camera_server" 2>/dev/null || true
    
    # Wait for graceful shutdown
    sleep 3
    
    # Kill remaining processes
    kill_existing_processes
    
    print_success "Graceful shutdown completed"
}

# Function for emergency stop
emergency_stop() {
    echo ""
    print_error "EMERGENCY STOP INITIATED!"
    
    # Immediate robot stop
    print_error "Stopping robot motion immediately..."
    timeout 2 ros2 service call /fr3_arm_controller/stop std_srvs/srv/Trigger 2>/dev/null || true
    
    # Kill all processes immediately
    print_error "Killing all processes..."
    pkill -9 -f "moveit" 2>/dev/null || true
    pkill -9 -f "franka" 2>/dev/null || true
    pkill -9 -f "ros2" 2>/dev/null || true
    pkill -9 -f "rviz2" 2>/dev/null || true
    pkill -9 -f "oculus_vr_server" 2>/dev/null || true
    pkill -9 -f "camera_server" 2>/dev/null || true
    
    print_error "Emergency stop completed"
}

# Function to display configuration
display_configuration() {
    echo ""
    echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
    echo -e "${CYAN}                    Configuration Summary                       ${NC}"
    echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
    echo -e "  Robot IP:          ${GREEN}$ROBOT_IP${NC}"
    echo -e "  Hardware:          $([ "$USE_FAKE_HARDWARE" = "true" ] && echo -e "${YELLOW}Fake${NC}" || echo -e "${GREEN}Real${NC}")"
    echo -e "  VR Mode:           ${GREEN}$VR_MODE${NC}"
    if [ ! -z "$VR_IP" ]; then
        echo -e "  VR IP:             ${GREEN}$VR_IP${NC}"
    fi
    echo -e "  Controller:        ${GREEN}$([ "$USE_RIGHT_CONTROLLER" = "true" ] && echo "Right" || echo "Left")${NC}"
    echo -e "  Cameras:           $([ "$ENABLE_CAMERAS" = "true" ] && echo -e "${GREEN}Enabled${NC}" || echo -e "${YELLOW}Disabled${NC}")"
    echo -e "  Recording:         $([ "$ENABLE_RECORDING" = "true" ] && echo -e "${GREEN}Enabled${NC}" || echo -e "${YELLOW}Disabled${NC}")"
    echo -e "  RViz:              $([ "$ENABLE_RVIZ" = "true" ] && echo -e "${GREEN}Enabled${NC}" || echo -e "${YELLOW}Disabled${NC}")"
    echo -e "  Hot Reload:        $([ "$HOT_RELOAD" = "true" ] && echo -e "${GREEN}Enabled${NC}" || echo -e "${YELLOW}Disabled${NC}")"
    echo -e "  Log Level:         ${GREEN}$LOG_LEVEL${NC}"
    echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
    echo ""
}

# Main execution
echo ""
echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
echo -e "${CYAN}           LBX Robotics Unified Launch System                   ${NC}"
echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"

# Setup steps
if ! check_ros2_environment; then exit 1; fi
if ! kill_existing_processes; then exit 1; fi
if ! perform_build; then exit 1; fi
if ! source_workspace; then exit 1; fi
if ! check_robot_connectivity; then
    print_warning "Continuing despite connectivity issues..."
fi
create_directories

# Display configuration
display_configuration

# Set up signal handling
trap graceful_shutdown SIGINT SIGTERM
trap emergency_stop SIGUSR1

# Construct launch arguments
LAUNCH_ARGS=""
LAUNCH_ARGS="$LAUNCH_ARGS robot_ip:=$ROBOT_IP"
LAUNCH_ARGS="$LAUNCH_ARGS use_fake_hardware:=$USE_FAKE_HARDWARE"
LAUNCH_ARGS="$LAUNCH_ARGS enable_rviz:=$ENABLE_RVIZ"
LAUNCH_ARGS="$LAUNCH_ARGS enable_cameras:=$ENABLE_CAMERAS"
LAUNCH_ARGS="$LAUNCH_ARGS enable_recording:=$ENABLE_RECORDING"
LAUNCH_ARGS="$LAUNCH_ARGS use_right_controller:=$USE_RIGHT_CONTROLLER"
LAUNCH_ARGS="$LAUNCH_ARGS vr_mode:=$VR_MODE"
LAUNCH_ARGS="$LAUNCH_ARGS hot_reload:=$HOT_RELOAD"
LAUNCH_ARGS="$LAUNCH_ARGS log_level:=$LOG_LEVEL"

if [ ! -z "$VR_IP" ]; then
    LAUNCH_ARGS="$LAUNCH_ARGS vr_ip:=$VR_IP"
fi

# Launch the system
print_info "Starting LBX Robotics System..."
print_info "Launch command: ros2 launch lbx_launch system_bringup.launch.py $LAUNCH_ARGS"
echo ""

# Start ROS2 daemon
print_info "Starting ROS2 daemon..."
ros2 daemon start 2>/dev/null || true

# Launch the system
ros2 launch lbx_launch system_bringup.launch.py $LAUNCH_ARGS

LAUNCH_EC=$?
if [ $LAUNCH_EC -ne 0 ]; then
    print_error "Launch script exited with error code $LAUNCH_EC"
    exit $LAUNCH_EC
fi

print_success "LBX Robotics System shut down gracefully"
exit 0 