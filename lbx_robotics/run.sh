#!/bin/bash
# LBX Robotics - Main Launch Script
# This script builds the lbx_robotics ROS2 workspace and launches the main system.
# It is designed to be robust, configurable, and use relative paths for portability.

# --- Configuration ---
DEFAULT_ROBOT_IP="192.168.1.59" # Default, can be overridden by franka_config.yaml or args
DEFAULT_LOG_LEVEL="INFO"

# --- Colors for Output ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# --- Helper Functions ---
show_help() {
    echo -e "${BLUE}LBX Robotics Launch Script${NC}"
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --robot-ip IP             Robot IP address (overrides config file)"
    echo "  --fake-hardware           Use fake hardware for testing (overrides config)"
    echo "  --no-fake-hardware        Use real hardware (overrides config)"
    echo "  --rviz / --no-rviz        Enable/Disable RViz (overrides config)"
    echo "  --log-level LEVEL         Set log level (DEBUG, INFO, WARN, ERROR)"
    echo "  --skip-build              Skip the colcon build step"
    echo "  --clean-build             Perform a clean build (removes build, install, log) (Now default behavior)"
    echo "  --shutdown                Gracefully shutdown any running system (experimental)"
    echo "  --help                    Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                                    # Use defaults from config files"
    echo "  $0 --robot-ip 192.168.1.100         # Custom robot IP"
    echo "  $0 --fake-hardware --no-rviz        # Test mode without RViz"
    echo "  $0 --skip-build                      # Skip colcon build"
    echo "  $0 --clean-build                   # Force a clean rebuild"
    echo ""
}

# --- Argument Parsing ---
ROBOT_IP_ARG=""
USE_FAKE_HARDWARE_ARG=""
ENABLE_RVIZ_ARG=""
LOG_LEVEL_ARG="$DEFAULT_LOG_LEVEL"
SKIP_BUILD_ARG="false"
CLEAN_BUILD_ARG="false"

while [[ $# -gt 0 ]]; do
    case $1 in
        --robot-ip) ROBOT_IP_ARG="$2"; shift 2 ;;       
        --fake-hardware) USE_FAKE_HARDWARE_ARG="true"; shift ;; 
        --no-fake-hardware) USE_FAKE_HARDWARE_ARG="false"; shift ;; 
        --rviz) ENABLE_RVIZ_ARG="true"; shift ;;      
        --no-rviz) ENABLE_RVIZ_ARG="false"; shift ;;   
        --log-level) LOG_LEVEL_ARG="$2"; shift 2 ;;    
        --skip-build) SKIP_BUILD_ARG="true"; shift ;; 
        --clean-build) CLEAN_BUILD_ARG="true"; shift ;; 
        --shutdown) echo -e "${BLUE}Graceful shutdown requested...${NC}"; pkill -f system_bringup.launch.py; exit 0 ;; # Simplified shutdown
        --help) show_help; exit 0 ;;                 
        *) echo -e "${RED}Unknown option: $1${NC}"; show_help; exit 1 ;;
    esac
done

# --- Script Setup ---
# Get the directory of this script to ensure relative paths work
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR" # lbx_robotics is the workspace root

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  LBX Robotics System Launcher${NC}"
echo -e "${BLUE}========================================${NC}"
echo "Workspace: $WORKSPACE_DIR"

# --- Environment Setup ---
check_ros2_environment() {
    echo -e "${BLUE}Checking ROS2 environment...${NC}"
    if ! command -v ros2 &> /dev/null; then
        echo -e "${RED}ERROR: ROS2 not found. Please source your ROS2 environment.${NC}"
        echo "Example: source /opt/ros/humble/setup.bash" >&2
        return 1
    fi
    echo -e "${GREEN}✓ ROS2 environment found ($ROS_DISTRO)${NC}"
    return 0
}

# --- Build Step ---
perform_build() {
    if [ "$SKIP_BUILD_ARG" = "true" ]; then
        echo -e "${YELLOW}Skipping build as requested.${NC}"
        return 0
    fi

    echo -e "${YELLOW}Performing build...${NC}"
    cd "$WORKSPACE_DIR" # Navigate to the workspace root (lbx_robotics)

    # Only clean if --clean-build was specified
    if [ "$CLEAN_BUILD_ARG" = "true" ]; then
        echo "Cleaning old build files (build/, install/, log/)..."
        rm -rf build/ install/ log/ 2>/dev/null || true
    fi

    echo "Sourcing ROS2 environment for build..."
    source "/opt/ros/$ROS_DISTRO/setup.bash" # Use the detected ROS_DISTRO
    
    echo -e "${BLUE}Building with: colcon build --symlink-install${NC}"
    if colcon build --symlink-install 2>&1; then # Build all packages in the workspace
        echo -e "${GREEN}✓ Build completed successfully.${NC}"
    else
        echo -e "${RED}ERROR: Build failed.${NC}" >&2
        return 1
    fi
    return 0
}

# --- Source Workspace ---
source_workspace() {
    echo "Sourcing workspace: $WORKSPACE_DIR/install/setup.bash"
    if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
        source "$WORKSPACE_DIR/install/setup.bash"
        echo -e "${GREEN}✓ Workspace sourced.${NC}"
        return 0
    else 
        echo -e "${RED}ERROR: Failed to source workspace. Build might be missing.${NC}" >&2
        return 1
    fi
}

# --- Main Execution ---
if ! check_ros2_environment; then exit 1; fi
if ! perform_build; then exit 1; fi
if ! source_workspace; then exit 1; fi

# Construct launch arguments
LAUNCH_ARGS=""
if [ -n "$ROBOT_IP_ARG" ]; then LAUNCH_ARGS+=" robot_ip:=$ROBOT_IP_ARG"; fi
if [ -n "$USE_FAKE_HARDWARE_ARG" ]; then LAUNCH_ARGS+=" use_fake_hardware:=$USE_FAKE_HARDWARE_ARG"; fi
if [ -n "$ENABLE_RVIZ_ARG" ]; then LAUNCH_ARGS+=" enable_rviz:=$ENABLE_RVIZ_ARG"; fi
if [ -n "$LOG_LEVEL_ARG" ]; then LAUNCH_ARGS+=" log_level:=$LOG_LEVEL_ARG"; fi

echo -e "${GREEN}Starting LBX Robotics System...${NC}"
echo "Launch file: lbx_launch system_bringup.launch.py"
echo "Arguments: $LAUNCH_ARGS"

ros2 launch lbx_launch system_bringup.launch.py $LAUNCH_ARGS

LAUNCH_EC=$?
if [ $LAUNCH_EC -ne 0 ]; then
    echo -e "${RED}Launch script exited with error code $LAUNCH_EC.${NC}" >&2
    exit $LAUNCH_EC
fi

echo -e "${GREEN}✓ LBX Robotics System shut down gracefully.${NC}"
exit 0 