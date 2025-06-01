#!/bin/bash
#
# Labelbox Robotics VR Teleoperation System
# 
# Usage: ./run_teleoperation.sh [options]
#
# Options:
#   --robot-ip <IP>         Robot IP address (default: 192.168.1.59)
#   --cameras               Enable camera system
#   --no-recording          Disable data recording
#   --left-controller       Use left VR controller instead of right
#   --network-vr <IP>       Use network VR mode with specified IP
#   --hot-reload            Enable hot reload for development
#   --verify-data           Enable data verification mode
#   --no-rviz               Disable RViz visualization
#   --help                  Show this help message

# Default values
ROBOT_IP="192.168.1.59"
ENABLE_CAMERAS="false"
ENABLE_RECORDING="true"
USE_RIGHT_CONTROLLER="true"
VR_MODE="usb"
VR_IP=""
HOT_RELOAD="false"
VERIFY_DATA="false"
USE_RVIZ="true"

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Function to print colored output
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

# Function to show help
show_help() {
    echo "Labelbox Robotics VR Teleoperation System"
    echo ""
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  --robot-ip <IP>         Robot IP address (default: 192.168.1.59)"
    echo "  --cameras               Enable camera system"
    echo "  --no-recording          Disable data recording"
    echo "  --left-controller       Use left VR controller instead of right"
    echo "  --network-vr <IP>       Use network VR mode with specified IP"
    echo "  --hot-reload            Enable hot reload for development"
    echo "  --verify-data           Enable data verification mode"
    echo "  --no-rviz               Disable RViz visualization"
    echo "  --help                  Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                                    # Run with defaults"
    echo "  $0 --robot-ip 192.168.1.100          # Custom robot IP"
    echo "  $0 --cameras --no-recording           # Cameras on, recording off"
    echo "  $0 --network-vr 192.168.1.50          # Network VR mode"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --robot-ip)
            ROBOT_IP="$2"
            shift 2
            ;;
        --cameras)
            ENABLE_CAMERAS="true"
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
        --network-vr)
            VR_MODE="network"
            VR_IP="$2"
            shift 2
            ;;
        --hot-reload)
            HOT_RELOAD="true"
            shift
            ;;
        --verify-data)
            VERIFY_DATA="true"
            shift
            ;;
        --no-rviz)
            USE_RVIZ="false"
            shift
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

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    print_error "ROS2 is not sourced. Please source your ROS2 installation."
    echo "Example: source /opt/ros/humble/setup.bash"
    exit 1
fi

print_info "Starting Labelbox Robotics VR Teleoperation System..."
print_info "ROS2 Distribution: $ROS_DISTRO"

# Build launch command
LAUNCH_CMD="ros2 launch lbx_robotics integrated_system.launch.py"
LAUNCH_CMD="$LAUNCH_CMD robot_ip:=$ROBOT_IP"
LAUNCH_CMD="$LAUNCH_CMD enable_cameras:=$ENABLE_CAMERAS"
LAUNCH_CMD="$LAUNCH_CMD enable_recording:=$ENABLE_RECORDING"
LAUNCH_CMD="$LAUNCH_CMD use_right_controller:=$USE_RIGHT_CONTROLLER"
LAUNCH_CMD="$LAUNCH_CMD vr_mode:=$VR_MODE"
LAUNCH_CMD="$LAUNCH_CMD use_rviz:=$USE_RVIZ"
LAUNCH_CMD="$LAUNCH_CMD hot_reload:=$HOT_RELOAD"
LAUNCH_CMD="$LAUNCH_CMD verify_data:=$VERIFY_DATA"

if [ ! -z "$VR_IP" ]; then
    LAUNCH_CMD="$LAUNCH_CMD vr_ip:=$VR_IP"
fi

# Display configuration
echo ""
echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
echo -e "${CYAN}                    Configuration Summary                       ${NC}"
echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
echo -e "  Robot IP:          ${GREEN}$ROBOT_IP${NC}"
echo -e "  VR Mode:           ${GREEN}$VR_MODE${NC}"
if [ ! -z "$VR_IP" ]; then
    echo -e "  VR IP:             ${GREEN}$VR_IP${NC}"
fi
echo -e "  Controller:        ${GREEN}$([ "$USE_RIGHT_CONTROLLER" = "true" ] && echo "Right" || echo "Left")${NC}"
echo -e "  Cameras:           $([ "$ENABLE_CAMERAS" = "true" ] && echo -e "${GREEN}Enabled${NC}" || echo -e "${YELLOW}Disabled${NC}")"
echo -e "  Recording:         $([ "$ENABLE_RECORDING" = "true" ] && echo -e "${GREEN}Enabled${NC}" || echo -e "${YELLOW}Disabled${NC}")"
echo -e "  RViz:              $([ "$USE_RVIZ" = "true" ] && echo -e "${GREEN}Enabled${NC}" || echo -e "${YELLOW}Disabled${NC}")"
echo -e "  Hot Reload:        $([ "$HOT_RELOAD" = "true" ] && echo -e "${GREEN}Enabled${NC}" || echo -e "${YELLOW}Disabled${NC}")"
echo -e "  Data Verification: $([ "$VERIFY_DATA" = "true" ] && echo -e "${GREEN}Enabled${NC}" || echo -e "${YELLOW}Disabled${NC}")"
echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
echo ""

# Create recordings directory if needed
if [ "$ENABLE_RECORDING" = "true" ]; then
    RECORDING_DIR="$HOME/lbx_recordings"
    if [ ! -d "$RECORDING_DIR" ]; then
        print_info "Creating recordings directory: $RECORDING_DIR"
        mkdir -p "$RECORDING_DIR"
    fi
fi

# Launch the system
print_info "Launching system..."
print_info "Command: $LAUNCH_CMD"
echo ""

# Set up trap to handle Ctrl+C gracefully
trap 'print_warning "Shutting down..."; exit 0' INT

# Execute the launch command
exec $LAUNCH_CMD 