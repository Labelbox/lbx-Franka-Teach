#!/bin/bash

# Oculus VR Server - MoveIt Edition Launch Script
# This script provides an easy way to launch the migrated VR server

set -e

# Default values
DEBUG=false
LEFT_CONTROLLER=false
SIMULATION=false
PERFORMANCE=false
NO_RECORDING=false
ENABLE_CAMERAS=false
HOT_RELOAD=false
ROBOT_IP="192.168.1.59"
CAMERA_CONFIG=""
COORD_TRANSFORM=""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_help() {
    echo -e "${BLUE}Oculus VR Server - MoveIt Edition${NC}"
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --debug                    Enable debug mode (no robot control)"
    echo "  --left-controller         Use left controller instead of right"
    echo "  --simulation              Use simulated FR3 robot"
    echo "  --performance             Enable performance mode (2x frequency)"
    echo "  --no-recording            Disable MCAP data recording"
    echo "  --enable-cameras          Enable camera recording"
    echo "  --hot-reload              Enable hot reload mode"
    echo "  --robot-ip IP             Robot IP address (default: $ROBOT_IP)"
    echo "  --camera-config PATH      Path to camera configuration file"
    echo "  --coord-transform X Y Z W Custom coordinate transformation"
    echo "  --check-deps              Check dependencies and exit"
    echo "  --help                    Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                        # Run with default settings"
    echo "  $0 --debug                # Run in debug mode"
    echo "  $0 --performance          # Run with performance optimizations"
    echo "  $0 --hot-reload           # Run with automatic restart on changes"
    echo "  $0 --enable-cameras       # Run with camera recording"
    echo ""
    echo "Prerequisites:"
    echo "  1. Start the robust Franka system first:"
    echo "     cd ros2_moveit_franka && ./run_robust_franka.sh --robot-ip $ROBOT_IP"
    echo ""
    echo "  2. Ensure the system shows 'Robot state: READY' in the output"
    echo ""
    echo "  3. Verify services are available:"
    echo "     ros2 service list | grep -E '(get_planning_scene|compute_cartesian_path|apply_planning_scene)'"
    echo ""
}

check_dependencies() {
    echo -e "${BLUE}Checking dependencies...${NC}"
    
    # Check if ROS 2 is sourced
    if ! command -v ros2 &> /dev/null; then
        echo -e "${RED}❌ ROS 2 not found. Please source your ROS 2 workspace.${NC}"
        return 1
    fi
    echo -e "${GREEN}✅ ROS 2 found${NC}"
    
    # Check if Python dependencies are available
    python3 -c "import rclpy, moveit_msgs, control_msgs" 2>/dev/null
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ ROS 2 Python dependencies found${NC}"
    else
        echo -e "${RED}❌ Missing ROS 2 Python dependencies${NC}"
        echo "Install with: pip install rclpy"
        return 1
    fi
    
    # Check if VR server file exists
    if [ ! -f "oculus_vr_server_moveit.py" ]; then
        echo -e "${RED}❌ oculus_vr_server_moveit.py not found${NC}"
        echo "Make sure you're in the correct directory"
        return 1
    fi
    echo -e "${GREEN}✅ VR server file found${NC}"
    
    # Check if MoveIt is running - updated to check for actual service names
    echo -e "${YELLOW}⚠️  Checking if MoveIt is running...${NC}"
    
    # Source ROS environment to ensure we can see services
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    fi
    
    # Set ROS_DOMAIN_ID to match the robust system
    export ROS_DOMAIN_ID=42
    
    # Check for the actual MoveIt services that are available
    timeout 10 bash -c 'ros2 service list' > /tmp/services_list 2>/dev/null
    if [ $? -eq 0 ] && ( grep -q "get_planning_scene\|compute_cartesian_path\|apply_planning_scene" /tmp/services_list ); then
        echo -e "${GREEN}✅ MoveIt services detected${NC}"
        rm -f /tmp/services_list
    else
        echo -e "${YELLOW}⚠️  MoveIt services not detected${NC}"
        echo "   Available services:"
        if [ -f /tmp/services_list ]; then
            grep -E "(planning|moveit|compute|move_)" /tmp/services_list | head -5 || echo "   No MoveIt-related services found"
            rm -f /tmp/services_list
        fi
        echo ""
        echo "   Make sure the robust Franka system is running in another terminal:"
        echo "   cd ros2_moveit_franka && ./run_robust_franka.sh --robot-ip $ROBOT_IP"
        echo ""
        echo "   Continue anyway? (y/N)"
        read -r response
        if [[ ! "$response" =~ ^[Yy]$ ]]; then
            return 1
        fi
    fi
    
    echo -e "${GREEN}✅ All dependencies check passed${NC}"
    return 0
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --debug)
            DEBUG=true
            shift
            ;;
        --left-controller)
            LEFT_CONTROLLER=true
            shift
            ;;
        --simulation)
            SIMULATION=true
            shift
            ;;
        --performance)
            PERFORMANCE=true
            shift
            ;;
        --no-recording)
            NO_RECORDING=true
            shift
            ;;
        --enable-cameras)
            ENABLE_CAMERAS=true
            shift
            ;;
        --hot-reload)
            HOT_RELOAD=true
            shift
            ;;
        --robot-ip)
            ROBOT_IP="$2"
            shift 2
            ;;
        --camera-config)
            CAMERA_CONFIG="$2"
            shift 2
            ;;
        --coord-transform)
            COORD_TRANSFORM="$2 $3 $4 $5"
            shift 5
            ;;
        --check-deps)
            check_dependencies
            exit $?
            ;;
        --help)
            print_help
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            print_help
            exit 1
            ;;
    esac
done

# Check dependencies
if ! check_dependencies; then
    echo -e "${RED}❌ Dependency check failed${NC}"
    exit 1
fi

# Build command
CMD="python3 oculus_vr_server_moveit.py"

if [ "$DEBUG" = true ]; then
    CMD="$CMD --debug"
fi

if [ "$LEFT_CONTROLLER" = true ]; then
    CMD="$CMD --left-controller"
fi

if [ "$SIMULATION" = true ]; then
    CMD="$CMD --simulation"
fi

if [ "$PERFORMANCE" = true ]; then
    CMD="$CMD --performance"
fi

if [ "$NO_RECORDING" = true ]; then
    CMD="$CMD --no-recording"
fi

if [ "$ENABLE_CAMERAS" = true ]; then
    CMD="$CMD --enable-cameras"
fi

if [ "$HOT_RELOAD" = true ]; then
    CMD="$CMD --hot-reload"
fi

if [ -n "$CAMERA_CONFIG" ]; then
    CMD="$CMD --camera-config $CAMERA_CONFIG"
fi

if [ -n "$COORD_TRANSFORM" ]; then
    CMD="$CMD --coord-transform $COORD_TRANSFORM"
fi

# Print configuration
echo -e "${BLUE}🎮 Starting Oculus VR Server - MoveIt Edition${NC}"
echo -e "${BLUE}=================================================${NC}"
echo "Configuration:"
echo "  Debug mode: $DEBUG"
echo "  Controller: $([ "$LEFT_CONTROLLER" = true ] && echo "LEFT" || echo "RIGHT")"
echo "  Simulation: $SIMULATION"
echo "  Performance mode: $PERFORMANCE"
echo "  Recording: $([ "$NO_RECORDING" = true ] && echo "DISABLED" || echo "ENABLED")"
echo "  Cameras: $([ "$ENABLE_CAMERAS" = true ] && echo "ENABLED" || echo "DISABLED")"
echo "  Hot reload: $HOT_RELOAD"
echo "  Robot IP: $ROBOT_IP"
if [ -n "$CAMERA_CONFIG" ]; then
    echo "  Camera config: $CAMERA_CONFIG"
fi
if [ -n "$COORD_TRANSFORM" ]; then
    echo "  Coordinate transform: $COORD_TRANSFORM"
fi
echo ""

# Final warning if not in debug mode
if [ "$DEBUG" = false ]; then
    echo -e "${YELLOW}⚠️  WARNING: Running in LIVE ROBOT CONTROL mode${NC}"
    echo -e "${YELLOW}   Make sure the robot is properly configured and safe to operate${NC}"
    echo -e "${YELLOW}   Press Ctrl+C at any time to stop${NC}"
    echo ""
    echo "Continue? (y/N)"
    read -r response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        echo "Cancelled."
        exit 0
    fi
fi

echo -e "${GREEN}🚀 Launching VR server...${NC}"
echo "Command: $CMD"
echo ""

# Execute the command
exec $CMD 