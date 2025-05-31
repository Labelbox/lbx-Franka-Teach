#!/bin/bash
# Robust Franka Launch Script
# This script launches the crash-proof Franka system with auto-restart capabilities

# Remove set -e to prevent script from exiting on non-critical errors
# set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default parameters
ROBOT_IP="192.168.1.59"
USE_FAKE_HARDWARE="false"
ENABLE_RVIZ="true"
ENABLE_HEALTH_MONITOR="true"
AUTO_RESTART="true"
LOG_LEVEL="INFO"
SKIP_BUILD="false"

# Help function
show_help() {
    echo -e "${BLUE}Robust Franka Launch Script${NC}"
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --robot-ip IP             Robot IP address (default: $ROBOT_IP)"
    echo "  --fake-hardware           Use fake hardware for testing"
    echo "  --no-rviz                 Disable RViz visualization"
    echo "  --no-health-monitor       Disable health monitoring"
    echo "  --no-auto-restart         Disable automatic restart"
    echo "  --log-level LEVEL         Set log level (DEBUG, INFO, WARN, ERROR)"
    echo "  --skip-build              Skip the fresh build step"
    echo "  --shutdown                Gracefully shutdown any running system"
    echo "  --emergency-stop          Emergency stop all robot processes"
    echo "  --help                    Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                                    # Use defaults"
    echo "  $0 --robot-ip 192.168.1.100         # Custom robot IP"
    echo "  $0 --fake-hardware --no-rviz        # Test mode without RViz"
    echo "  $0 --skip-build                      # Skip fresh build"
    echo "  $0 --shutdown                        # Graceful shutdown"
    echo "  $0 --emergency-stop                  # Emergency stop"
    echo ""
    echo "Shutdown Controls:"
    echo "  Ctrl+C                               # Graceful shutdown"
    echo "  Ctrl+Z + kill -9 \$pid               # Emergency stop"
    echo ""
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --robot-ip)
            ROBOT_IP="$2"
            shift 2
            ;;
        --fake-hardware)
            USE_FAKE_HARDWARE="true"
            shift
            ;;
        --no-rviz)
            ENABLE_RVIZ="false"
            shift
            ;;
        --no-health-monitor)
            ENABLE_HEALTH_MONITOR="false"
            shift
            ;;
        --no-auto-restart)
            AUTO_RESTART="false"
            shift
            ;;
        --log-level)
            LOG_LEVEL="$2"
            shift 2
            ;;
        --skip-build)
            SKIP_BUILD="true"
            shift
            ;;
        --shutdown)
            echo -e "${BLUE}Graceful shutdown requested${NC}"
            graceful_shutdown
            exit 0
            ;;
        --emergency-stop)
            echo -e "${RED}Emergency stop requested${NC}"
            emergency_stop
            ;;
        --help)
            show_help
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            show_help
            exit 1
            ;;
    esac
done

# Function to check if ROS2 is sourced
check_ros2_environment() {
    echo -e "${BLUE}Checking ROS2 environment...${NC}"
    if ! command -v ros2 &> /dev/null; then
        echo -e "${RED}ERROR: ROS2 not found. Please source your ROS2 environment first.${NC}"
        echo "Example: source /opt/ros/humble/setup.bash"
        exit 1
    fi
    echo -e "${GREEN}✓ ROS2 environment found${NC}"
}

# Function to kill existing ROS and MoveIt processes
kill_existing_processes() {
    echo -e "${YELLOW}Stopping existing ROS and MoveIt processes...${NC}"
    
    # Kill ROS2 processes with better error handling
    echo "Killing ROS2 daemon..."
    if ros2 daemon stop 2>/dev/null; then
        echo "✓ ROS2 daemon stopped"
    else
        echo "✓ ROS2 daemon was not running"
    fi
    
    # Kill specific MoveIt and Franka processes
    echo "Killing MoveIt processes..."
    pkill -f "moveit" 2>/dev/null || echo "✓ No moveit processes found"
    pkill -f "robot_state_publisher" 2>/dev/null || echo "✓ No robot_state_publisher processes found"
    pkill -f "joint_state_publisher" 2>/dev/null || echo "✓ No joint_state_publisher processes found"
    pkill -f "controller_manager" 2>/dev/null || echo "✓ No controller_manager processes found"
    pkill -f "spawner" 2>/dev/null || echo "✓ No spawner processes found"
    
    echo "Killing Franka processes..."
    # Be more specific to avoid killing this script
    pkill -f "franka_hardware" 2>/dev/null || echo "✓ No franka_hardware processes found"
    pkill -f "franka_gripper" 2>/dev/null || echo "✓ No franka_gripper processes found"
    pkill -f "franka_robot_state_broadcaster" 2>/dev/null || echo "✓ No franka_robot_state_broadcaster processes found"
    pkill -f "robust_franka_control" 2>/dev/null || echo "✓ No robust_franka_control processes found"
    pkill -f "system_health_monitor" 2>/dev/null || echo "✓ No system_health_monitor processes found"
    
    echo "Killing RViz..."
    pkill -f "rviz2" 2>/dev/null || echo "✓ No rviz2 processes found"
    
    echo "Killing other ROS nodes..."
    # Be more specific here too
    pkill -f "ros2 run" 2>/dev/null || echo "✓ No ros2 run processes found"
    pkill -f "ros2 launch" 2>/dev/null || echo "✓ No ros2 launch processes found"
    
    # Wait for processes to terminate
    echo "Waiting for processes to terminate..."
    sleep 2
    
    # More specific force kill
    echo "Force killing any remaining processes..."
    pkill -9 -f "moveit" 2>/dev/null || true
    pkill -9 -f "franka_hardware" 2>/dev/null || true
    pkill -9 -f "rviz2" 2>/dev/null || true
    
    echo -e "${GREEN}✓ Existing processes terminated${NC}"
}

# Function to perform fresh build
perform_fresh_build() {
    if [ "$SKIP_BUILD" = "true" ]; then
        echo -e "${YELLOW}Skipping fresh build as requested${NC}"
        return 0
    fi
    
    echo -e "${YELLOW}Performing fresh build...${NC}"
    
    # Remove old build artifacts
    echo "Cleaning old build files..."
    rm -rf build/ install/ log/ 2>/dev/null || true
    
    # Source ROS2 environment for building
    echo "Sourcing ROS2 environment..."
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        echo "✓ ROS2 environment sourced"
    else
        echo -e "${RED}ERROR: ROS2 setup file not found${NC}"
        return 1
    fi
    
    # Source Franka workspace for MoveIt dependencies
    echo "Sourcing Franka workspace..."
    if [ -f "/home/labelbox/franka_ros2_ws/install/setup.bash" ]; then
        source /home/labelbox/franka_ros2_ws/install/setup.bash
        echo "✓ Franka workspace sourced for build"
    else
        echo -e "${YELLOW}⚠ Warning: Franka workspace not found at expected location${NC}"
        echo "This may cause build issues if MoveIt dependencies are missing"
    fi
    
    # Note: We no longer check for moveit_commander since we use ROS 2 native interface
    echo "✓ Using ROS 2 native MoveIt interface (no Python dependencies required)"
    
    # Build the package with colcon
    echo -e "${BLUE}Building with: colcon build --packages-select ros2_moveit_franka${NC}"
    if colcon build --packages-select ros2_moveit_franka --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1; then
        echo -e "${GREEN}✓ Build completed successfully${NC}"
    else
        echo -e "${RED}ERROR: Build failed${NC}"
        echo "Try running manually: colcon build --packages-select ros2_moveit_franka"
        return 1
    fi
    
    # Fix ROS2 directory structure for executables
    echo "Fixing ROS2 directory structure..."
    if fix_ros2_directory_structure; then
        echo -e "${GREEN}✓ Directory structure fixed${NC}"
    else
        echo -e "${RED}ERROR: Failed to fix directory structure${NC}"
        return 1
    fi
    
    # Source the newly built workspace
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
        echo -e "${GREEN}✓ Workspace sourced${NC}"
    else
        echo -e "${RED}ERROR: Failed to source workspace${NC}"
        return 1
    fi
    
    return 0
}

# Function to fix ROS2 directory structure
fix_ros2_directory_structure() {
    echo "Creating expected ROS2 directory structure..."
    
    # Create the lib/package_name directory that ROS2 launch expects
    local lib_dir="install/ros2_moveit_franka/lib/ros2_moveit_franka"
    local bin_dir="install/ros2_moveit_franka/bin"
    
    if [ ! -d "$bin_dir" ]; then
        echo -e "${RED}ERROR: bin directory not found after build${NC}"
        return 1
    fi
    
    # Create the expected directory
    mkdir -p "$lib_dir"
    
    # Copy executables to the expected location
    if [ -d "$bin_dir" ]; then
        for executable in "$bin_dir"/*; do
            if [ -f "$executable" ] && [ -x "$executable" ]; then
                local filename=$(basename "$executable")
                cp "$executable" "$lib_dir/$filename"
                chmod +x "$lib_dir/$filename"
                echo "✓ Copied $filename to lib directory"
            fi
        done
    fi
    
    # Verify executables are in place
    if [ -f "$lib_dir/robust_franka_control" ] && [ -f "$lib_dir/system_health_monitor" ]; then
        echo "✓ All executables found in expected location"
        return 0
    else
        echo -e "${RED}ERROR: Executables not found in expected location${NC}"
        return 1
    fi
}

# Function to check if package is built
check_package_built() {
    echo -e "${BLUE}Checking if package is built...${NC}"
    
    # Check both locations for robustness
    local lib_executable="install/ros2_moveit_franka/lib/ros2_moveit_franka/robust_franka_control"
    local bin_executable="install/ros2_moveit_franka/bin/robust_franka_control"
    
    if [ -f "$lib_executable" ] && [ -x "$lib_executable" ]; then
        echo -e "${GREEN}✓ Package built successfully (lib location)${NC}"
        return 0
    elif [ -f "$bin_executable" ] && [ -x "$bin_executable" ]; then
        echo -e "${YELLOW}Package built but needs directory fix...${NC}"
        # Try to fix the directory structure
        if fix_ros2_directory_structure; then
            echo -e "${GREEN}✓ Directory structure fixed${NC}"
            return 0
        else
            echo -e "${RED}ERROR: Failed to fix directory structure${NC}"
            return 1
        fi
    else
        echo -e "${RED}ERROR: Package not built properly.${NC}"
        if [ "$SKIP_BUILD" = "true" ]; then
            echo "Try running without --skip-build flag"
        fi
        return 1
    fi
}

# Function to check robot connectivity
check_robot_connectivity() {
    if [ "$USE_FAKE_HARDWARE" = "false" ]; then
        echo -e "${YELLOW}Checking robot connectivity to $ROBOT_IP...${NC}"
        if ping -c 1 -W 2 "$ROBOT_IP" > /dev/null 2>&1; then
            echo -e "${GREEN}✓ Robot is reachable${NC}"
        else
            echo -e "${YELLOW}⚠ Warning: Robot at $ROBOT_IP is not reachable${NC}"
            echo "Continuing anyway... (use --fake-hardware for testing without robot)"
        fi
    else
        echo -e "${BLUE}Using fake hardware - skipping connectivity check${NC}"
    fi
}

# Function to setup environment
setup_environment() {
    echo -e "${BLUE}Setting up environment...${NC}"
    
    # Source ROS2 base environment
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        echo "✓ ROS2 base environment sourced"
    fi
    
    # Source Franka workspace if it exists
    if [ -f "/home/labelbox/franka_ros2_ws/install/setup.bash" ]; then
        source /home/labelbox/franka_ros2_ws/install/setup.bash
        echo "✓ Franka workspace sourced"
    fi
    
    # Source workspace (only if not already done in build step)
    if [ "$SKIP_BUILD" = "true" ] && [ -f "install/setup.bash" ]; then
        source install/setup.bash
        echo "✓ Local workspace sourced"
    fi
    
    # Set ROS_DOMAIN_ID if not set
    if [ -z "$ROS_DOMAIN_ID" ]; then
        export ROS_DOMAIN_ID=42
        echo "Set ROS_DOMAIN_ID to $ROS_DOMAIN_ID"
    fi
    
    # Ensure Python can find ROS packages (for diagnostics)
    export PYTHONPATH="/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH"
    if [ -d "/home/labelbox/franka_ros2_ws/install" ]; then
        export PYTHONPATH="/home/labelbox/franka_ros2_ws/install/lib/python3.10/site-packages:$PYTHONPATH"
    fi
    echo "✓ Python path configured for ROS packages"
    
    # Verify MoveIt services will be available (instead of Python packages)
    echo "✓ Using ROS 2 native MoveIt interface (service-based)"
    echo "  Services will be checked at runtime: /move_action, /get_planning_scene"
    
    # Start ROS2 daemon fresh
    echo "Starting ROS2 daemon..."
    if ros2 daemon start 2>/dev/null; then
        echo "✓ ROS2 daemon started"
    else
        echo "✓ ROS2 daemon already running"
    fi
    
    echo -e "${GREEN}✓ Environment setup complete${NC}"
}

# Function to graceful shutdown the robot system
graceful_shutdown() {
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE}  Initiating Graceful System Shutdown${NC}"
    echo -e "${BLUE}========================================${NC}"
    
    # Step 0: Stop the recovery daemon first to prevent restarts during shutdown
    echo -e "${YELLOW}Step 0: Stopping recovery daemon...${NC}"
    pkill -SIGTERM -f "franka_recovery_daemon" 2>/dev/null && echo "✓ Stopped recovery daemon" || echo "✓ Recovery daemon not running"
    sleep 1
    
    # Step 1: Stop robot motion safely
    echo -e "${YELLOW}Step 1: Stopping robot motion safely...${NC}"
    if timeout 3 ros2 topic list 2>/dev/null | grep -q "/fr3_arm_controller"; then
        echo "Sending stop command to arm controller..."
        timeout 5 ros2 service call /fr3_arm_controller/stop std_srvs/srv/Trigger 2>/dev/null || echo "✓ Controller stop failed or already stopped"
    else
        echo "✓ Arm controller not available"
    fi
    
    # Step 2: Stop our robust control nodes first
    echo -e "${YELLOW}Step 2: Stopping robust control nodes...${NC}"
    pkill -SIGTERM -f "robust_franka_control" 2>/dev/null && echo "✓ Stopped robust_franka_control" || echo "✓ robust_franka_control not running"
    pkill -SIGTERM -f "system_health_monitor" 2>/dev/null && echo "✓ Stopped system_health_monitor" || echo "✓ system_health_monitor not running"
    
    # Give nodes time to shutdown gracefully
    sleep 3
    
    # Step 3: Stop controllers in proper order
    echo -e "${YELLOW}Step 3: Stopping controllers...${NC}"
    if timeout 3 ros2 node list 2>/dev/null | grep -q "controller_manager"; then
        echo "Stopping fr3_arm_controller..."
        timeout 5 ros2 service call /controller_manager/stop_controller controller_manager_msgs/srv/StopController "{name: fr3_arm_controller}" 2>/dev/null || echo "✓ Controller already stopped"
        
        echo "Stopping franka_robot_state_broadcaster..."
        timeout 5 ros2 service call /controller_manager/stop_controller controller_manager_msgs/srv/StopController "{name: franka_robot_state_broadcaster}" 2>/dev/null || echo "✓ Broadcaster already stopped"
        
        echo "Stopping joint_state_broadcaster..."
        timeout 5 ros2 service call /controller_manager/stop_controller controller_manager_msgs/srv/StopController "{name: joint_state_broadcaster}" 2>/dev/null || echo "✓ Joint broadcaster already stopped"
    else
        echo "✓ Controller manager not running"
    fi
    
    # Step 4: Stop MoveIt and planning
    echo -e "${YELLOW}Step 4: Stopping MoveIt components...${NC}"
    pkill -SIGTERM -f "move_group" 2>/dev/null && echo "✓ Stopped move_group" || echo "✓ move_group not running"
    
    # Step 5: Stop hardware interface
    echo -e "${YELLOW}Step 5: Stopping hardware interface...${NC}"
    pkill -SIGTERM -f "ros2_control_node" 2>/dev/null && echo "✓ Stopped ros2_control_node" || echo "✓ ros2_control_node not running"
    
    # Step 6: Stop gripper
    echo -e "${YELLOW}Step 6: Stopping gripper...${NC}"
    pkill -SIGTERM -f "franka_gripper_node" 2>/dev/null && echo "✓ Stopped franka_gripper_node" || echo "✓ franka_gripper_node not running"
    
    # Step 7: Stop state publishers
    echo -e "${YELLOW}Step 7: Stopping state publishers...${NC}"
    pkill -SIGTERM -f "robot_state_publisher" 2>/dev/null && echo "✓ Stopped robot_state_publisher" || echo "✓ robot_state_publisher not running"
    pkill -SIGTERM -f "joint_state_publisher" 2>/dev/null && echo "✓ Stopped joint_state_publisher" || echo "✓ joint_state_publisher not running"
    
    # Step 8: Stop visualization
    echo -e "${YELLOW}Step 8: Stopping visualization...${NC}"
    pkill -SIGTERM -f "rviz2" 2>/dev/null && echo "✓ Stopped rviz2" || echo "✓ rviz2 not running"
    
    # Wait for graceful shutdown
    echo -e "${YELLOW}Waiting for graceful shutdown...${NC}"
    sleep 3
    
    # Step 9: Force kill any remaining processes
    echo -e "${YELLOW}Step 9: Cleaning up remaining processes...${NC}"
    pkill -9 -f "moveit" 2>/dev/null || true
    pkill -9 -f "franka" 2>/dev/null || true
    pkill -9 -f "rviz2" 2>/dev/null || true
    pkill -9 -f "ros2_control" 2>/dev/null || true
    
    # Step 10: Stop ROS2 daemon
    echo -e "${YELLOW}Step 10: Stopping ROS2 daemon...${NC}"
    ros2 daemon stop 2>/dev/null && echo "✓ ROS2 daemon stopped" || echo "✓ ROS2 daemon already stopped"
    
    echo -e "${GREEN}✓ Graceful shutdown completed successfully${NC}"
    echo -e "${BLUE}========================================${NC}"
}

# Function to cleanup on exit (enhanced)
cleanup() {
    echo -e "\n${YELLOW}Shutdown signal received...${NC}"
    graceful_shutdown
}

# Function to handle emergency stop
emergency_stop() {
    echo -e "\n${RED}EMERGENCY STOP INITIATED!${NC}"
    
    # Immediate robot stop
    echo -e "${RED}Stopping robot motion immediately...${NC}"
    timeout 2 ros2 service call /fr3_arm_controller/stop std_srvs/srv/Trigger 2>/dev/null || true
    
    # Kill all processes immediately
    echo -e "${RED}Stopping all processes...${NC}"
    pkill -9 -f "franka_recovery_daemon" 2>/dev/null || true
    pkill -9 -f "moveit" 2>/dev/null || true
    pkill -9 -f "franka" 2>/dev/null || true
    pkill -9 -f "ros2_control" 2>/dev/null || true
    pkill -9 -f "rviz2" 2>/dev/null || true
    
    # Force kill any hanging ROS service calls
    pkill -9 -f "ros2 service call" 2>/dev/null || true
    
    echo -e "${RED}Emergency stop completed${NC}"
    exit 1
}

# Function to monitor system
monitor_system() {
    echo -e "${BLUE}Monitoring system health...${NC}"
    echo -e "${GREEN}System is running! Use Ctrl+C for graceful shutdown${NC}"
    echo -e "${YELLOW}For emergency stop: kill -USR1 $$${NC}"
    echo ""
    
    while true; do
        sleep 5
        
        # Check if main processes are running
        if ! pgrep -f "robust_franka_control" > /dev/null; then
            echo -e "${RED}WARNING: Robust Franka Control not running${NC}"
        fi
        
        if [ "$ENABLE_HEALTH_MONITOR" = "true" ]; then
            if ! pgrep -f "system_health_monitor" > /dev/null; then
                echo -e "${RED}WARNING: System Health Monitor not running${NC}"
            fi
        fi
        
        # Check MoveIt processes
        if ! pgrep -f "moveit" > /dev/null; then
            echo -e "${RED}WARNING: MoveIt processes not found${NC}"
        fi
        
        # Check controller status
        if ros2 node list 2>/dev/null | grep -q "controller_manager"; then
            controller_status="✓"
        else
            controller_status="✗"
        fi
        
        # Check robot connection
        if ros2 topic list 2>/dev/null | grep -q "robot_state"; then
            robot_status="✓"
        else
            robot_status="✗"
        fi
        
        # Basic system info
        CPU=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1 | cut -d' ' -f2)
        MEM=$(free | grep Mem | awk '{printf("%.1f", $3/$2 * 100.0)}')
        PROCESSES=$(pgrep -f "franka\|moveit" | wc -l)
        
        echo -e "${GREEN}$(date '+%H:%M:%S')${NC} - CPU: ${CPU}% | Memory: ${MEM}% | Processes: ${PROCESSES} | Controller: ${controller_status} | Robot: ${robot_status}"
    done
}

# Function to verify system startup
verify_system_startup() {
    echo -e "${YELLOW}Verifying system startup...${NC}"
    
    # Wait for processes to start
    sleep 10
    
    # Check if key processes are running
    local errors=0
    
    if ! pgrep -f "robot_state_publisher" > /dev/null; then
        echo -e "${RED}✗ robot_state_publisher not running${NC}"
        ((errors++))
    else
        echo -e "${GREEN}✓ robot_state_publisher running${NC}"
    fi
    
    if ! pgrep -f "controller_manager" > /dev/null; then
        echo -e "${RED}✗ controller_manager not running${NC}"
        ((errors++))
    else
        echo -e "${GREEN}✓ controller_manager running${NC}"
    fi
    
    if [ "$ENABLE_HEALTH_MONITOR" = "true" ]; then
        if ! pgrep -f "system_health_monitor" > /dev/null; then
            echo -e "${RED}✗ system_health_monitor not running${NC}"
            ((errors++))
        else
            echo -e "${GREEN}✓ system_health_monitor running${NC}"
        fi
    fi
    
    if [ "$ENABLE_RVIZ" = "true" ]; then
        if ! pgrep -f "rviz2" > /dev/null; then
            echo -e "${YELLOW}⚠ rviz2 not running (may take longer to start)${NC}"
        else
            echo -e "${GREEN}✓ rviz2 running${NC}"
        fi
    fi
    
    if [ $errors -gt 0 ]; then
        echo -e "${YELLOW}⚠ Some components failed to start, but continuing...${NC}"
    else
        echo -e "${GREEN}✓ All critical components started successfully${NC}"
    fi
}

# Main execution starts here
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Robust Franka Production System${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Print configuration
echo -e "${YELLOW}Configuration:${NC}"
echo "  Robot IP: $ROBOT_IP"
echo "  Fake Hardware: $USE_FAKE_HARDWARE"
echo "  RViz: $ENABLE_RVIZ"
echo "  Health Monitor: $ENABLE_HEALTH_MONITOR"
echo "  Auto Restart: $AUTO_RESTART"
echo "  Log Level: $LOG_LEVEL"
echo "  Skip Build: $SKIP_BUILD"
echo ""

# Perform setup steps with error handling
echo -e "${BLUE}Starting setup process...${NC}"

if ! check_ros2_environment; then
    echo -e "${RED}Failed to verify ROS2 environment${NC}"
    exit 1
fi

if ! kill_existing_processes; then
    echo -e "${RED}Failed to kill existing processes${NC}"
    exit 1
fi

if ! perform_fresh_build; then
    echo -e "${RED}Failed to perform fresh build${NC}"
    exit 1
fi

if ! check_package_built; then
    echo -e "${RED}Failed to verify package build${NC}"
    exit 1
fi

if ! check_robot_connectivity; then
    echo -e "${YELLOW}Robot connectivity check had issues, but continuing...${NC}"
fi

if ! setup_environment; then
    echo -e "${RED}Failed to setup environment${NC}"
    exit 1
fi

echo -e "${GREEN}✓ All setup steps completed successfully${NC}"
echo ""

# Set up signal handling
trap cleanup SIGINT SIGTERM
trap emergency_stop SIGUSR1

# Launch the robust system
echo -e "${GREEN}Starting Robust Franka System...${NC}"

ros2 launch ros2_moveit_franka franka_robust_production.launch.py \
    robot_ip:="$ROBOT_IP" \
    use_fake_hardware:="$USE_FAKE_HARDWARE" \
    enable_rviz:="$ENABLE_RVIZ" \
    enable_health_monitor:="$ENABLE_HEALTH_MONITOR" \
    auto_restart:="$AUTO_RESTART" \
    log_level:="$LOG_LEVEL" &

LAUNCH_PID=$!

# Wait a bit for launch to start
sleep 3

# Check if launch started successfully
if ! kill -0 $LAUNCH_PID 2>/dev/null; then
    echo -e "${RED}ERROR: Failed to start the robust system${NC}"
    echo "Check the logs for more details:"
    echo "  ros2 log list"
    echo "  ros2 log view <node_name>"
    exit 1
fi

echo -e "${GREEN}✓ Robust Franka System launched successfully!${NC}"
echo ""

# Verify system components
verify_system_startup

# Monitor the system
monitor_system 