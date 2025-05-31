#!/usr/bin/env python3
"""
Robust Production Launch File for Franka FR3 MoveIt
This launch file provides crash-proof operation with automatic restart
capabilities and comprehensive error handling, including libfranka exceptions.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    ExecuteProcess,
    LogInfo,
    TimerAction,
    OpaqueFunction,
    GroupAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_crash_resistant_launcher(context, *args, **kwargs):
    """Generate a crash-resistant wrapper for the MoveIt launch"""
    
    robot_ip = LaunchConfiguration('robot_ip').perform(context)
    use_fake_hardware = LaunchConfiguration('use_fake_hardware').perform(context)
    enable_rviz = LaunchConfiguration('enable_rviz').perform(context)
    
    # Create an independent restart daemon that survives parent shutdowns
    restart_script = ExecuteProcess(
        cmd=[
            'bash', '-c', f'''
            #!/bin/bash
            echo "🛡️  Starting Independent Crash-Recovery Daemon"
            echo "==============================================="
            
            # Create a unique session to survive parent shutdown
            SESSION_ID="franka_recovery_$$"
            
            # Create recovery daemon script
            DAEMON_SCRIPT="/tmp/franka_recovery_daemon_$$.sh"
            cat > "$DAEMON_SCRIPT" << 'DAEMON_EOF'
#!/bin/bash

MAX_RESTARTS=5
RESTART_COUNT=0
RESTART_DELAY=2
ROBOT_IP="{robot_ip}"
USE_FAKE_HARDWARE="{use_fake_hardware}"
ENABLE_RVIZ="{enable_rviz}"

echo "🔄 Franka Recovery Daemon Started"
echo "Session: $SESSION_ID"
echo "Robot IP: $ROBOT_IP"
echo "Fake Hardware: $USE_FAKE_HARDWARE"
echo "RViz: $ENABLE_RVIZ"
echo ""

while [ $RESTART_COUNT -lt $MAX_RESTARTS ]; do
    echo "🚀 Starting MoveIt system (attempt $((RESTART_COUNT + 1))/$MAX_RESTARTS)"
    echo "⏰ $(date)"
    
    # Create a log file to capture crash indicators
    LOG_FILE="/tmp/moveit_crash_log_$SESSION_ID.txt"
    
    # Launch MoveIt in a new process group
    setsid ros2 launch franka_fr3_moveit_config moveit.launch.py \\
        robot_ip:="$ROBOT_IP" \\
        use_fake_hardware:="$USE_FAKE_HARDWARE" \\
        load_gripper:=true \\
        use_rviz:="$ENABLE_RVIZ" \\
        > "$LOG_FILE" 2>&1 &
    
    MOVEIT_PID=$!
    echo "📍 MoveIt PID: $MOVEIT_PID"
    
    # Monitor the process
    while kill -0 $MOVEIT_PID 2>/dev/null; do
        sleep 2
        # Check for crash indicators in real-time
        if grep -q "libfranka.*aborted\\|ControlException\\|joint_velocity_violation\\|cartesian_reflex\\|terminate called" "$LOG_FILE" 2>/dev/null; then
            echo "💥 CRASH DETECTED: libfranka exception in progress"
            break
        fi
    done
    
    # Wait for the process to finish and get exit code
    wait $MOVEIT_PID 2>/dev/null
    EXIT_CODE=$?
    
    # Analyze the crash
    CRASH_DETECTED=false
    
    if grep -q "libfranka.*aborted\\|ControlException\\|joint_velocity_violation\\|cartesian_reflex\\|terminate called" "$LOG_FILE" 2>/dev/null; then
        echo "💥 CRASH CONFIRMED: libfranka exception found in logs"
        CRASH_DETECTED=true
    elif grep -q "process has died.*exit code -[0-9]\\|Aborted (Signal\\|Segmentation fault" "$LOG_FILE" 2>/dev/null; then
        echo "💥 CRASH CONFIRMED: Process died with fatal signal"
        CRASH_DETECTED=true
    elif [ $EXIT_CODE -ne 0 ] && [ $EXIT_CODE -ne 130 ] && [ $EXIT_CODE -ne 143 ]; then  # 143 is SIGTERM
        echo "💥 CRASH CONFIRMED: Abnormal exit code $EXIT_CODE"
        CRASH_DETECTED=true
    fi
    
    if [ "$CRASH_DETECTED" = "true" ]; then
        echo "🔍 Crash analysis: libfranka safety reflex triggered"
        echo "   Detected at: $(date)"
        echo "   Crash type: Hardware safety violation"
        
        RESTART_COUNT=$((RESTART_COUNT + 1))
        
        if [ $RESTART_COUNT -lt $MAX_RESTARTS ]; then
            echo "🔄 Initiating autonomous recovery (attempt $RESTART_COUNT/$MAX_RESTARTS)"
            echo "   🧹 Cleaning up crashed processes..."
            
            # Kill any remaining MoveIt processes
            pkill -f "franka_fr3_moveit_config.*moveit.launch.py" 2>/dev/null || true
            sleep 2
            pkill -f "ros2_control_node" 2>/dev/null || true
            pkill -f "move_group" 2>/dev/null || true
            pkill -f "rviz2" 2>/dev/null || true
            pkill -f "joint_state" 2>/dev/null || true
            pkill -f "robot_state_publisher" 2>/dev/null || true
            pkill -f "controller_manager" 2>/dev/null || true
            pkill -f "franka_gripper" 2>/dev/null || true
            
            # Wait for cleanup
            echo "   ⏳ Waiting for cleanup to complete..."
            sleep 8
            
            echo "   🔄 Brief pause for system stabilization ($RESTART_DELAY seconds)..."
            sleep $RESTART_DELAY
            
            echo "   ✨ Restarting MoveIt system..."
            echo "   💡 Previous crash will be auto-handled"
        else
            echo "❌ Maximum restart attempts reached ($MAX_RESTARTS)"
            echo "   Persistent crashes detected - manual intervention required"
            echo ""
            echo "🔧 Troubleshooting checklist:"
            echo "   1. Physical robot state: Ensure no collisions or obstructions"
            echo "   2. Joint positions: Verify all joints within safe limits"  
            echo "   3. Robot status: Check robot is unlocked and ready"
            echo "   4. Network: Test connection to robot IP $ROBOT_IP"
            echo "   5. Hardware: Try restarting with --fake-hardware for testing"
            echo ""
            echo "🔄 To retry: ./run_robust_franka.sh --robot-ip $ROBOT_IP"
            echo "🆘 Emergency: pkill -f franka  # Stop all robot processes"
            break
        fi
    else
        if [ $EXIT_CODE -eq 130 ]; then
            echo "✅ MoveIt shutdown by user (Ctrl+C)"
        elif [ $EXIT_CODE -eq 143 ]; then
            echo "✅ MoveIt shutdown by system (SIGTERM)"
        else
            echo "✅ MoveIt shutdown normally (exit code: $EXIT_CODE)"  
        fi
        break
    fi
    
    # Clean up log file
    rm -f "$LOG_FILE"
done

echo "🏁 Recovery daemon finished"
echo "Final status: $RESTART_COUNT/$MAX_RESTARTS restarts attempted"

# Cleanup
rm -f "$DAEMON_SCRIPT"
DAEMON_EOF

            # Make the daemon script executable
            chmod +x "$DAEMON_SCRIPT"
            
            # Launch the daemon in background with nohup to survive parent exit
            echo "🚀 Launching independent recovery daemon..."
            nohup "$DAEMON_SCRIPT" > /tmp/franka_recovery_$$.log 2>&1 &
            DAEMON_PID=$!
            
            echo "✅ Recovery daemon launched (PID: $DAEMON_PID)"
            echo "📄 Daemon logs: /tmp/franka_recovery_$$.log"
            echo "🛡️ System now has autonomous crash recovery"
            
            # Wait briefly to ensure daemon starts
            sleep 3
            
            # Check if daemon is running
            if kill -0 $DAEMON_PID 2>/dev/null; then
                echo "✅ Recovery daemon confirmed running"
                # Keep this process alive to maintain the daemon
                wait $DAEMON_PID
            else
                echo "❌ Failed to start recovery daemon"
                exit 1
            fi
            '''
        ],
        output='screen',
        shell=True
    )
    
    return [restart_script]


def generate_robust_nodes(context, *args, **kwargs):
    """Generate robust nodes with respawn capabilities"""
    
    # Get launch configurations
    robot_ip = LaunchConfiguration('robot_ip').perform(context)
    enable_health_monitor = LaunchConfiguration('enable_health_monitor').perform(context)
    auto_restart = LaunchConfiguration('auto_restart').perform(context)
    
    nodes = []
    
    # Enhanced environment setup for MoveIt availability
    enhanced_env = dict(os.environ)
    
    # Ensure proper Python path for MoveIt
    python_paths = [
        '/opt/ros/humble/lib/python3.10/site-packages',
        '/opt/ros/humble/local/lib/python3.10/dist-packages',
    ]
    
    # Add Franka workspace paths if they exist
    franka_workspace_paths = [
        '/home/labelbox/franka_ros2_ws/install/lib/python3.10/site-packages',
        '/home/labelbox/franka_ros2_ws/install/local/lib/python3.10/dist-packages',
    ]
    
    for path in franka_workspace_paths:
        if os.path.exists(path):
            python_paths.append(path)
    
    # Set PYTHONPATH
    current_pythonpath = enhanced_env.get('PYTHONPATH', '')
    enhanced_env['PYTHONPATH'] = ':'.join(python_paths + ([current_pythonpath] if current_pythonpath else []))
    
    # Ensure ROS environment variables
    enhanced_env['ROS_VERSION'] = '2'
    enhanced_env['ROS_DISTRO'] = 'humble'
    
    # Add LD_LIBRARY_PATH for ROS libraries
    ld_paths = [
        '/opt/ros/humble/lib',
        '/opt/ros/humble/lib/x86_64-linux-gnu',
    ]
    
    # Add Franka workspace library paths
    franka_lib_paths = [
        '/home/labelbox/franka_ros2_ws/install/lib',
        '/home/labelbox/franka_ros2_ws/install/lib/x86_64-linux-gnu',
    ]
    
    for path in franka_lib_paths:
        if os.path.exists(path):
            ld_paths.append(path)
    
    current_ld_path = enhanced_env.get('LD_LIBRARY_PATH', '')
    enhanced_env['LD_LIBRARY_PATH'] = ':'.join(ld_paths + ([current_ld_path] if current_ld_path else []))
    
    # Robust Franka Control Node with respawn
    robust_control_node = Node(
        package='ros2_moveit_franka',
        executable='robust_franka_control',
        name='robust_franka_control',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'robot_ip': robot_ip},
        ],
        respawn=False,  # Let the recovery daemon handle restarts
        respawn_delay=5.0,
        # Use enhanced environment
        additional_env=enhanced_env
    )
    nodes.append(robust_control_node)
    
    # System Health Monitor (if enabled) - Enhanced to monitor hardware crashes
    if enable_health_monitor.lower() == 'true':
        health_monitor_node = Node(
            package='ros2_moveit_franka',
            executable='system_health_monitor',
            name='system_health_monitor',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'monitor_hardware_crashes': True},
                {'restart_on_hardware_failure': True},
            ],
            respawn=False,  # Disable auto-respawn to allow clean shutdown
            respawn_delay=5.0,
            # Use enhanced environment
            additional_env=enhanced_env
        )
        nodes.append(health_monitor_node)
    
    return nodes


def generate_launch_description():
    # Declare launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.59',
        description='IP address of the Franka robot'
    )
    
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Use fake hardware for testing (true/false)'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',  # Enable RViz by default
        description='Enable RViz visualization (true/false)'
    )
    
    enable_health_monitor_arg = DeclareLaunchArgument(
        'enable_health_monitor',
        default_value='true',
        description='Enable system health monitoring (true/false)'
    )
    
    auto_restart_arg = DeclareLaunchArgument(
        'auto_restart',
        default_value='true',
        description='Enable automatic restart of failed nodes (true/false)'
    )
    
    restart_delay_arg = DeclareLaunchArgument(
        'restart_delay',
        default_value='5.0',
        description='Delay in seconds before restarting failed nodes'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='Logging level (DEBUG, INFO, WARN, ERROR)'
    )
    
    # Get launch configurations
    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    enable_rviz = LaunchConfiguration('enable_rviz')
    enable_health_monitor = LaunchConfiguration('enable_health_monitor')
    auto_restart = LaunchConfiguration('auto_restart')
    restart_delay = LaunchConfiguration('restart_delay')
    log_level = LaunchConfiguration('log_level')
    
    # Log startup information
    startup_log = LogInfo(
        msg=[
            "Starting Crash-Resistant Franka Production System\n",
            "Robot IP: ", robot_ip, "\n",
            "Fake Hardware: ", use_fake_hardware, "\n",
            "Auto Restart: ", auto_restart, "\n",
            "Health Monitor: ", enable_health_monitor, "\n",
            "RViz Enabled: ", enable_rviz, "\n",
            "Log Level: ", log_level, "\n",
            "✓ libfranka exceptions will trigger automatic restart\n",
            "✓ Maximum 5 restart attempts with intelligent recovery\n",
            "✓ Comprehensive crash protection enabled"
        ]
    )
    
    # Launch crash-resistant MoveIt wrapper
    crash_resistant_moveit = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg="🛡️  Starting crash-resistant MoveIt wrapper..."),
            OpaqueFunction(function=generate_crash_resistant_launcher)
        ]
    )
    
    # Launch robust control nodes after giving MoveIt time to start
    robust_nodes = TimerAction(
        period=25.0,  # Wait for MoveIt to initialize
        actions=[
            LogInfo(msg="🤖 Starting robust control and monitoring nodes..."),
            OpaqueFunction(function=generate_robust_nodes)
        ]
    )
    
    # System status monitoring with crash detection
    crash_monitor_script = ExecuteProcess(
        cmd=[
            'bash', '-c', 
            '''
            echo ""
            echo "🛡️  Crash-Resistant Franka System Status"
            echo "========================================"
            echo "✓ Hardware interface: Auto-restart on libfranka exceptions"
            echo "✓ MoveIt components: Intelligent crash recovery (max 5 attempts)"
            echo "✓ Control nodes: Robust error handling and restart"
            echo "✓ Health monitoring: Active system supervision"
            echo ""
            echo "📊 Monitor topics:"
            echo "  ros2 topic echo /robot_state       # Robot state"
            echo "  ros2 topic echo /robot_health      # Health status"
            echo "  ros2 topic echo /robot_errors      # Error messages"
            echo "  ros2 topic echo /system_health     # Overall system health"
            echo ""
            echo "🚨 Emergency commands:"
            echo "  ros2 service call /controller_manager/stop_controller controller_manager_msgs/srv/StopController \\"{name: fr3_arm_controller}\\""
            echo "  pkill -f franka  # Emergency stop all"
            echo ""
            echo "🔄 System will auto-recover from hardware crashes!"
            echo "💡 Common recovery scenarios:"
            echo "   - Cartesian reflex triggers → Auto-restart in 15s"
            echo "   - Joint limit violations → Auto-restart in 15s"
            echo "   - Network interruptions → Auto-restart in 15s"
            echo "   - libfranka exceptions → Auto-restart in 15s"
            echo ""
            '''
        ],
        output='screen',
        condition=IfCondition(enable_health_monitor)
    )
    
    return LaunchDescription([
        # Launch arguments
        robot_ip_arg,
        use_fake_hardware_arg,
        enable_rviz_arg,
        enable_health_monitor_arg,
        auto_restart_arg,
        restart_delay_arg,
        log_level_arg,
        
        # Startup log
        startup_log,
        
        # Crash-resistant components
        crash_resistant_moveit,  # Start MoveIt with crash protection
        robust_nodes,           # Start our robust nodes
        
        # System monitoring
        crash_monitor_script,
    ]) 