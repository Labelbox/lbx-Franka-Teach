#!/usr/bin/env python3
"""
Main System Integration for Labelbox Robotics VR Teleoperation

This script orchestrates the complete VR-based robot control system with:
- Health monitoring and diagnostics
- Graceful startup sequence
- Real-time status updates
- High-performance teleoperation at 45Hz
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import time
import threading
import os
import sys
import yaml
import signal
import asyncio  # Add asyncio import
from datetime import datetime
from typing import Dict, Optional, List
import numpy as np
import json

# Import system components with error handling
try:
    from .system_manager import SystemManager
except ImportError as e:
    print(f"Warning: Could not import SystemManager: {e}", file=sys.stderr)
    SystemManager = None

try:
    from .franka_controller import FrankaController
except ImportError as e:
    print(f"Warning: Could not import FrankaController: {e}", file=sys.stderr)
    FrankaController = None

# ROS2 messages
from std_msgs.msg import String, Bool, Header
from std_srvs.srv import Empty, Trigger
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from lbx_interfaces.msg import SystemStatus, VRControllerState, RecordingStatus

# ROS 2 Diagnostics
from diagnostic_updater import Updater, DiagnosticTask, DiagnosticStatus

# ANSI color codes for pretty printing
class Colors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    WARNING = '\033[93m'
    YELLOW = '\033[93m'
    FAIL = '\033[91m'
    RED = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class SystemDiagnosticTask(DiagnosticTask):
    """Simple diagnostic task that reports main_system display status"""
    
    def __init__(self, node):
        super().__init__("LBX Robotics Display Node")
        self.node = node
    
    def run(self, stat):
        """Run diagnostic check and report status"""
        # This node is just a display, always healthy if running
        stat.summary(DiagnosticStatus.OK, "Display node operational")
        
        # Add basic information
        stat.add("Node Type", "Display/Monitor")
        stat.add("Display Rate", f"{self.node.diagnostic_summary_interval}s")
        stat.add("Cameras Enabled", str(self.node.launch_params.get('enable_cameras', False)))
        stat.add("Recording Enabled", str(self.node.launch_params.get('enable_recording', False)))
        
        return stat


class LabelboxRoboticsSystem(Node):
    """Main system integration node for VR teleoperation"""
    
    def __init__(self, config_path: str, launch_params: Dict):
        super().__init__('labelbox_robotics_system')
        
        self.config_path = config_path
        self.launch_params = launch_params
        self.running = True
        
        # Load configuration
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # System state
        self.system_ready = False
        self.vr_healthy = False
        self.moveit_healthy = False
        self.cameras_healthy = True  # Default true if not enabled
        self.robot_healthy = False
        
        # System control state
        self.current_system_mode = "initializing"
        self.teleoperation_enabled = False
        self.recording_active = False
        self.calibration_valid = False
        
        # Service clients for system control
        self.callback_group = ReentrantCallbackGroup()
        self.calibrate_client = self.create_client(
            Trigger, '/system/start_calibration', callback_group=self.callback_group
        )
        self.start_teleop_client = self.create_client(
            Trigger, '/system/start_teleoperation', callback_group=self.callback_group
        )
        self.stop_system_client = self.create_client(
            Trigger, '/system/stop', callback_group=self.callback_group
        )
        
        # Subscribe to system state
        self.state_sub = self.create_subscription(
            String,
            '/system/state',
            self.system_state_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        )
        
        self.status_sub = self.create_subscription(
            SystemStatus,
            '/system/status',
            self.system_status_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        )
        
        # Subscribe to VR controller state for real-time button monitoring
        self.vr_state_sub = self.create_subscription(
            VRControllerState,
            '/vr/controller_state',
            self.vr_controller_state_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        # Simple diagnostic system - ROS2 best practice
        self.diagnostic_updater = Updater(self)
        self.diagnostic_updater.setHardwareID("lbx_robotics_system")
        self.diagnostic_updater.add(SystemDiagnosticTask(self))
        
        # Signal handling
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # Store latest states
        self.latest_system_status = None
        self.latest_vr_state = None
        self.latest_diagnostics = {}
        
        # Diagnostic collection and summary
        self.node_diagnostics = {}  # Store diagnostics from all nodes
        self.last_diagnostic_summary_time = 0
        self.diagnostic_summary_interval = 5.0  # Print summary every 5 seconds (changed from 30.0)
        
        # QoS profile for diagnostics subscription
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to diagnostics from all nodes (lightweight monitoring for health status)
        self.diagnostics_subscription = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            qos_profile
        )
    
    def system_state_callback(self, msg: String):
        """Handle system state updates"""
        self.current_system_mode = msg.data
        self.get_logger().info(f"System mode changed to: {msg.data}")
    
    def system_status_callback(self, msg: SystemStatus):
        """Handle detailed system status updates"""
        self.teleoperation_enabled = msg.teleoperation_enabled
        self.recording_active = msg.recording_active
        self.calibration_valid = (msg.calibration_mode == "valid" or self.current_system_mode == "teleoperation")
        self.latest_system_status = msg
    
    def vr_controller_state_callback(self, msg: VRControllerState):
        """Handle VR controller state updates for button monitoring"""
        self.latest_vr_state = msg
        
        # TODO: Add logic here to trigger actions based on VR button presses
        # For example:
        # - A/X button: Start/stop recording
        # - B/Y button: Mark recording as successful
        # - Menu button: Start calibration
        
    def signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        self.get_logger().info("\n🛑 Shutdown signal received...")
        self.running = False
        self.cleanup()
        sys.exit(0)
    
    def print_welcome_message(self):
        """Print ASCII welcome message and configuration summary"""
        # Clear screen
        os.system('clear' if os.name == 'posix' else 'cls')
        
        # ASCII Art
        print(f"{Colors.CYAN}{Colors.BOLD}")
        print("""
        ╔═════════════════════════════════════════════════════════════════════╗
        ║                                                                     ║
        ║  ██╗      █████╗ ██████╗ ███████╗██╗     ██████╗  ██████╗ ██╗  ██╗  ║
        ║  ██║     ██╔══██╗██╔══██╗██╔════╝██║     ██╔══██╗██╔═══██╗╚██╗██╔╝  ║
        ║  ██║     ███████║██████╔╝█████╗  ██║     ██████╔╝██║   ██║ ╚███╔╝   ║
        ║  ██║     ██╔══██║██╔══██╗██╔══╝  ██║     ██╔══██╗██║   ██║ ██╔██╗   ║
        ║  ███████╗██║  ██║██████╔╝███████╗███████╗██████╔╝╚██████╔╝██╔╝ ██╗  ║
        ║  ╚══════╝╚═╝  ╚═╝╚═════╝ ╚══════╝╚══════╝╚═════╝  ╚═════╝ ╚═╝  ╚═╝  ║
        ║                                                                     ║
        ║                    R O B O T I C S   S Y S T E M                    ║
        ║                                                                     ║
        ╚═════════════════════════════════════════════════════════════════════╝
        """)
        print(f"{Colors.ENDC}")
        
        # System description
        print(f"{Colors.GREEN}🤖 VR-Based Franka Robot Teleoperation System{Colors.ENDC}")
        print(f"   High-performance control at 45Hz with MoveIt integration\n")
        
        # Configuration summary
        print(f"{Colors.BLUE}{Colors.BOLD}📋 Configuration Summary:{Colors.ENDC}")
        print(f"   ├─ 🎮 VR Mode: {self._get_vr_mode()}")
        print(f"   ├─ 🎯 Control Rate: {Colors.BOLD}45Hz{Colors.ENDC} robot commands")
        print(f"   ├─ 📹 Cameras: {self._get_camera_status()}")
        print(f"   ├─ 💾 Recording: {self._get_recording_status()}")
        print(f"   ├─ 🔄 Hot Reload: {self._get_hotreload_status()}")
        print(f"   ├─ 🔍 Data Verification: {self._get_verification_status()}")
        print(f"   └─ 🤖 Robot IP: {Colors.BOLD}{self.config['robot']['robot_ip']}{Colors.ENDC}\n")
        
        # Force flush output to ensure it appears immediately
        sys.stdout.flush()
    
    def _get_vr_mode(self):
        """Get VR connection mode description"""
        if self.launch_params.get('vr_ip'):
            return f"{Colors.BOLD}Network{Colors.ENDC} ({self.launch_params['vr_ip']})"
        return f"{Colors.BOLD}USB{Colors.ENDC} (Direct connection)"
    
    def _get_camera_status(self):
        """Get camera configuration status"""
        if not self.launch_params.get('enable_cameras', False):
            return f"{Colors.WARNING}Disabled{Colors.ENDC}"
        
        camera_config = self.launch_params.get('camera_config', 'auto')
        if camera_config == 'auto':
            return f"{Colors.GREEN}Auto-discovery{Colors.ENDC}"
        return f"{Colors.GREEN}Enabled{Colors.ENDC} ({os.path.basename(camera_config)})"
    
    def _get_recording_status(self):
        """Get recording configuration status"""
        if self.config['recording']['enabled']:
            return f"{Colors.GREEN}Enabled{Colors.ENDC} (MCAP format)"
        return f"{Colors.WARNING}Disabled{Colors.ENDC}"
    
    def _get_hotreload_status(self):
        """Get hot reload status"""
        if self.launch_params.get('hot_reload', False):
            return f"{Colors.GREEN}Enabled{Colors.ENDC}"
        return f"{Colors.WARNING}Disabled{Colors.ENDC}"
    
    def _get_verification_status(self):
        """Get data verification status"""
        if self.launch_params.get('verify_data', False):
            return f"{Colors.GREEN}Enabled{Colors.ENDC}"
        return f"{Colors.WARNING}Disabled{Colors.ENDC}"
    
    async def wait_for_message(self, topic, msg_type, timeout=2.0):
        """Wait for a single message on a topic"""
        received_msg = None
        
        def msg_callback(msg):
            nonlocal received_msg
            received_msg = msg
        
        sub = self.create_subscription(msg_type, topic, msg_callback, 1)
        
        start_time = time.time()
        while received_msg is None and (time.time() - start_time) < timeout:
            await asyncio.sleep(0.1)
        
        self.destroy_subscription(sub)
        return received_msg
    
    def enter_calibration_mode(self):
        """Guide user through calibration"""
        print(f"\n{Colors.BLUE}{Colors.BOLD}🎯 Calibration Mode{Colors.ENDC}")
        print("─" * 50)
        
        # Call calibration service
        if self.calibrate_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Starting calibration sequence...")
            future = self.calibrate_client.call_async(Trigger.Request())
            # Don't block, let the system continue
        
        print(f"\n{Colors.CYAN}Forward Direction Calibration:{Colors.ENDC}")
        print("   1. Hold the {Colors.BOLD}joystick button{Colors.ENDC} on your VR controller")
        print("   2. Move the controller in your desired {Colors.BOLD}forward direction{Colors.ENDC}")
        print("   3. Move at least {Colors.BOLD}3mm{Colors.ENDC} to register the direction")
        print("   4. Release the joystick button to complete calibration")
        print(f"\n{Colors.CYAN}Origin Calibration:{Colors.ENDC}")
        print("   • Press and release the {Colors.BOLD}grip button{Colors.ENDC} to set origin")
        print("   • The current VR and robot positions will be synchronized")
        print(f"\n{Colors.GREEN}Ready for calibration. Waiting for joystick button...{Colors.ENDC}\n")
    
    def start_teleoperation(self):
        """Start main teleoperation mode"""
        print(f"\n{Colors.GREEN}{Colors.BOLD}🎮 Teleoperation Active!{Colors.ENDC}")
        print("─" * 50)
        
        # Display current system mode
        mode_color = Colors.GREEN if self.current_system_mode == "teleoperation" else Colors.YELLOW
        print(f"\n{Colors.BOLD}System Mode:{Colors.ENDC} {mode_color}{self.current_system_mode.upper()}{Colors.ENDC}")
        
        if self.vr_healthy:
            print(f"\n{Colors.CYAN}VR Controller Commands:{Colors.ENDC}")
            print("   • {Colors.BOLD}Grip{Colors.ENDC}: Hold to enable robot movement")
            print("   • {Colors.BOLD}Trigger{Colors.ENDC}: Control gripper (pull to close)")
            print("   • {Colors.BOLD}A/X{Colors.ENDC}: Start/stop recording")
            print("   • {Colors.BOLD}B/Y{Colors.ENDC}: Mark recording as successful")
            print("   • {Colors.BOLD}Menu{Colors.ENDC}: Start calibration sequence")
            print("   • {Colors.BOLD}Joystick Press{Colors.ENDC}: Emergency stop")
            
            print(f"\n{Colors.CYAN}System Control:{Colors.ENDC}")
            if self.current_system_mode == "idle":
                print("   • System is idle - use Menu button to start calibration")
            elif self.current_system_mode == "calibrating":
                print(f"   • Hold {Colors.BOLD}JOYSTICK{Colors.ENDC} and move to set forward direction")
                print(f"   • Press {Colors.BOLD}GRIP{Colors.ENDC} to set origin point")
            elif self.current_system_mode == "teleoperation":
                print(f"   • {Colors.BOLD}GRIP{Colors.ENDC}: Enable robot movement")
                print(f"   • {Colors.BOLD}TRIGGER{Colors.ENDC}: Control gripper")
                print(f"   • {Colors.BOLD}A/X{Colors.ENDC}: Start/stop recording")
                print(f"   • {Colors.BOLD}B/Y{Colors.ENDC}: Mark recording successful")
            
            # Show recording status
            if self.recording_active:
                print(f"\n{Colors.RED}● RECORDING IN PROGRESS{Colors.ENDC}")
            
        else:
            print(f"\n{Colors.WARNING}🎮 VR Graceful Fallback Mode:{Colors.ENDC}")
            print(f"   • VR controller not connected - using alternative control methods")
            print(f"   • {Colors.CYAN}All other features remain fully functional{Colors.ENDC}")
            print(f"   • Recording, cameras, and robot operation continue normally")
            print(f"\n{Colors.CYAN}Alternative Controls:{Colors.ENDC}")
            print("   • Use keyboard/mouse interfaces if available")
            print("   • Record data without VR input for testing")
            print("   • Monitor robot status and diagnostics")
            print(f"\n{Colors.BLUE}VR Reconnection:{Colors.ENDC}")
            print("   • System will automatically detect and reconnect VR when available")
            print("   • No restart required - hot-pluggable VR support")
        
        print(f"\n{Colors.WARNING}Press Ctrl+C to stop the system{Colors.ENDC}\n")
    
    def cleanup(self):
        """Clean up resources and shutdown ROS nodes"""
        print(f"\n\n{Colors.CYAN}Shutting down systems...{Colors.ENDC}")
        
        # First attempt graceful ROS node shutdown
        self._graceful_node_shutdown()
        
        # Then cleanup ROS2 processes
        self._cleanup_ros_processes()
        
        print(f"{Colors.GREEN}✅ Shutdown complete{Colors.ENDC}")
    
    def _graceful_node_shutdown(self):
        """Attempt graceful shutdown of ROS nodes"""
        print(f"{Colors.CYAN}Attempting graceful node shutdown...{Colors.ENDC}")
        
        try:
            # Stop robot motion first for safety
            if self.robot_healthy:
                print("  • Stopping robot motion...")
                stop_client = self.create_client(Trigger, '/system/stop')
                if stop_client.wait_for_service(timeout_sec=2.0):
                    future = stop_client.call_async(Trigger.Request())
                    # Don't wait for response, just send the command
                
            # Request system components to shutdown
            shutdown_topics = [
                '/system/shutdown',
                '/vr/shutdown', 
                '/cameras/shutdown',
                '/recording/shutdown'
            ]
            
            for topic in shutdown_topics:
                try:
                    shutdown_pub = self.create_publisher(Bool, topic, 1)
                    # Give it a moment to connect
                    time.sleep(0.1)
                    shutdown_msg = Bool()
                    shutdown_msg.data = True
                    shutdown_pub.publish(shutdown_msg)
                    print(f"  • Sent shutdown signal to {topic}")
                    self.destroy_publisher(shutdown_pub)
                except Exception as e:
                    # Silent continue - this is cleanup
                    pass
            
            # Give nodes time to shutdown gracefully
            print("  • Waiting for graceful shutdown...")
            time.sleep(2.0)
            
        except Exception as e:
            print(f"  • Warning: Error during graceful shutdown: {e}")
    
    def _cleanup_ros_processes(self):
        """Cleanup ROS processes if graceful shutdown fails"""
        print(f"{Colors.CYAN}Cleaning up ROS processes...{Colors.ENDC}")
        
        import subprocess
        import signal
        import os
        
        # List of process patterns to kill (in order of priority)
        process_patterns = [
            # VR and teleoperation processes (highest priority)
            "oculus_node",           # The oculus executable
            "oculus_reader",         # The oculus node name
            "lbx_input_oculus",      # The package process
            "vr_teleop_node",        # VR teleoperation node
            "system_manager",        # System manager
            
            # Robot control processes
            "robot_control_node",    # Robot control
            "system_orchestrator",   # System orchestrator  
            "system_monitor",        # System monitor
            "franka_hardware",       # Franka hardware interface
            "controller_manager",    # ROS2 controller manager
            
            # MoveIt processes
            "move_group",            # MoveIt move_group
            "moveit",                # Any moveit processes
            
            # Camera processes
            "camera_node",           # Camera node
            "vision_camera_node",    # Vision camera node
            "realsense",             # RealSense processes
            
            # Data recording
            "mcap_recorder_node",    # MCAP recorder
            "data_recorder",         # Data recorder
            
            # Visualization
            "rviz2",                 # RViz
            
            # General ROS processes
            "ros2 run",              # ROS2 run commands
            "ros2 launch",           # ROS2 launch commands
        ]
        
        killed_processes = []
        
        for pattern in process_patterns:
            try:
                # Use pkill to find and terminate processes matching the pattern
                result = subprocess.run(
                    ['pkill', '-f', pattern],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                if result.returncode == 0:
                    killed_processes.append(pattern)
                    print(f"  ✓ Stopped processes matching: {pattern}")
                else:
                    # Check if any processes exist
                    check_result = subprocess.run(
                        ['pgrep', '-f', pattern],
                        capture_output=True,
                        text=True,
                        timeout=1
                    )
                    if check_result.returncode != 0:
                        pass  # No processes found, which is good
                        
            except subprocess.TimeoutExpired:
                print(f"  ⚠ Timeout killing: {pattern}")
            except Exception as e:
                # Silent continue - this is cleanup
                pass
        
        # Wait for processes to terminate
        if killed_processes:
            print("  • Waiting for processes to terminate...")
            time.sleep(3.0)
        
        # Force kill any stubborn processes
        stubborn_patterns = ["oculus_node", "moveit", "franka", "rviz2"]
        for pattern in stubborn_patterns:
            try:
                subprocess.run(
                    ['pkill', '-9', '-f', pattern],
                    capture_output=True,
                    timeout=1
                )
            except:
                pass
        
        # Stop ROS2 daemon last
        try:
            print("  • Stopping ROS2 daemon...")
            subprocess.run(['ros2', 'daemon', 'stop'], capture_output=True, timeout=3)
        except:
            pass
        
        if killed_processes:
            print(f"  ✓ Cleaned up {len(killed_processes)} process types")
        else:
            print("  ✓ No processes needed cleanup")
    
    def _get_bool_param(self, param_name: str, default: bool = False) -> bool:
        """Helper to safely convert launch parameters to boolean values"""
        param_value = self.launch_params.get(param_name, default)
        if isinstance(param_value, bool):
            return param_value
        else:
            return str(param_value).lower() in ('true', '1', 'yes', 'on')
    
    async def run(self):
        """Main run loop with passive monitoring approach"""
        try:
            # Welcome message - always show first
            self.print_welcome_message()
            
            # Initial status message
            print(f"\n{Colors.GREEN}{Colors.BOLD}🚀 System Starting Up{Colors.ENDC}")
            print("─" * 50)
            print(f"\n{Colors.CYAN}System components will initialize in sequence:{Colors.ENDC}")
            print(f"   1️⃣  Cameras (if enabled) - initializing first")
            print(f"   2️⃣  Robot control and MoveIt - starting after camera init")
            print(f"   3️⃣  VR input and teleoperation nodes")
            print(f"   4️⃣  Data recording (if enabled)")
            print(f"\n{Colors.YELLOW}Components will appear as they become ready...{Colors.ENDC}")
            
            # Start passive monitoring
            print(f"\n{Colors.CYAN}📊 System monitoring active{Colors.ENDC}")
            print(f"   • Components will be detected automatically")
            print(f"   • Status updates every 5 seconds")
            print(f"   • Detailed diagnostics: ros2 topic echo /diagnostics")
            print(f"\n{Colors.WARNING}Press Ctrl+C to stop the system{Colors.ENDC}\n")
            
            # Force flush output
            sys.stdout.flush()
            
            # Initialize monitoring state
            self._last_health_check = time.time()
            self.last_diagnostic_summary_time = time.time()
            self._last_mode_display = ""
            self._startup_phase = True
            self._startup_start_time = time.time()
            
            # Main monitoring loop
            while self.running:
                try:
                    # Update diagnostics
                    self.diagnostic_updater.update()
                    
                    # During startup phase (first 30 seconds), check component availability more frequently
                    if self._startup_phase and (time.time() - self._startup_start_time) < 30.0:
                        # Quick component detection during startup
                        await self._detect_components()
                        
                        # Exit startup phase once core components are ready
                        if self.robot_healthy and self.moveit_healthy:
                            self._startup_phase = False
                            print(f"\n{Colors.GREEN}✅ Core system components initialized!{Colors.ENDC}\n")
                    
                    # Display mode change notifications
                    if self.current_system_mode != self._last_mode_display:
                        self._last_mode_display = self.current_system_mode
                        
                        # Clear and show mode change
                        print(f"\n{Colors.BLUE}{'=' * 60}{Colors.ENDC}")
                        print(f"🔄 {Colors.BOLD}SYSTEM MODE CHANGED: {Colors.GREEN}{self.current_system_mode.upper()}{Colors.ENDC}")
                        print(f"{Colors.BLUE}{'=' * 60}{Colors.ENDC}\n")
                        
                        # Show mode-specific guidance
                        if self.current_system_mode == "calibrating":
                            self.enter_calibration_mode()
                        elif self.current_system_mode == "teleoperation":
                            self.start_teleoperation()
                        elif self.current_system_mode == "idle":
                            if self.vr_healthy:
                                print(f"{Colors.CYAN}System is idle. Press MENU button on VR controller to start calibration.{Colors.ENDC}")
                            else:
                                print(f"{Colors.CYAN}System is idle. Waiting for VR controller connection...{Colors.ENDC}")
                    
                    # Periodic health checks (every 10 seconds after startup)
                    if time.time() - self._last_health_check > 10.0:
                        await self._detect_components()
                        self._last_health_check = time.time()
                    
                    # Short sleep for responsive monitoring
                    await asyncio.sleep(1.0)
                    
                except Exception as e:
                    self.get_logger().error(f"Error in main loop: {e}")
                    await asyncio.sleep(1.0)
                    
        except KeyboardInterrupt:
            print("\n\nKeyboard interrupt received")
        except Exception as e:
            self.get_logger().error(f"Critical error in main system: {e}")
            import traceback
            traceback.print_exc()
    
    async def _detect_components(self):
        """Passively detect component availability without blocking"""
        try:
            # Check VR if not healthy
            if not self.vr_healthy:
                vr_msg = await self.wait_for_message('/vr/right_controller/pose', PoseStamped, timeout=0.5)
                if vr_msg is None:
                    vr_msg = await self.wait_for_message('/vr/left_controller/pose', PoseStamped, timeout=0.5)
                if vr_msg:
                    self.vr_healthy = True
                    self.get_logger().info("🎮 VR controller detected!")
                    print(f"\n{Colors.GREEN}✅ VR Controller connected!{Colors.ENDC}")
                    if self.current_system_mode == "idle":
                        print(f"{Colors.CYAN}Press MENU button to start calibration{Colors.ENDC}\n")
            
            # Check robot connection if not healthy
            if not self.robot_healthy:
                joint_msg = await self.wait_for_message('/joint_states', JointState, timeout=0.5)
                if joint_msg and len(joint_msg.position) >= 7:
                    self.robot_healthy = True
                    self.get_logger().info("🤖 Robot connected!")
                    print(f"\n{Colors.GREEN}✅ Robot connection established!{Colors.ENDC}\n")
            
            # Check MoveIt services if not healthy
            if not self.moveit_healthy:
                # Just check one key service as indicator
                ik_client = self.create_client(Trigger, '/compute_ik')  # Dummy service type
                if ik_client.service_is_ready():
                    self.moveit_healthy = True
                    self.get_logger().info("🔧 MoveIt services ready!")
                    print(f"\n{Colors.GREEN}✅ MoveIt services available!{Colors.ENDC}\n")
                self.destroy_client(ik_client)
                
        except Exception as e:
            # Silent failure - this is just detection, not critical
            pass

    def diagnostics_callback(self, msg):
        """Lightweight callback for basic health status tracking"""
        for status in msg.status:
            # Extract basic system health from main_system diagnostics only
            if 'main_system' in status.name:
                for item in status.values:
                    if item.key == 'vr_connected':
                        self.vr_healthy = (item.value == 'True')
                    elif item.key == 'robot_connected':
                        self.robot_healthy = (item.value == 'True')
                    elif item.key == 'cameras_healthy':
                        self.cameras_healthy = (item.value == 'True')
                    elif item.key == 'system_state':
                        self.current_system_mode = item.value
                    elif item.key == 'recording_active':
                        self.recording_active = (item.value == 'True')
                    elif item.key == 'moveit_ready':
                        self.moveit_healthy = (item.value == 'True')
    
    def _diagnostic_level_to_string(self, level):
        """Convert diagnostic level to human-readable string"""
        level_map = {
            DiagnosticStatus.OK: "OK",
            DiagnosticStatus.WARN: "WARNING", 
            DiagnosticStatus.ERROR: "ERROR",
            DiagnosticStatus.STALE: "STALE"
        }
        return level_map.get(level, f"UNKNOWN({level})")


async def async_main():
    """Async main entry point"""
    # Initialize ROS2
    try:
        rclpy.init()
    except Exception as e:
        print(f"Failed to initialize ROS2: {e}", file=sys.stderr)
        return
    
    # Parse launch parameters from environment and ROS parameters
    launch_params = {
        'vr_ip': os.environ.get('VR_IP', ''),
        'enable_cameras': os.environ.get('ENABLE_CAMERAS', 'false').lower() == 'true',
        'camera_config': os.environ.get('CAMERA_CONFIG', 'auto'),
        'hot_reload': os.environ.get('HOT_RELOAD', 'false').lower() == 'true',
        'verify_data': os.environ.get('VERIFY_DATA', 'false').lower() == 'true',
        'use_fake_hardware': os.environ.get('USE_FAKE_HARDWARE', 'false').lower() == 'true',
        'robot_ip': os.environ.get('ROBOT_IP', '192.168.1.59'),
        'enable_rviz': os.environ.get('ENABLE_RVIZ', 'true').lower() == 'true',
        'enable_recording': os.environ.get('ENABLE_RECORDING', 'true').lower() == 'true',
    }
    
    # Configuration path - fix to use correct relative path
    config_path = None
    try:
        from ament_index_python.packages import get_package_share_directory
        config_path = os.path.join(
            get_package_share_directory('lbx_franka_control'),
            'config',
            'franka_vr_control_config.yaml'
        )
    except Exception as e:
        print(f"Warning: Could not get package share directory: {e}", file=sys.stderr)
        
    if not config_path or not os.path.exists(config_path):
        # Fallback for development/source builds
        config_path = os.path.join(
            os.path.dirname(__file__),
            '../../../../configs/control/franka_vr_control_config.yaml'
        )
    
    # Additional fallback to workspace config if installed config not found
    if not os.path.exists(config_path):
        workspace_config = os.path.join(
            os.path.dirname(__file__),
            '../../../../../../../configs/control/franka_vr_control_config.yaml'
        )
        if os.path.exists(workspace_config):
            config_path = workspace_config
        else:
            # Create a minimal default config if none found
            config_path = '/tmp/default_franka_config.yaml'
            default_config = {
                'robot': {'robot_ip': '192.168.1.59'},
                'recording': {'enabled': True}
            }
            try:
                import yaml
                with open(config_path, 'w') as f:
                    yaml.dump(default_config, f)
            except Exception as e:
                print(f"Warning: Could not create default config: {e}", file=sys.stderr)
    
    # Create main system node
    system = None
    executor = None
    try:
        system = LabelboxRoboticsSystem(config_path, launch_params)
        
        # Update config with launch parameters
        if launch_params.get('robot_ip'):
            system.config['robot']['robot_ip'] = launch_params['robot_ip']
        
        # Create executor
        executor = MultiThreadedExecutor()
        executor.add_node(system)
        
        # Run system with proper executor integration
        # Start the executor in a separate thread
        import threading
        import concurrent.futures
        
        # Create a future for the system run task
        loop = asyncio.get_event_loop()
        
        # Run executor in a separate thread
        def spin_executor():
            try:
                executor.spin()
            except Exception as e:
                print(f"Executor error: {e}")
        
        executor_thread = threading.Thread(target=spin_executor, daemon=True)
        executor_thread.start()
        
        # Give the executor a moment to start
        await asyncio.sleep(0.1)
        
        # Run the main system loop
        await system.run()
        
    except KeyboardInterrupt:
        print("\n\nKeyboard interrupt received")
    except Exception as e:
        print(f"Error in main system: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Shutdown in the correct order
        if system:
            system.running = False
            try:
                system.cleanup()
            except Exception as e:
                print(f"Error during system cleanup: {e}")
        
        if executor:
            try:
                executor.shutdown(timeout_sec=5.0)
            except Exception as e:
                print(f"Error shutting down executor: {e}")
        
        if system:
            try:
                system.destroy_node()
            except Exception as e:
                print(f"Error destroying system node: {e}")
        
        # Only shutdown RCL if it's still initialized
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"Error during RCL shutdown: {e}")


def main():
    """Synchronous main entry point for ROS2 entry_points"""
    import asyncio
    try:
        asyncio.run(async_main())
    except KeyboardInterrupt:
        print("\nShutdown requested")
    except Exception as e:
        print(f"Error in main system: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Final cleanup
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass  # Ignore errors during final cleanup


if __name__ == '__main__':
    main() 