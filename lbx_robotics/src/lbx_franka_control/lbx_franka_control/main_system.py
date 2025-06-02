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

# Import system components
from .system_manager import SystemManager
from .franka_controller import FrankaController

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
        
        # Subscribe to diagnostics from all nodes
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
        """Clean up resources"""
        print(f"\n\n{Colors.CYAN}Shutting down systems...{Colors.ENDC}")
        
        # Cleanup will be handled by ROS2 shutdown
    
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
        """Callback for receiving diagnostics from all nodes"""
        for status in msg.status:
            node_name = status.name
            if node_name not in self.node_diagnostics:
                self.node_diagnostics[node_name] = {}
            
            # Store the summary/status message
            self.node_diagnostics[node_name]['Summary'] = status.message
            self.node_diagnostics[node_name]['Level'] = self._diagnostic_level_to_string(status.level)
            
            # Store individual key-value pairs
            for item in status.values:
                self.node_diagnostics[node_name][item.key] = item.value
            
            # Extract system health from main_system diagnostics
            if 'main_system' in node_name:
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
                    # Check if MoveIt is ready based on the 'moveit_ready' key from system_monitor
                    elif item.key == 'moveit_ready': # Assuming system_monitor adds this key
                        self.moveit_healthy = (item.value == 'True')

                # Fallback for moveit_healthy based on message if 'moveit_ready' key is not present
                # This part should ideally be less reliant on string parsing if a dedicated key is available
                if not any(item.key == 'moveit_ready' for item in status.values):
                    if 'fully operational' in status.message.lower() or \
                       ('vr fallback' in status.message.lower() and self.robot_healthy): # VR fallback implies MoveIt is OK if robot is OK
                        self.moveit_healthy = True
                    elif 'moveit pending' in status.message.lower() or not self.robot_healthy:
                        self.moveit_healthy = False
                    # else, keep previous state or default to False if unsure
        
        # Check if it's time to print a diagnostic summary
        current_time = time.time()
        if current_time - self.last_diagnostic_summary_time > self.diagnostic_summary_interval:
            self.last_diagnostic_summary_time = current_time
            self.print_diagnostic_summary()
    
    def _diagnostic_level_to_string(self, level):
        """Convert diagnostic level to human-readable string"""
        level_map = {
            DiagnosticStatus.OK: "OK",
            DiagnosticStatus.WARN: "WARNING", 
            DiagnosticStatus.ERROR: "ERROR",
            DiagnosticStatus.STALE: "STALE"
        }
        return level_map.get(level, f"UNKNOWN({level})")
    
    def print_diagnostic_summary(self):
        """Print a beautiful system health summary from diagnostics"""
        
        # Clear previous output for a clean look
        print("\n" * 1)
        
        # Main header with simple line splitters
        print(f"{Colors.CYAN}{'=' * 80}{Colors.ENDC}")
        print(f"{Colors.BOLD}{Colors.GREEN}                     📊 SYSTEM HEALTH DIAGNOSTICS 📊{Colors.ENDC}")
        print(f"{Colors.CYAN}{'=' * 80}{Colors.ENDC}")
        
        # Timestamp
        current_time = datetime.now().strftime("%H:%M:%S")
        print(f"🕐 Time: {Colors.BOLD}{current_time}{Colors.ENDC}")
        print(f"{Colors.CYAN}{'-' * 80}{Colors.ENDC}")
        
        # Extract information from diagnostics
        system_diagnostics = {}
        vr_performance = {}
        control_performance = {}
        camera_diagnostics_data = {} # Renamed to avoid conflict with the other camera_diagnostics
        resource_info = {}
        
        # Consolidate oculus_reader specific diagnostics for VR section
        oculus_node_diagnostics = None

        for node_name, diagnostics_content in self.node_diagnostics.items():
            if 'main_system' in node_name: # Should be specific like self.get_name()
                system_diagnostics = diagnostics_content
                # System state updates are handled in diagnostics_callback directly
            
            elif node_name == 'oculus_reader:vr_controller': # Specific key for VR performance
                vr_performance = diagnostics_content # This now holds all KVs for this diagnostic source
            elif node_name.startswith('oculus_reader') and not oculus_node_diagnostics: # General oculus_reader status
                 oculus_node_diagnostics = diagnostics_content

            elif node_name.startswith("Camera ") and " (Unavailable)" not in node_name: # Match active cameras by task name
                # Extracts <camera_id> from "Camera <camera_id>"
                cam_id_key = node_name.split("Camera ", 1)[1]
                camera_diagnostics_data[cam_id_key] = diagnostics_content
            elif node_name.startswith("Camera ") and " (Unavailable)" in node_name:
                cam_id_key = node_name.split("Camera ", 1)[1].replace(" (Unavailable)", "")
                # Optionally store unavailable camera diagnostics if needed for detailed display beyond status
                # For now, the status is primary, handled by CameraNodeOverallStatusTask for summary
                # and IndividualCameraDiagnosticTask for the specific 'unavailable' status message.
                pass # Or store if you want to display more from it.
            
            elif 'control_loop' in node_name: # Matches 'robot_control_node:control_loop'
                control_performance = diagnostics_content

            elif 'resources' in node_name:
                resource_info = diagnostics_content
        
        # System Mode - Most prominent display
        mode_color = Colors.GREEN if self.current_system_mode == "teleoperation" else Colors.YELLOW
        if self.current_system_mode == "error":
            mode_color = Colors.FAIL
        print(f"🤖 {Colors.BOLD}SYSTEM MODE:{Colors.ENDC} {mode_color}{Colors.BOLD}{self.current_system_mode.upper()}{Colors.ENDC}")
        
        # Recording Status - Prominent display
        if self.recording_active:
            print(f"🔴 {Colors.BOLD}RECORDING STATUS:{Colors.ENDC} {Colors.RED}{Colors.BOLD}● RECORDING ON{Colors.ENDC}")
        else:
            print(f"⚫ {Colors.BOLD}RECORDING STATUS:{Colors.ENDC} {Colors.CYAN}○ RECORDING OFF{Colors.ENDC}")
        
        # VR Controller Performance & Status
        print(f"\n{Colors.BOLD}🎮 VR STATUS & PERFORMANCE:{Colors.ENDC}")
        if self.vr_healthy:
            # Try to get specific performance data from 'oculus_reader:vr_controller'
            target_fps_vr = vr_performance.get('target_rate', 'N/A') 
            actual_fps_vr = vr_performance.get('current_rate', 'N/A')
            efficiency_vr = vr_performance.get('efficiency', 'N/A')
            status_msg_vr = vr_performance.get('Summary', 'Connected')

            print(f"   • Status: {Colors.GREEN}{status_msg_vr}{Colors.ENDC}")
            print(f"   • Target FPS: {Colors.CYAN}{target_fps_vr} Hz{Colors.ENDC}")
            
            try: actual_fps_float = float(actual_fps_vr)
            except ValueError: actual_fps_float = 0.0
            fps_color = Colors.GREEN if actual_fps_float >= 55 else Colors.WARNING if actual_fps_float >= 30 else Colors.FAIL
            print(f"   • Actual FPS: {fps_color}{actual_fps_vr} Hz{Colors.ENDC}")
            
            if efficiency_vr != 'N/A': 
                try: efficiency_float = float(efficiency_vr)
                except ValueError: efficiency_float = 0.0
                eff_color = Colors.GREEN if efficiency_float >= 90 else Colors.WARNING if efficiency_float >= 70 else Colors.FAIL
                print(f"   • Efficiency: {eff_color}{efficiency_vr}%{Colors.ENDC}")
        elif oculus_node_diagnostics: # General oculus_reader status if specific performance data not found but oculus_reader is publishing
            oc_status_msg = oculus_node_diagnostics.get('Summary', 'Graceful Fallback')
            oc_level = oculus_node_diagnostics.get('Level', 'WARNING')
            oc_color = Colors.GREEN if oc_level == 'OK' else Colors.WARNING if oc_level == 'WARNING' else Colors.FAIL
            print(f"   {oc_color}• {oc_status_msg}{Colors.ENDC}")
            print(f"   {Colors.YELLOW}• Performance data not found under 'oculus_reader:vr_controller' diagnostics.{Colors.ENDC}")
        else:
            print(f"   {Colors.WARNING}• VR Controller not connected or no diagnostics received.{Colors.ENDC}")
        
        # Robot Control Loop Performance (only shown during teleoperation)
        if self.current_system_mode == "teleoperation" and control_performance:
            print(f"\n{Colors.BOLD}🤖 ROBOT CONTROL LOOP:{Colors.ENDC}")
            target_rate = control_performance.get('target', 45.0)
            actual_rate = control_performance.get('actual', 0.0)
            efficiency = control_performance.get('efficiency', 0.0)
            
            if actual_rate > 0:
                fps_color = Colors.GREEN if actual_rate >= 40 else Colors.WARNING if actual_rate >= 20 else Colors.FAIL
                print(f"   • Target Rate: {Colors.CYAN}{target_rate:.0f} Hz{Colors.ENDC}")
                print(f"   • Actual Rate: {fps_color}{actual_rate:.1f} Hz{Colors.ENDC}")
                eff_color = Colors.GREEN if efficiency >= 90 else Colors.WARNING if efficiency >= 70 else Colors.FAIL
                print(f"   • Efficiency: {eff_color}{efficiency:.1f}%{Colors.ENDC}")
            else:
                print(f"   {Colors.WARNING}• Control loop not active yet{Colors.ENDC}")
        
        # Camera System Performance
        if self.cameras_healthy and camera_diagnostics_data:
            print(f"\n{Colors.BOLD}{Colors.BLUE}📹 CAMERA SYSTEM PERFORMANCE{Colors.ENDC}")
            print(f"{Colors.CYAN}{'=' * 60}{Colors.ENDC}")
            
            # Load camera config to get expected cameras and their positions
            expected_cameras_map = {}
            camera_config_path = self.launch_params.get('camera_config', '')
            # self.get_logger().info(f"Attempting to load expected_cameras from: {camera_config_path}") # Already logged earlier
            if os.path.exists(camera_config_path):
                try:
                    with open(camera_config_path, 'r') as f:
                        config_from_file = yaml.safe_load(f)
                        if config_from_file and 'cameras' in config_from_file:
                            for cam_id_yaml, cam_cfg_yaml in config_from_file.get('cameras', {}).items():
                                if cam_cfg_yaml.get('enabled', False):
                                    expected_cameras_map[cam_id_yaml] = {
                                        'serial': cam_cfg_yaml.get('serial_number', 'N/A'),
                                        'position': cam_cfg_yaml.get('metadata', {}).get('position', 'Unknown') # Get position from metadata
                                    }
                            # self.get_logger().info(f"Loaded {len(expected_cameras_map)} expected cameras with positions from config.")
                except Exception as e:
                    self.get_logger().error(f"Failed to load/parse camera config for positions in main_system: {camera_config_path} - {e}")
            
            main_cam_node_status = self.node_diagnostics.get('vision_camera_node:Camera System Status', {})
            cam_node_summary = main_cam_node_status.get('Summary', 'Camera node status not directly available')
            cam_node_level = main_cam_node_status.get('Level', 'OK')
            cam_node_color = Colors.GREEN if cam_node_level == 'OK' else Colors.WARNING if cam_node_level == 'WARNING' else Colors.FAIL

            print(f"  🔧 {Colors.BOLD}CAMERA NODE STATUS:{Colors.ENDC} {cam_node_color}{cam_node_summary}{Colors.ENDC}")
            print(f"  📷 {Colors.BOLD}Discovered & Reporting Cameras:{Colors.ENDC} {Colors.CYAN}{len(camera_diagnostics_data)}{Colors.ENDC}")
            
            print(f"\n  📊 {Colors.BOLD}INDIVIDUAL CAMERA PERFORMANCE:{Colors.ENDC}")
            print(f"  {'.' * 56}") # Dotted line
            
            # Iterate through camera_diagnostics_data which should now be keyed by camera_id
            for cam_id_diag, cam_diag_content in camera_diagnostics_data.items():
                # cam_id_diag is the camera_id from the diagnostic task name
                # cam_diag_content is the dictionary of KVs for this specific camera
                
                # Get configured position using cam_id_diag from expected_cameras_map
                # serial = cam_diag_content.get('Actual SN', cam_diag_content.get('Configured SN/Idx', 'Unknown')) # Prioritize actual SN if available
                # position = cam_config.get('metadata', {}).get('position', 'N/A')
                serial = cam_diag_content.get('Actual SN', cam_diag_content.get('Configured SN/Idx', 'Unknown'))
                position = expected_cameras_map.get(cam_id_diag, {}).get('position', 'Config Pos N/A')

                status_summary = cam_diag_content.get('Summary', 'Status N/A')
                level = cam_diag_content.get('Level', 'OK')

                print(f"\n  📷 {Colors.BOLD}{cam_id_diag}{Colors.ENDC} (SN: {serial}, Pos: {position}):")
                
                if level == 'ERROR':
                    print(f"     {Colors.FAIL}• Status: {status_summary}{Colors.ENDC}")
                    continue
                elif level == 'WARNING':
                    print(f"     {Colors.WARNING}• Status: {status_summary}{Colors.ENDC}")
                else:
                    print(f"     {Colors.GREEN}• Status: {status_summary}{Colors.ENDC}")

                # Color stream performance
                color_target = cam_diag_content.get('Target Color FPS', 'N/A')
                color_current = cam_diag_content.get('Actual Color FPS', 'N/A')
                color_efficiency = cam_diag_content.get('Color Efficiency (%)', 'N/A')
                color_frames = cam_diag_content.get('Color Frames Published', 'N/A')
                
                if color_current != 'N/A':
                    try: color_current_float = float(color_current)
                    except ValueError: color_current_float = 0.0
                    print(f"     • Color FPS (Target: {color_target}): ", end="")
                    fps_color = Colors.GREEN if color_current_float >= (float(color_target)*0.9 if color_target !='N/A' else 25) else Colors.WARNING if color_current_float >= (float(color_target)*0.7 if color_target != 'N/A' else 15) else Colors.FAIL
                    print(f"{fps_color}{color_current} Hz{Colors.ENDC} (Eff: {color_efficiency}%) ({color_frames})")
                
                # Depth stream performance
                depth_target = cam_diag_content.get('Target Depth FPS', 'N/A')
                depth_current = cam_diag_content.get('Actual Depth FPS', 'N/A')
                depth_efficiency = cam_diag_content.get('Depth Efficiency (%)', 'N/A')
                depth_frames = cam_diag_content.get('Depth Frames Published', 'N/A')
                
                # Check if depth is enabled for this camera from the config
                depth_enabled_in_config = False
                if cam_id_diag in expected_cameras_map: # Check if cam_id_diag is a valid key
                    # This assumes expected_cameras_map stores the full config for the camera, 
                    # or that camera_config_path needs to be parsed again to get specific depth enabled status
                    # For simplicity, let's assume the camera_node only provides depth diagnostics if enabled.
                    # A more robust way would be to check the original camera_config_file content.
                    # The IndividualCameraDiagnosticTask already checks this from its own camera_config.
                    # So if depth_current is not N/A, it implies it was enabled and measured.
                    depth_enabled_in_config = True # Simpler: if depth_current is present, assume it was enabled

                if depth_current != 'N/A' and depth_enabled_in_config:
                    try: depth_current_float = float(depth_current)
                    except ValueError: depth_current_float = 0.0
                    print(f"     • Depth FPS (Target: {depth_target}): ", end="")
                    fps_color = Colors.GREEN if depth_current_float >= (float(depth_target)*0.9 if depth_target !='N/A' else 25) else Colors.WARNING if depth_current_float >= (float(depth_target)*0.7 if depth_target != 'N/A' else 15) else Colors.FAIL
                    print(f"{fps_color}{depth_current} Hz{Colors.ENDC} (Eff: {depth_efficiency}%) ({depth_frames})")
            
            # Summary
            connected_count = sum(1 for cam_diag_content in camera_diagnostics_data.values() 
                                if cam_diag_content.get('Level') != 'ERROR')
            
            print(f"\n  {'.' * 56}") # Dotted line
            print(f"  📈 {Colors.BOLD}SUMMARY:{Colors.ENDC} {connected_count}/{len(expected_cameras_map)} configured cameras reported as operational by diagnostics.")
            if connected_count < len(expected_cameras_map) and len(expected_cameras_map) > 0:
                print(f"  💡 {Colors.CYAN}System may be operating with partial camera coverage or config mismatch.{Colors.ENDC}")
            elif len(expected_cameras_map) == 0 and self.cameras_healthy:
                print(f"  💡 {Colors.YELLOW}Cameras are enabled, but no cameras found in the loaded config file: {camera_config_path}{Colors.ENDC}")
            
            print(f"{Colors.CYAN}{'=' * 60}{Colors.ENDC}")
        
        # System Resources
        if resource_info:
            cpu_val = resource_info.get('cpu_percent', '0.0') # These are KVs, so value is string
            mem_val = resource_info.get('memory_percent', '0.0')
            try:
                cpu = float(cpu_val)
                memory = float(mem_val)
            except ValueError:
                cpu = 0.0
                memory = 0.0
            
            if cpu > 0 or memory > 0:
                print(f"\n{Colors.BOLD}{Colors.BLUE}💻 SYSTEM RESOURCES{Colors.ENDC}")
                print(f"{Colors.CYAN}{'-' * 40}{Colors.ENDC}")
                
                cpu_color = Colors.GREEN if cpu < 80 else Colors.WARNING if cpu < 90 else Colors.FAIL
                mem_color = Colors.GREEN if memory < 80 else Colors.WARNING if memory < 90 else Colors.FAIL
                
                print(f"  • CPU Usage: {cpu_color}{cpu:.1f}%{Colors.ENDC}")
                print(f"  • Memory Usage: {mem_color}{memory:.1f}%{Colors.ENDC}")
        
        # Detailed MoveIt Services Status
        print(f"\n{Colors.BOLD}{Colors.BLUE}🔧 MOVEIT SERVICES STATUS{Colors.ENDC}")
        print(f"{Colors.CYAN}{'-' * 40}{Colors.ENDC}")
        moveit_overall_ready = system_diagnostics.get('moveit_ready', 'False') == 'True'
        
        if moveit_overall_ready:
            print(f"  {Colors.GREEN}✅ All core MoveIt services reported as ready.{Colors.ENDC}")
        else:
            print(f"  {Colors.WARNING}⚠️ Some MoveIt services may not be ready:{Colors.ENDC}")

        ik_status = system_diagnostics.get('moveit_ik_service', 'Pending')
        planner_status = system_diagnostics.get('moveit_planner_service', 'Pending')
        scene_status = system_diagnostics.get('moveit_scene_service', 'Pending')

        print(f"    ├─ IK Service (/compute_ik): {Colors.GREEN if ik_status == 'Ready' else Colors.WARNING}{ik_status}{Colors.ENDC}")
        print(f"    ├─ Planner Service (/plan_kinematic_path): {Colors.GREEN if planner_status == 'Ready' else Colors.WARNING}{planner_status}{Colors.ENDC}")
        print(f"    └─ Scene Service (/get_planning_scene): {Colors.GREEN if scene_status == 'Ready' else Colors.WARNING}{scene_status}{Colors.ENDC}")

        # Other nodes status (filtered)
        if self.node_diagnostics:
            print(f"\n{Colors.BOLD}{Colors.BLUE}📋 OTHER NODE STATUS (Filtered){Colors.ENDC}")
            print(f"{Colors.CYAN}{'-' * 40}{Colors.ENDC}")
            
            # Filter out nodes we've already shown details for or are part of other sections
            shown_nodes_prefixes = [
                'main_system', 
                'oculus_reader:vr_controller', 
                # 'vision_camera_node:camera_', # Covered by new parsing
                'Camera ', # Covers active and unavailable individual camera tasks
                'Camera Node Status', # Covers the overall camera node status task
                'resources', 
                self.get_name()
            ]
            # Also filter specific camera node summary if we list cameras individually
            # The camera_diagnostics keys are like 'realsense_D435_12345'
            # The node names are like 'vision_camera_node:camera_realsense_D435_12345'

            other_nodes_to_print = []
            oculus_reader_summaries = set() # To avoid duplicate oculus_reader general messages

            for node_name, diagnostics_content in sorted(self.node_diagnostics.items()):
                is_shown_elsewhere = False
                for prefix in shown_nodes_prefixes:
                    if node_name.startswith(prefix):
                        is_shown_elsewhere = True
                        break
                if is_shown_elsewhere:
                    continue
                
                # Consolidate oculus_reader general messages
                if node_name.startswith('oculus_reader'):
                    summary_msg = diagnostics_content.get('Summary', 'No status')
                    if summary_msg in oculus_reader_summaries:
                        continue # Skip if this exact summary for oculus_reader was already added
                    oculus_reader_summaries.add(summary_msg)

                other_nodes_to_print.append((node_name, diagnostics_content))

            for node_name, diagnostics in other_nodes_to_print[:7]:  # Limit to avoid clutter
                level = diagnostics.get('Level', 'OK')
                level_color = Colors.CYAN # Default
                level_icon = "🔵"
                if level == 'OK': level_color = Colors.GREEN; level_icon = "🟢"
                elif level == 'WARNING': level_color = Colors.WARNING; level_icon = "🟡"
                elif level == 'ERROR': level_color = Colors.FAIL; level_icon = "🔴"
                
                display_name = node_name.replace(':', ' -> ')
                if len(display_name) > 35:
                    display_name = display_name[:32] + "..."
                
                summary = diagnostics.get('Summary', 'No status')
                if len(summary) > 40:
                    summary = summary[:37] + "..."
                
                print(f"  {level_icon} {Colors.BOLD}{display_name:<35}{Colors.ENDC}: {level_color}{summary}{Colors.ENDC}")
        
        # Footer with quick actions
        print(f"\n{Colors.CYAN}{'=' * 80}{Colors.ENDC}")
        print(f"{Colors.BOLD}{Colors.BLUE}🚀 QUICK ACTIONS{Colors.ENDC}")
        print(f"{Colors.CYAN}{'-' * 20}{Colors.ENDC}")
        print(f"• Live diagnostics: {Colors.YELLOW}ros2 topic echo /diagnostics{Colors.ENDC}")
        print(f"• Check nodes: {Colors.YELLOW}ros2 node list{Colors.ENDC}")
        print(f"• Emergency stop: {Colors.RED}Ctrl+C{Colors.ENDC}")
        print(f"{Colors.CYAN}{'=' * 80}{Colors.ENDC}")
        
        # Force flush output
        sys.stdout.flush()


async def async_main():
    """Async main entry point"""
    # Initialize ROS2
    rclpy.init()
    
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
    try:
        from ament_index_python.packages import get_package_share_directory
        config_path = os.path.join(
            get_package_share_directory('lbx_franka_control'),
            'config',
            'franka_vr_control_config.yaml'
        )
    except:
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
            import yaml
            with open(config_path, 'w') as f:
                yaml.dump(default_config, f)
    
    # Create main system node
    system = LabelboxRoboticsSystem(config_path, launch_params)
    
    # Update config with launch parameters
    if launch_params.get('robot_ip'):
        system.config['robot']['robot_ip'] = launch_params['robot_ip']
    
    # Create executor
    executor = MultiThreadedExecutor()
    executor.add_node(system)
    
    # Run system with proper executor integration
    try:
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
        system.running = False
        system.cleanup()
        executor.shutdown(timeout_sec=5.0)
        system.destroy_node()
        rclpy.shutdown()


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


if __name__ == '__main__':
    main() 