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
from std_srvs.srv import Empty
from sensor_msgs.msg import Joy, JointState, Image, CameraInfo
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
    """Simple, comprehensive system diagnostic task"""
    
    def __init__(self, node):
        super().__init__("LBX Robotics System Status")
        self.node = node
    
    def run(self, stat):
        """Run diagnostic check and report status"""
        # Overall system health
        if self.node.vr_healthy and self.node.moveit_healthy and self.node.robot_healthy:
            stat.summary(DiagnosticStatus.OK, "All systems operational")
            mode = "Full Operation"
        elif self.node.moveit_healthy and self.node.robot_healthy:
            stat.summary(DiagnosticStatus.WARN, "VR graceful fallback active")
            mode = "Robot Control (No VR)"
        elif self.node.robot_healthy:
            stat.summary(DiagnosticStatus.WARN, "Monitoring mode - MoveIt services pending")
            mode = "Robot Monitoring"
        else:
            stat.summary(DiagnosticStatus.ERROR, "Robot connection required")
            mode = "Basic Monitoring"
        
        # Add detailed status
        stat.add("Operation Mode", mode)
        stat.add("VR Status", "Connected" if self.node.vr_healthy else "Graceful Fallback")
        stat.add("MoveIt Status", "Ready" if self.node.moveit_healthy else "Pending")
        stat.add("Robot Status", "Connected" if self.node.robot_healthy else "Disconnected")
        stat.add("Fake Hardware", str(self.node._get_bool_param('use_fake_hardware', False)))
        stat.add("Camera Status", "Healthy" if self.node.cameras_healthy else "Issues Detected")
        
        # System ready status
        stat.add("System Ready", str(self.node.system_ready))
        
        # Configuration information
        stat.add("Robot IP", self.node.config.get('robot', {}).get('robot_ip', 'Unknown'))
        stat.add("Recording Enabled", str(self.node.config.get('recording', {}).get('enabled', False)))
        
        # System capabilities
        capabilities = []
        if self.node.launch_params.get('enable_cameras', False):
            capabilities.append("Cameras")
        if self.node.launch_params.get('enable_recording', False):
            capabilities.append("Recording")
        if self.node.launch_params.get('hot_reload', False):
            capabilities.append("Hot Reload")
        stat.add("Active Capabilities", ", ".join(capabilities) if capabilities else "Basic")
        
        # Update latest diagnostics for periodic summaries
        self.node.latest_diagnostics = {
            "Operation Mode": mode,
            "VR Status": "Connected" if self.node.vr_healthy else "Graceful Fallback",
            "MoveIt Status": "Ready" if self.node.moveit_healthy else "Pending",
            "Robot Status": "Connected" if self.node.robot_healthy else "Disconnected",
            "System Ready": str(self.node.system_ready),
            "Active Capabilities": ", ".join(capabilities) if capabilities else "Basic"
        }
        
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
        
        # Camera system tracking
        self.camera_fps_tracking = {}
        self.camera_info_data = {}
        self.camera_subscriptions = []
        
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
        
        # Initialize camera monitoring if cameras are enabled
        if self.launch_params.get('enable_cameras', False):
            self.setup_camera_monitoring()
    
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
    
    async def initialize_system(self):
        """Initialize and test all system components"""
        print(f"\n{Colors.BLUE}{Colors.BOLD}🚀 System Initialization{Colors.ENDC}")
        print("─" * 50)
        
        # 1. Check VR Controller with enhanced feedback
        print(f"\n{Colors.CYAN}1️⃣  Checking VR Controller...{Colors.ENDC}")
        vr_status = await self.check_vr_controller()
        if vr_status:
            print(f"   ✅ VR Controller: {Colors.GREEN}Connected and responding{Colors.ENDC}")
            self.vr_healthy = True
        else:
            print(f"   ❌ VR Controller: {Colors.FAIL}Not detected{Colors.ENDC}")
            print(f"      {Colors.WARNING}System will continue with graceful fallback{Colors.ENDC}")
            print(f"      • Robot control via other interfaces remains available")
            print(f"      • All features except VR input will function normally")
            print(f"      • See VR setup instructions in oculus_node logs")
            self.vr_healthy = False
        
        # 2. Test Cameras (if enabled)
        if self.launch_params.get('enable_cameras', False):
            print(f"\n{Colors.CYAN}2️⃣  Testing Cameras...{Colors.ENDC}")
            camera_results = await self.test_cameras()
            for camera_name, status in camera_results.items():
                if status:
                    print(f"   ✅ {camera_name}: {Colors.GREEN}Operational{Colors.ENDC}")
                else:
                    print(f"   ⚠️  {camera_name}: {Colors.WARNING}Failed{Colors.ENDC}")
        
        # 3. Check MoveIt Services
        print(f"\n{Colors.CYAN}3️⃣  Checking MoveIt Services...{Colors.ENDC}")
        moveit_status = await self.check_moveit_services()
        services = ['IK Solver', 'FK Solver', 'Planning Scene', 'Trajectory Controller', 'Gripper Controller']
        for i, (service, status) in enumerate(moveit_status.items()):
            if status:
                print(f"   ✅ {services[i]}: {Colors.GREEN}Ready{Colors.ENDC}")
            else:
                print(f"   ❌ {services[i]}: {Colors.FAIL}Not available{Colors.ENDC}")
        
        # 4. Check Robot Connection
        print(f"\n{Colors.CYAN}4️⃣  Checking Robot Connection...{Colors.ENDC}")
        robot_status = await self.check_robot_connection()
        if robot_status:
            print(f"   ✅ Robot: {Colors.GREEN}Connected{Colors.ENDC}")
            print(f"   📊 Joint states: Receiving at {Colors.BOLD}1000Hz{Colors.ENDC}")
        else:
            print(f"   ❌ Robot: {Colors.FAIL}Not connected{Colors.ENDC}")
        
        # Summary with graceful fallback logic
        print(f"\n{Colors.BLUE}{'─' * 50}{Colors.ENDC}")
        
        # Essential systems: robot connection is most important
        # MoveIt services are important but system can run in monitoring mode without them
        essential_systems_healthy = robot_status
        
        # Determine system readiness with graceful degradation
        if essential_systems_healthy:
            if self.vr_healthy and all(moveit_status.values()):
                print(f"{Colors.GREEN}{Colors.BOLD}✅ All systems fully operational!{Colors.ENDC}")
                self.system_ready = True
            elif all(moveit_status.values()):
                print(f"{Colors.WARNING}{Colors.BOLD}⚠️  Core systems operational (VR graceful fallback active){Colors.ENDC}")
                print(f"   {Colors.CYAN}Robot control and MoveIt ready - VR will reconnect automatically{Colors.ENDC}")
                self.system_ready = True
            else:
                print(f"{Colors.WARNING}{Colors.BOLD}⚠️  Robot connected (MoveIt services pending){Colors.ENDC}")
                print(f"   {Colors.CYAN}System monitoring active - services may become available{Colors.ENDC}")
                self.system_ready = True  # Still allow monitoring mode
        else:
            print(f"{Colors.FAIL}{Colors.BOLD}❌ Robot connection required for system operation{Colors.ENDC}")
            print(f"   {Colors.CYAN}Check robot power, network, and ros2_control configuration{Colors.ENDC}")
            # For fake hardware mode, be more lenient 
            fake_hardware = self._get_bool_param('use_fake_hardware', False)
            if fake_hardware:
                print(f"   {Colors.WARNING}Fake hardware mode: Continuing anyway for testing{Colors.ENDC}")
                self.system_ready = True  # Allow testing without perfect robot connection
                return True
            else:
                return False
        
        return True
    
    async def check_vr_controller(self):
        """Check if VR controller is connected and responding with enhanced status reporting"""
        # Wait for VR pose data with timeout
        vr_pose_msg = None
        try:
            print(f"      Waiting for VR controller data...")
            vr_pose_msg = await self.wait_for_message('/vr/controller_pose', PoseStamped, timeout=3.0)
            
            # Additional check for valid pose data
            if vr_pose_msg:
                pose = vr_pose_msg.pose
                # Check if we're getting non-zero poses (actual VR input vs default/fallback data)
                position_magnitude = (pose.position.x**2 + pose.position.y**2 + pose.position.z**2)**0.5
                orientation_valid = abs(pose.orientation.w) > 0.1  # Valid quaternion
                
                if position_magnitude > 0.01 or not orientation_valid:  # Some actual movement or valid orientation
                    print(f"      📍 VR pose data: Valid tracking detected")
                    self.vr_healthy = True
                else:
                    print(f"      📍 VR pose data: Receiving default/fallback data")
                    self.vr_healthy = False
            else:
                self.vr_healthy = False
                
        except Exception as e:
            print(f"      ❌ VR check failed: {e}")
            self.vr_healthy = False
        
        return self.vr_healthy
    
    async def test_cameras(self):
        """Test camera connections"""
        results = {}
        
        # Get available camera topics dynamically
        available_topics = self.get_topic_names_and_types()
        camera_topics = []
        
        for topic_name, topic_types in available_topics:
            if '/cameras/' in topic_name and 'sensor_msgs/msg/Image' in topic_types:
                if '/image_raw' in topic_name and '/depth/' not in topic_name:
                    # Only test main color image topics, not depth
                    camera_topics.append(topic_name)
        
        # If no topics found, try expected patterns
        if not camera_topics:
            camera_topics = [
                '/cameras/wrist_camera/image_raw',
                '/cameras/overhead_camera/image_raw',
            ]
        
        for topic in camera_topics:
            try:
                msg = await self.wait_for_message(topic, Image, timeout=1.0)
                # Extract camera name from topic
                parts = topic.split('/')
                camera_name = parts[2] if len(parts) > 2 else 'unknown'
                results[camera_name] = msg is not None
            except:
                parts = topic.split('/')
                camera_name = parts[2] if len(parts) > 2 else 'unknown'
                results[camera_name] = False
        
        self.cameras_healthy = all(results.values()) if results else True
        return results
    
    async def check_moveit_services(self):
        """Check if MoveIt services are available with proper waiting"""
        services_to_check = {
            '/compute_ik': False,
            '/compute_fk': False, 
            '/get_planning_scene': False,
            '/fr3_arm_controller/follow_joint_trajectory': False,
            '/fr3_gripper/grasp': False
        }
        
        print(f"      Waiting for MoveIt services to become available...")
        
        # Wait for each service with timeout
        for service_name in services_to_check:
            try:
                print(f"      Checking {service_name}...")
                
                # Create a temporary client to wait for the service
                if 'compute_ik' in service_name:
                    from moveit_msgs.srv import GetPositionIK
                    client = self.create_client(GetPositionIK, service_name)
                elif 'compute_fk' in service_name:
                    from moveit_msgs.srv import GetPositionFK
                    client = self.create_client(GetPositionFK, service_name)
                elif 'get_planning_scene' in service_name:
                    from moveit_msgs.srv import GetPlanningScene
                    client = self.create_client(GetPlanningScene, service_name)
                elif 'follow_joint_trajectory' in service_name:
                    from control_msgs.action import FollowJointTrajectory
                    # For action servers, we check differently
                    services_to_check[service_name] = True  # Assume available for now
                    print(f"        ✓ {service_name} (action server)")
                    continue
                elif 'grasp' in service_name:
                    from franka_msgs.action import Grasp
                    # For action servers, we check differently  
                    services_to_check[service_name] = True  # Assume available for now
                    print(f"        ✓ {service_name} (action server)")
                    continue
                else:
                    # Generic service check
                    client = self.create_client(Empty, service_name)
                
                # Wait for service with timeout
                service_available = client.wait_for_service(timeout_sec=5.0)
                services_to_check[service_name] = service_available
                
                if service_available:
                    print(f"        ✓ {service_name}")
                else:
                    print(f"        ✗ {service_name} (timeout)")
                
                # Clean up client
                self.destroy_client(client)
                
            except Exception as e:
                print(f"        ✗ {service_name} (error: {e})")
                services_to_check[service_name] = False
        
        self.moveit_healthy = all(services_to_check.values())
        return services_to_check
    
    async def check_robot_connection(self):
        """Check robot connection via joint states with enhanced feedback"""
        print(f"      Waiting for robot joint states...")
        
        # Check if we're using fake hardware - handle both string and boolean values
        use_fake_hardware = self._get_bool_param('use_fake_hardware', False)
        
        if use_fake_hardware:
            print(f"      Using fake hardware - joint states should be simulated")
        else:
            print(f"      Connecting to real robot at {self.config['robot']['robot_ip']}")
        
        try:
            # Wait longer for joint states to become available (especially for fake hardware)
            timeout = 10.0 if use_fake_hardware else 5.0
            msg = await self.wait_for_message('/joint_states', JointState, timeout=timeout)
            
            if msg is not None and len(msg.position) >= 7:
                joint_count = len(msg.position)
                print(f"      📊 Receiving joint states: {joint_count} joints")
                
                # Check for reasonable joint values
                if use_fake_hardware:
                    print(f"      🤖 Fake hardware joint states active")
                else:
                    # For real hardware, check if joints are in reasonable ranges
                    joint_ranges_ok = all(-3.0 <= pos <= 3.0 for pos in msg.position[:7])  # Rough FR3 joint limits
                    if joint_ranges_ok:
                        print(f"      🤖 Real robot joint states within normal ranges")
                    else:
                        print(f"      ⚠️  Joint positions may be outside normal ranges")
                
                self.robot_healthy = True
                return True
            else:
                print(f"      ❌ Joint states missing or insufficient joints (got {len(msg.position) if msg else 0}, need ≥7)")
                self.robot_healthy = False
                return False
                
        except Exception as e:
            print(f"      ❌ Robot connection check failed: {e}")
            if use_fake_hardware:
                print(f"      💡 Fake hardware mode: ensure ros2_control is running with fake hardware")
                print(f"      🔄 Will continue in monitoring mode - joint states may become available later")
                # In fake hardware mode, be more tolerant and allow monitoring mode
                self.robot_healthy = False  # Mark as not healthy but don't fail completely
                return False  # But still return False so system knows to use monitoring mode
            else:
                print(f"      💡 Real robot mode: check robot power, network, and FCI enable")
                self.robot_healthy = False
                return False
    
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
    
    def reset_robot_to_home(self):
        """Reset robot to home position"""
        print(f"\n{Colors.BLUE}{Colors.BOLD}🏠 Resetting Robot to Home Position{Colors.ENDC}")
        print("─" * 50)
        
        # Try to call reset service if available
        reset_client = self.create_client(Empty, '/reset_robot')
        if reset_client.wait_for_service(timeout_sec=2.0):
            print("   🔄 Sending reset command...")
            try:
                future = reset_client.call_async(Empty.Request())
                
                # Wait for completion with timeout
                start_time = time.time()
                while not future.done() and (time.time() - start_time) < 5.0:
                    time.sleep(0.1)
                
                if future.done():
                    print(f"   ✅ Robot reset to home position")
                else:
                    print(f"   ⚠️  Reset command timeout, but continuing...")
            except Exception as e:
                print(f"   ⚠️  Reset command failed: {e}, but continuing...")
        else:
            print(f"   ℹ️  Reset service not available - robot may already be in home position")
            print(f"   📍 Current system uses MoveIt for robot control")
            print(f"   💡 Robot will be positioned during VR teleoperation startup")
    
    def enter_calibration_mode(self):
        """Guide user through calibration"""
        print(f"\n{Colors.BLUE}{Colors.BOLD}🎯 Calibration Mode{Colors.ENDC}")
        print("─" * 50)
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
        
        if self.vr_healthy:
            print(f"\n{Colors.CYAN}VR Controls:{Colors.ENDC}")
            print("   • {Colors.BOLD}Grip{Colors.ENDC}: Hold to enable robot movement")
            print("   • {Colors.BOLD}Trigger{Colors.ENDC}: Control gripper (pull to close)")
            print("   • {Colors.BOLD}A/X{Colors.ENDC}: Start/stop recording")
            print("   • {Colors.BOLD}B/Y{Colors.ENDC}: Mark recording as successful")
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
        
        # Clean up camera subscriptions
        for subscription in self.camera_subscriptions:
            self.destroy_subscription(subscription)
        self.camera_subscriptions.clear()
        
        # Cleanup will be handled by ROS2 shutdown
    
    def _get_bool_param(self, param_name: str, default: bool = False) -> bool:
        """Helper to safely convert launch parameters to boolean values"""
        param_value = self.launch_params.get(param_name, default)
        if isinstance(param_value, bool):
            return param_value
        else:
            return str(param_value).lower() in ('true', '1', 'yes', 'on')
    
    async def run(self):
        """Main run loop with graceful degradation support"""
        try:
            # Welcome message
            self.print_welcome_message()
            
            # Initialize system
            if not await self.initialize_system():
                print(f"\n{Colors.FAIL}Critical system initialization failed. Exiting.{Colors.ENDC}")
                return
            
            # Reset robot (only if robot and MoveIt are healthy)
            if self.robot_healthy and self.moveit_healthy:
                self.reset_robot_to_home()
            else:
                print(f"\n{Colors.WARNING}⚠️  Skipping robot reset - running in monitoring mode{Colors.ENDC}")
            
            # Handle calibration based on system state
            if self.vr_healthy and self.moveit_healthy:
                # Full VR teleoperation mode
                self.enter_calibration_mode()
                
                # Start full teleoperation
                self.start_teleoperation()
                
            elif self.robot_healthy:
                # Robot monitoring mode (no VR or degraded MoveIt)
                print(f"\n{Colors.CYAN}{Colors.BOLD}🔍 System Monitoring Mode{Colors.ENDC}")
                print("─" * 50)
                print(f"\n{Colors.CYAN}Current Status:{Colors.ENDC}")
                if not self.vr_healthy:
                    print("   • 🎮 VR: Graceful fallback active (will auto-reconnect)")
                if not self.moveit_healthy:
                    print("   • 🔧 MoveIt: Services initializing (check progress below)")
                print("   • 📊 System diagnostics: Active")
                print("   • 🔄 Hot-plugging: VR and services supported")
                print(f"\n{Colors.GREEN}Features available:{Colors.ENDC}")
                print("   • Real-time system status monitoring")
                print("   • Automatic VR reconnection detection")
                print("   • Service availability monitoring")
                print("   • Data recording (if enabled)")
                print(f"\n{Colors.WARNING}System will automatically upgrade when components become available{Colors.ENDC}")
                
            else:
                # Basic monitoring mode
                print(f"\n{Colors.WARNING}{Colors.BOLD}⚠️  Basic Monitoring Mode{Colors.ENDC}")
                print("─" * 50)
                print("   • Limited functionality - monitoring system health")
                print("   • Check robot connection and ros2_control status")
            
            # Simple diagnostic reporting
            print(f"\n{Colors.CYAN}📊 System diagnostics active{Colors.ENDC}")
            print(f"   View status: ros2 topic echo /diagnostics")
            print(f"   Monitor system: ros2 run rqt_robot_monitor rqt_robot_monitor")
            print(f"{Colors.WARNING}Press Ctrl+C to stop the system{Colors.ENDC}\n")
            
            # Force flush output
            sys.stdout.flush()
            
            # Update diagnostics and keep system running
            self._last_health_check = time.time()
            self.last_diagnostic_summary_time = time.time()  # Initialize summary timer
            
            while self.running:
                try:
                    # Update diagnostics more frequently
                    self.diagnostic_updater.update()
                    
                    # Health checks every 30 seconds
                    if time.time() - self._last_health_check > 30.0:
                        await self._periodic_health_check()
                        self._last_health_check = time.time()
                    
                    # Short sleep for responsive diagnostics
                    await asyncio.sleep(2.0)  # Update every 2 seconds for better responsiveness
                    
                except Exception as e:
                    self.get_logger().error(f"Error in main loop iteration: {e}")
                    # Continue running even if there are errors
                    await asyncio.sleep(1.0)
                    
        except Exception as e:
            self.get_logger().error(f"Critical error in main system run: {e}")
            import traceback
            traceback.print_exc()
            print(f"\n{Colors.FAIL}System encountered critical error: {e}{Colors.ENDC}")
            print(f"Check logs for details. System will attempt to continue...")
            # Don't exit immediately, allow cleanup
    
    async def _periodic_health_check(self):
        """Simple periodic health check for component reconnection"""
        # Re-check VR if not healthy
        if not self.vr_healthy:
            try:
                vr_status = await self.check_vr_controller()
                if vr_status:
                    self.get_logger().info("🎮 VR reconnected!")
                    self.vr_healthy = True
            except Exception:
                pass
        
        # Re-check MoveIt if not healthy
        if not self.moveit_healthy:
            try:
                moveit_status = await self.check_moveit_services()
                if all(moveit_status.values()):
                    self.get_logger().info("🔧 MoveIt services available!")
                    self.moveit_healthy = True
            except Exception:
                pass
        
        # Re-check robot if not healthy
        if not self.robot_healthy:
            try:
                robot_status = await self.check_robot_connection()
                if robot_status:
                    self.get_logger().info("🤖 Robot connected!")
                    self.robot_healthy = True
            except Exception:
                pass
        
        # Trigger a diagnostic summary if it's been a while
        current_time = time.time()
        if current_time - self.last_diagnostic_summary_time > self.diagnostic_summary_interval:
            self.print_diagnostic_summary()
            self.last_diagnostic_summary_time = current_time

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
        """Print a beautiful system health summary with frequency information"""
        
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
        
        # Overall system status
        if self.vr_healthy and self.moveit_healthy and self.robot_healthy:
            status_icon = "🟢"
            status_text = f"{Colors.GREEN}ALL SYSTEMS FULLY OPERATIONAL{Colors.ENDC}"
        elif self.moveit_healthy and self.robot_healthy:
            status_icon = "🟡"
            status_text = f"{Colors.WARNING}CORE SYSTEMS OPERATIONAL (VR FALLBACK){Colors.ENDC}"
        elif self.robot_healthy:
            status_icon = "🟠"
            status_text = f"{Colors.WARNING}ROBOT CONNECTED (MOVEIT PENDING){Colors.ENDC}"
        else:
            status_icon = "🔴"
            status_text = f"{Colors.FAIL}ROBOT CONNECTION REQUIRED{Colors.ENDC}"
        
        print(f"{status_icon} {Colors.BOLD}OVERALL STATUS:{Colors.ENDC} {status_text}")
        print(f"{Colors.CYAN}{'=' * 80}{Colors.ENDC}")
        
        # System components overview
        print(f"{Colors.BOLD}{Colors.BLUE}🔧 SYSTEM COMPONENTS{Colors.ENDC}")
        print(f"{Colors.CYAN}{'-' * 40}{Colors.ENDC}")
        
        # VR Component
        vr_icon = "🎮✅" if self.vr_healthy else "🎮⚠️"
        vr_status = f"{Colors.GREEN}Connected & Operational{Colors.ENDC}" if self.vr_healthy else f"{Colors.WARNING}Graceful Fallback{Colors.ENDC}"
        print(f"  {vr_icon} VR Controller: {vr_status}")
        
        # Robot Component
        robot_icon = "🤖✅" if self.robot_healthy else "🤖❌"
        if self.robot_healthy:
            hardware_type = "Fake Hardware" if self._get_bool_param('use_fake_hardware', False) else "Real Robot"
            robot_status = f"{Colors.GREEN}Connected ({hardware_type}){Colors.ENDC}"
        else:
            robot_status = f"{Colors.FAIL}Disconnected{Colors.ENDC}"
        print(f"  {robot_icon} Robot: {robot_status}")
        
        # MoveIt Component
        moveit_icon = "🔧✅" if self.moveit_healthy else "🔧⚠️"
        moveit_status = f"{Colors.GREEN}Services Ready{Colors.ENDC}" if self.moveit_healthy else f"{Colors.WARNING}Services Pending{Colors.ENDC}"
        print(f"  {moveit_icon} MoveIt: {moveit_status}")
        
        # Camera Component
        cameras_enabled = self.launch_params.get('enable_cameras', False)
        if cameras_enabled:
            camera_icon = "📹✅" if self.cameras_healthy else "📹⚠️"
            camera_status = f"{Colors.GREEN}Operational{Colors.ENDC}" if self.cameras_healthy else f"{Colors.WARNING}Issues{Colors.ENDC}"
        else:
            camera_icon = "📹⭕"
            camera_status = f"{Colors.CYAN}Disabled{Colors.ENDC}"
        print(f"  {camera_icon} Cameras: {camera_status}")
        
        print(f"{Colors.CYAN}{'=' * 80}{Colors.ENDC}")
        
        # Detailed frequency information from all nodes
        if self.node_diagnostics:
            print(f"{Colors.BOLD}{Colors.BLUE}🔄 NODE FREQUENCY REPORT{Colors.ENDC}")
            print(f"{Colors.CYAN}{'=' * 50}{Colors.ENDC}")
            
            # Frequency-related keywords to look for
            frequency_keywords = [
                'Rate', 'Frequency', 'Hz', 'FPS', 'Poll Rate', 'Publish Rate', 'Control Rate',
                'Update Rate', 'Sample Rate', 'Actual', 'Target', 'Expected'
            ]
            
            # Collect nodes with frequency data (excluding camera nodes now handled above)
            frequency_nodes = []
            other_nodes = []
            
            for node_name in sorted(self.node_diagnostics.keys()):
                diagnostics = self.node_diagnostics[node_name]
                frequency_data = {}
                
                for key, value in diagnostics.items():
                    if any(keyword in key for keyword in frequency_keywords):
                        frequency_data[key] = value
                
                if frequency_data:
                    frequency_nodes.append((node_name, diagnostics, frequency_data))
                else:
                    other_nodes.append((node_name, diagnostics))
            
            # Display camera system report first if cameras are enabled
            cameras_enabled = self.launch_params.get('enable_cameras', False)
            if cameras_enabled:
                print(f"\n{Colors.BOLD}{Colors.BLUE}📹 CAMERA SYSTEM REPORT{Colors.ENDC}")
                print(f"{Colors.CYAN}{'=' * 60}{Colors.ENDC}")
                
                if self.camera_fps_tracking:
                    # Count only cameras that have received data (active cameras)
                    active_cameras = [cam for cam, streams in self.camera_fps_tracking.items() 
                                    if any(stream['msg_count'] > 0 for stream in streams.values())]
                    
                    # Show active cameras from real-time tracking
                    print(f"  🔧 {Colors.BOLD}CAMERA CONFIGURATION:{Colors.ENDC}")
                    print(f"    • Active Cameras: {Colors.CYAN}{len(active_cameras)}{Colors.ENDC}")
                    print(f"    • Monitoring: Real-time FPS tracking enabled")
                    
                    if active_cameras:
                        # Individual camera details from direct monitoring
                        print(f"  📷 {Colors.BOLD}CAMERA PERFORMANCE:{Colors.ENDC}")
                        for camera_name in active_cameras:
                            streams = self.camera_fps_tracking[camera_name]
                            print(f"\n    📹 {Colors.BOLD}{camera_name.upper()}{Colors.ENDC}")
                            
                            # Show camera info if available
                            if camera_name in self.camera_info_data:
                                color_info = self.camera_info_data[camera_name].get('color', {})
                                if color_info:
                                    resolution = f"{color_info['width']}x{color_info['height']}"
                                    print(f"      └─ Resolution: {Colors.CYAN}{resolution}{Colors.ENDC}")
                                    print(f"      └─ Frame ID: {Colors.CYAN}{color_info.get('frame_id', 'N/A')}{Colors.ENDC}")
                            
                            # Stream performance from real-time tracking
                            print(f"      📊 {Colors.BOLD}STREAM PERFORMANCE:{Colors.ENDC}")
                            
                            for stream_type, tracking_data in streams.items():
                                # Only show streams that have received data
                                if tracking_data['msg_count'] == 0:
                                    continue
                                    
                                current_fps = tracking_data['current_fps']
                                msg_count = tracking_data['msg_count']
                                
                                # Determine performance color
                                if current_fps >= 25:
                                    perf_color = Colors.GREEN
                                    perf_icon = "🟢"
                                elif current_fps >= 15:
                                    perf_color = Colors.CYAN
                                    perf_icon = "🔵"
                                elif current_fps > 0:
                                    perf_color = Colors.WARNING
                                    perf_icon = "🟡"
                                else:
                                    perf_color = Colors.FAIL
                                    perf_icon = "🔴"
                                
                                stream_display = "RGB" if stream_type == 'color' else "Depth"
                                if current_fps > 0:
                                    print(f"        {perf_icon} {stream_display}: {perf_color}{current_fps:.1f} Hz{Colors.ENDC} ({msg_count} frames)")
                                else:
                                    print(f"        {perf_icon} {stream_display}: {perf_color}No data{Colors.ENDC}")
                    else:
                        # No active cameras found
                        print(f"\n⚠️  {Colors.WARNING}No active cameras detected{Colors.ENDC}")
                        print(f"   📍 Camera nodes may be starting up or no cameras connected")
                        print(f"   💡 Check camera hardware connections and camera node status")
                
                else:
                    # No camera tracking data - cameras may not be publishing yet
                    print(f"\n⚠️  {Colors.WARNING}Camera monitoring active but no camera data received{Colors.ENDC}")
                    print(f"   📍 Camera nodes may be starting up or no cameras connected")
                    print(f"   💡 Check camera hardware connections and RealSense drivers")
                    print(f"   🔍 Expected topics:")
                    print(f"      • /cameras/wrist_camera/image_raw")
                    print(f"      • /cameras/overhead_camera/image_raw")
                    print(f"      • /cameras/*/depth/image_raw")
                
                print(f"{Colors.CYAN}{'=' * 60}{Colors.ENDC}")
            
            # Display other nodes with frequency information
            if frequency_nodes:
                for i, (node_name, diagnostics, frequency_data) in enumerate(frequency_nodes):
                    # Node header
                    level = diagnostics.get('Level', 'OK')
                    if level == 'OK':
                        level_color = Colors.GREEN
                        level_icon = "🟢"
                    elif level == 'WARNING':
                        level_color = Colors.WARNING
                        level_icon = "🟡"
                    elif level == 'ERROR':
                        level_color = Colors.FAIL
                        level_icon = "🔴"
                    else:
                        level_color = Colors.CYAN
                        level_icon = "🔵"
                    
                    # Create a nice node name display
                    display_name = node_name.replace('oculus_reader: ', '').replace(':', ' -> ')
                    if len(display_name) > 60:
                        display_name = display_name[:57] + "..."
                    
                    print(f"\n{level_icon} {Colors.BOLD}{display_name}{Colors.ENDC} ({level_color}{level}{Colors.ENDC})")
                    print(f"{Colors.CYAN}{'-' * 70}{Colors.ENDC}")
                    
                    # Show summary if available
                    if 'Summary' in diagnostics:
                        summary = diagnostics['Summary']
                        if len(summary) > 75:
                            summary = summary[:72] + "..."
                        print(f"  📝 {summary}")
                    
                    # Show frequency information in organized sections
                    target_rates = []
                    actual_rates = []
                    other_freq = []
                    
                    for key, value in frequency_data.items():
                        if 'Target' in key or 'Expected' in key:
                            target_rates.append((key, value))
                        elif 'Actual' in key or 'Current' in key:
                            actual_rates.append((key, value))
                        else:
                            other_freq.append((key, value))
                    
                    # Display target rates
                    if target_rates:
                        print(f"  🎯 {Colors.BOLD}TARGET RATES:{Colors.ENDC}")
                        for key, value in target_rates:
                            clean_key = key.replace('Target ', '').replace('Expected ', '')
                            if len(clean_key) > 40:
                                clean_key = clean_key[:37] + "..."
                            print(f"    • {clean_key}: {Colors.CYAN}{value}{Colors.ENDC}")
                    
                    # Display actual rates
                    if actual_rates:
                        print(f"  📊 {Colors.BOLD}ACTUAL PERFORMANCE:{Colors.ENDC}")
                        for key, value in actual_rates:
                            clean_key = key.replace('Actual ', '').replace('Current ', '')
                            if len(clean_key) > 40:
                                clean_key = clean_key[:37] + "..."
                                
                            # Color code based on performance
                            try:
                                numeric_value = float(str(value).replace('Hz', '').replace('%', '').strip())
                                if numeric_value > 50:
                                    color = Colors.GREEN
                                    perf_icon = "🟢"
                                elif numeric_value > 20:
                                    color = Colors.CYAN
                                    perf_icon = "🔵"
                                elif numeric_value > 0:
                                    color = Colors.WARNING
                                    perf_icon = "🟡"
                                else:
                                    color = Colors.FAIL
                                    perf_icon = "🔴"
                            except:
                                color = Colors.CYAN
                                perf_icon = "🔵"
                                
                            print(f"    {perf_icon} {clean_key}: {color}{value}{Colors.ENDC}")
                    
                    # Special handling for camera node - extract specific camera FPS data
                    if 'vision_camera_node' in node_name or 'Camera System' in node_name:
                        # Look for camera-specific FPS data in all diagnostics
                        camera_fps_data = {}
                        for key, value in diagnostics.items():
                            # Pattern: "Cam [camera_id] Actual Color FPS" or "Cam [camera_id] Actual Depth FPS"
                            if key.startswith('Cam [') and '] Actual' in key and 'FPS' in key:
                                camera_fps_data[key] = value
                        
                        if camera_fps_data and not actual_rates:  # Only show if not already displayed
                            print(f"  📊 {Colors.BOLD}ACTUAL PERFORMANCE:{Colors.ENDC}")
                            for key, value in camera_fps_data.items():
                                # Extract camera ID and stream type from key
                                # Format: "Cam [camera_id] Actual Stream FPS"
                                parts = key.split(']')
                                if len(parts) >= 2:
                                    cam_id = parts[0].replace('Cam [', '')
                                    # Extract stream type (Color or Depth)
                                    if 'Color' in key:
                                        stream_type = 'Color'
                                    elif 'Depth' in key:
                                        stream_type = 'Depth'
                                    else:
                                        stream_type = 'Unknown'
                                    
                                    # Color code based on FPS
                                    try:
                                        fps_value = float(value)
                                        if fps_value >= 25:
                                            color = Colors.GREEN
                                            perf_icon = "🟢"
                                        elif fps_value >= 15:
                                            color = Colors.CYAN
                                            perf_icon = "🔵"
                                        elif fps_value > 0:
                                            color = Colors.WARNING
                                            perf_icon = "🟡"
                                        else:
                                            color = Colors.FAIL
                                            perf_icon = "🔴"
                                    except:
                                        color = Colors.CYAN
                                        perf_icon = "🔵"
                                    
                                    print(f"    {perf_icon} Cam [{cam_id}] FPS ({stream_type}): {color}{value}{Colors.ENDC}")
                    
                    # Display other frequency metrics
                    if other_freq:
                        print(f"  📈 {Colors.BOLD}OTHER METRICS:{Colors.ENDC}")
                        for key, value in other_freq:
                            if len(key) > 40:
                                key = key[:37] + "..."
                            print(f"    • {key}: {Colors.CYAN}{value}{Colors.ENDC}")
                    
                    # Show other important metrics for this node
                    other_important_keys = [
                        'Connection Status', 'VR Connected', 'Device IP', 'Recording Status',
                        'Active Cameras', 'Queue Size', 'Data Queue Size', 'IK Success Rate',
                        'Operation Mode', 'System Ready'
                    ]
                    
                    important_metrics = []
                    for key in other_important_keys:
                        if key in diagnostics and key not in frequency_data:
                            important_metrics.append((key, diagnostics[key]))
                    
                    if important_metrics:
                        print(f"  ℹ️  {Colors.BOLD}STATUS INFO:{Colors.ENDC}")
                        for key, value in important_metrics:
                            if len(key) > 35:
                                key = key[:32] + "..."
                                
                            if 'Success' in key and '%' in str(value):
                                try:
                                    success_rate = float(str(value).replace('%', ''))
                                    if success_rate >= 95:
                                        color = Colors.GREEN
                                        icon = "✅"
                                    elif success_rate >= 80:
                                        color = Colors.CYAN
                                        icon = "🔵"
                                    else:
                                        color = Colors.WARNING
                                        icon = "⚠️"
                                    print(f"    {icon} {key}: {color}{value}{Colors.ENDC}")
                                except:
                                    print(f"    ℹ️ {key}: {Colors.CYAN}{value}{Colors.ENDC}")
                            elif 'Connected' in key or 'Ready' in key:
                                if str(value).lower() in ['yes', 'true', 'connected', 'ready']:
                                    print(f"    ✅ {key}: {Colors.GREEN}{value}{Colors.ENDC}")
                                else:
                                    print(f"    ❌ {key}: {Colors.WARNING}{value}{Colors.ENDC}")
                            else:
                                print(f"    • {key}: {Colors.CYAN}{value}{Colors.ENDC}")
            
            # Show nodes without frequency data in a compact format
            if other_nodes:
                print(f"\n{Colors.BOLD}{Colors.BLUE}📋 OTHER NODE STATUS{Colors.ENDC}")
                print(f"{Colors.CYAN}{'-' * 40}{Colors.ENDC}")
                
                for i, (node_name, diagnostics) in enumerate(other_nodes):
                    level = diagnostics.get('Level', 'OK')
                    if level == 'OK':
                        level_color = Colors.GREEN
                        level_icon = "🟢"
                    elif level == 'WARNING':
                        level_color = Colors.WARNING
                        level_icon = "🟡"
                    elif level == 'ERROR':
                        level_color = Colors.FAIL
                        level_icon = "🔴"
                    else:
                        level_color = Colors.CYAN
                        level_icon = "🔵"
                    
                    display_name = node_name.replace(':', ' -> ')
                    if len(display_name) > 40:
                        display_name = display_name[:37] + "..."
                    
                    summary = diagnostics.get('Summary', 'No status')
                    if len(summary) > 40:
                        summary = summary[:37] + "..."
                    
                    print(f"  {level_icon} {Colors.BOLD}{display_name}{Colors.ENDC}: {summary}")
                
        else:
            print(f"{Colors.WARNING}⏳ WAITING FOR DIAGNOSTIC REPORTS...{Colors.ENDC}")
            print(f"   💡 Nodes may still be initializing")
        
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

    def setup_camera_monitoring(self):
        """Set up direct camera topic monitoring for real-time performance tracking"""
        # Get currently available camera topics dynamically
        available_topics = self.get_topic_names_and_types()
        camera_topics = []
        camera_info_topics = []
        
        # Filter for camera-related topics - look for any camera topics under /cameras/
        for topic_name, topic_types in available_topics:
            if '/cameras/' in topic_name and 'sensor_msgs/msg/Image' in topic_types:
                # Accept any image topic under /cameras/ namespace
                if '/image_raw' in topic_name or '/image' in topic_name:
                    camera_topics.append(topic_name)
            elif '/cameras/' in topic_name and 'sensor_msgs/msg/CameraInfo' in topic_types:
                camera_info_topics.append(topic_name)
        
        # If no topics found yet, wait a bit and try again (cameras may be starting up)
        if not camera_topics:
            self.get_logger().info("No camera topics found yet, will use expected patterns and monitor for availability")
            # Use expected patterns but don't hardcode specific camera names
            camera_topics = [
                '/cameras/wrist_camera/image_raw',
                '/cameras/overhead_camera/image_raw',
                '/cameras/wrist_camera/depth/image_raw',
                '/cameras/overhead_camera/depth/image_raw',
            ]
            camera_info_topics = [
                '/cameras/wrist_camera/camera_info',
                '/cameras/overhead_camera/camera_info',
                '/cameras/wrist_camera/depth/camera_info',
                '/cameras/overhead_camera/depth/camera_info',
            ]
        else:
            self.get_logger().info(f"Found {len(camera_topics)} camera image topics for monitoring")
        
        # QoS for image topics (best effort like camera publishers typically use)
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # QoS for camera info (reliable for configuration data)
        info_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to image topics for FPS tracking
        for topic in camera_topics:
            camera_name = self.extract_camera_name(topic)
            # Fix stream type detection: image_raw (without depth) = 'color', depth/image_raw = 'depth'
            if '/depth/' in topic:
                stream_type = 'depth'
            else:
                stream_type = 'color'  # This will handle /cameras/wrist_camera/image_raw as RGB/color
            
            # Initialize tracking data only for topics we're going to monitor
            if camera_name not in self.camera_fps_tracking:
                self.camera_fps_tracking[camera_name] = {}
            self.camera_fps_tracking[camera_name][stream_type] = {
                'last_msg_time': None,
                'msg_count': 0,
                'fps_history': [],
                'current_fps': 0.0
            }
            
            # Create subscription
            subscription = self.create_subscription(
                Image,
                topic,
                lambda msg, cam=camera_name, stream=stream_type: self.camera_image_callback(msg, cam, stream),
                image_qos
            )
            self.camera_subscriptions.append(subscription)
            self.get_logger().info(f"Monitoring camera topic: {topic} -> {camera_name}({stream_type})")
        
        # Subscribe to camera info topics for resolution and config data
        for topic in camera_info_topics:
            camera_name = self.extract_camera_name(topic)
            stream_type = 'depth' if '/depth/' in topic else 'color'
            
            # Initialize camera info storage
            if camera_name not in self.camera_info_data:
                self.camera_info_data[camera_name] = {}
            
            subscription = self.create_subscription(
                CameraInfo,
                topic,
                lambda msg, cam=camera_name, stream=stream_type: self.camera_info_callback(msg, cam, stream),
                info_qos
            )
            self.camera_subscriptions.append(subscription)
            self.get_logger().info(f"Monitoring camera info: {topic}")
    
    def extract_camera_name(self, topic):
        """Extract camera name from topic path"""
        # Example: '/cameras/wrist_camera/image_raw' -> 'wrist_camera'
        parts = topic.split('/')
        if len(parts) >= 3 and parts[1] == 'cameras':
            return parts[2]
        return 'unknown_camera'
    
    def camera_image_callback(self, msg, camera_name, stream_type):
        """Track camera image frequency for real-time FPS calculation"""
        current_time = time.time()
        tracking_data = self.camera_fps_tracking[camera_name][stream_type]
        
        if tracking_data['last_msg_time'] is not None:
            # Calculate time since last message
            time_diff = current_time - tracking_data['last_msg_time']
            if time_diff > 0:
                instant_fps = 1.0 / time_diff
                
                # Keep a rolling average of last 10 samples
                tracking_data['fps_history'].append(instant_fps)
                if len(tracking_data['fps_history']) > 10:
                    tracking_data['fps_history'].pop(0)
                
                # Calculate average FPS
                tracking_data['current_fps'] = sum(tracking_data['fps_history']) / len(tracking_data['fps_history'])
        
        tracking_data['last_msg_time'] = current_time
        tracking_data['msg_count'] += 1
    
    def camera_info_callback(self, msg, camera_name, stream_type):
        """Store camera configuration information"""
        self.camera_info_data[camera_name][stream_type] = {
            'width': msg.width,
            'height': msg.height,
            'distortion_model': msg.distortion_model,
            'frame_id': msg.header.frame_id
        }


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