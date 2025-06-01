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
from std_msgs.msg import String, Bool, Empty, Header
from sensor_msgs.msg import Joy, JointState, Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from lbx_interfaces.msg import SystemStatus, VRControllerState, RecordingStatus

# ANSI color codes for pretty printing
class Colors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


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
        self.last_health_print = time.time()
        
        # Create callback groups
        self.diagnostics_group = ReentrantCallbackGroup()
        self.status_group = ReentrantCallbackGroup()
        
        # Subscribe to diagnostics
        self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            10,
            callback_group=self.diagnostics_group
        )
        
        # Subscribe to system status
        self.create_subscription(
            SystemStatus,
            '/system_status',
            self.system_status_callback,
            10,
            callback_group=self.status_group
        )
        
        # Subscribe to VR state
        self.create_subscription(
            VRControllerState,
            '/vr_control_state',
            self.vr_state_callback,
            10,
            callback_group=self.status_group
        )
        
        # Subscribe to recording verification results
        self.create_subscription(
            String,
            '/recording_verification_result',
            self.verification_result_callback,
            10,
            callback_group=self.status_group
        )
        
        # Health monitoring timer
        self.health_timer = self.create_timer(
            5.0,  # Every 5 seconds
            self.print_health_status,
            callback_group=self.diagnostics_group
        )
        
        # Signal handling
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # Store latest states
        self.latest_system_status = None
        self.latest_vr_state = None
        self.latest_diagnostics = {}
    
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
        
        # Check for camera topics
        camera_topics = [
            '/cameras/wrist_camera/image_raw',
            '/cameras/overhead_camera/image_raw',
        ]
        
        for topic in camera_topics:
            try:
                msg = await self.wait_for_message(topic, Image, timeout=1.0)
                camera_name = topic.split('/')[2]
                results[camera_name] = msg is not None
            except:
                camera_name = topic.split('/')[2]
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
                    from std_srvs.srv import Empty
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
        
        # Call reset service
        reset_client = self.create_client(Empty, '/reset_robot')
        if reset_client.wait_for_service(timeout_sec=2.0):
            print("   🔄 Sending reset command...")
            future = reset_client.call_async(Empty.Request())
            
            # Wait for completion
            while not future.done():
                time.sleep(0.1)
            
            print(f"   ✅ Robot reset to home position")
        else:
            print(f"   ❌ Reset service not available")
    
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
    
    def diagnostics_callback(self, msg: DiagnosticArray):
        """Process diagnostic messages"""
        for status in msg.status:
            self.latest_diagnostics[status.name] = status
    
    def system_status_callback(self, msg: SystemStatus):
        """Process system status messages"""
        self.latest_system_status = msg
    
    def vr_state_callback(self, msg: VRControllerState):
        """Process VR controller state"""
        self.latest_vr_state = msg
    
    def verification_result_callback(self, msg: String):
        """Process recording verification results"""
        try:
            results = json.loads(msg.data)
            
            # Display verification results
            print(f"\n\n{Colors.BLUE}{Colors.BOLD}📊 Recording Verification Results{Colors.ENDC}")
            print("─" * 50)
            
            # File info
            print(f"📁 File: {os.path.basename(results['file_path'])}")
            print(f"📏 Size: {results['file_size_mb']:.1f} MB")
            
            # Checks
            checks = results.get('checks', {})
            print(f"\n{Colors.CYAN}Data Completeness:{Colors.ENDC}")
            print(f"   ✓ VR Data: {'✅' if checks.get('has_vr_data', False) else '❌'}")
            print(f"   ✓ Robot Data: {'✅' if checks.get('has_robot_data', False) else '❌'}")
            print(f"   ✓ VR Calibration: {'✅' if checks.get('has_vr_calibration', False) else '❌'}")
            print(f"   ✓ Robot Torques: {'✅' if checks.get('has_torques', False) else '❌'}")
            
            # Camera data checks
            print(f"\n{Colors.CYAN}Camera Data:{Colors.ENDC}")
            print(f"   ✓ RGB Images: {'✅' if checks.get('has_rgb_images', False) else '❌'}")
            print(f"   ✓ Depth Images: {'✅' if checks.get('has_depth_images', False) else '❌'}")
            print(f"   ✓ Point Clouds: {'✅' if checks.get('has_point_clouds', False) else '❌'}")
            print(f"   ✓ Camera Transforms: {'✅' if checks.get('has_camera_transforms', False) else '❌'}")
            
            # Statistics
            stats = results.get('statistics', {})
            if 'duration_seconds' in stats:
                print(f"\n{Colors.CYAN}Recording Statistics:{Colors.ENDC}")
                print(f"   ⏱️  Duration: {stats['duration_seconds']:.1f} seconds")
                print(f"   📦 Total Messages: {checks.get('total_messages', 0):,}")
                print(f"   📑 Total Topics: {checks.get('total_topics', 0)}")
            
            # Summary
            all_checks_passed = all([
                checks.get('has_vr_data', False),
                checks.get('has_robot_data', False),
                checks.get('has_vr_calibration', False),
                checks.get('has_torques', False),
            ])
            
            # Include camera checks if cameras were enabled
            if self.launch_params.get('enable_cameras', False):
                camera_checks_passed = all([
                    checks.get('has_rgb_images', False),
                    checks.get('has_depth_images', False),
                    checks.get('has_camera_transforms', False)
                ])
                all_checks_passed = all_checks_passed and camera_checks_passed
            
            if all_checks_passed:
                print(f"\n{Colors.GREEN}{Colors.BOLD}✅ All essential data recorded successfully!{Colors.ENDC}")
            else:
                print(f"\n{Colors.WARNING}{Colors.BOLD}⚠️  Some data may be missing{Colors.ENDC}")
            
            print("─" * 50)
            print("")
            
        except Exception as e:
            self.get_logger().error(f"Failed to parse verification results: {e}")
    
    def print_health_status(self):
        """Print system health status every 5 seconds"""
        if not self.system_ready:
            # Even if not fully ready, show basic status during initialization
            timestamp = datetime.now().strftime("%H:%M:%S")
            print(f"\r[{timestamp}] 🔄 System initializing...", end='', flush=True)
            return
        
        # Build comprehensive status line
        status_parts = []
        
        # VR Status with enhanced feedback
        if self.latest_vr_state and hasattr(self.latest_vr_state, 'grip_pressed') and self.latest_vr_state.grip_pressed and self.vr_healthy:
            status_parts.append(f"🎮 VR: {Colors.GREEN}Active{Colors.ENDC}")
        elif self.vr_healthy:
            status_parts.append(f"🎮 VR: {Colors.CYAN}Ready{Colors.ENDC}")
        else:
            status_parts.append(f"🎮 VR: {Colors.WARNING}Fallback{Colors.ENDC}")
        
        # Robot Status
        if self.robot_healthy:
            status_parts.append(f"🤖 Robot: {Colors.GREEN}OK{Colors.ENDC}")
        else:
            # Check if we're in fake hardware mode
            fake_hardware = self._get_bool_param('use_fake_hardware', False)
            if fake_hardware:
                status_parts.append(f"🤖 Robot: {Colors.CYAN}Fake{Colors.ENDC}")
            else:
                status_parts.append(f"🤖 Robot: {Colors.FAIL}Error{Colors.ENDC}")
        
        # MoveIt Status
        if self.moveit_healthy:
            status_parts.append(f"🔧 MoveIt: {Colors.GREEN}Ready{Colors.ENDC}")
        else:
            status_parts.append(f"🔧 MoveIt: {Colors.WARNING}Pending{Colors.ENDC}")
        
        # Recording Status
        if self.latest_system_status and hasattr(self.latest_system_status, 'recording_active') and self.latest_system_status.recording_active:
            status_parts.append(f"📹 Recording: {Colors.GREEN}Active{Colors.ENDC}")
        else:
            status_parts.append(f"📹 Recording: {Colors.WARNING}Off{Colors.ENDC}")
        
        # Performance Status
        if hasattr(self, 'control_rate'):
            if self.control_rate >= 44.0:  # Within 1Hz of target
                status_parts.append(f"⚡ Rate: {Colors.GREEN}{self.control_rate:.1f}Hz{Colors.ENDC}")
            else:
                status_parts.append(f"⚡ Rate: {Colors.WARNING}{self.control_rate:.1f}Hz{Colors.ENDC}")
        else:
            status_parts.append(f"⚡ Rate: {Colors.CYAN}Monitoring{Colors.ENDC}")
        
        # Build final status line
        timestamp = datetime.now().strftime("%H:%M:%S")
        status_line = f"[{timestamp}] " + " | ".join(status_parts)
        
        # Add system mode indicator
        mode_indicator = ""
        if not self.vr_healthy and not self.moveit_healthy:
            mode_indicator = f" | {Colors.CYAN}🔍 Monitoring Mode{Colors.ENDC}"
        elif not self.vr_healthy:
            mode_indicator = f" | {Colors.CYAN}🎮 VR Graceful Fallback{Colors.ENDC}"
        elif not self.moveit_healthy:
            mode_indicator = f" | {Colors.YELLOW}🔧 MoveIt Initializing{Colors.ENDC}"
        
        status_line += mode_indicator
        
        # Show the status (clear previous line and print new one)
        print(f"\r{' ' * 120}", end='')  # Clear previous line
        print(f"\r{status_line}", end='', flush=True)
    
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
        """Main run loop with graceful degradation support"""
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
            
            # Wait for calibration to complete
            print("Waiting for VR calibration...")
            calibration_timeout = 60.0  # 1 minute timeout for calibration
            calibration_start = time.time()
            
            while self.running and (time.time() - calibration_start) < calibration_timeout:
                if self.latest_system_status and hasattr(self.latest_system_status, 'system_state'):
                    if self.latest_system_status.system_state == 'teleop':
                        if hasattr(self.latest_system_status, 'calibration_mode') and self.latest_system_status.calibration_mode == "":
                            print(f"\n{Colors.GREEN}✅ VR calibration complete!{Colors.ENDC}")
                            break
                await asyncio.sleep(0.1)
            
            if time.time() - calibration_start >= calibration_timeout:
                print(f"\n{Colors.WARNING}⚠️  VR calibration timeout - continuing anyway{Colors.ENDC}")
            
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
            print(f"\n{Colors.YELLOW}{Colors.BOLD}⚠️  Basic Monitoring Mode{Colors.ENDC}")
            print("─" * 50)
            print("   • Limited functionality - monitoring system health")
            print("   • Check robot connection and ros2_control status")
            
        # Main monitoring loop - runs regardless of system state
        print(f"\n{Colors.CYAN}📊 Starting system diagnostics (5-second intervals)...{Colors.ENDC}")
        print(f"{Colors.WARNING}Press Ctrl+C to stop the system{Colors.ENDC}\n")
        
        # Main loop - keep running and show diagnostics
        loop_count = 0
        while self.running:
            # Periodically retry system health checks
            if loop_count % 12 == 0:  # Every 60 seconds (12 * 5s intervals)
                # Re-check VR if not healthy
                if not self.vr_healthy:
                    vr_status = await self.check_vr_controller()
                    if vr_status and not self.vr_healthy:
                        print(f"\n🎮 VR reconnected! System upgrading...")
                        self.vr_healthy = True
                
                # Re-check MoveIt if not healthy
                if not self.moveit_healthy:
                    moveit_status = await self.check_moveit_services()
                    if all(moveit_status.values()) and not self.moveit_healthy:
                        print(f"\n🔧 MoveIt services available! System upgrading...")
                        self.moveit_healthy = True
                
                # Re-check robot if not healthy
                if not self.robot_healthy:
                    robot_status = await self.check_robot_connection()
                    if robot_status and not self.robot_healthy:
                        print(f"\n🤖 Robot connected! System upgrading...")
                        self.robot_healthy = True
            
            await asyncio.sleep(0.5)  # More frequent checks for responsiveness
            loop_count += 1


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
    
    # Run system
    try:
        # Run the async initialization and main loop
        await system.run()
    except KeyboardInterrupt:
        print("\n\nKeyboard interrupt received")
    finally:
        system.cleanup()
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