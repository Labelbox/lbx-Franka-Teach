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
        
        # 1. Check VR Controller
        print(f"\n{Colors.CYAN}1️⃣  Checking VR Controller...{Colors.ENDC}")
        vr_status = await self.check_vr_controller()
        if vr_status:
            print(f"   ✅ VR Controller: {Colors.GREEN}Connected and responding{Colors.ENDC}")
        else:
            print(f"   ❌ VR Controller: {Colors.FAIL}Not detected{Colors.ENDC}")
            print(f"      Please ensure Oculus Quest is connected and oculus_reader is running")
        
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
        
        # Summary
        print(f"\n{Colors.BLUE}{'─' * 50}{Colors.ENDC}")
        all_healthy = vr_status and all(moveit_status.values()) and robot_status
        if all_healthy:
            print(f"{Colors.GREEN}{Colors.BOLD}✅ All systems operational!{Colors.ENDC}")
            self.system_ready = True
        else:
            print(f"{Colors.FAIL}{Colors.BOLD}❌ Some systems need attention{Colors.ENDC}")
            return False
        
        return True
    
    async def check_vr_controller(self):
        """Check if VR controller is connected and responding"""
        # Wait for VR pose data
        vr_pose_msg = None
        try:
            vr_pose_msg = await self.wait_for_message('/vr/controller_pose', PoseStamped, timeout=2.0)
            self.vr_healthy = vr_pose_msg is not None
        except:
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
        """Check if MoveIt services are available"""
        services = {
            '/compute_ik': False,
            '/compute_fk': False,
            '/get_planning_scene': False,
            '/fr3_arm_controller/follow_joint_trajectory': False,
            '/fr3_gripper/grasp': False
        }
        
        for service_name in services:
            try:
                # Check if service exists
                service_names = self.get_service_names_and_types()
                services[service_name] = any(service_name in s[0] for s in service_names)
            except:
                services[service_name] = False
        
        self.moveit_healthy = all(services.values())
        return services
    
    async def check_robot_connection(self):
        """Check robot connection via joint states"""
        try:
            msg = await self.wait_for_message('/joint_states', JointState, timeout=2.0)
            self.robot_healthy = msg is not None and len(msg.position) >= 7
        except:
            self.robot_healthy = False
        
        return self.robot_healthy
    
    async def wait_for_message(self, topic, msg_type, timeout=2.0):
        """Wait for a single message on a topic"""
        received_msg = None
        
        def msg_callback(msg):
            nonlocal received_msg
            received_msg = msg
        
        sub = self.create_subscription(msg_type, topic, msg_callback, 1)
        
        start_time = time.time()
        while received_msg is None and (time.time() - start_time) < timeout:
            await self.async_sleep(0.1)
        
        self.destroy_subscription(sub)
        return received_msg
    
    async def async_sleep(self, duration):
        """Async sleep helper"""
        await rclpy.task.sleep(duration)
    
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
        print(f"\n{Colors.CYAN}Controls:{Colors.ENDC}")
        print("   • {Colors.BOLD}Grip{Colors.ENDC}: Hold to enable robot movement")
        print("   • {Colors.BOLD}Trigger{Colors.ENDC}: Control gripper (pull to close)")
        print("   • {Colors.BOLD}A/X{Colors.ENDC}: Start/stop recording")
        print("   • {Colors.BOLD}B/Y{Colors.ENDC}: Mark recording as successful")
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
            return
        
        # Build status line
        status_parts = []
        
        # VR Status
        if self.latest_vr_state and self.latest_vr_state.grip_pressed:
            status_parts.append(f"🎮 VR: {Colors.GREEN}Active{Colors.ENDC}")
        else:
            status_parts.append(f"🎮 VR: {Colors.CYAN}Ready{Colors.ENDC}")
        
        # Robot Status
        if self.robot_healthy:
            status_parts.append(f"🤖 Robot: {Colors.GREEN}OK{Colors.ENDC}")
        else:
            status_parts.append(f"🤖 Robot: {Colors.FAIL}Error{Colors.ENDC}")
        
        # Recording Status
        if self.latest_system_status and self.latest_system_status.recording_active:
            status_parts.append(f"📹 Recording: {Colors.GREEN}Active{Colors.ENDC}")
        else:
            status_parts.append(f"📹 Recording: {Colors.WARNING}Off{Colors.ENDC}")
        
        # Performance
        if hasattr(self, 'control_rate'):
            if self.control_rate >= 44.0:  # Within 1Hz of target
                status_parts.append(f"⚡ Rate: {Colors.GREEN}{self.control_rate:.1f}Hz{Colors.ENDC}")
            else:
                status_parts.append(f"⚡ Rate: {Colors.WARNING}{self.control_rate:.1f}Hz{Colors.ENDC}")
        
        # Print status line
        timestamp = datetime.now().strftime("%H:%M:%S")
        status_line = f"[{timestamp}] " + " | ".join(status_parts)
        print(f"\r{status_line}", end='', flush=True)
    
    def cleanup(self):
        """Clean up resources"""
        print(f"\n\n{Colors.CYAN}Shutting down systems...{Colors.ENDC}")
        # Cleanup will be handled by ROS2 shutdown
    
    async def run(self):
        """Main run loop"""
        # Welcome message
        self.print_welcome_message()
        
        # Initialize system
        if not await self.initialize_system():
            print(f"\n{Colors.FAIL}System initialization failed. Please check errors above.{Colors.ENDC}")
            return
        
        # Reset robot
        self.reset_robot_to_home()
        
        # Calibration mode
        self.enter_calibration_mode()
        
        # Wait for calibration to complete
        print("Waiting for calibration...")
        while self.running:
            if self.latest_system_status and self.latest_system_status.system_state == 'teleop':
                if self.latest_system_status.calibration_mode == "":
                    print(f"\n{Colors.GREEN}✅ Calibration complete!{Colors.ENDC}")
                    break
            await self.async_sleep(0.1)
        
        # Start teleoperation
        self.start_teleoperation()
        
        # Main loop - just keep running
        while self.running:
            await self.async_sleep(0.1)


async def async_main():
    """Async main entry point"""
    # Initialize ROS2
    rclpy.init()
    
    # Parse launch parameters
    launch_params = {
        'vr_ip': os.environ.get('VR_IP', ''),
        'enable_cameras': os.environ.get('ENABLE_CAMERAS', 'false').lower() == 'true',
        'camera_config': os.environ.get('CAMERA_CONFIG', 'auto'),
        'hot_reload': os.environ.get('HOT_RELOAD', 'false').lower() == 'true',
        'verify_data': os.environ.get('VERIFY_DATA', 'false').lower() == 'true',
    }
    
    # Configuration path
    config_path = os.path.join(
        os.path.dirname(__file__),
        '../../../configs/control/franka_vr_control_config.yaml'
    )
    
    # Create main system node
    system = LabelboxRoboticsSystem(config_path, launch_params)
    
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