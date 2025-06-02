#!/usr/bin/env python3
"""
System Monitor Node

Centralized monitoring for the VR teleoperation system:
- Tracks VR controller performance (target: 60Hz)
- Tracks camera FPS for each camera stream
- Monitors robot control loop rates (target: 45Hz)
- Monitors system health and resource usage
- Publishes comprehensive diagnostics for display
- Enhanced display with actual FPS monitoring
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Joy, JointState, Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from lbx_interfaces.msg import SystemStatus, VRControllerState, RecordingStatus
from std_msgs.msg import String
import time
import psutil
import numpy as np
import os
import yaml
import json
import sys
from datetime import datetime
from typing import Dict, List, Optional, Tuple

# Import MoveIt services
from moveit_msgs.srv import GetPositionIK, GetMotionPlan, GetPlanningScene

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


class PerformanceTracker:
    """Track performance metrics for a specific component"""
    
    def __init__(self, name: str, target_rate: float):
        self.name = name
        self.target_rate = target_rate
        self.msg_count = 0
        self.last_msg_time = None
        self.rate_history = []
        self.max_history = 10
        self.last_rate_calc_time = time.time()
        self.current_rate = 0.0
    
    def update(self):
        """Called when a new message is received"""
        current_time = time.time()
        
        if self.last_msg_time is not None:
            time_diff = current_time - self.last_msg_time
            if time_diff > 0:
                instant_rate = 1.0 / time_diff
                self.rate_history.append(instant_rate)
                if len(self.rate_history) > self.max_history:
                    self.rate_history.pop(0)
                
                # Calculate average rate
                self.current_rate = sum(self.rate_history) / len(self.rate_history)
        
        self.last_msg_time = current_time
        self.msg_count += 1
    
    def get_efficiency(self) -> float:
        """Get efficiency as percentage of target rate"""
        if self.target_rate > 0:
            return (self.current_rate / self.target_rate) * 100
        return 0.0
    
    def get_status(self) -> Dict:
        """Get current status as dictionary"""
        return {
            'name': self.name,
            'target_rate': self.target_rate,
            'current_rate': self.current_rate,
            'efficiency': self.get_efficiency(),
            'msg_count': self.msg_count,
            'healthy': self.current_rate >= self.target_rate * 0.8
        }


class SystemMonitor(Node):
    """Centralized system monitoring node"""
    
    def __init__(self):
        super().__init__('system_monitor')
        
        # Load configuration parameters
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('camera_config', '')
        self.declare_parameter('enable_cameras', False)
        
        self.publish_rate = self.get_parameter('publish_rate').value
        self.camera_config_path = self.get_parameter('camera_config').value
        self.cameras_enabled = self.get_parameter('enable_cameras').value
        
        # Performance trackers
        self.vr_tracker = PerformanceTracker('vr_controller', 60.0)
        self.robot_control_tracker = PerformanceTracker('robot_control', 45.0)
        self.joint_state_tracker = PerformanceTracker('joint_states', 1000.0)
        self.camera_trackers = {}  # Will be populated based on camera config
        
        # System state tracking
        self.system_state = "initializing"
        self.recording_active = False
        self.teleoperation_enabled = False
        self.calibration_mode = "uncalibrated"
        
        # Component health
        self.vr_connected = False
        self.robot_connected = False
        self.moveit_ready = False
        self.cameras_healthy = True
        
        # Create service clients for MoveIt health check
        # Use a ReentrantCallbackGroup if these checks are done in a timer callback
        # that might run concurrently with other callbacks.
        # For simplicity, if check_component_health is called synchronously by the main timer,
        # a specific callback group might not be strictly needed here unless other parts of the node use one.
        self.compute_ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.plan_kinematic_path_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.get_planning_scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
        
        # Create diagnostics publisher
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            10
        )
        
        # Subscribe to diagnostics to read camera node status
        self.diagnostics_sub = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.read_diagnostics_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        
        # Store camera diagnostics from camera node
        self.camera_diagnostics_from_node = {}
        
        # Store VR diagnostics from oculus node
        self.vr_diagnostics_from_node = {}
        
        # Display control and system state tracking
        self.last_display_time = time.time()
        self.display_interval = 5.0  # Display every 5 seconds
        self.current_system_mode = "initializing"
        self.recording_active = False
        self.teleoperation_enabled = False
        self.calibration_mode = "uncalibrated"
        
        # Timer for publishing diagnostics
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_diagnostics)
        
        # Subscribe to system state
        self.system_state_sub = self.create_subscription(
            String,
            '/system/state',
            self.system_state_callback,
            1
        )
        
        # Subscribe to system status
        self.system_status_sub = self.create_subscription(
            SystemStatus,
            '/system/status',
            self.system_status_callback,
            1
        )
        
        # Subscribe to recording status
        self.recording_status_sub = self.create_subscription(
            RecordingStatus,
            '/recording_status',
            self.recording_status_callback,
            1
        )
        
        # VR monitoring
        self.vr_right_sub = self.create_subscription(
            PoseStamped,
            '/vr/right_controller/pose',
            lambda msg: self.vr_pose_callback(msg, 'right'),
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        self.vr_left_sub = self.create_subscription(
            PoseStamped,
            '/vr/left_controller/pose',
            lambda msg: self.vr_pose_callback(msg, 'left'),
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        # Joint state monitoring
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        # Robot control monitoring - subscribe to command topic
        self.robot_cmd_sub = self.create_subscription(
            JointState,
            '/robot/joint_position_commands',
            self.robot_command_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        # Initialize camera monitoring if enabled
        if self.cameras_enabled:
            self.setup_camera_monitoring()
        
        # Resource monitoring
        self.last_resource_check = time.time()
        self.resource_check_interval = 2.0  # Check every 2 seconds
        
        self.get_logger().info('System Monitor initialized')
        self.get_logger().info(f'  • Publish rate: {self.publish_rate} Hz')
        self.get_logger().info(f'  • Cameras enabled: {self.cameras_enabled}')
        if self.cameras_enabled:
            self.get_logger().info(f'  • Camera config: {self.camera_config_path}')
    
    def setup_camera_monitoring(self):
        """Set up camera performance monitoring based on config"""
        if not self.camera_config_path:
            self.get_logger().warn("No camera config path provided, skipping camera monitoring")
            return
            
        # Try multiple possible paths for the camera config
        possible_paths = [
            self.camera_config_path,  # Use provided path first
        ]
        
        # If the provided path doesn't exist, try alternative locations
        if not os.path.exists(self.camera_config_path):
            workspace_root = os.environ.get('COLCON_WS', os.getcwd())
            possible_paths.extend([
                os.path.join(workspace_root, 'lbx_robotics', 'configs', 'sensors', 'realsense_cameras.yaml'),
                os.path.join(workspace_root, 'configs', 'sensors', 'realsense_cameras.yaml'),
                os.path.join(os.path.dirname(workspace_root), 'lbx_robotics', 'configs', 'sensors', 'realsense_cameras.yaml'),
            ])
        
        config_found = False
        camera_config = None
        
        for config_path in possible_paths:
            if os.path.exists(config_path):
                try:
                    with open(config_path, 'r') as f:
                        camera_config = yaml.safe_load(f)
                    self.get_logger().info(f"Using camera config: {config_path}")
                    config_found = True
                    break
                except Exception as e:
                    self.get_logger().warn(f"Failed to load camera config {config_path}: {e}")
                    continue
        
        if not config_found:
            self.get_logger().warn("No valid camera config found, camera monitoring disabled")
            return
        
        try:
            # QoS for camera topics
            image_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
            
            # Create trackers and subscriptions for each enabled camera
            for cam_id, cam_cfg in camera_config.get('cameras', {}).items():
                if not cam_cfg.get('enabled', False):
                    continue
                
                # Get the base topic from camera config
                topics_config = cam_cfg.get('topics', {})
                base_topic = topics_config.get('base', f"/cameras/cam_{cam_id}")
                
                serial = cam_cfg.get('serial_number', '')
                target_fps = cam_cfg.get('color', {}).get('fps', 30)
                
                # Create trackers for color and depth streams
                color_tracker_name = f"{cam_id}_color"
                depth_tracker_name = f"{cam_id}_depth"
                
                self.camera_trackers[color_tracker_name] = PerformanceTracker(color_tracker_name, target_fps)
                self.camera_trackers[depth_tracker_name] = PerformanceTracker(depth_tracker_name, target_fps)
                
                # Store camera metadata
                for tracker_name in [color_tracker_name, depth_tracker_name]:
                    tracker = self.camera_trackers[tracker_name]
                    tracker.camera_id = cam_id
                    tracker.serial = serial
                    tracker.camera_type = cam_cfg.get('type', 'unknown')
                    tracker.base_topic = base_topic
                
                # Subscribe to camera topics using the base topic from config
                # Color stream: {base_topic}/image_raw
                color_topic = f"{base_topic}/image_raw"
                try:
                    self.create_subscription(
                        Image,
                        color_topic,
                        lambda msg, t=color_tracker_name: self.camera_image_callback(msg, t),
                        image_qos
                    )
                    self.get_logger().info(f"Monitoring camera: {cam_id} color @ {color_topic}")
                except Exception as e:
                    self.get_logger().warn(f"Failed to subscribe to {color_topic}: {e}")
                
                # Depth stream: {base_topic}/depth/image_raw (if depth enabled)
                depth_config = cam_cfg.get('depth', {})
                if depth_config.get('enabled', True):
                    depth_topic = f"{base_topic}/depth/image_raw"
                    try:
                        self.create_subscription(
                            Image,
                            depth_topic,
                            lambda msg, t=depth_tracker_name: self.camera_image_callback(msg, t),
                            image_qos
                        )
                        self.get_logger().info(f"Monitoring camera: {cam_id} depth @ {depth_topic}")
                    except Exception as e:
                        self.get_logger().warn(f"Failed to subscribe to {depth_topic}: {e}")
        
        except Exception as e:
            self.get_logger().error(f"Failed to setup camera monitoring: {e}")
    
    def system_state_callback(self, msg: String):
        """Track system state changes"""
        self.current_system_mode = msg.data
    
    def system_status_callback(self, msg: SystemStatus):
        """Track detailed system status"""
        self.teleoperation_enabled = msg.teleoperation_enabled
        self.calibration_mode = msg.calibration_mode
    
    def recording_status_callback(self, msg: RecordingStatus):
        """Track recording status"""
        self.recording_active = msg.recording_active
    
    def vr_pose_callback(self, msg: PoseStamped, controller: str):
        """Monitor VR pose messages"""
        self.vr_tracker.update()
        self.vr_connected = True
    
    def joint_state_callback(self, msg: JointState):
        """Monitor joint state messages"""
        if len(msg.position) >= 7:  # Ensure it's robot joint states
            self.joint_state_tracker.update()
            self.robot_connected = True
    
    def robot_command_callback(self, msg: JointState):
        """Monitor robot control commands"""
        self.robot_control_tracker.update()
    
    def camera_image_callback(self, msg: Image, tracker_name: str):
        """Monitor camera image messages"""
        if tracker_name in self.camera_trackers:
            self.camera_trackers[tracker_name].update()
    
    def get_system_resources(self) -> Dict:
        """Get current system resource usage"""
        try:
            cpu_percent = psutil.cpu_percent(interval=0.1)
            memory = psutil.virtual_memory()
            
            return {
                'cpu_percent': cpu_percent,
                'memory_percent': memory.percent,
                'memory_available_gb': memory.available / (1024**3),
                'memory_used_gb': memory.used / (1024**3)
            }
        except Exception:
            return {
                'cpu_percent': 0.0,
                'memory_percent': 0.0,
                'memory_available_gb': 0.0,
                'memory_used_gb': 0.0
            }
    
    def check_component_health(self):
        """Check health of all components"""
        current_time = time.time()
        
        # VR health - check if receiving messages
        vr_timeout = 2.0
        self.vr_connected = (self.vr_tracker.last_msg_time and 
                           (current_time - self.vr_tracker.last_msg_time) < vr_timeout)
        
        # Robot health - check if receiving joint states
        robot_timeout = 2.0
        self.robot_connected = (self.joint_state_tracker.last_msg_time and 
                              (current_time - self.joint_state_tracker.last_msg_time) < robot_timeout)
        
        # MoveIt health - check if key services are available
        # Note: service_is_ready() is non-blocking
        ik_ready = self.compute_ik_client.service_is_ready()
        planner_ready = self.plan_kinematic_path_client.service_is_ready()
        scene_ready = self.get_planning_scene_client.service_is_ready()
        
        if ik_ready and planner_ready and scene_ready:
            if not self.moveit_ready: # Log only on change
                self.get_logger().info("✅ MoveIt services are now ready.")
            self.moveit_ready = True
        else:
            if self.moveit_ready: # Log only on change
                self.get_logger().warn("⚠️ MoveIt services are no longer ready.")
                if not ik_ready: self.get_logger().warn("   - /compute_ik is NOT ready.")
                if not planner_ready: self.get_logger().warn("   - /plan_kinematic_path is NOT ready.")
                if not scene_ready: self.get_logger().warn("   - /get_planning_scene is NOT ready.")
            self.moveit_ready = False
        
        # Camera health - at least one camera should be working
        if self.cameras_enabled and self.camera_trackers:
            working_cameras = sum(1 for tracker in self.camera_trackers.values() 
                                if tracker.current_rate > tracker.target_rate * 0.5)
            total_cameras = len(self.camera_trackers) // 2  # Divide by 2 since we track color and depth separately
            self.cameras_healthy = working_cameras >= max(1, total_cameras // 2)  # At least half should work
        else:
            self.cameras_healthy = True  # If no cameras expected, consider healthy
    
    def publish_diagnostics(self):
        """Publish comprehensive diagnostic information"""
        # Update component health
        self.check_component_health()
        
        # Create diagnostic array
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        # System Overview
        system_status = DiagnosticStatus()
        system_status.name = "lbx_robotics_system:main_system"
        system_status.hardware_id = "lbx_system"
        
        # Determine overall health based on component status
        if self.robot_connected:
            if self.moveit_ready:
                if self.vr_connected:
                    system_status.level = DiagnosticStatus.OK
                    system_status.message = f"System fully operational - {self.system_state}"
                else:
                    system_status.level = DiagnosticStatus.WARN
                    system_status.message = f"Core system operational (VR fallback) - {self.system_state}"
            else: # MoveIt not ready
                system_status.level = DiagnosticStatus.WARN
                system_status.message = f"Robot connected, MoveIt services pending - {self.system_state}"
        else: # Robot not connected
            system_status.level = DiagnosticStatus.ERROR
            system_status.message = "Robot connection required"
        
        system_status.values = [
            KeyValue(key="system_state", value=self.system_state),
            KeyValue(key="recording_active", value=str(self.recording_active)),
            KeyValue(key="teleoperation_enabled", value=str(self.teleoperation_enabled)),
            KeyValue(key="calibration_mode", value=self.calibration_mode),
            KeyValue(key="vr_connected", value=str(self.vr_connected)),
            KeyValue(key="robot_connected", value=str(self.robot_connected)),
            KeyValue(key="cameras_healthy", value=str(self.cameras_healthy)),
            KeyValue(key="moveit_ready", value=str(self.moveit_ready)),
            KeyValue(key="moveit_ik_service", value="Ready" if self.compute_ik_client.service_is_ready() else "Pending"),
            KeyValue(key="moveit_planner_service", value="Ready" if self.plan_kinematic_path_client.service_is_ready() else "Pending"),
            KeyValue(key="moveit_scene_service", value="Ready" if self.get_planning_scene_client.service_is_ready() else "Pending")
        ]
        diag_array.status.append(system_status)
        
        # VR Controller Performance - Enhanced to show actual status from oculus node
        vr_status = DiagnosticStatus()
        vr_status.name = "system_monitor:vr_system_overview"
        vr_status.hardware_id = "vr_system"
        
        # Get VR diagnostics from oculus node
        vr_connection_diag = self.vr_diagnostics_from_node.get("vr_controller") or \
                            self.vr_diagnostics_from_node.get("Oculus Connection")
        vr_data_rates_diag = self.vr_diagnostics_from_node.get("Oculus Data Rates")
        
        if vr_connection_diag or vr_data_rates_diag:
            # Use actual VR diagnostics
            vr_connected = False
            connection_status = "Unknown"
            
            # Parse connection status
            if vr_connection_diag:
                connection_status = vr_connection_diag.message
                vr_connected = vr_connection_diag.level == DiagnosticStatus.OK
            
            # Parse performance data
            target_poll_rate = "N/A"
            actual_poll_rate = "N/A"
            target_publish_rate = "N/A"
            actual_publish_rate = "N/A"
            queue_size = "N/A"
            efficiency = 0.0
            
            if vr_data_rates_diag:
                for kv in vr_data_rates_diag.values:
                    if kv.key == "Target Poll Rate (Hz)":
                        target_poll_rate = kv.value
                    elif kv.key == "Actual Poll Rate (Hz)":
                        actual_poll_rate = kv.value
                    elif kv.key == "Target Publish Rate (Hz)":
                        target_publish_rate = kv.value
                    elif kv.key == "Actual Publish Rate (Hz)":
                        actual_publish_rate = kv.value
                    elif kv.key == "Data Queue Size":
                        queue_size = kv.value
                    elif kv.key == "VR Connected":
                        vr_connected = vr_connected or (kv.value == "Yes")
            
            # Calculate efficiency if we have numeric data
            try:
                if target_publish_rate != "N/A" and actual_publish_rate != "N/A":
                    target_val = float(target_publish_rate)
                    actual_val = float(actual_publish_rate)
                    if target_val > 0:
                        efficiency = (actual_val / target_val) * 100
            except ValueError:
                efficiency = 0.0
            
            # Set status level based on connection and performance
            if vr_connected:
                if efficiency >= 90:
                    vr_status.level = DiagnosticStatus.OK
                elif efficiency >= 70:
                    vr_status.level = DiagnosticStatus.WARN
                else:
                    vr_status.level = DiagnosticStatus.ERROR
                vr_status.message = f"VR Connected - Poll:{actual_poll_rate}Hz, Pub:{actual_publish_rate}Hz ({efficiency:.1f}%)"
            else:
                vr_status.level = DiagnosticStatus.ERROR
                vr_status.message = f"VR Disconnected - {connection_status}"
            
            vr_status.values = [
                KeyValue(key="vr_connected", value="Yes" if vr_connected else "No"),
                KeyValue(key="connection_status", value=connection_status),
                KeyValue(key="target_poll_rate_hz", value=target_poll_rate),
                KeyValue(key="actual_poll_rate_hz", value=actual_poll_rate),
                KeyValue(key="target_publish_rate_hz", value=target_publish_rate),
                KeyValue(key="actual_publish_rate_hz", value=actual_publish_rate),
                KeyValue(key="efficiency_percent", value=f"{efficiency:.1f}"),
                KeyValue(key="data_queue_size", value=queue_size),
            ]
        else:
            # Fallback to pose tracking if no VR diagnostics available
            if self.vr_connected:
                vr_perf = self.vr_tracker.get_status()
                if vr_perf['efficiency'] >= 90:
                    vr_status.level = DiagnosticStatus.OK
                elif vr_perf['efficiency'] >= 70:
                    vr_status.level = DiagnosticStatus.WARN
                else:
                    vr_status.level = DiagnosticStatus.ERROR
                
                vr_status.message = f"VR Controller @ {vr_perf['current_rate']:.1f} Hz ({vr_perf['efficiency']:.1f}% efficiency)"
                vr_status.values = [
                    KeyValue(key="target_rate", value=f"{self.vr_tracker.target_rate}"),
                    KeyValue(key="current_rate", value=f"{self.vr_tracker.current_rate:.1f}"),
                    KeyValue(key="efficiency", value=f"{self.vr_tracker.get_efficiency():.1f}"),
                    KeyValue(key="msg_count", value=str(self.vr_tracker.msg_count)),
                ]
            else:
                vr_status.level = DiagnosticStatus.ERROR
                vr_status.message = "VR Controller not connected"
                vr_status.values = [
                    KeyValue(key="vr_connected", value="No"),
                    KeyValue(key="diagnostics_available", value="No"),
                ]
        
        diag_array.status.append(vr_status)
        
        # Robot Control Performance
        if self.system_state == "teleoperation" and self.teleoperation_enabled:
            control_status = DiagnosticStatus()
            control_status.name = "robot_control_node:control_loop"
            control_status.hardware_id = "control_system"
            
            control_perf = self.robot_control_tracker.get_status()
            if control_perf['current_rate'] > 0:
                if control_perf['efficiency'] >= 90:
                    control_status.level = DiagnosticStatus.OK
                elif control_perf['efficiency'] >= 70:
                    control_status.level = DiagnosticStatus.WARN
                else:
                    control_status.level = DiagnosticStatus.ERROR
                
                control_status.message = f"Control loop @ {control_perf['current_rate']:.1f} Hz ({control_perf['efficiency']:.1f}% efficiency)"
            else:
                control_status.level = DiagnosticStatus.WARN
                control_status.message = "Control loop starting up"
            
            control_status.values = [
                KeyValue(key="target_rate", value=f"{self.robot_control_tracker.target_rate}"),
                KeyValue(key="current_rate", value=f"{self.robot_control_tracker.current_rate:.1f}"),
                KeyValue(key="efficiency", value=f"{self.robot_control_tracker.get_efficiency():.1f}"),
                KeyValue(key="msg_count", value=str(self.robot_control_tracker.msg_count)),
            ]
            diag_array.status.append(control_status)
        
        # System Resources
        if time.time() - self.last_resource_check > self.resource_check_interval:
            self.last_resource_check = time.time()
            resources = self.get_system_resources()
            
            resource_status = DiagnosticStatus()
            resource_status.name = "system_monitor:resources"
            resource_status.hardware_id = "compute"
            
            if resources['cpu_percent'] < 80 and resources['memory_percent'] < 80:
                resource_status.level = DiagnosticStatus.OK
                resource_status.message = "System resources normal"
            elif resources['cpu_percent'] < 90 and resources['memory_percent'] < 90:
                resource_status.level = DiagnosticStatus.WARN
                resource_status.message = "System resources elevated"
            else:
                resource_status.level = DiagnosticStatus.ERROR
                resource_status.message = "System resources critical"
            
            resource_status.values = [
                KeyValue(key="cpu_percent", value=f"{resources['cpu_percent']:.1f}"),
                KeyValue(key="memory_percent", value=f"{resources['memory_percent']:.1f}"),
                KeyValue(key="memory_available_gb", value=f"{resources['memory_available_gb']:.1f}"),
                KeyValue(key="memory_used_gb", value=f"{resources['memory_used_gb']:.1f}"),
            ]
            diag_array.status.append(resource_status)
        
        # Publish diagnostics
        self.diagnostics_pub.publish(diag_array)
        
        # Check if it's time to display diagnostic summary
        current_time = time.time()
        if current_time - self.last_display_time > self.display_interval:
            self.last_display_time = current_time
            self.print_enhanced_diagnostic_summary()

    def print_enhanced_diagnostic_summary(self):
        """Print comprehensive system health summary with individual camera FPS data"""
        
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
        
        # VR Controller Performance & Status - Enhanced
        print(f"\n{Colors.BOLD}🎮 VR STATUS & PERFORMANCE:{Colors.ENDC}")
        
        # Get VR diagnostics from oculus node (using the exact cleaned keys)
        vr_connection_diag = self.vr_diagnostics_from_node.get("Oculus Connection")
        vr_data_rates_diag = self.vr_diagnostics_from_node.get("Oculus Data Rates")
        
        if vr_connection_diag and vr_data_rates_diag:
            vr_connected = vr_connection_diag.level == DiagnosticStatus.OK
            connection_status = vr_connection_diag.message
            
            target_poll_rate = self._get_diagnostic_value(vr_data_rates_diag, 'Target Poll Rate (Hz)')
            actual_poll_rate = self._get_diagnostic_value(vr_data_rates_diag, 'Actual Poll Rate (Hz)')
            target_publish_rate = self._get_diagnostic_value(vr_data_rates_diag, 'Target Publish Rate (Hz)')
            actual_publish_rate = self._get_diagnostic_value(vr_data_rates_diag, 'Actual Publish Rate (Hz)')
            queue_size = self._get_diagnostic_value(vr_data_rates_diag, 'Data Queue Size')
            
            print(f"   • Connection: {Colors.GREEN if vr_connected else Colors.FAIL}{connection_status}{Colors.ENDC}")
            
            if vr_connected and actual_publish_rate != 'N/A':
                try:
                    actual_publish_float = float(actual_publish_rate)
                    efficiency_float = 0.0
                    if target_publish_rate != 'N/A':
                        target_pub = float(target_publish_rate)
                        if target_pub > 0:
                            efficiency_float = (actual_publish_float / target_pub) * 100
                except ValueError:
                    actual_publish_float = 0.0
                    efficiency_float = 0.0
                
                if actual_poll_rate != 'N/A':
                    try:
                        poll_rate_float = float(actual_poll_rate)
                        poll_color = Colors.GREEN if poll_rate_float >= 55 else Colors.WARNING if poll_rate_float >= 30 else Colors.FAIL
                        print(f"   • Poll Rate: {poll_color}{actual_poll_rate} Hz{Colors.ENDC} (Target: {target_poll_rate} Hz)")
                    except ValueError: pass
                
                fps_color = Colors.GREEN if actual_publish_float >= 55 else Colors.WARNING if actual_publish_float >= 30 else Colors.FAIL
                print(f"   • Publish Rate: {fps_color}{actual_publish_rate} Hz{Colors.ENDC} (Target: {target_publish_rate} Hz)")
                
                eff_color = Colors.GREEN if efficiency_float >= 90 else Colors.WARNING if efficiency_float >= 70 else Colors.FAIL
                print(f"   • Efficiency: {eff_color}{efficiency_float:.1f}%{Colors.ENDC}")
                
                if queue_size != 'N/A':
                    try:
                        queue_int = int(queue_size)
                        queue_color = Colors.GREEN if queue_int < 5 else Colors.WARNING if queue_int < 10 else Colors.FAIL
                        print(f"   • Queue Size: {queue_color}{queue_size}{Colors.ENDC}")
                    except ValueError: pass
            elif vr_connected: # Connected but no rate data
                 print(f"   {Colors.YELLOW}• Performance data pending...{Colors.ENDC}")
            else: # Not connected
                print(f"   {Colors.FAIL}• VR Controller disconnected or no data available{Colors.ENDC}")
        else:
            print(f"   {Colors.WARNING}• VR diagnostics not yet received from oculus_reader.{Colors.ENDC}")
        
        # Individual Camera Diagnostics Section
        if self.cameras_enabled or self.camera_diagnostics_from_node:
            # Camera System Overview (using the cleaned key "Camera System Status")
            camera_system_overview = self.camera_diagnostics_from_node.get("Camera System Status")
            if camera_system_overview:
                cameras_detected = self._get_diagnostic_value(camera_system_overview, 'cameras_detected')
                cameras_active = self._get_diagnostic_value(camera_system_overview, 'cameras_active')
                cameras_configured = self._get_diagnostic_value(camera_system_overview, 'cameras_configured')
                overview_status_msg = camera_system_overview.message
                overview_level_str = self._diagnostic_level_to_string(camera_system_overview.level)
                overview_color = Colors.GREEN if overview_level_str == 'OK' else Colors.WARNING if overview_level_str == 'WARNING' else Colors.FAIL
                
                print(f"\n🔧 {Colors.BOLD}CAMERA SYSTEM OVERVIEW:{Colors.ENDC}")
                print(f"   Status: {overview_color}{overview_status_msg}{Colors.ENDC}")
                print(f"   Configured: {cameras_configured} | Detected: {cameras_detected} | Active: {cameras_active}")
            
            print(f"\n📊 {Colors.BOLD}INDIVIDUAL CAMERA PERFORMANCE:{Colors.ENDC}")
            print(f"{Colors.CYAN}{'-' * 60}{Colors.ENDC}")
            
            individual_cameras_found_and_displayed = False
            
            for diag_key_name, cam_status_obj in self.camera_diagnostics_from_node.items():
                # Process active individual cameras: "Camera <ID>" (exact match after cleaning)
                if diag_key_name.startswith("Camera ") and "(Unavailable)" not in diag_key_name and diag_key_name != "Camera System Status":
                    individual_cameras_found_and_displayed = True
                    cam_id_str = diag_key_name.replace("Camera ", "").strip()
                    serial_num = self._get_diagnostic_value(cam_status_obj, 'Actual SN') or self._get_diagnostic_value(cam_status_obj, 'serial_number') # Ensure this matches camera_node.py
                    status_msg_str = cam_status_obj.message
                    level_str_val = self._diagnostic_level_to_string(cam_status_obj.level)
                    
                    status_color_val = Colors.GREEN if level_str_val == 'OK' else Colors.WARNING if level_str_val == 'WARNING' else Colors.FAIL
                    
                    print(f"\n📷 {Colors.BOLD}{Colors.BLUE}Camera: {cam_id_str}{Colors.ENDC}")
                    print(f"   Serial Number: {Colors.CYAN}{serial_num}{Colors.ENDC}")
                    print(f"   Status: {status_color_val}{level_str_val} - {status_msg_str}{Colors.ENDC}")
                    
                    rgb_target_fps = self._get_diagnostic_value(cam_status_obj, 'Target Color FPS')
                    rgb_actual_fps = self._get_diagnostic_value(cam_status_obj, 'Actual Color FPS')
                    rgb_efficiency = self._get_diagnostic_value(cam_status_obj, 'Color Efficiency (%)')
                    rgb_frames = self._get_diagnostic_value(cam_status_obj, 'Color Frames Published')
                    
                    if rgb_actual_fps != 'N/A' and rgb_target_fps != 'N/A': # Ensure we have data to process
                        try:
                            rgb_actual_float = float(rgb_actual_fps)
                            rgb_target_float = float(rgb_target_fps)
                            fps_color = Colors.GREEN if rgb_actual_float >= (rgb_target_float * 0.9) else Colors.WARNING if rgb_actual_float >= (rgb_target_float * 0.7) else Colors.FAIL
                            eff_val = "N/A"
                            if rgb_efficiency != 'N/A': 
                                try: eff_val = f"{float(rgb_efficiency):.1f}" 
                                except: pass # Keep N/A if conversion fails

                            print(f"   📸 {Colors.BOLD}RGB Stream:{Colors.ENDC}")
                            print(f"      • FPS: {fps_color}{rgb_actual_fps} Hz{Colors.ENDC} (Target: {rgb_target_fps} Hz)")
                            print(f"      • Efficiency: {fps_color}{eff_val}%{Colors.ENDC}")
                            if rgb_frames != 'N/A': print(f"      • Frames Published: {Colors.CYAN}{rgb_frames}{Colors.ENDC}")
                        except ValueError: print(f"   📸 {Colors.BOLD}RGB Stream:{Colors.ENDC} {Colors.WARNING}Data format error (RGB){Colors.ENDC}")
                    else: print(f"   📸 {Colors.BOLD}RGB Stream:{Colors.ENDC} {Colors.FAIL}No FPS data{Colors.ENDC}")
                    
                    depth_target_fps = self._get_diagnostic_value(cam_status_obj, 'Target Depth FPS')
                    depth_actual_fps = self._get_diagnostic_value(cam_status_obj, 'Actual Depth FPS')
                    depth_efficiency = self._get_diagnostic_value(cam_status_obj, 'Depth Efficiency (%)')
                    depth_frames = self._get_diagnostic_value(cam_status_obj, 'Depth Frames Published')
                    
                    if depth_actual_fps != 'N/A' and depth_target_fps != 'N/A': # Ensure we have data
                        try:
                            depth_actual_float = float(depth_actual_fps)
                            depth_target_float = float(depth_target_fps)
                            fps_color = Colors.GREEN if depth_actual_float >= (depth_target_float * 0.9) else Colors.WARNING if depth_actual_float >= (depth_target_float * 0.7) else Colors.FAIL
                            eff_val = "N/A"
                            if depth_efficiency != 'N/A': 
                                try: eff_val = f"{float(depth_efficiency):.1f}" 
                                except: pass

                            print(f"   🔍 {Colors.BOLD}Depth Stream:{Colors.ENDC}")
                            print(f"      • FPS: {fps_color}{depth_actual_fps} Hz{Colors.ENDC} (Target: {depth_target_fps} Hz)")
                            print(f"      • Efficiency: {fps_color}{eff_val}%{Colors.ENDC}")
                            if depth_frames != 'N/A': print(f"      • Frames Published: {Colors.CYAN}{depth_frames}{Colors.ENDC}")
                        except ValueError: print(f"   🔍 {Colors.BOLD}Depth Stream:{Colors.ENDC} {Colors.WARNING}Data format error (Depth){Colors.ENDC}")
                    else: print(f"   🔍 {Colors.BOLD}Depth Stream:{Colors.ENDC} {Colors.FAIL}No FPS data{Colors.ENDC}")
                    print(f"   {Colors.CYAN}{'.' * 40}{Colors.ENDC}")

            # Process unavailable cameras: "Camera <ID> (Unavailable)" (exact match after cleaning)
            for diag_key_name, cam_status_obj in self.camera_diagnostics_from_node.items():
                if diag_key_name.startswith("Camera ") and "(Unavailable)" in diag_key_name:
                    individual_cameras_found_and_displayed = True
                    cam_id_str = diag_key_name.replace("Camera ", "").replace(" (Unavailable)", "").strip()
                    configured_sn = self._get_diagnostic_value(cam_status_obj, 'Configured SN/Idx')
                    status_msg_str = cam_status_obj.message
                    
                    print(f"\n📷 {Colors.BOLD}{Colors.RED}Camera: {cam_id_str} (UNAVAILABLE){Colors.ENDC}")
                    print(f"   Configured SN: {Colors.CYAN}{configured_sn}{Colors.ENDC}")
                    print(f"   Status: {Colors.FAIL}{status_msg_str}{Colors.ENDC}")
                    print(f"   📸 RGB Stream: {Colors.FAIL}Not available{Colors.ENDC}")
                    print(f"   🔍 Depth Stream: {Colors.FAIL}Not available{Colors.ENDC}")
                    print(f"   {Colors.CYAN}{'.' * 40}{Colors.ENDC}")
            
            if not individual_cameras_found_and_displayed:
                # Fallback for old-style keys or if new keys are somehow missed
                legacy_or_unprocessed_keys = [k for k in self.camera_diagnostics_from_node.keys() if k != "Camera System Status"]
                if legacy_or_unprocessed_keys:
                    print(f"{Colors.WARNING}Displaying other (possibly legacy or unprocessed) camera diagnostics:{Colors.ENDC}")
                    for key in legacy_or_unprocessed_keys:
                        status = self.camera_diagnostics_from_node[key]
                        level_str = self._diagnostic_level_to_string(status.level)
                        level_color = Colors.GREEN if level_str == 'OK' else Colors.WARNING if level_str == 'WARNING' else Colors.FAIL
                        print(f"  • {Colors.BOLD}{key}{Colors.ENDC}: {level_color}{level_str}{Colors.ENDC} - {status.message}")
                        for kv in status.values: print(f"      {kv.key}: {kv.value}")
                else:
                    print(f"{Colors.WARNING}No individual camera performance data found.{Colors.ENDC}")
                    print(f"{Colors.CYAN}💡 Verify camera_node is publishing diagnostics with names like 'Camera <ID>'.{Colors.ENDC}")
        
        # System Resources
        try:
            resources = self.get_system_resources()
            cpu = resources['cpu_percent']
            memory = resources['memory_percent']
            
            if cpu > 0 or memory > 0:
                print(f"\n💻 SYSTEM RESOURCES{Colors.ENDC}")
                print(f"{Colors.CYAN}{'-' * 40}{Colors.ENDC}")
                
                cpu_color = Colors.GREEN if cpu < 80 else Colors.WARNING if cpu < 90 else Colors.FAIL
                mem_color = Colors.GREEN if memory < 80 else Colors.WARNING if memory < 90 else Colors.FAIL
                
                print(f"  • CPU Usage: {cpu_color}{cpu:.1f}%{Colors.ENDC}")
                print(f"  • Memory Usage: {mem_color}{memory:.1f}%{Colors.ENDC}")
        except Exception:
            pass
        
        # MoveIt Services Status
        print(f"\n🔧 MOVEIT SERVICES STATUS{Colors.ENDC}")
        print(f"{Colors.CYAN}{'-' * 40}{Colors.ENDC}")
        
        if self.moveit_ready:
            print(f"  {Colors.GREEN}✅ All core MoveIt services reported as ready.{Colors.ENDC}")
        else:
            print(f"  {Colors.WARNING}⚠️ Some MoveIt services may not be ready{Colors.ENDC}")

        ik_ready = self.compute_ik_client.service_is_ready()
        planner_ready = self.plan_kinematic_path_client.service_is_ready()
        scene_ready = self.get_planning_scene_client.service_is_ready()

        print(f"    ├─ IK Service (/compute_ik): {Colors.GREEN if ik_ready else Colors.WARNING}{'Ready' if ik_ready else 'Pending'}{Colors.ENDC}")
        print(f"    ├─ Planner Service (/plan_kinematic_path): {Colors.GREEN if planner_ready else Colors.WARNING}{'Ready' if planner_ready else 'Pending'}{Colors.ENDC}")
        print(f"    └─ Scene Service (/get_planning_scene): {Colors.GREEN if scene_ready else Colors.WARNING}{'Ready' if scene_ready else 'Pending'}{Colors.ENDC}")

        # Footer with quick actions
        print(f"\n🚀 QUICK ACTIONS{Colors.ENDC}")
        print(f"{Colors.CYAN}{'-' * 20}{Colors.ENDC}")
        print(f"• Live diagnostics: {Colors.YELLOW}ros2 topic echo /diagnostics{Colors.ENDC}")
        print(f"• Check camera topics: {Colors.YELLOW}ros2 topic list | grep cam{Colors.ENDC}")
        print(f"• Check camera FPS: {Colors.YELLOW}ros2 topic hz /cameras/cam_<SERIAL>/image_raw{Colors.ENDC}")
        print(f"• Emergency stop: {Colors.RED}Ctrl+C{Colors.ENDC}")
        print(f"{Colors.CYAN}{'=' * 80}{Colors.ENDC}")
        
        # Force flush output
        sys.stdout.flush()

    def _get_diagnostic_value(self, status: DiagnosticStatus, key: str) -> str:
        """Helper to extract value from diagnostic status by key"""
        for kv in status.values:
            if kv.key == key:
                return kv.value
        return 'N/A'
    
    def _diagnostic_level_to_string(self, level: int) -> str:
        """Convert diagnostic level to human-readable string"""
        level_map = {
            DiagnosticStatus.OK: "OK",
            DiagnosticStatus.WARN: "WARNING", 
            DiagnosticStatus.ERROR: "ERROR",
            DiagnosticStatus.STALE: "STALE"
        }
        return level_map.get(level, f"UNKNOWN({level})")

    def read_diagnostics_callback(self, msg: DiagnosticArray):
        """Read camera and VR node status from diagnostics"""
        for status in msg.status:
            # Capture camera diagnostics
            if status.name.startswith("vision_camera_node:"):
                # Remove prefix and strip leading/trailing whitespace for clean keys
                cam_diag_name = status.name.replace("vision_camera_node:", "").strip()
                self.camera_diagnostics_from_node[cam_diag_name] = status
            
            # Capture VR diagnostics from oculus node - handle multiple diagnostic types
            elif status.name.startswith("oculus_reader:"):
                # Store VR diagnostics by their specific name (remove prefix and strip)
                vr_diag_name = status.name.replace("oculus_reader:", "").strip()
                self.vr_diagnostics_from_node[vr_diag_name] = status
            # Keep this for backward compatibility or other oculus diagnostics an old node might publish
            elif "Oculus" in status.name and status.name.startswith("oculus_reader: "): 
                vr_diag_name = status.name.replace("oculus_reader: ", "").strip()
                self.vr_diagnostics_from_node[vr_diag_name] = status


def main(args=None):
    rclpy.init(args=args)
    
    monitor = SystemMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info("Shutting down system monitor...")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 