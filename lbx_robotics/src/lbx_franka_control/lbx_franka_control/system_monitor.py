#!/usr/bin/env python3
"""
System Monitor Node

Centralized monitoring for the VR teleoperation system:
- Tracks VR controller performance (target: 60Hz)
- Tracks camera FPS for each camera stream
- Monitors robot control loop rates (target: 45Hz)
- Monitors system health and resource usage
- Publishes comprehensive diagnostics for display
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
from typing import Dict, List, Optional, Tuple

# Import MoveIt services
from moveit_msgs.srv import GetPositionIK, GetMotionPlan, GetPlanningScene


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
        if not self.camera_config_path or not os.path.exists(self.camera_config_path):
            self.get_logger().warn(f"Camera config not found: {self.camera_config_path}")
            return
        
        try:
            with open(self.camera_config_path, 'r') as f:
                camera_config = yaml.safe_load(f)
            
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
                
                # Subscribe to camera topics (using the pattern camera node uses)
                if serial:
                    # Color stream
                    color_topic = f"/cameras/cam_realsense_{serial}/image_raw"
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
                    
                    # Depth stream
                    depth_topic = f"/cameras/cam_realsense_{serial}/depth/image_raw"
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
        self.system_state = msg.data
    
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
        
        # VR Controller Performance
        vr_status = DiagnosticStatus()
        vr_status.name = "oculus_reader:vr_controller"
        vr_status.hardware_id = "oculus_quest"
        
        if self.vr_connected:
            vr_perf = self.vr_tracker.get_status()
            if vr_perf['efficiency'] >= 90:
                vr_status.level = DiagnosticStatus.OK
            elif vr_perf['efficiency'] >= 70:
                vr_status.level = DiagnosticStatus.WARN
            else:
                vr_status.level = DiagnosticStatus.ERROR
            
            vr_status.message = f"VR Controller @ {vr_perf['current_rate']:.1f} Hz ({vr_perf['efficiency']:.1f}% efficiency)"
        else:
            vr_status.level = DiagnosticStatus.ERROR
            vr_status.message = "VR Controller not connected"
        
        vr_status.values = [
            KeyValue(key="target_rate", value=f"{self.vr_tracker.target_rate}"),
            KeyValue(key="current_rate", value=f"{self.vr_tracker.current_rate:.1f}"),
            KeyValue(key="efficiency", value=f"{self.vr_tracker.get_efficiency():.1f}"),
            KeyValue(key="msg_count", value=str(self.vr_tracker.msg_count)),
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
        
        # Camera Performance
        if self.cameras_enabled and self.camera_trackers:
            # Group cameras by ID
            camera_groups = {}
            for tracker_name, tracker in self.camera_trackers.items():
                cam_id = getattr(tracker, 'camera_id', 'unknown')
                if cam_id not in camera_groups:
                    camera_groups[cam_id] = {}
                
                stream_type = 'color' if 'color' in tracker_name else 'depth'
                camera_groups[cam_id][stream_type] = tracker
            
            # Create diagnostic for each camera
            for cam_id, streams in camera_groups.items():
                cam_status = DiagnosticStatus()
                cam_status.name = f"vision_camera_node:camera_{cam_id}"
                cam_status.hardware_id = f"camera_{cam_id}"
                
                # Check if any stream is working
                color_tracker = streams.get('color')
                depth_tracker = streams.get('depth')
                
                working_streams = []
                total_efficiency = 0
                stream_count = 0
                
                for stream_type, tracker in streams.items():
                    if tracker and tracker.current_rate > 0:
                        working_streams.append(f"{stream_type}:{tracker.current_rate:.1f}Hz")
                        total_efficiency += tracker.get_efficiency()
                        stream_count += 1
                
                if working_streams:
                    avg_efficiency = total_efficiency / stream_count if stream_count > 0 else 0
                    if avg_efficiency >= 90:
                        cam_status.level = DiagnosticStatus.OK
                    elif avg_efficiency >= 70:
                        cam_status.level = DiagnosticStatus.WARN
                    else:
                        cam_status.level = DiagnosticStatus.ERROR
                    
                    cam_status.message = f"Camera active: {', '.join(working_streams)}"
                else:
                    cam_status.level = DiagnosticStatus.ERROR
                    cam_status.message = "Camera not publishing"
                
                # Add detailed values
                if color_tracker:
                    cam_status.values.extend([
                        KeyValue(key="color_target_fps", value=str(color_tracker.target_rate)),
                        KeyValue(key="color_current_fps", value=f"{color_tracker.current_rate:.1f}"),
                        KeyValue(key="color_efficiency", value=f"{color_tracker.get_efficiency():.1f}"),
                        KeyValue(key="color_msg_count", value=str(color_tracker.msg_count)),
                    ])
                
                if depth_tracker:
                    cam_status.values.extend([
                        KeyValue(key="depth_target_fps", value=str(depth_tracker.target_rate)),
                        KeyValue(key="depth_current_fps", value=f"{depth_tracker.current_rate:.1f}"),
                        KeyValue(key="depth_efficiency", value=f"{depth_tracker.get_efficiency():.1f}"),
                        KeyValue(key="depth_msg_count", value=str(depth_tracker.msg_count)),
                    ])
                
                if hasattr(color_tracker or depth_tracker, 'serial'):
                    cam_status.values.append(KeyValue(key="serial_number", value=getattr(color_tracker or depth_tracker, 'serial', '')))
                
                diag_array.status.append(cam_status)
        
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