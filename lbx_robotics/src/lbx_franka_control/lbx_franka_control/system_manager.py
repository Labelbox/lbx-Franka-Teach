#!/usr/bin/env python3
"""
System Manager Node - Simplified Recording and Service Manager

This node provides system-level services and recording management:
- Recording control (start/stop/mark success)
- System-wide services (reset_robot, emergency_stop)
- NO direct robot control (handled by robot_control_node)
- NO VR processing (handled by vr_teleop_node)

The node acts as a lightweight coordinator for recording and system services.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.action import ActionClient

import time
import threading
import yaml
import os
import shutil
from typing import Dict, Optional
from datetime import datetime

# ROS 2 messages
from std_msgs.msg import String, Bool, Int32
from std_srvs.srv import Empty, Trigger
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

# Recording interfaces
from lbx_interfaces.msg import RecordingStatus
from lbx_interfaces.srv import StartRecording, StopRecording


class SystemManager(Node):
    """Simplified system manager for recording and system services"""
    
    def __init__(self, config_path: str):
        super().__init__('lbx_system_manager')
        
        # Load configuration
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # State
        self.recording_active = False
        self.recording_start_time = None
        self.recording_file_path = None
        self.recording_success = False
        
        # Callback group
        self.callback_group = ReentrantCallbackGroup()
        
        # Publishers
        self.recording_status_pub = self.create_publisher(
            RecordingStatus, '/recording_status', 1
        )
        
        # Subscribers for recording triggers from VR
        self.start_recording_sub = self.create_subscription(
            Bool,
            '/recording/start_request',
            self.handle_start_recording_request,
            1
        )
        
        self.stop_recording_sub = self.create_subscription(
            Int32,
            '/recording/stop_request',
            self.handle_stop_recording_request,
            1
        )
        
        # Services for recording control
        self.create_service(Empty, 'start_recording', self.start_recording_callback)
        self.create_service(Empty, 'stop_recording', self.stop_recording_callback)
        self.create_service(Empty, 'mark_recording_success', self.mark_recording_success_callback)
        
        # System-wide services
        self.create_service(Empty, 'reset_robot', self.reset_robot_callback)
        self.create_service(Empty, 'emergency_stop', self.emergency_stop_callback)
        
        # Service clients to other nodes
        self.robot_reset_client = self.create_client(Trigger, '/robot/reset_to_home')
        self.robot_emergency_stop_client = self.create_client(Trigger, '/robot/emergency_stop')
        
        # Data recorder interface (if enabled)
        if self.config['recording']['enabled']:
            self.data_recorder = DataRecorderInterface(self, self.config)
        else:
            self.data_recorder = None
        
        # Status timer
        self.status_timer = self.create_timer(1.0, self.publish_recording_status)
        
        self.get_logger().info('📹 System Manager (Recording & Services) initialized')
        self.get_logger().info(f'   Recording: {"enabled" if self.config["recording"]["enabled"] else "disabled"}')
        
        # Don't wait for recorder services during init - check asynchronously
        if self.data_recorder:
            self.create_timer(2.0, self._check_recorder_services_async, callback_group=self.callback_group)
    
    def _check_recorder_services_async(self):
        """Check recorder services asynchronously after startup"""
        # Cancel timer after first run
        self.destroy_timer(self._timers[-1])
        
        if self.data_recorder:
            self.data_recorder._check_services_async()
    
    def handle_start_recording_request(self, msg: Bool):
        """Handle recording start request from VR controller"""
        if msg.data:
            if self.recording_active:
                # Toggle off
                self.stop_recording(success=False)
            else:
                # Toggle on
                self.start_recording()
    
    def handle_stop_recording_request(self, msg: Int32):
        """Handle recording stop request from VR controller"""
        # msg.data: 0 = normal stop, 1 = mark as success
        success = (msg.data == 1)
        self.stop_recording(success=success)
    
    def start_recording(self):
        """Start recording"""
        if not self.config['recording']['enabled']:
            self.get_logger().warn("Recording is disabled in configuration")
            return
        
        if self.recording_active:
            self.get_logger().warn("Recording already active")
            return
        
        if self.data_recorder:
            success = self.data_recorder.start_recording()
            if success:
                self.recording_active = True
                self.recording_start_time = time.time()
                self.recording_success = False
                self.get_logger().info("🎬 Recording started")
    
    def stop_recording(self, success: bool = False):
        """Stop recording"""
        if not self.recording_active:
            self.get_logger().warn("No active recording to stop")
            return
        
        self.recording_success = success
        
        if self.data_recorder:
            self.data_recorder.stop_recording(success)
        
        self.recording_active = False
        duration = time.time() - self.recording_start_time if self.recording_start_time else 0
        
        status_str = "✅ successful" if success else "⏹️  stopped"
        self.get_logger().info(f"Recording {status_str} (duration: {duration:.1f}s)")
        
        # Check auto-mark success threshold
        if not success and duration > self.config['recording'].get('auto_mark_success_threshold', 2.0):
            self.get_logger().info(f"Auto-marking as successful (>{self.config['recording']['auto_mark_success_threshold']}s)")
            self._mark_recording_successful("Auto-marked based on duration")
    
    def _mark_recording_successful(self, message: str):
        """Mark the last recording as successful"""
        if self.data_recorder:
            self.data_recorder.mark_recording_successful(message)
    
    def publish_recording_status(self):
        """Publish recording status at 1Hz"""
        msg = RecordingStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.recording_active = self.recording_active
        msg.current_file = self.recording_file_path if self.recording_file_path else ""
        msg.duration_seconds = time.time() - self.recording_start_time if self.recording_start_time and self.recording_active else 0.0
        msg.marked_successful = self.recording_success
        
        self.recording_status_pub.publish(msg)
    
    # Service callbacks
    def start_recording_callback(self, request, response):
        """Service callback to start recording"""
        self.start_recording()
        return response
    
    def stop_recording_callback(self, request, response):
        """Service callback to stop recording"""
        self.stop_recording(success=False)
        return response
    
    def mark_recording_success_callback(self, request, response):
        """Service callback to mark recording as successful"""
        if self.recording_active:
            self.stop_recording(success=True)
        else:
            self._mark_recording_successful("Marked via service")
        return response
    
    def reset_robot_callback(self, request, response):
        """Service callback to reset robot to home position"""
        self.get_logger().info("🏠 Reset robot requested")
        
        if self.robot_reset_client.wait_for_service(timeout_sec=2.0):
            future = self.robot_reset_client.call_async(Trigger.Request())
            # Don't block - let the robot_control_node handle it
            self.get_logger().info("Reset command sent to robot control node")
        else:
            self.get_logger().error("Robot reset service not available")
        
        return response
    
    def emergency_stop_callback(self, request, response):
        """Service callback for emergency stop"""
        self.get_logger().warn("🛑 EMERGENCY STOP requested")
        
        # Stop any active recording
        if self.recording_active:
            self.stop_recording(success=False)
        
        # Send emergency stop to robot
        if self.robot_emergency_stop_client.wait_for_service(timeout_sec=1.0):
            future = self.robot_emergency_stop_client.call_async(Trigger.Request())
            self.get_logger().info("Emergency stop sent to robot control node")
        else:
            self.get_logger().error("Robot emergency stop service not available")
        
        return response


class DataRecorderInterface:
    """Interface to data recording service"""
    
    def __init__(self, node: Node, config: Dict):
        self.node = node
        self.config = config
        self.services_ready = False
        
        # Service clients
        self.start_recording_client = node.create_client(
            StartRecording, '/data_recorder/start_recording'
        )
        self.stop_recording_client = node.create_client(
            StopRecording, '/data_recorder/stop_recording'
        )
        
        # Don't block during initialization
        self.node.get_logger().info("Data recorder interface created, checking services asynchronously...")
    
    def _check_services_async(self):
        """Check if recording services are available (non-blocking)"""
        start_ready = self.start_recording_client.service_is_ready()
        stop_ready = self.stop_recording_client.service_is_ready()
        
        if start_ready and stop_ready:
            self.services_ready = True
            self.node.get_logger().info("✅ Data recorder services ready")
        else:
            self.node.get_logger().warn("⚠️  Data recorder services not available yet")
            if not start_ready:
                self.node.get_logger().warn("   - Start recording service not ready")
            if not stop_ready:
                self.node.get_logger().warn("   - Stop recording service not ready")
    
    def start_recording(self) -> bool:
        """Start recording via service"""
        if not self.services_ready:
            self.node.get_logger().error("Recording services not ready")
            return False
        
        request = StartRecording.Request()
        request.recording_name = f"recording_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        
        try:
            future = self.start_recording_client.call_async(request)
            future.add_done_callback(self._start_recording_done)
        except Exception as e:
            self.node.get_logger().error(f"Failed to start recording call: {e}")
            return False
        
        return True  # Assume success, callback will handle actual result
    
    def _start_recording_done(self, future):
        """Handle start recording response"""
        try:
            response = future.result()
            if response.success:
                self.node.recording_file_path = response.recording_id
                self.node.get_logger().info(f"Recording started: {response.recording_id}")
            else:
                self.node.get_logger().error(f"Failed to start recording: {response.message}")
                self.node.recording_active = False
        except Exception as e:
            self.node.get_logger().error(f"Recording start exception: {e}")
            self.node.recording_active = False
    
    def stop_recording(self, success: bool):
        """Stop recording via service"""
        if not self.services_ready:
            self.node.get_logger().error("Recording services not ready")
            return
        
        request = StopRecording.Request()
        request.success = success
        request.recording_id = self.node.recording_file_path if self.node.recording_file_path else ""
        
        try:
            future = self.stop_recording_client.call_async(request)
            future.add_done_callback(lambda f: self._stop_recording_done(f, success))
        except Exception as e:
            self.node.get_logger().error(f"Failed to stop recording call: {e}")
    
    def _stop_recording_done(self, future, success: bool):
        """Handle stop recording response"""
        try:
            response = future.result()
            if response.success:
                self.node.get_logger().info(f"Recording stopped: {response.final_recording_path}")
                
                # Move to success folder if marked successful
                if success and response.final_recording_path:
                    self._move_to_success_folder(response.final_recording_path)
            else:
                self.node.get_logger().error(f"Failed to stop recording: {response.message}")
        except Exception as e:
            self.node.get_logger().error(f"Recording stop exception: {e}")
    
    def mark_recording_successful(self, message: str):
        """Mark the last recording as successful by moving it"""
        if self.node.recording_file_path:
            self._move_to_success_folder(self.node.recording_file_path)
    
    def _move_to_success_folder(self, recording_path: str):
        """Move recording to success folder"""
        try:
            # Create success directory if it doesn't exist
            base_dir = os.path.dirname(recording_path)
            success_dir = os.path.join(base_dir, 'success')
            os.makedirs(success_dir, exist_ok=True)
            
            # Move file
            filename = os.path.basename(recording_path)
            success_path = os.path.join(success_dir, filename)
            
            if os.path.exists(recording_path):
                shutil.move(recording_path, success_path)
                self.node.get_logger().info(f"✅ Moved recording to success folder: {filename}")
            else:
                self.node.get_logger().warn(f"Recording file not found: {recording_path}")
        except Exception as e:
            self.node.get_logger().error(f"Failed to move recording to success folder: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    # Get config path from parameter or environment
    config_path = os.environ.get('CONFIG_FILE', '')
    if not config_path:
        # Try default locations
        from ament_index_python.packages import get_package_share_directory
        try:
            config_path = os.path.join(
                get_package_share_directory('lbx_franka_control'),
                'config',
                'franka_vr_control_config.yaml'
            )
        except:
            # Fallback
            config_path = 'configs/control/franka_vr_control_config.yaml'
    
    system_manager = SystemManager(config_path)
    
    try:
        rclpy.spin(system_manager)
    except KeyboardInterrupt:
        system_manager.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        system_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 