#!/usr/bin/env python3
"""
System Orchestrator Node

A lightweight orchestrator that manages system state and coordinates between nodes.
Does NOT directly control the robot - only sends commands to other nodes.

Responsibilities:
- System state management (idle, calibrating, teleoperation)
- Service server for high-level commands
- Health monitoring and status publishing
- Startup sequencing
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import time
from enum import Enum
from typing import Optional

# ROS2 messages and services
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, SetBool
from lbx_interfaces.msg import SystemStatus

# Diagnostics
from diagnostic_updater import Updater, DiagnosticTask, DiagnosticStatus


class SystemState(Enum):
    """System operational states"""
    INITIALIZING = "initializing"
    IDLE = "idle"
    CALIBRATING = "calibrating"
    TELEOPERATION = "teleoperation"
    ERROR = "error"
    SHUTDOWN = "shutdown"


class SystemHealthTask(DiagnosticTask):
    """Diagnostic task for system health"""
    
    def __init__(self, orchestrator):
        super().__init__("System Orchestrator Status")
        self.orchestrator = orchestrator
    
    def run(self, stat):
        """Report system health"""
        state = self.orchestrator.current_state
        
        if state == SystemState.ERROR:
            stat.summary(DiagnosticStatus.ERROR, f"System in error state: {self.orchestrator.error_message}")
        elif state == SystemState.INITIALIZING:
            stat.summary(DiagnosticStatus.WARN, "System initializing")
        elif state in [SystemState.IDLE, SystemState.CALIBRATING, SystemState.TELEOPERATION]:
            stat.summary(DiagnosticStatus.OK, f"System operational - {state.value}")
        else:
            stat.summary(DiagnosticStatus.WARN, f"System in {state.value} state")
        
        # Add detailed status
        stat.add("Current State", state.value)
        stat.add("Robot Ready", str(self.orchestrator.robot_ready))
        stat.add("VR Ready", str(self.orchestrator.vr_ready))
        stat.add("Calibration Valid", str(self.orchestrator.calibration_valid))
        stat.add("Teleoperation Active", str(self.orchestrator.teleoperation_active))
        
        return stat


class SystemOrchestrator(Node):
    """Lightweight system orchestrator following ROS2 best practices"""
    
    def __init__(self):
        super().__init__('system_orchestrator')
        
        # State management
        self.current_state = SystemState.INITIALIZING
        self.error_message = ""
        
        # Component readiness flags
        self.robot_ready = False
        self.vr_ready = False
        self.calibration_valid = False
        self.teleoperation_active = False
        
        # Callback group for services
        self.service_callback_group = ReentrantCallbackGroup()
        
        # Publishers
        self.status_pub = self.create_publisher(
            SystemStatus, 
            '/system/status', 
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        )
        
        self.state_pub = self.create_publisher(
            String,
            '/system/state',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        )
        
        # Service servers for system commands
        self.create_service(
            Trigger,
            '/system/initialize',
            self.initialize_system_callback,
            callback_group=self.service_callback_group
        )
        
        self.create_service(
            Trigger,
            '/system/start_calibration',
            self.start_calibration_callback,
            callback_group=self.service_callback_group
        )
        
        self.create_service(
            Trigger,
            '/system/start_teleoperation',
            self.start_teleoperation_callback,
            callback_group=self.service_callback_group
        )
        
        self.create_service(
            Trigger,
            '/system/stop',
            self.stop_system_callback,
            callback_group=self.service_callback_group
        )
        
        self.create_service(
            Trigger,
            '/system/reset',
            self.reset_system_callback,
            callback_group=self.service_callback_group
        )
        
        # Service clients to other nodes
        self.robot_reset_client = self.create_client(Trigger, '/robot/reset_to_home')
        self.robot_enable_client = self.create_client(SetBool, '/robot/enable_control')
        self.vr_calibrate_client = self.create_client(Trigger, '/vr/calibrate')
        self.vr_enable_client = self.create_client(SetBool, '/vr/enable_teleoperation')
        
        # Subscribers for component status
        self.create_subscription(
            Bool,
            '/robot/ready',
            self.robot_ready_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        self.create_subscription(
            Bool,
            '/vr/ready',
            self.vr_ready_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        self.create_subscription(
            Bool,
            '/vr/calibration_valid',
            self.calibration_valid_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        # Status publishing timer (1 Hz)
        self.create_timer(1.0, self.publish_status)
        
        # Diagnostics
        self.diagnostic_updater = Updater(self)
        self.diagnostic_updater.setHardwareID("system_orchestrator")
        self.diagnostic_updater.add(SystemHealthTask(self))
        
        # Start initialization sequence
        self.get_logger().info("System Orchestrator started")
        self.create_timer(2.0, self.auto_initialize, callback_group=self.service_callback_group)
    
    def auto_initialize(self):
        """Auto-initialize system after startup"""
        # Cancel timer after first run
        self.destroy_timer(self._timers[-1])
        
        # Start initialization
        self.get_logger().info("Starting automatic system initialization...")
        self.initialize_system()
    
    def change_state(self, new_state: SystemState):
        """Change system state and publish update"""
        old_state = self.current_state
        self.current_state = new_state
        
        self.get_logger().info(f"State transition: {old_state.value} -> {new_state.value}")
        
        # Publish state change
        msg = String()
        msg.data = new_state.value
        self.state_pub.publish(msg)
    
    def robot_ready_callback(self, msg: Bool):
        """Handle robot ready status updates"""
        self.robot_ready = msg.data
    
    def vr_ready_callback(self, msg: Bool):
        """Handle VR ready status updates"""
        self.vr_ready = msg.data
    
    def calibration_valid_callback(self, msg: Bool):
        """Handle calibration validity updates"""
        self.calibration_valid = msg.data
    
    def publish_status(self):
        """Publish system status at 1Hz"""
        status = SystemStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.system_state = self.current_state.value
        status.teleoperation_enabled = self.teleoperation_active
        status.controller_connected = self.vr_ready
        status.calibration_mode = "active" if self.current_state == SystemState.CALIBRATING else ""
        status.recording_active = False  # Handled by data recorder
        
        self.status_pub.publish(status)
        
        # Update diagnostics
        self.diagnostic_updater.update()
    
    def initialize_system(self):
        """Initialize system components in sequence"""
        self.get_logger().info("Initializing system components...")
        
        # Step 1: Reset robot to home position
        if self.robot_reset_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("Resetting robot to home position...")
            future = self.robot_reset_client.call_async(Trigger.Request())
            future.add_done_callback(self._robot_reset_done)
        else:
            self.get_logger().error("Robot reset service not available")
            self.change_state(SystemState.ERROR)
            self.error_message = "Robot reset service not available"
    
    def _robot_reset_done(self, future):
        """Handle robot reset completion"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("✅ Robot reset to home position")
                self.change_state(SystemState.IDLE)
                self.get_logger().info("System ready for operation")
                self.get_logger().info("Use '/system/start_calibration' to begin VR calibration")
            else:
                self.get_logger().error(f"Robot reset failed: {response.message}")
                self.change_state(SystemState.ERROR)
                self.error_message = f"Robot reset failed: {response.message}"
        except Exception as e:
            self.get_logger().error(f"Robot reset exception: {e}")
            self.change_state(SystemState.ERROR)
            self.error_message = str(e)
    
    # Service callbacks
    def initialize_system_callback(self, request, response):
        """Handle system initialization request"""
        if self.current_state != SystemState.INITIALIZING:
            self.change_state(SystemState.INITIALIZING)
            self.initialize_system()
            response.success = True
            response.message = "System initialization started"
        else:
            response.success = False
            response.message = "System already initializing"
        return response
    
    def start_calibration_callback(self, request, response):
        """Handle calibration start request"""
        if self.current_state != SystemState.IDLE:
            response.success = False
            response.message = f"Cannot start calibration from {self.current_state.value} state"
            return response
        
        if not self.robot_ready:
            response.success = False
            response.message = "Robot not ready"
            return response
        
        if not self.vr_ready:
            response.success = False
            response.message = "VR controller not ready"
            return response
        
        # Start calibration
        self.change_state(SystemState.CALIBRATING)
        
        if self.vr_calibrate_client.wait_for_service(timeout_sec=2.0):
            future = self.vr_calibrate_client.call_async(Trigger.Request())
            future.add_done_callback(self._calibration_started)
            response.success = True
            response.message = "Calibration started"
        else:
            response.success = False
            response.message = "VR calibration service not available"
            self.change_state(SystemState.IDLE)
        
        return response
    
    def _calibration_started(self, future):
        """Handle calibration start response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Calibration sequence started")
            else:
                self.get_logger().error(f"Failed to start calibration: {response.message}")
                self.change_state(SystemState.IDLE)
        except Exception as e:
            self.get_logger().error(f"Calibration start exception: {e}")
            self.change_state(SystemState.IDLE)
    
    def start_teleoperation_callback(self, request, response):
        """Handle teleoperation start request"""
        if self.current_state not in [SystemState.IDLE, SystemState.CALIBRATING]:
            response.success = False
            response.message = f"Cannot start teleoperation from {self.current_state.value} state"
            return response
        
        if not self.calibration_valid:
            response.success = False
            response.message = "Calibration not valid - please calibrate first"
            return response
        
        # Enable robot control
        if self.robot_enable_client.wait_for_service(timeout_sec=2.0):
            robot_req = SetBool.Request()
            robot_req.data = True
            robot_future = self.robot_enable_client.call_async(robot_req)
            
            # Enable VR teleoperation
            if self.vr_enable_client.wait_for_service(timeout_sec=2.0):
                vr_req = SetBool.Request()
                vr_req.data = True
                vr_future = self.vr_enable_client.call_async(vr_req)
                
                self.change_state(SystemState.TELEOPERATION)
                self.teleoperation_active = True
                response.success = True
                response.message = "Teleoperation started"
            else:
                response.success = False
                response.message = "VR enable service not available"
        else:
            response.success = False
            response.message = "Robot enable service not available"
        
        return response
    
    def stop_system_callback(self, request, response):
        """Handle system stop request"""
        self.get_logger().info("Stopping system...")
        
        # Disable teleoperation if active
        if self.teleoperation_active:
            if self.robot_enable_client.wait_for_service(timeout_sec=1.0):
                req = SetBool.Request()
                req.data = False
                self.robot_enable_client.call_async(req)
            
            if self.vr_enable_client.wait_for_service(timeout_sec=1.0):
                req = SetBool.Request()
                req.data = False
                self.vr_enable_client.call_async(req)
        
        self.teleoperation_active = False
        self.change_state(SystemState.IDLE)
        response.success = True
        response.message = "System stopped"
        return response
    
    def reset_system_callback(self, request, response):
        """Handle system reset request"""
        self.get_logger().info("Resetting system...")
        
        # Stop any active operations
        self.stop_system_callback(request, response)
        
        # Reset flags
        self.calibration_valid = False
        self.error_message = ""
        
        # Re-initialize
        self.change_state(SystemState.INITIALIZING)
        self.initialize_system()
        
        response.success = True
        response.message = "System reset initiated"
        return response


def main(args=None):
    rclpy.init(args=args)
    
    orchestrator = SystemOrchestrator()
    
    # Use single-threaded executor for simplicity
    # The orchestrator is lightweight and doesn't need multi-threading
    try:
        rclpy.spin(orchestrator)
    except KeyboardInterrupt:
        orchestrator.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        orchestrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 