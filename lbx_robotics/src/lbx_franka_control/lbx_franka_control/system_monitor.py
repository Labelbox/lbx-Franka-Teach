#!/usr/bin/env python3
"""
System Monitor Node

Publishes diagnostic information about the VR teleoperation system health
"""

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import PoseStamped
from lbx_interfaces.msg import SystemStatus, VRControllerState
import time
import psutil
import numpy as np


class SystemMonitor(Node):
    """Monitor system health and publish diagnostics"""
    
    def __init__(self):
        super().__init__('system_monitor')
        
        # Create diagnostics publisher
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            10
        )
        
        # Create timer for diagnostics
        self.declare_parameter('publish_rate', 1.0)
        publish_rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_diagnostics)
        
        # Subscribe to system topics for monitoring
        self.vr_pose_sub = self.create_subscription(
            PoseStamped,
            '/vr/controller_pose',
            self.vr_pose_callback,
            10
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.system_status_sub = self.create_subscription(
            SystemStatus,
            '/system_status',
            self.system_status_callback,
            10
        )
        
        self.vr_state_sub = self.create_subscription(
            VRControllerState,
            '/vr_control_state',
            self.vr_state_callback,
            10
        )
        
        # Monitoring state
        self.last_vr_msg_time = None
        self.last_joint_msg_time = None
        self.last_system_status = None
        self.last_vr_state = None
        self.vr_msg_count = 0
        self.joint_msg_count = 0
        self.vr_rate = 0.0
        self.joint_rate = 0.0
        self.last_rate_calc_time = time.time()
        
        # Control performance metrics
        self.control_latencies = []
        self.max_latency_window = 100
        
        self.get_logger().info('System Monitor started')
    
    def vr_pose_callback(self, msg):
        """Monitor VR pose messages"""
        self.last_vr_msg_time = time.time()
        self.vr_msg_count += 1
    
    def joint_state_callback(self, msg):
        """Monitor joint state messages"""
        self.last_joint_msg_time = time.time()
        self.joint_msg_count += 1
    
    def system_status_callback(self, msg):
        """Monitor system status"""
        self.last_system_status = msg
    
    def vr_state_callback(self, msg):
        """Monitor VR controller state"""
        self.last_vr_state = msg
    
    def calculate_rates(self):
        """Calculate message rates"""
        current_time = time.time()
        time_diff = current_time - self.last_rate_calc_time
        
        if time_diff > 0:
            self.vr_rate = self.vr_msg_count / time_diff
            self.joint_rate = self.joint_msg_count / time_diff
            
            # Reset counters
            self.vr_msg_count = 0
            self.joint_msg_count = 0
            self.last_rate_calc_time = current_time
    
    def publish_diagnostics(self):
        """Publish diagnostic information"""
        # Calculate rates
        self.calculate_rates()
        
        # Create diagnostic array
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        # VR System Diagnostics
        vr_status = DiagnosticStatus()
        vr_status.name = "vr_system"
        vr_status.hardware_id = "oculus_quest"
        
        if self.last_vr_msg_time and (time.time() - self.last_vr_msg_time) < 2.0:
            vr_status.level = DiagnosticStatus.OK
            vr_status.message = f"VR system operational ({self.vr_rate:.1f} Hz)"
        else:
            vr_status.level = DiagnosticStatus.ERROR
            vr_status.message = "VR system not responding"
        
        vr_status.values = [
            KeyValue(key="rate_hz", value=f"{self.vr_rate:.1f}"),
            KeyValue(key="controller_connected", value=str(self.last_vr_state is not None)),
            KeyValue(key="grip_pressed", value=str(self.last_vr_state.grip_pressed if self.last_vr_state else False)),
        ]
        diag_array.status.append(vr_status)
        
        # Robot System Diagnostics
        robot_status = DiagnosticStatus()
        robot_status.name = "robot_system"
        robot_status.hardware_id = "franka_fr3"
        
        if self.last_joint_msg_time and (time.time() - self.last_joint_msg_time) < 2.0:
            if self.joint_rate > 500:  # Expecting ~1000Hz
                robot_status.level = DiagnosticStatus.OK
                robot_status.message = f"Robot connected ({self.joint_rate:.0f} Hz)"
            else:
                robot_status.level = DiagnosticStatus.WARN
                robot_status.message = f"Robot rate low ({self.joint_rate:.0f} Hz)"
        else:
            robot_status.level = DiagnosticStatus.ERROR
            robot_status.message = "Robot not responding"
        
        robot_status.values = [
            KeyValue(key="joint_state_rate_hz", value=f"{self.joint_rate:.0f}"),
            KeyValue(key="connected", value=str(self.last_joint_msg_time is not None)),
        ]
        diag_array.status.append(robot_status)
        
        # Control System Diagnostics
        control_status = DiagnosticStatus()
        control_status.name = "control_system"
        control_status.hardware_id = "system_manager"
        
        if self.last_system_status:
            control_status.level = DiagnosticStatus.OK
            control_status.message = f"System state: {self.last_system_status.system_state}"
            control_status.values = [
                KeyValue(key="state", value=self.last_system_status.system_state),
                KeyValue(key="recording_active", value=str(self.last_system_status.recording_active)),
                KeyValue(key="teleoperation_enabled", value=str(self.last_system_status.teleoperation_enabled)),
            ]
        else:
            control_status.level = DiagnosticStatus.WARN
            control_status.message = "System manager not reporting"
        
        diag_array.status.append(control_status)
        
        # System Resources Diagnostics
        resource_status = DiagnosticStatus()
        resource_status.name = "system_resources"
        resource_status.hardware_id = "compute"
        
        cpu_percent = psutil.cpu_percent(interval=0.1)
        memory = psutil.virtual_memory()
        
        if cpu_percent < 80 and memory.percent < 80:
            resource_status.level = DiagnosticStatus.OK
            resource_status.message = "System resources normal"
        elif cpu_percent < 90 and memory.percent < 90:
            resource_status.level = DiagnosticStatus.WARN
            resource_status.message = "System resources elevated"
        else:
            resource_status.level = DiagnosticStatus.ERROR
            resource_status.message = "System resources critical"
        
        resource_status.values = [
            KeyValue(key="cpu_percent", value=f"{cpu_percent:.1f}"),
            KeyValue(key="memory_percent", value=f"{memory.percent:.1f}"),
            KeyValue(key="memory_available_gb", value=f"{memory.available / (1024**3):.1f}"),
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
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 