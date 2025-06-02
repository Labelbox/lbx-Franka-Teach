#!/usr/bin/env python3
"""
UI Node

Simple terminal UI for interacting with the system.
Only sends commands to other nodes - no direct control.

Responsibilities:
- Display system status
- Accept user commands
- Call services on other nodes
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import threading
import sys
import termios
import tty
from typing import Optional

# ROS2 messages and services
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from lbx_interfaces.msg import SystemStatus


class Colors:
    """Terminal color codes"""
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class UINode(Node):
    """Simple UI node for system interaction"""
    
    def __init__(self):
        super().__init__('ui_node')
        
        # State
        self.system_state = "unknown"
        self.system_status = None
        self.running = True
        
        # Service clients
        self.initialize_client = self.create_client(Trigger, '/system/initialize')
        self.calibrate_client = self.create_client(Trigger, '/system/start_calibration')
        self.start_teleop_client = self.create_client(Trigger, '/system/start_teleoperation')
        self.stop_client = self.create_client(Trigger, '/system/stop')
        self.reset_client = self.create_client(Trigger, '/system/reset')
        
        # Subscribers
        self.state_sub = self.create_subscription(
            String,
            '/system/state',
            self.state_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        )
        
        self.status_sub = self.create_subscription(
            SystemStatus,
            '/system/status',
            self.status_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        )
        
        # Start UI thread
        self.ui_thread = threading.Thread(target=self.ui_loop)
        self.ui_thread.start()
        
        # Display timer
        self.create_timer(0.5, self.display_status)
        
        self.get_logger().info("UI Node started")
    
    def state_callback(self, msg: String):
        """Handle system state updates"""
        self.system_state = msg.data
    
    def status_callback(self, msg: SystemStatus):
        """Handle system status updates"""
        self.system_status = msg
    
    def display_status(self):
        """Display current system status"""
        # Clear screen
        print("\033[2J\033[H", end='')
        
        # Header
        print(f"{Colors.BOLD}{Colors.BLUE}")
        print("╔═══════════════════════════════════════════════════════╗")
        print("║         LABELBOX FRANKA CONTROL SYSTEM UI             ║")
        print("╚═══════════════════════════════════════════════════════╝")
        print(f"{Colors.ENDC}")
        
        # System State
        state_color = Colors.GREEN if self.system_state == "teleoperation" else Colors.YELLOW
        print(f"\n{Colors.BOLD}System State:{Colors.ENDC} {state_color}{self.system_state.upper()}{Colors.ENDC}")
        
        # Status details
        if self.system_status:
            print(f"\n{Colors.BOLD}Status Details:{Colors.ENDC}")
            print(f"  • Controller Connected: {'✓' if self.system_status.controller_connected else '✗'}")
            print(f"  • Teleoperation Active: {'✓' if self.system_status.teleoperation_enabled else '✗'}")
            print(f"  • Recording Active: {'✓' if self.system_status.recording_active else '✗'}")
        
        # Commands
        print(f"\n{Colors.BOLD}Commands:{Colors.ENDC}")
        print("  [i] Initialize System")
        print("  [c] Start Calibration")
        print("  [t] Start Teleoperation")
        print("  [s] Stop System")
        print("  [r] Reset System")
        print("  [q] Quit")
        
        print(f"\n{Colors.CYAN}Press a key to execute command...{Colors.ENDC}")
    
    def get_key(self):
        """Get single keypress"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key
    
    def call_service(self, client, service_name: str):
        """Call a service and display result"""
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"{service_name} service not available")
            return
        
        request = Trigger.Request()
        future = client.call_async(request)
        
        # Wait for response
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f"{service_name}: {response.message}")
            else:
                self.get_logger().error(f"{service_name} failed: {response.message}")
        else:
            self.get_logger().error(f"{service_name} call failed")
    
    def ui_loop(self):
        """Main UI loop"""
        while self.running and rclpy.ok():
            key = self.get_key()
            
            if key == 'i':
                self.call_service(self.initialize_client, "Initialize")
            elif key == 'c':
                self.call_service(self.calibrate_client, "Calibration")
            elif key == 't':
                self.call_service(self.start_teleop_client, "Start Teleoperation")
            elif key == 's':
                self.call_service(self.stop_client, "Stop")
            elif key == 'r':
                self.call_service(self.reset_client, "Reset")
            elif key == 'q':
                self.running = False
                self.get_logger().info("Shutting down UI...")
                rclpy.shutdown()
    
    def destroy_node(self):
        """Clean shutdown"""
        self.running = False
        if self.ui_thread.is_alive():
            self.ui_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = UINode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 