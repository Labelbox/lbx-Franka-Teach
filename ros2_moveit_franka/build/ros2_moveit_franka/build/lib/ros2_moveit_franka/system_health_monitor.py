#!/usr/bin/env python3
"""
System Health Monitor for Robust Franka Control
Monitors system health, logs diagnostics, and can restart components
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import time
import threading
import subprocess
import psutil
import json
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional
from enum import Enum


class SystemHealthStatus(Enum):
    """System health status enumeration"""
    HEALTHY = "healthy"
    WARNING = "warning"
    CRITICAL = "critical"
    UNKNOWN = "unknown"


@dataclass
class HealthMetrics:
    """System health metrics"""
    timestamp: float
    robot_state: str
    robot_healthy: bool
    cpu_usage: float
    memory_usage: float
    franka_process_running: bool
    moveit_process_running: bool
    network_connectivity: bool
    last_error: Optional[str]
    uptime: float


class SystemHealthMonitor(Node):
    """
    System health monitor for the Franka robot system
    """
    
    def __init__(self):
        super().__init__('system_health_monitor')
        
        # Configuration
        self.monitor_interval = 2.0  # seconds
        self.restart_threshold = 3  # consecutive critical failures
        self.auto_restart_enabled = True
        
        # State tracking
        self.start_time = time.time()
        self.consecutive_failures = 0
        self.last_robot_state = "unknown"
        self.last_robot_health = False
        self.system_status = SystemHealthStatus.UNKNOWN
        
        # Threading
        self.callback_group = ReentrantCallbackGroup()
        self.health_lock = threading.Lock()
        
        # Subscribers
        self.robot_state_subscriber = self.create_subscription(
            String,
            'robot_state',
            self.robot_state_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.robot_health_subscriber = self.create_subscription(
            Bool,
            'robot_health',
            self.robot_health_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.robot_errors_subscriber = self.create_subscription(
            String,
            'robot_errors',
            self.robot_errors_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Publishers
        self.system_health_publisher = self.create_publisher(
            String,
            'system_health',
            10,
            callback_group=self.callback_group
        )
        
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray,
            'diagnostics',
            10,
            callback_group=self.callback_group
        )
        
        self.health_metrics_publisher = self.create_publisher(
            String,
            'health_metrics',
            10,
            callback_group=self.callback_group
        )
        
        # Timers
        self.health_timer = self.create_timer(
            self.monitor_interval,
            self.health_monitor_callback,
            callback_group=self.callback_group
        )
        
        self.diagnostics_timer = self.create_timer(
            5.0,  # Publish diagnostics every 5 seconds
            self.publish_diagnostics,
            callback_group=self.callback_group
        )
        
        self.get_logger().info("System Health Monitor initialized")
    
    def robot_state_callback(self, msg: String):
        """Track robot state changes"""
        with self.health_lock:
            old_state = self.last_robot_state
            self.last_robot_state = msg.data
            
            if old_state != msg.data:
                self.get_logger().info(f"Robot state changed: {old_state} -> {msg.data}")
                
                # Reset failure counter on successful state transitions
                if msg.data == "ready":
                    self.consecutive_failures = 0
    
    def robot_health_callback(self, msg: Bool):
        """Track robot health status"""
        with self.health_lock:
            self.last_robot_health = msg.data
    
    def robot_errors_callback(self, msg: String):
        """Log and track robot errors"""
        self.get_logger().warn(f"Robot error reported: {msg.data}")
        
        # Increment failure counter for critical errors
        if "libfranka" in msg.data.lower() or "connection" in msg.data.lower():
            with self.health_lock:
                self.consecutive_failures += 1
                self.get_logger().warn(f"Critical error detected. Consecutive failures: {self.consecutive_failures}")
    
    def health_monitor_callback(self):
        """Main health monitoring callback"""
        try:
            # Collect health metrics
            metrics = self.collect_health_metrics()
            
            # Determine system health status
            health_status = self.evaluate_system_health(metrics)
            
            # Update system status
            with self.health_lock:
                self.system_status = health_status
            
            # Publish health status
            self.publish_health_status(health_status)
            
            # Publish detailed metrics
            self.publish_health_metrics(metrics)
            
            # Take corrective action if needed
            if health_status == SystemHealthStatus.CRITICAL and self.auto_restart_enabled:
                self.handle_critical_health()
                
        except Exception as e:
            self.get_logger().error(f"Health monitoring failed: {str(e)}")
    
    def collect_health_metrics(self) -> HealthMetrics:
        """Collect comprehensive system health metrics"""
        current_time = time.time()
        
        # System metrics
        cpu_usage = psutil.cpu_percent(interval=0.1)
        memory_info = psutil.virtual_memory()
        memory_usage = memory_info.percent
        
        # Process checks
        franka_running = self.is_process_running("franka")
        moveit_running = self.is_process_running("moveit") or self.is_process_running("robot_state_publisher")
        
        # Network connectivity check
        network_ok = self.check_network_connectivity()
        
        # Robot state
        with self.health_lock:
            robot_state = self.last_robot_state
            robot_healthy = self.last_robot_health
        
        return HealthMetrics(
            timestamp=current_time,
            robot_state=robot_state,
            robot_healthy=robot_healthy,
            cpu_usage=cpu_usage,
            memory_usage=memory_usage,
            franka_process_running=franka_running,
            moveit_process_running=moveit_running,
            network_connectivity=network_ok,
            last_error=None,  # Could be expanded to track last error
            uptime=current_time - self.start_time
        )
    
    def is_process_running(self, process_name: str) -> bool:
        """Check if a process with given name is running"""
        try:
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    # Check process name
                    if process_name.lower() in proc.info['name'].lower():
                        return True
                    
                    # Check command line arguments
                    cmdline = ' '.join(proc.info['cmdline'] or [])
                    if process_name.lower() in cmdline.lower():
                        return True
                        
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
            return False
        except Exception as e:
            self.get_logger().warn(f"Failed to check process {process_name}: {str(e)}")
            return False
    
    def check_network_connectivity(self) -> bool:
        """Check network connectivity to robot"""
        try:
            # Simple ping test (adjust IP as needed)
            result = subprocess.run(
                ['ping', '-c', '1', '-W', '2', '192.168.1.59'],
                capture_output=True,
                timeout=5
            )
            return result.returncode == 0
        except Exception as e:
            self.get_logger().debug(f"Network check failed: {str(e)}")
            return False
    
    def evaluate_system_health(self, metrics: HealthMetrics) -> SystemHealthStatus:
        """Evaluate overall system health based on metrics"""
        
        # Critical conditions
        if (not metrics.robot_healthy and 
            metrics.robot_state in ["error", "disconnected"]):
            return SystemHealthStatus.CRITICAL
        
        if not metrics.network_connectivity:
            return SystemHealthStatus.CRITICAL
        
        if metrics.cpu_usage > 90 or metrics.memory_usage > 90:
            return SystemHealthStatus.CRITICAL
        
        # Warning conditions
        if metrics.robot_state in ["recovering", "initializing"]:
            return SystemHealthStatus.WARNING
        
        if not metrics.franka_process_running or not metrics.moveit_process_running:
            return SystemHealthStatus.WARNING
        
        if metrics.cpu_usage > 70 or metrics.memory_usage > 70:
            return SystemHealthStatus.WARNING
        
        # Healthy conditions
        if (metrics.robot_healthy and 
            metrics.robot_state in ["ready", "moving"] and
            metrics.network_connectivity):
            return SystemHealthStatus.HEALTHY
        
        return SystemHealthStatus.UNKNOWN
    
    def publish_health_status(self, status: SystemHealthStatus):
        """Publish current health status"""
        try:
            msg = String()
            msg.data = status.value
            self.system_health_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish health status: {str(e)}")
    
    def publish_health_metrics(self, metrics: HealthMetrics):
        """Publish detailed health metrics as JSON"""
        try:
            msg = String()
            msg.data = json.dumps(asdict(metrics), indent=2)
            self.health_metrics_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish health metrics: {str(e)}")
    
    def publish_diagnostics(self):
        """Publish ROS diagnostics messages"""
        try:
            diag_array = DiagnosticArray()
            diag_array.header.stamp = self.get_clock().now().to_msg()
            
            # System health diagnostic
            system_diag = DiagnosticStatus()
            system_diag.name = "franka_system_health"
            system_diag.hardware_id = "franka_robot"
            
            if self.system_status == SystemHealthStatus.HEALTHY:
                system_diag.level = DiagnosticStatus.OK
                system_diag.message = "System is healthy"
            elif self.system_status == SystemHealthStatus.WARNING:
                system_diag.level = DiagnosticStatus.WARN
                system_diag.message = "System has warnings"
            elif self.system_status == SystemHealthStatus.CRITICAL:
                system_diag.level = DiagnosticStatus.ERROR
                system_diag.message = "System is in critical state"
            else:
                system_diag.level = DiagnosticStatus.STALE
                system_diag.message = "System status unknown"
            
            # Add key values
            with self.health_lock:
                system_diag.values = [
                    KeyValue(key="robot_state", value=self.last_robot_state),
                    KeyValue(key="robot_healthy", value=str(self.last_robot_health)),
                    KeyValue(key="consecutive_failures", value=str(self.consecutive_failures)),
                    KeyValue(key="uptime", value=f"{time.time() - self.start_time:.1f}s"),
                ]
            
            diag_array.status.append(system_diag)
            self.diagnostics_publisher.publish(diag_array)
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish diagnostics: {str(e)}")
    
    def handle_critical_health(self):
        """Handle critical health conditions"""
        with self.health_lock:
            if self.consecutive_failures >= self.restart_threshold:
                self.get_logger().warn(
                    f"Critical health detected with {self.consecutive_failures} consecutive failures. "
                    f"Attempting system recovery..."
                )
                
                # Reset counter to prevent rapid restart attempts
                self.consecutive_failures = 0
                
                # Attempt recovery in a separate thread
                recovery_thread = threading.Thread(target=self.attempt_system_recovery)
                recovery_thread.start()
    
    def attempt_system_recovery(self):
        """Attempt to recover the system"""
        try:
            self.get_logger().info("Starting system recovery procedure...")
            
            # Stop current processes gracefully
            self.get_logger().info("Stopping existing Franka processes...")
            subprocess.run(['pkill', '-f', 'robust_franka_control'], capture_output=True)
            time.sleep(2.0)
            
            # Wait a bit for cleanup
            time.sleep(3.0)
            
            # Restart the robust control node
            self.get_logger().info("Restarting robust franka control node...")
            subprocess.Popen([
                'ros2', 'run', 'ros2_moveit_franka', 'robust_franka_control'
            ])
            
            self.get_logger().info("System recovery attempt completed")
            
        except Exception as e:
            self.get_logger().error(f"System recovery failed: {str(e)}")
    
    def get_system_info(self) -> Dict:
        """Get comprehensive system information for logging"""
        try:
            return {
                'cpu_usage': psutil.cpu_percent(),
                'memory_usage': psutil.virtual_memory().percent,
                'disk_usage': psutil.disk_usage('/').percent,
                'load_average': psutil.getloadavg(),
                'uptime': time.time() - self.start_time,
                'robot_state': self.last_robot_state,
                'robot_healthy': self.last_robot_health,
                'system_status': self.system_status.value,
            }
        except Exception as e:
            self.get_logger().error(f"Failed to get system info: {str(e)}")
            return {}


def main(args=None):
    """Main entry point"""
    try:
        rclpy.init(args=args)
        
        node = SystemHealthMonitor()
        
        # Use multi-threaded executor
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            node.get_logger().info("Starting system health monitor...")
            executor.spin()
        except KeyboardInterrupt:
            node.get_logger().info("Keyboard interrupt received")
        except Exception as e:
            node.get_logger().error(f"Unexpected error: {str(e)}")
        finally:
            node.destroy_node()
            executor.shutdown()
            
    except Exception as e:
        print(f"Failed to initialize system health monitor: {str(e)}")
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main() 