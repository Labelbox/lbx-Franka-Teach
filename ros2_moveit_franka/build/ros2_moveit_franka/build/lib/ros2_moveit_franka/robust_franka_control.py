#!/usr/bin/env python3
"""
Robust Franka Control Node with Exception Handling and Auto-Recovery
This node provides a crash-proof interface to the Franka robot with automatic
restart capabilities and comprehensive error handling.

ROS 2 Version: Uses direct service calls to MoveIt instead of moveit_commander
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# ROS 2 MoveIt service interfaces
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetPlanningScene
from moveit_msgs.msg import (
    PlanningScene, RobotState, JointConstraint, Constraints,
    PositionIKRequest, RobotTrajectory, MotionPlanRequest
)
from moveit_msgs.action import MoveGroup

# Standard ROS 2 messages
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState

# Handle franka_msgs import with fallback
try:
    from franka_msgs.msg import FrankaState
    FRANKA_MSGS_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: Failed to import franka_msgs: {e}")
    FRANKA_MSGS_AVAILABLE = False
    # Create dummy message for graceful failure
    class DummyFrankaState:
        def __init__(self):
            self.robot_mode = 0
    FrankaState = DummyFrankaState

import time
import threading
import traceback
import sys
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Dict, Any
import signal


class RobotState(Enum):
    """Robot state enumeration for state machine"""
    INITIALIZING = "initializing"
    READY = "ready"
    MOVING = "moving"
    ERROR = "error"
    RECOVERING = "recovering"
    DISCONNECTED = "disconnected"


@dataclass
class RecoveryConfig:
    """Configuration for recovery behavior"""
    max_retries: int = 5
    retry_delay: float = 2.0
    connection_timeout: float = 10.0
    emergency_stop_timeout: float = 1.0
    health_check_interval: float = 1.0


class RobustFrankaControl(Node):
    """
    Robust Franka control node with exception handling and auto-recovery
    Uses ROS 2 service calls to MoveIt instead of moveit_commander
    """
    
    def __init__(self):
        super().__init__('robust_franka_control')
        
        self.get_logger().info("Using ROS 2 native MoveIt interface (service calls)")
        
        # Recovery configuration
        self.recovery_config = RecoveryConfig()
        
        # State management
        self.robot_state = RobotState.INITIALIZING
        self.retry_count = 0
        self.last_error = None
        self.shutdown_requested = False
        
        # Threading and synchronization
        self.callback_group = ReentrantCallbackGroup()
        self.state_lock = threading.Lock()
        self.recovery_thread = None
        
        # MoveIt service clients (ROS 2 approach)
        self.move_group_client = ActionClient(
            self, MoveGroup, '/move_action', callback_group=self.callback_group
        )
        self.planning_scene_client = self.create_client(
            GetPlanningScene, '/get_planning_scene', callback_group=self.callback_group
        )
        self.ik_client = self.create_client(
            GetPositionIK, '/compute_ik', callback_group=self.callback_group
        )
        self.fk_client = self.create_client(
            GetPositionFK, '/compute_fk', callback_group=self.callback_group
        )
        
        # Current robot state
        self.current_joint_state = None
        self.planning_group = "panda_arm"  # Default planning group
        
        # Publishers and subscribers
        self.state_publisher = self.create_publisher(
            String, 'robot_state', 10, callback_group=self.callback_group
        )
        self.error_publisher = self.create_publisher(
            String, 'robot_errors', 10, callback_group=self.callback_group
        )
        self.health_publisher = self.create_publisher(
            Bool, 'robot_health', 10, callback_group=self.callback_group
        )
        
        # Command subscriber
        self.command_subscriber = self.create_subscription(
            PoseStamped,
            'target_pose',
            self.pose_command_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Joint state subscriber for current robot state
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Franka state subscriber for monitoring (only if franka_msgs available)
        if FRANKA_MSGS_AVAILABLE:
            self.franka_state_subscriber = self.create_subscription(
                FrankaState,
                'franka_robot_state_broadcaster/robot_state',
                self.franka_state_callback,
                10,
                callback_group=self.callback_group
            )
        else:
            self.get_logger().warn("franka_msgs not available - Franka state monitoring disabled")
        
        # Health monitoring timer
        self.health_timer = self.create_timer(
            self.recovery_config.health_check_interval,
            self.health_check_callback,
            callback_group=self.callback_group
        )
        
        # Status reporting timer
        self.status_timer = self.create_timer(
            1.0,  # Report status every second
            self.status_report_callback,
            callback_group=self.callback_group
        )
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        self.get_logger().info("Robust Franka Control Node initialized")
        
        # Start initialization in a separate thread
        self.initialization_thread = threading.Thread(target=self.initialize_robot)
        self.initialization_thread.start()
    
    def signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        self.get_logger().info(f"Received signal {signum}, initiating graceful shutdown...")
        self.shutdown_requested = True
        self.set_robot_state(RobotState.DISCONNECTED)
    
    def set_robot_state(self, new_state: RobotState):
        """Thread-safe state setter"""
        with self.state_lock:
            old_state = self.robot_state
            self.robot_state = new_state
            self.get_logger().info(f"Robot state changed: {old_state.value} -> {new_state.value}")
    
    def get_robot_state(self) -> RobotState:
        """Thread-safe state getter"""
        with self.state_lock:
            return self.robot_state
    
    def joint_state_callback(self, msg: JointState):
        """Update current joint state"""
        self.current_joint_state = msg
    
    def initialize_robot(self):
        """Initialize robot connection with error handling"""
        max_init_retries = 3
        init_retry_count = 0
        
        while init_retry_count < max_init_retries and not self.shutdown_requested:
            try:
                self.get_logger().info(f"Initializing robot connection (attempt {init_retry_count + 1}/{max_init_retries})")
                
                # Wait for MoveIt services to be available
                self.get_logger().info("Waiting for MoveIt services...")
                
                if not self.move_group_client.wait_for_server(timeout_sec=10.0):
                    raise Exception("MoveGroup action server not available")
                
                if not self.planning_scene_client.wait_for_service(timeout_sec=5.0):
                    raise Exception("Planning scene service not available")
                
                self.get_logger().info("✓ MoveGroup action server available")
                self.get_logger().info("✓ Planning scene service available")
                
                # Test connection by getting planning scene
                if self.test_robot_connection():
                    self.get_logger().info("Successfully connected to MoveIt!")
                    self.set_robot_state(RobotState.READY)
                    self.retry_count = 0
                    self.last_error = None
                    break
                else:
                    raise Exception("Robot connection test failed")
                
            except Exception as e:
                init_retry_count += 1
                error_msg = f"Initialization failed (attempt {init_retry_count}): {str(e)}"
                self.get_logger().error(error_msg)
                self.publish_error(error_msg)
                
                if init_retry_count >= max_init_retries:
                    self.get_logger().error("Max initialization retries reached. Setting error state.")
                    self.set_robot_state(RobotState.ERROR)
                    self.last_error = str(e)
                    break
                else:
                    time.sleep(self.recovery_config.retry_delay)
    
    def pose_command_callback(self, msg: PoseStamped):
        """Handle pose command with error handling"""
        if self.get_robot_state() != RobotState.READY:
            self.get_logger().warn(f"Ignoring pose command - robot not ready (state: {self.robot_state.value})")
            return
        
        try:
            self.execute_pose_command(msg.pose)
        except Exception as e:
            self.handle_execution_error(e, "pose_command")
    
    def execute_pose_command(self, target_pose: Pose):
        """Execute pose command using ROS 2 MoveIt action"""
        self.set_robot_state(RobotState.MOVING)
        
        try:
            self.get_logger().info(f"Executing pose command: {target_pose.position}")
            
            # Create MoveGroup goal
            goal = MoveGroup.Goal()
            goal.request.group_name = self.planning_group
            goal.request.num_planning_attempts = 5
            goal.request.allowed_planning_time = 10.0
            goal.request.max_velocity_scaling_factor = 0.3
            goal.request.max_acceleration_scaling_factor = 0.3
            
            # Set target pose
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "panda_link0"
            pose_stamped.pose = target_pose
            goal.request.goal_constraints.append(self.create_pose_constraint(pose_stamped))
            
            # Send goal and wait for result
            self.get_logger().info("Sending goal to MoveGroup...")
            future = self.move_group_client.send_goal_async(goal)
            
            # This is a simplified synchronous approach
            # In production, you'd want to handle this asynchronously
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            
            if future.result() is not None:
                goal_handle = future.result()
                if goal_handle.accepted:
                    self.get_logger().info("Goal accepted, waiting for result...")
                    result_future = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
                    
                    if result_future.result() is not None:
                        result = result_future.result()
                        if result.result.error_code.val == 1:  # SUCCESS
                            self.get_logger().info("Motion completed successfully")
                            self.set_robot_state(RobotState.READY)
                        else:
                            raise Exception(f"Motion planning failed with error code: {result.result.error_code.val}")
                    else:
                        raise Exception("Failed to get motion result")
                else:
                    raise Exception("Goal was rejected by MoveGroup")
            else:
                raise Exception("Failed to send goal to MoveGroup")
                
        except Exception as e:
            self.handle_execution_error(e, "execute_pose")
            raise
    
    def create_pose_constraint(self, pose_stamped: PoseStamped) -> Constraints:
        """Create pose constraints for MoveIt planning"""
        constraints = Constraints()
        # This is a simplified version - in practice you'd create proper constraints
        # For now, we'll use this as a placeholder
        return constraints
    
    def handle_execution_error(self, error: Exception, context: str):
        """Handle execution errors with recovery logic"""
        error_msg = f"Error in {context}: {str(error)}"
        self.get_logger().error(error_msg)
        self.publish_error(error_msg)
        
        self.set_robot_state(RobotState.ERROR)
        self.last_error = str(error)
        
        # Start recovery if not already running
        if not self.recovery_thread or not self.recovery_thread.is_alive():
            self.recovery_thread = threading.Thread(target=self.recovery_procedure)
            self.recovery_thread.start()
    
    def recovery_procedure(self):
        """Comprehensive recovery procedure"""
        self.get_logger().info("Starting recovery procedure...")
        self.set_robot_state(RobotState.RECOVERING)
        
        recovery_start_time = time.time()
        
        while self.retry_count < self.recovery_config.max_retries and not self.shutdown_requested:
            try:
                self.retry_count += 1
                self.get_logger().info(f"Recovery attempt {self.retry_count}/{self.recovery_config.max_retries}")
                
                # Wait before retry
                time.sleep(self.recovery_config.retry_delay)
                
                # Test basic functionality
                if self.test_robot_connection():
                    self.get_logger().info("Recovery successful!")
                    self.set_robot_state(RobotState.READY)
                    self.retry_count = 0
                    self.last_error = None
                    return
                
            except Exception as e:
                error_msg = f"Recovery attempt {self.retry_count} failed: {str(e)}"
                self.get_logger().error(error_msg)
                self.publish_error(error_msg)
                
                # Check if we've exceeded recovery time
                if time.time() - recovery_start_time > 60.0:  # 60 second recovery timeout
                    break
        
        # Recovery failed
        self.get_logger().error("Recovery procedure failed. Manual intervention required.")
        self.set_robot_state(RobotState.ERROR)
    
    def test_robot_connection(self) -> bool:
        """Test robot connection and basic functionality"""
        try:
            # Test planning scene service
            if not self.planning_scene_client.service_is_ready():
                self.get_logger().warn("Planning scene service not ready")
                return False
            
            # Try to get planning scene
            request = GetPlanningScene.Request()
            future = self.planning_scene_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                self.get_logger().info("Robot connection test passed")
                return True
            else:
                self.get_logger().warn("Failed to get planning scene")
                return False
            
        except Exception as e:
            self.get_logger().error(f"Robot connection test failed: {str(e)}")
            return False
    
    def franka_state_callback(self, msg: FrankaState):
        """Monitor Franka state for errors"""
        if not FRANKA_MSGS_AVAILABLE:
            return
            
        try:
            # Check for robot errors in the state message
            if hasattr(msg, 'robot_mode') and msg.robot_mode == 4:  # Error mode
                self.get_logger().warn("Franka robot is in error mode")
                if self.get_robot_state() == RobotState.READY:
                    self.handle_execution_error(Exception("Robot entered error mode"), "franka_state")
                    
        except Exception as e:
            self.get_logger().error(f"Error processing Franka state: {str(e)}")
    
    def health_check_callback(self):
        """Periodic health check"""
        try:
            current_state = self.get_robot_state()
            is_healthy = current_state in [RobotState.READY, RobotState.MOVING]
            
            # Publish health status
            health_msg = Bool()
            health_msg.data = is_healthy
            self.health_publisher.publish(health_msg)
            
            # If we're in ready state, do a quick connection test
            if current_state == RobotState.READY:
                try:
                    # Quick non-intrusive test
                    if not self.planning_scene_client.service_is_ready():
                        self.get_logger().warn("Health check: Planning scene service not ready")
                        self.handle_execution_error(Exception("Planning scene service not ready"), "health_check")
                except Exception as e:
                    self.get_logger().warn(f"Health check detected connection issue: {str(e)}")
                    self.handle_execution_error(e, "health_check")
                    
        except Exception as e:
            self.get_logger().error(f"Health check failed: {str(e)}")
    
    def status_report_callback(self):
        """Publish regular status reports"""
        try:
            # Publish current state
            state_msg = String()
            state_msg.data = self.robot_state.value
            self.state_publisher.publish(state_msg)
            
            # Log status periodically (every 10 seconds)
            if hasattr(self, '_last_status_log'):
                if time.time() - self._last_status_log > 10.0:
                    self._log_status()
                    self._last_status_log = time.time()
            else:
                self._last_status_log = time.time()
                
        except Exception as e:
            self.get_logger().error(f"Status report failed: {str(e)}")
    
    def _log_status(self):
        """Log comprehensive status information"""
        status_info = {
            'state': self.robot_state.value,
            'retry_count': self.retry_count,
            'last_error': self.last_error,
            'move_group_available': self.move_group_client.server_is_ready(),
            'planning_scene_available': self.planning_scene_client.service_is_ready(),
            'has_joint_state': self.current_joint_state is not None,
            'franka_msgs_available': FRANKA_MSGS_AVAILABLE,
        }
        
        if self.current_joint_state is not None:
            status_info['joint_count'] = len(self.current_joint_state.position)
        
        self.get_logger().info(f"Status: {status_info}")
    
    def publish_error(self, error_message: str):
        """Publish error message"""
        try:
            error_msg = String()
            error_msg.data = f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {error_message}"
            self.error_publisher.publish(error_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish error: {str(e)}")
    
    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down robust franka control node...")
        self.shutdown_requested = True
        
        # Wait for recovery thread to finish
        if self.recovery_thread and self.recovery_thread.is_alive():
            self.recovery_thread.join(timeout=5.0)
        
        # Wait for initialization thread to finish
        if hasattr(self, 'initialization_thread') and self.initialization_thread.is_alive():
            self.initialization_thread.join(timeout=5.0)
        
        super().destroy_node()


def main(args=None):
    """Main entry point"""
    try:
        rclpy.init(args=args)
        
        # Create robust control node
        node = RobustFrankaControl()
        
        # Use multi-threaded executor for better concurrency
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            node.get_logger().info("Starting robust franka control node...")
            executor.spin()
        except KeyboardInterrupt:
            node.get_logger().info("Keyboard interrupt received")
        except Exception as e:
            node.get_logger().error(f"Unexpected error in main loop: {str(e)}")
            traceback.print_exc()
        finally:
            node.destroy_node()
            executor.shutdown()
            
    except Exception as e:
        print(f"Failed to initialize ROS2: {str(e)}")
        traceback.print_exc()
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main() 