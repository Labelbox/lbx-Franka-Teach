#!/usr/bin/env python3
"""
Robot Control Node

Dedicated node for robot control operations using MoveIt.
Uses FrankaController internally for all the sophisticated control logic.

Responsibilities:
- MoveIt interface for motion planning and execution
- Service server for robot commands (reset_to_home, enable_control)
- Velocity command processing
- Robot state publishing
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import threading
import time
import yaml
import os
from typing import Optional, Dict

# ROS2 messages and services
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

# Import the existing FrankaController with all its logic
from .franka_controller import FrankaController

# Custom messages
from lbx_interfaces.msg import VelocityCommand

# For robot state
from dataclasses import dataclass


@dataclass
class RobotState:
    """Robot state for FrankaController compatibility"""
    pos: np.ndarray
    quat: np.ndarray
    gripper: float
    joint_positions: Optional[np.ndarray] = None


class RobotControlNode(Node):
    """Dedicated robot control node using FrankaController"""
    
    def __init__(self):
        super().__init__('robot_control_node')
        
        # Parameters
        self.declare_parameter('config_file', '')
        self.declare_parameter('robot_name', 'fr3')
        self.declare_parameter('control_rate', 45.0)
        
        config_file = self.get_parameter('config_file').value
        
        # Load configuration
        if config_file and os.path.exists(config_file):
            with open(config_file, 'r') as f:
                self.config = yaml.safe_load(f)
        else:
            # Use default configuration
            self.config = self._get_default_config()
        
        # State
        self.robot_ready = False
        self.control_enabled = False
        self.robot_state = RobotState(
            pos=np.array([0.5, 0.0, 0.5]),
            quat=np.array([0.0, 0.0, 0.0, 1.0]),
            gripper=0.0
        )
        
        # Callback groups
        self.service_callback_group = ReentrantCallbackGroup()
        self.control_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Create FrankaController with all the sophisticated logic
        self.controller = FrankaController(self, self.config)
        
        # Publishers
        self.ready_pub = self.create_publisher(
            Bool, 
            '/robot/ready',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        )
        
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/robot/current_pose',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        self.velocity_command_sub = self.create_subscription(
            VelocityCommand,
            '/robot/velocity_command',
            self.velocity_command_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=self.control_callback_group
        )
        
        # Service servers
        self.create_service(
            Trigger,
            '/robot/reset_to_home',
            self.reset_to_home_callback,
            callback_group=self.service_callback_group
        )
        
        self.create_service(
            SetBool,
            '/robot/enable_control',
            self.enable_control_callback,
            callback_group=self.service_callback_group
        )
        
        self.create_service(
            Trigger,
            '/robot/emergency_stop',
            self.emergency_stop_callback,
            callback_group=self.service_callback_group
        )
        
        # Timers
        self.create_timer(1.0, self.publish_status)
        self.create_timer(1.0, self.publish_ready_status)  # Publish ready status
        
        # Initialize state tracking
        self.has_valid_joint_states = False
        
        # Check if controller is ready
        if self.controller.is_fully_operational():
            self.robot_ready = True
            self.get_logger().info("✅ Robot control node ready with FrankaController")
            
            # Don't auto-reset to home on startup - let system orchestrator handle it
            # This prevents conflicts and robot crashes
            self.get_logger().info("Waiting for reset command from system orchestrator...")
        else:
            self.get_logger().warn("⚠️  Robot control node started but some services not ready")
    
    def _get_default_config(self):
        """Get default configuration matching franka_vr_control_config.yaml structure"""
        return {
            'robot': {
                'planning_group': 'fr3_arm',
                'base_frame': 'fr3_link0',
                'end_effector_link': 'fr3_hand_tcp',
                'joint_names': [f'fr3_joint{i}' for i in range(1, 8)],
                'home_positions': [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
                'workspace_min': [0.1, -0.6, 0.1],
                'workspace_max': [0.8, 0.6, 0.8],
                'robot_ip': '192.168.1.60',
            },
            'moveit': {
                'service_wait_timeout_sec': 10.0,
                'ik_timeout_sec': 0.1,
                'ik_attempts': 10,
                'fk_timeout_sec': 0.1,
                'planning_scene_timeout_sec': 0.5,
                'trajectory_duration_single_point': 0.1,
                'trajectory_duration_reset': 5.0,
                'velocity_scale_factor': 1.2,
                'max_joint_velocity': 2.0,
                'path_tolerance_position': 0.01,
                'path_tolerance_velocity': 0.1,
                'path_tolerance_acceleration': 0.5,
                'goal_tolerance_position': 0.005,
                'goal_tolerance_velocity': 0.05,
                'goal_tolerance_acceleration': 0.5,
            },
            'vr_control': {
                'control_hz': 45.0,
                'max_lin_delta': 0.01,
                'max_rot_delta': 0.05,
                'min_command_interval': 0.02,
                'pose_smoothing_enabled': True,
                'pose_smoothing_alpha': 0.3,
                'adaptive_smoothing': True,
                'max_gripper_smoothing_delta': 0.1,
            },
            'gripper': {
                'trigger_threshold': 0.5,
                'open_width': 0.08,
                'close_width': 0.0,
                'speed': 0.1,
                'grasp_force': 40.0,
                'epsilon_inner': 0.005,
                'epsilon_outer': 0.005,
            },
            'constants': {
                'GRIPPER_OPEN': 1,
                'GRIPPER_CLOSE': 0,
            },
            'debug': {
                'debug_ik_failures': False,
                'debug_comm_stats': False,
                'debug_moveit': False,
            }
        }
    
    def joint_state_callback(self, msg: JointState):
        """Update current joint state and robot pose"""
        # Update controller's joint state
        self.controller.joint_state = msg
        
        # Extract joint positions for our robot
        joint_positions = []
        for joint_name in self.config['robot']['joint_names']:
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                joint_positions.append(msg.position[idx])
        
        if len(joint_positions) == 7:
            # Update robot state joint positions
            self.robot_state.joint_positions = np.array(joint_positions)
            
            # Get end-effector pose using FK
            pos, quat = self.controller.get_current_end_effector_pose(joint_positions)
            if pos is not None and quat is not None:
                self.robot_state.pos = pos
                self.robot_state.quat = quat
                
                # Publish current pose
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = self.config['robot']['base_frame']
                pose_msg.pose.position.x = pos[0]
                pose_msg.pose.position.y = pos[1]
                pose_msg.pose.position.z = pos[2]
                pose_msg.pose.orientation.x = quat[0]
                pose_msg.pose.orientation.y = quat[1]
                pose_msg.pose.orientation.z = quat[2]
                pose_msg.pose.orientation.w = quat[3]
                self.pose_pub.publish(pose_msg)
            
            # Update gripper state
            self.robot_state.gripper = self.controller.get_current_gripper_state(msg)
            
            # Mark that we have valid joint states
            self.has_valid_joint_states = True
    
    def velocity_command_callback(self, msg: VelocityCommand):
        """Handle velocity commands"""
        if not self.control_enabled:
            return
        
        # Convert to numpy array for FrankaController
        velocities = np.array(msg.velocities[:7])  # 3 linear + 3 angular + 1 gripper
        
        # Create action info dict
        action_info = {}
        
        # Check if we have trigger value in the message header (custom field)
        # Otherwise use gripper velocity as proxy
        if hasattr(msg, 'trigger_value'):
            action_info['trigger_value'] = msg.trigger_value
        else:
            # Use gripper velocity as trigger proxy (>0 means close)
            action_info['trigger_value'] = 1.0 if velocities[6] > 0 else 0.0
        
        # Let FrankaController handle all the sophisticated control
        self.controller.execute_command(velocities, action_info, self.robot_state)
    
    def publish_status(self):
        """Publish robot ready status"""
        # Update readiness based on controller status
        self.robot_ready = self.controller.is_fully_operational()
        
        msg = Bool()
        msg.data = self.robot_ready
        self.ready_pub.publish(msg)
    
    def publish_ready_status(self):
        """Publish robot ready status"""
        # Update readiness based on controller status
        self.robot_ready = self.controller.is_fully_operational()
        
        msg = Bool()
        msg.data = self.robot_ready
        self.ready_pub.publish(msg)
    
    # Service callbacks
    def reset_to_home_callback(self, request, response):
        """Service callback for reset to home"""
        success = self.reset_to_home()
        response.success = success
        response.message = "Robot reset to home position" if success else "Failed to reset robot"
        return response
    
    def enable_control_callback(self, request, response):
        """Service callback for enabling/disabling control"""
        self.control_enabled = request.data
        
        if self.control_enabled:
            self.get_logger().info("Robot control enabled")
        else:
            self.get_logger().info("Robot control disabled")
            # Reset controller state when disabling
            self.controller._last_gripper_command = None
        
        response.success = True
        response.message = f"Control {'enabled' if self.control_enabled else 'disabled'}"
        return response
    
    def emergency_stop_callback(self, request, response):
        """Service callback for emergency stop"""
        self.controller.emergency_stop()
        self.control_enabled = False
        response.success = True
        response.message = "Emergency stop executed"
        return response
    
    def reset_to_home(self) -> bool:
        """Reset robot to home position using FrankaController"""
        try:
            # Use FrankaController's sophisticated reset logic
            pos, quat, joint_positions = self.controller.reset_robot()
            
            # Update robot state
            self.robot_state.pos = pos
            self.robot_state.quat = quat
            if joint_positions is not None:
                self.robot_state.joint_positions = joint_positions
            
            return True
        except Exception as e:
            self.get_logger().error(f"Exception during reset to home: {e}")
            return False


def main(args=None):
    rclpy.init(args=args)
    
    node = RobotControlNode()
    
    # Use multi-threaded executor for concurrent operations
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        # Print stats before shutdown
        if hasattr(node, 'controller'):
            node.controller.print_stats()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 