#!/usr/bin/env python3
"""
VR Teleoperation Node

Processes VR controller input and converts it to robot velocity commands.
Handles VR-specific calibration and coordinate transformations.

Responsibilities:
- Subscribe to VR controller input
- Perform coordinate transformations
- Apply safety limits and scaling
- Publish velocity commands to robot control node
- Handle VR calibration
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import json
import os
import yaml
from typing import Optional, Dict, Tuple
from dataclasses import dataclass
from scipy.spatial.transform import Rotation as R

# ROS2 messages and services
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose
from sensor_msgs.msg import JointState, Joy

# Custom messages
from lbx_interfaces.msg import VelocityCommand


@dataclass
class CalibrationData:
    """VR calibration data"""
    translation_offset: np.ndarray  # Translation offset between VR and robot
    rotation_offset: np.ndarray     # Rotation offset (quaternion)
    scale_factor: float             # Scale factor for position mapping
    calibrated: bool = False


class VRTeleopNode(Node):
    """VR teleoperation processing node"""
    
    def __init__(self):
        super().__init__('vr_teleop_node')
        
        # Parameters
        self.declare_parameter('config_file', '')
        self.declare_parameter('control_rate', 45.0)
        self.declare_parameter('calibration_file', 'vr_calibration_data.json')
        
        config_file = self.get_parameter('config_file').value
        self.calibration_file = self.get_parameter('calibration_file').value
        
        # Load configuration
        if config_file and os.path.exists(config_file):
            with open(config_file, 'r') as f:
                self.config = yaml.safe_load(f)
        else:
            self.config = self._get_default_config()
        
        # State
        self.vr_ready = False
        self.teleoperation_enabled = False
        self.calibration = CalibrationData(
            translation_offset=np.zeros(3),
            rotation_offset=np.array([0.0, 0.0, 0.0, 1.0]),
            scale_factor=1.0,
            calibrated=False
        )
        
        # Current states
        self.last_vr_pose = None
        self.last_vr_buttons = None
        self.current_robot_pose = None
        self.target_pose = None
        self.target_quaternion = None
        
        # Control variables
        self.last_action_time = 0.0
        self.control_active = False
        
        # Callback groups
        self.service_callback_group = ReentrantCallbackGroup()
        self.control_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Publishers
        self.ready_pub = self.create_publisher(
            Bool,
            '/vr/ready',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        )
        
        self.calibration_valid_pub = self.create_publisher(
            Bool,
            '/vr/calibration_valid',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        )
        
        self.velocity_command_pub = self.create_publisher(
            VelocityCommand,
            '/robot/velocity_command',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        )
        
        self.target_pose_pub = self.create_publisher(
            PoseStamped,
            '/vr/target_pose',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        # Subscribers
        self.vr_pose_sub = self.create_subscription(
            PoseStamped,
            '/vr/controller_pose',
            self.vr_pose_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=self.control_callback_group
        )
        
        self.vr_buttons_sub = self.create_subscription(
            Joy,
            '/vr/controller_joy',
            self.vr_buttons_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        self.robot_pose_sub = self.create_subscription(
            PoseStamped,
            '/robot/current_pose',
            self.robot_pose_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        # Service servers
        self.create_service(
            Trigger,
            '/vr/calibrate',
            self.calibrate_callback,
            callback_group=self.service_callback_group
        )
        
        self.create_service(
            SetBool,
            '/vr/enable_teleoperation',
            self.enable_teleoperation_callback,
            callback_group=self.service_callback_group
        )
        
        self.create_service(
            Trigger,
            '/vr/save_calibration',
            self.save_calibration_callback,
            callback_group=self.service_callback_group
        )
        
        # Status timer
        self.create_timer(1.0, self.publish_status)
        
        # Load calibration if exists
        self.load_calibration()
        
        self.get_logger().info("VR Teleoperation node started")
    
    def _get_default_config(self):
        """Get default configuration"""
        return {
            'vr_control': {
                'velocity_scale': 1.0,
                'rotation_scale': 1.0,
                'min_position_change': 0.0001,  # 0.1mm
                'workspace_origin': [0.5, 0.0, 0.3],
            },
            'calibration': {
                'position_scale': 1.0,
            },
            'gripper': {
                'trigger_threshold': 0.1,
            },
            'constants': {
                'GRIPPER_OPEN': 1,
                'GRIPPER_CLOSE': 0,
            }
        }
    
    def load_calibration(self):
        """Load calibration from file if exists"""
        if os.path.exists(self.calibration_file):
            try:
                with open(self.calibration_file, 'r') as f:
                    data = json.load(f)
                
                self.calibration.translation_offset = np.array(data.get('translation_offset', [0, 0, 0]))
                self.calibration.rotation_offset = np.array(data.get('rotation_offset', [0, 0, 0, 1]))
                self.calibration.scale_factor = data.get('scale_factor', 1.0)
                self.calibration.calibrated = True
                
                self.get_logger().info(f"Loaded calibration from {self.calibration_file}")
                self.get_logger().info(f"Translation offset: {self.calibration.translation_offset}")
                self.get_logger().info(f"Rotation offset: {self.calibration.rotation_offset}")
                self.get_logger().info(f"Scale factor: {self.calibration.scale_factor}")
            except Exception as e:
                self.get_logger().error(f"Failed to load calibration: {e}")
    
    def save_calibration(self):
        """Save calibration to file"""
        try:
            data = {
                'translation_offset': self.calibration.translation_offset.tolist(),
                'rotation_offset': self.calibration.rotation_offset.tolist(),
                'scale_factor': float(self.calibration.scale_factor)
            }
            
            with open(self.calibration_file, 'w') as f:
                json.dump(data, f, indent=2)
            
            self.get_logger().info(f"Saved calibration to {self.calibration_file}")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to save calibration: {e}")
            return False
    
    def vr_pose_callback(self, msg: PoseStamped):
        """Handle VR controller pose updates"""
        if not self.vr_ready:
            self.vr_ready = True
            self.get_logger().info("VR controller connected")
        
        # Only process if teleoperation is enabled
        if self.teleoperation_enabled and self.calibration.calibrated:
            self.process_vr_input(msg.pose)
    
    def vr_buttons_callback(self, msg: Joy):
        """Handle VR button updates"""
        self.last_vr_buttons = msg
    
    def robot_pose_callback(self, msg: PoseStamped):
        """Handle robot pose updates"""
        self.current_robot_pose = msg
    
    def publish_status(self):
        """Publish VR status"""
        # VR ready status
        ready_msg = Bool()
        ready_msg.data = self.vr_ready
        self.ready_pub.publish(ready_msg)
        
        # Calibration valid status
        calib_msg = Bool()
        calib_msg.data = self.calibration.calibrated
        self.calibration_valid_pub.publish(calib_msg)
    
    def process_vr_input(self, vr_pose: Pose):
        """Process VR pose and generate velocity commands"""
        # Convert VR pose to numpy arrays
        vr_position = np.array([
            vr_pose.position.x,
            vr_pose.position.y,
            vr_pose.position.z
        ])
        
        vr_quaternion = np.array([
            vr_pose.orientation.x,
            vr_pose.orientation.y,
            vr_pose.orientation.z,
            vr_pose.orientation.w
        ])
        
        # Apply calibration transformation
        robot_position, robot_quaternion = self.apply_calibration(vr_position, vr_quaternion)
        
        # Compute target pose
        self.target_pose = robot_position
        self.target_quaternion = robot_quaternion
        
        # Publish target pose for visualization
        target_pose_msg = PoseStamped()
        target_pose_msg.header.stamp = self.get_clock().now().to_msg()
        target_pose_msg.header.frame_id = "world"
        target_pose_msg.pose.position.x = robot_position[0]
        target_pose_msg.pose.position.y = robot_position[1]
        target_pose_msg.pose.position.z = robot_position[2]
        target_pose_msg.pose.orientation.x = robot_quaternion[0]
        target_pose_msg.pose.orientation.y = robot_quaternion[1]
        target_pose_msg.pose.orientation.z = robot_quaternion[2]
        target_pose_msg.pose.orientation.w = robot_quaternion[3]
        self.target_pose_pub.publish(target_pose_msg)
        
        # Only send velocity commands if we have robot state
        if self.current_robot_pose is None:
            return
        
        # Get current robot pose
        current_pos = np.array([
            self.current_robot_pose.pose.position.x,
            self.current_robot_pose.pose.position.y,
            self.current_robot_pose.pose.position.z
        ])
        
        current_quat = np.array([
            self.current_robot_pose.pose.orientation.x,
            self.current_robot_pose.pose.orientation.y,
            self.current_robot_pose.pose.orientation.z,
            self.current_robot_pose.pose.orientation.w
        ])
        
        # Compute velocity command (similar to original implementation)
        velocity_cmd = self.compute_velocity_command(
            robot_position, robot_quaternion,
            current_pos, current_quat
        )
        
        # Publish velocity command
        self.velocity_command_pub.publish(velocity_cmd)
        
        # Store last pose
        self.last_vr_pose = vr_pose
    
    def apply_calibration(self, vr_position: np.ndarray, vr_quaternion: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Apply calibration transformation to VR pose"""
        # Apply scale factor
        scaled_position = vr_position * self.calibration.scale_factor
        
        # Apply translation offset
        robot_position = scaled_position + self.calibration.translation_offset
        
        # Apply rotation offset
        vr_rotation = R.from_quat(vr_quaternion)
        offset_rotation = R.from_quat(self.calibration.rotation_offset)
        robot_rotation = offset_rotation * vr_rotation
        robot_quaternion = robot_rotation.as_quat()
        
        return robot_position, robot_quaternion
    
    def compute_velocity_command(
        self, 
        target_pos: np.ndarray, 
        target_quat: np.ndarray,
        current_pos: np.ndarray,
        current_quat: np.ndarray
    ) -> VelocityCommand:
        """Compute velocity command from target and current poses
        
        This matches the original oculus_vr_server_moveit.py implementation
        where velocities are computed as normalized direction vectors.
        """
        # Position difference
        pos_diff = target_pos - current_pos
        
        # Check if movement is significant
        if np.linalg.norm(pos_diff) < self.config['vr_control']['min_position_change']:
            # No significant movement - send zeros
            linear_vel = np.zeros(3)
        else:
            # Normalize position difference to get velocity direction
            # This matches the original implementation where velocities are [-1, 1]
            linear_vel = pos_diff / np.linalg.norm(pos_diff) if np.linalg.norm(pos_diff) > 0 else np.zeros(3)
            linear_vel *= self.config['vr_control']['velocity_scale']
        
        # Orientation difference
        current_rot = R.from_quat(current_quat)
        target_rot = R.from_quat(target_quat)
        rot_diff = target_rot * current_rot.inv()
        
        # Convert to axis-angle
        rotvec = rot_diff.as_rotvec()
        
        # Normalize rotation velocity
        angular_vel = rotvec / np.linalg.norm(rotvec) if np.linalg.norm(rotvec) > 0 else np.zeros(3)
        angular_vel *= self.config['vr_control']['rotation_scale']
        
        # Get gripper command from VR buttons
        gripper_vel = 0.0
        trigger_value = 0.0
        
        if self.last_vr_buttons is not None:
            # Joy message from oculus_node:
            # axes[0,1] = joystick x,y
            # axes[2] = grip analog (0-1)
            # axes[3] = trigger analog (0-1)
            if len(self.last_vr_buttons.axes) >= 4:
                trigger_value = self.last_vr_buttons.axes[3]
            
            # Convert trigger to gripper velocity
            # Positive velocity closes gripper, negative opens
            if trigger_value > self.config['gripper']['trigger_threshold']:
                gripper_vel = 1.0  # Close
            else:
                gripper_vel = -1.0  # Open
        
        # Create velocity command message
        velocity_cmd = VelocityCommand()
        velocity_cmd.header.stamp = self.get_clock().now().to_msg()
        velocity_cmd.velocities = [
            float(linear_vel[0]),
            float(linear_vel[1]),
            float(linear_vel[2]),
            float(angular_vel[0]),
            float(angular_vel[1]),
            float(angular_vel[2]),
            float(gripper_vel)
        ]
        
        # Add trigger value to message for FrankaController
        velocity_cmd.trigger_value = trigger_value
        
        return velocity_cmd
    
    # Service callbacks
    def calibrate_callback(self, request, response):
        """Handle calibration request"""
        self.get_logger().info("Starting VR calibration...")
        
        if not self.vr_ready:
            response.success = False
            response.message = "VR controller not ready"
            return response
        
        if not self.current_robot_pose:
            response.success = False
            response.message = "Robot pose not available"
            return response
        
        if self.last_vr_pose is None:
            response.success = False
            response.message = "No VR pose data available"
            return response
        
        # Get current VR pose
        vr_pos = np.array([
            self.last_vr_pose.position.x,
            self.last_vr_pose.position.y,
            self.last_vr_pose.position.z
        ])
        
        vr_quat = np.array([
            self.last_vr_pose.orientation.x,
            self.last_vr_pose.orientation.y,
            self.last_vr_pose.orientation.z,
            self.last_vr_pose.orientation.w
        ])
        
        # Get current robot pose
        robot_pos = np.array([
            self.current_robot_pose.pose.position.x,
            self.current_robot_pose.pose.position.y,
            self.current_robot_pose.pose.position.z
        ])
        
        robot_quat = np.array([
            self.current_robot_pose.pose.orientation.x,
            self.current_robot_pose.pose.orientation.y,
            self.current_robot_pose.pose.orientation.z,
            self.current_robot_pose.pose.orientation.w
        ])
        
        # Compute calibration parameters
        # Translation offset = robot_pos - vr_pos * scale
        # For now, use scale factor from config or 1.0
        scale = self.config['calibration'].get('position_scale', 1.0)
        self.calibration.scale_factor = scale
        self.calibration.translation_offset = robot_pos - vr_pos * scale
        
        # Rotation offset = robot_rot * vr_rot.inv()
        vr_rot = R.from_quat(vr_quat)
        robot_rot = R.from_quat(robot_quat)
        offset_rot = robot_rot * vr_rot.inv()
        self.calibration.rotation_offset = offset_rot.as_quat()
        
        self.calibration.calibrated = True
        
        self.get_logger().info(f"Calibration complete:")
        self.get_logger().info(f"  Translation offset: {self.calibration.translation_offset}")
        self.get_logger().info(f"  Rotation offset: {self.calibration.rotation_offset}")
        self.get_logger().info(f"  Scale factor: {self.calibration.scale_factor}")
        
        response.success = True
        response.message = "Calibration successful"
        return response
    
    def enable_teleoperation_callback(self, request, response):
        """Handle teleoperation enable/disable request"""
        self.teleoperation_enabled = request.data
        
        if self.teleoperation_enabled:
            if not self.calibration.calibrated:
                response.success = False
                response.message = "Calibration required before enabling teleoperation"
                self.teleoperation_enabled = False
            else:
                response.success = True
                response.message = "Teleoperation enabled"
                self.get_logger().info("VR teleoperation enabled")
        else:
            response.success = True
            response.message = "Teleoperation disabled"
            self.get_logger().info("VR teleoperation disabled")
        
        return response
    
    def save_calibration_callback(self, request, response):
        """Handle save calibration request"""
        if self.save_calibration():
            response.success = True
            response.message = "Calibration saved successfully"
        else:
            response.success = False
            response.message = "Failed to save calibration"
        return response


def main(args=None):
    rclpy.init(args=args)
    
    node = VRTeleopNode()
    
    # Use multi-threaded executor
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 