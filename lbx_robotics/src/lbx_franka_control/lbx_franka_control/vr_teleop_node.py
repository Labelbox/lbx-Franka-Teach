#!/usr/bin/env python3
"""
VR Teleoperation Node

Processes VR controller input and converts it to robot velocity commands.
Handles VR-specific calibration and coordinate transformations.

This node implements the exact VR-to-robot pipeline from oculus_vr_server_moveit.py:
1. VR motion captured and transformed
2. Motion differences converted to velocity commands using gains
3. Velocities scaled to position deltas using max_delta parameters
4. Publishes normalized velocity commands for robot_control_node

Responsibilities:
- Subscribe to VR controller input
- Perform coordinate transformations and calibration
- Apply safety limits and scaling
- Publish velocity commands to robot control node
- Handle VR calibration (forward direction and origin)
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
import time
import threading
from typing import Optional, Dict, Tuple
from dataclasses import dataclass
from scipy.spatial.transform import Rotation as R
import copy

# ROS2 messages and services
from std_msgs.msg import Bool, String, Int32
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose
from sensor_msgs.msg import JointState, Joy

# Custom messages
from lbx_interfaces.msg import VelocityCommand, VRControllerState


@dataclass
class CalibrationData:
    """VR calibration data"""
    translation_offset: np.ndarray  # Translation offset between VR and robot
    rotation_offset: np.ndarray     # Rotation offset (quaternion)
    scale_factor: float             # Scale factor for position mapping
    calibrated: bool = False


class VRTeleopNode(Node):
    """VR teleoperation processing node with sophisticated control logic"""
    
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
        
        # Initialize state from config
        self.controller_id = "r" if self.config['vr_control']['use_right_controller'] else "l"
        self.control_hz = self.config['vr_control']['control_hz']
        
        # Initialize transformation matrices (preserved exactly from original)
        self.global_to_env_mat = self.vec_to_reorder_mat(self.config['vr_control']['coord_transform'])
        self.vr_to_global_mat = np.eye(4)
        
        # State - matching original system_manager implementation
        self.reset_state()
        
        # Thread-safe VR state
        self._vr_state_lock = threading.Lock()
        self._latest_vr_pose = None
        self._latest_vr_buttons = None
        
        # Calibration
        self.calibration = CalibrationData(
            translation_offset=np.zeros(3),
            rotation_offset=np.array([0.0, 0.0, 0.0, 1.0]),
            scale_factor=1.0,
            calibrated=False
        )
        
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
        
        self.vr_control_state_pub = self.create_publisher(
            VRControllerState,
            '/vr/controller_state',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
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
            '/vr/controller_buttons',
            self.vr_buttons_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        self.robot_pose_sub = self.create_subscription(
            PoseStamped,
            '/robot/current_pose',
            self.robot_pose_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        # Services for commands
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
        
        # Services to handle recording requests (pass-through to system)
        self.start_recording_pub = self.create_publisher(
            Bool, '/recording/start_request', 1
        )
        
        self.stop_recording_pub = self.create_publisher(
            Int32, '/recording/stop_request', 1  # Int32 for success/failure code
        )
        
        # Status timer
        self.create_timer(1.0, self.publish_status)
        
        # Control loop timer - runs at control_hz
        self.control_timer = self.create_timer(
            1.0 / self.control_hz,
            self.control_loop,
            callback_group=self.control_callback_group
        )
        
        self.get_logger().info('🎮 VR Teleoperation node initialized')
        self.get_logger().info(f'   Control rate: {self.control_hz}Hz')
        self.get_logger().info(f'   Controller: {"Right" if self.controller_id == "r" else "Left"}')
    
    def reset_state(self):
        """Reset internal state - exactly as in original system_manager"""
        self._state = {
            "poses": {},
            "buttons": {"A": False, "B": False, "X": False, "Y": False},
            "movement_enabled": False,
            "controller_on": True,
        }
        self.update_sensor = True
        self.reset_origin = True
        self.reset_orientation = True
        self.robot_origin = None
        self.vr_origin = None
        self.vr_state = None
        
        # Robot state
        self.robot_pos = None
        self.robot_quat = None
        self.robot_euler = None
        self.current_robot_pose = None
        
        # Calibration state
        self.prev_joystick_state = False
        self.prev_grip_state = False
        self.calibrating_forward = False
        self.calibration_start_pose = None
        self.calibration_start_time = None
        self.vr_neutral_pose = None
        
        # Control state
        self.teleoperation_enabled = False
        self.vr_ready = False
        self._last_vr_pos = None
        self._last_action = np.zeros(7)
        self._last_command_time = 0.0
        
        # Recording state
        self.prev_a_button = False
        self.prev_b_button = False
    
    def _get_default_config(self):
        """Get default configuration matching franka_vr_control_config.yaml structure"""
        return {
            'vr_control': {
                'use_right_controller': True,
                'control_hz': 45.0,
                'translation_sensitivity': 3.0,
                'rotation_sensitivity': 2.0,
                'max_lin_vel': 0.5,
                'max_rot_vel': 1.0,
                'workspace_radius': 1.5,
                'neutral_gripper_val': 0.0,
                'coord_transform': [1, -3, 2],  # X, -Z, Y
                'translation_gain': 1.5,
                'rotation_gain': 2.0,
                'max_lin_delta': 0.01,
                'max_rot_delta': 0.05,
                'scale_factor': 1.5,
                'rotation_scale': 2.0,
                'fixed_orientation': False,
                'position_smoothing_enabled': True,
                'position_smoothing_alpha': 0.3,
            },
            'gripper': {
                'trigger_threshold': 0.5,
                'open_width': 0.08,
                'close_width': 0.0,
            },
            'recording': {
                'auto_mark_success_threshold': 2.0,
            },
            'constants': {
                'GRIPPER_OPEN': 1,
                'GRIPPER_CLOSE': 0,
            }
        }
    
    # Preserved transformation functions exactly from original
    def vec_to_reorder_mat(self, vec):
        """Convert reordering vector to transformation matrix"""
        X = np.zeros((len(vec), len(vec)))
        for i in range(X.shape[0]):
            ind = int(abs(vec[i])) - 1
            X[i, ind] = np.sign(vec[i])
        return X
    
    def quat_to_euler(self, quat, degrees=False):
        """Convert quaternion to euler angles"""
        euler = R.from_quat(quat).as_euler("xyz", degrees=degrees)
        return euler
    
    def euler_to_quat(self, euler, degrees=False):
        """Convert euler angles to quaternion"""
        return R.from_euler("xyz", euler, degrees=degrees).as_quat()
    
    def quat_diff(self, target, source):
        """Calculate quaternion difference"""
        result = R.from_quat(target) * R.from_quat(source).inv()
        return result.as_quat()
    
    def add_angles(self, delta, source, degrees=False):
        """Add two sets of euler angles"""
        delta_rot = R.from_euler("xyz", delta, degrees=degrees)
        source_rot = R.from_euler("xyz", source, degrees=degrees)
        new_rot = delta_rot * source_rot
        return new_rot.as_euler("xyz", degrees=degrees)
    
    def vr_pose_callback(self, msg: PoseStamped):
        """Handle VR controller pose updates"""
        # Convert PoseStamped to 4x4 transformation matrix
        pose_mat = np.eye(4)
        pose_mat[:3, 3] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        
        quat = [msg.pose.orientation.x, msg.pose.orientation.y, 
                msg.pose.orientation.z, msg.pose.orientation.w]
        pose_mat[:3, :3] = R.from_quat(quat).as_matrix()
        
        # Update state
        with self._vr_state_lock:
            self._latest_vr_pose = pose_mat
            if not self.vr_ready:
                self.vr_ready = True
                self.get_logger().info("✅ VR controller connected")
    
    def vr_buttons_callback(self, msg: Joy):
        """Handle VR controller button updates - matching original format"""
        with self._vr_state_lock:
            self._latest_vr_buttons = {}
            
            # Map Joy message to button states (following oculus_reader format)
            if len(msg.buttons) >= 7:
                # A/X button - button[0] on right, button[3] on left  
                if self.controller_id == "r":
                    self._latest_vr_buttons["A"] = bool(msg.buttons[0])
                    self._latest_vr_buttons["B"] = bool(msg.buttons[1])
                else:
                    self._latest_vr_buttons["X"] = bool(msg.buttons[0])
                    self._latest_vr_buttons["Y"] = bool(msg.buttons[1])
                
                # Controller-specific buttons with uppercase prefix
                self._latest_vr_buttons[self.controller_id.upper() + "G"] = bool(msg.buttons[4])  # Grip
                self._latest_vr_buttons[self.controller_id.upper() + "J"] = bool(msg.buttons[6])  # Joystick press
                
                # Trigger as continuous value - exactly as in original
                if len(msg.axes) >= 3:
                    trigger_key = "rightTrig" if self.controller_id == "r" else "leftTrig"
                    self._latest_vr_buttons[trigger_key] = [msg.axes[2]]  # Index trigger as list
    
    def robot_pose_callback(self, msg: PoseStamped):
        """Handle robot pose updates"""
        self.current_robot_pose = msg
        
        # Update internal robot state
        self.robot_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.robot_quat = np.array([
            msg.pose.orientation.x, msg.pose.orientation.y, 
            msg.pose.orientation.z, msg.pose.orientation.w
        ])
        self.robot_euler = self.quat_to_euler(self.robot_quat)
    
    def control_loop(self):
        """Main control loop - runs at control_hz"""
        # Get latest VR state
        with self._vr_state_lock:
            vr_pose = self._latest_vr_pose.copy() if self._latest_vr_pose is not None else None
            vr_buttons = copy.deepcopy(self._latest_vr_buttons) if self._latest_vr_buttons else {}
        
        if vr_pose is None or self.robot_pos is None:
            return
        
        # Update state
        self._state["poses"][self.controller_id] = vr_pose
        self._state["buttons"] = vr_buttons
        self._state["movement_enabled"] = vr_buttons.get(self.controller_id.upper() + "G", False)
        
        # Handle calibration (preserved exactly from original)
        self._handle_calibration()
        
        # Handle recording controls
        self._handle_recording_controls()
        
        # Publish VR controller state for main_system display
        self._publish_vr_state()
        
        # Calculate and publish velocity commands if enabled
        if self._state["movement_enabled"] and self.teleoperation_enabled:
            action, action_info = self._calculate_action()
            self._last_action = action.copy()
            
            # Get gripper state directly from trigger
            trigger_key = "rightTrig" if self.controller_id == "r" else "leftTrig"
            trigger_data = self._state["buttons"].get(trigger_key, [0.0])
            
            # Handle both tuple and list formats
            if isinstance(trigger_data, (tuple, list)) and len(trigger_data) > 0:
                trigger_value = trigger_data[0]
            else:
                trigger_value = 0.0
            
            # Create velocity command
            velocity_cmd = VelocityCommand()
            velocity_cmd.header.stamp = self.get_clock().now().to_msg()
            velocity_cmd.velocities = action.tolist()  # 7D: [lin_x, lin_y, lin_z, ang_x, ang_y, ang_z, gripper]
            velocity_cmd.trigger_value = trigger_value
            
            self.velocity_command_pub.publish(velocity_cmd)
        else:
            self._last_action = np.zeros(7)
    
    def _handle_calibration(self):
        """Handle forward direction calibration - preserved exactly from original"""
        if self.controller_id not in self._state["poses"]:
            return
        
        pose_matrix = self._state["poses"][self.controller_id]
        
        # Get current button states
        current_grip = self._state["buttons"].get(self.controller_id.upper() + "G", False)
        current_joystick = self._state["buttons"].get(self.controller_id.upper() + "J", False)
        
        # Detect edge transitions
        grip_toggled = self.prev_grip_state != current_grip
        joystick_pressed = current_joystick and not self.prev_joystick_state
        joystick_released = not current_joystick and self.prev_joystick_state
        
        # Update control flags
        self.update_sensor = self.update_sensor or current_grip
        self.reset_origin = self.reset_origin or grip_toggled
        
        # Forward Direction Calibration
        if joystick_pressed:
            self.calibrating_forward = True
            self.calibration_start_pose = pose_matrix.copy()
            self.calibration_start_time = time.time()
            self.get_logger().info("🎯 Forward calibration started - Move controller in desired forward direction")
        
        elif joystick_released and self.calibrating_forward:
            self.calibrating_forward = False
            
            if self.calibration_start_pose is not None:
                # Get movement vector
                start_pos = self.calibration_start_pose[:3, 3]
                end_pos = pose_matrix[:3, 3]
                movement_vec = end_pos - start_pos
                movement_distance = np.linalg.norm(movement_vec)
                
                # Apply calibration if movement is significant (3mm threshold)
                if movement_distance > 0.003:
                    self._apply_forward_calibration(movement_vec, movement_distance)
                else:
                    self.get_logger().warn("⚠️  Insufficient movement for calibration (< 3mm)")
        
        # Origin Calibration (Grip button)
        if grip_toggled:
            if self.vr_origin is None or self.robot_origin is None:
                self.get_logger().info("🎯 Setting initial VR-Robot origin mapping")
            else:
                self.get_logger().info("🔄 Resetting VR-Robot origin mapping")
            
            # Set origins
            self.vr_origin = pose_matrix[:3, 3].copy()
            self.robot_origin = self.robot_pos.copy() if self.robot_pos is not None else np.array([0.5, 0.0, 0.5])
            self.reset_origin = False
            
            # Also store neutral orientation
            self.vr_neutral_pose = pose_matrix.copy()
            
            # Mark calibration as complete
            self.calibration.calibrated = True
            self.teleoperation_enabled = True
            
            self.get_logger().info(f"✅ Origin calibration complete")
            self.get_logger().info(f"   VR origin: {self.vr_origin}")
            self.get_logger().info(f"   Robot origin: {self.robot_origin}")
        
        # Update previous states
        self.prev_grip_state = current_grip
        self.prev_joystick_state = current_joystick
    
    def _apply_forward_calibration(self, movement_vec, movement_distance):
        """Apply forward direction calibration - preserved from original"""
        # Normalize movement vector
        forward_dir = movement_vec / movement_distance
        
        # Calculate new transformation matrix
        # The forward direction becomes the new X-axis
        new_x = forward_dir
        
        # Use world up as temporary Y
        world_up = np.array([0, 0, 1])
        
        # Calculate right vector (cross product)
        new_y = np.cross(world_up, new_x)
        new_y = new_y / np.linalg.norm(new_y)
        
        # Recalculate up vector
        new_z = np.cross(new_x, new_y)
        new_z = new_z / np.linalg.norm(new_z)
        
        # Build rotation matrix
        rotation_matrix = np.eye(3)
        rotation_matrix[:, 0] = new_x
        rotation_matrix[:, 1] = new_y
        rotation_matrix[:, 2] = new_z
        
        # Update VR to global transformation
        self.vr_to_global_mat = np.eye(4)
        self.vr_to_global_mat[:3, :3] = rotation_matrix
        
        self.get_logger().info("✅ Forward direction calibrated")
        self.get_logger().info(f"   Movement distance: {movement_distance*1000:.1f}mm")
        self.get_logger().info(f"   Forward direction: {new_x}")
        
        # Store calibration
        self.calibration.rotation_offset = R.from_matrix(rotation_matrix).as_quat()
    
    def _handle_recording_controls(self):
        """Handle recording start/stop based on VR buttons"""
        # Get current button states
        current_a = self._state["buttons"].get("A", False) or self._state["buttons"].get("X", False)
        current_b = self._state["buttons"].get("B", False) or self._state["buttons"].get("Y", False)
        
        # Detect A button press (start/stop recording)
        if current_a and not self.prev_a_button:
            self.get_logger().info("🎬 A/X button pressed - toggle recording")
            msg = Bool()
            msg.data = True
            self.start_recording_pub.publish(msg)
        
        # Detect B button press (mark success)
        if current_b and not self.prev_b_button:
            self.get_logger().info("✅ B/Y button pressed - mark recording successful")
            msg = Int32()
            msg.data = 1  # Success code
            self.stop_recording_pub.publish(msg)
        
        self.prev_a_button = current_a
        self.prev_b_button = current_b
    
    def _process_reading(self):
        """Process current readings - preserved from original"""
        # Get current pose from VR controller
        if self.controller_id not in self._state["poses"]:
            return None, None, None
        
        pose_matrix = self._state["poses"][self.controller_id]
        
        # Transform: VR -> Global -> Environment
        global_pos = self.vr_to_global_mat @ pose_matrix
        env_pos = self.global_to_env_mat @ global_pos[:3, 3]
        
        # Apply scale and offset
        if self.vr_origin is not None and self.robot_origin is not None:
            # Position relative to VR origin
            vr_relative = env_pos - self.global_to_env_mat @ self.vr_origin
            
            # Scale and offset to robot space
            robot_pos = self.robot_origin + vr_relative * self.config['vr_control']['scale_factor']
        else:
            # Fallback to direct mapping
            robot_pos = env_pos
        
        # Process rotation
        vr_quat = R.from_matrix(pose_matrix[:3, :3]).as_quat()
        
        if self.config['vr_control']['fixed_orientation']:
            # Use fixed orientation
            robot_quat = self.robot_quat if self.robot_quat is not None else np.array([0, 0, 0, 1])
        else:
            # Apply rotation scaling
            if self.vr_neutral_pose is not None:
                # Calculate rotation relative to neutral pose
                neutral_quat = R.from_matrix(self.vr_neutral_pose[:3, :3]).as_quat()
                relative_rot = self.quat_diff(vr_quat, neutral_quat)
                
                # Scale rotation
                relative_euler = self.quat_to_euler(relative_rot)
                scaled_euler = relative_euler * self.config['vr_control']['rotation_scale']
                
                # Apply to robot
                if self.robot_euler is not None:
                    robot_euler = self.add_angles(scaled_euler, self.robot_euler)
                    robot_quat = self.euler_to_quat(robot_euler)
                else:
                    robot_quat = self.euler_to_quat(scaled_euler)
            else:
                robot_quat = vr_quat
        
        # Get gripper value from trigger
        trigger_key = "rightTrig" if self.controller_id == "r" else "leftTrig"
        trigger_data = self._state["buttons"].get(trigger_key, [0.0])
        
        if isinstance(trigger_data, (tuple, list)) and len(trigger_data) > 0:
            gripper_val = trigger_data[0]
        else:
            gripper_val = 0.0
        
        return robot_pos, robot_quat, gripper_val
    
    def _calculate_action(self):
        """Calculate action from VR input - preserved from original"""
        # Process current reading
        target_pos, target_quat, gripper_val = self._process_reading()
        
        if target_pos is None or self.robot_pos is None:
            return np.zeros(7), {}
        
        # Initialize action info
        action_info = {
            "target_pos": target_pos,
            "target_quat": target_quat,
            "gripper_val": gripper_val
        }
        
        # Smooth position if enabled
        if self.config['vr_control'].get('position_smoothing_enabled', True):
            alpha = self.config['vr_control'].get('position_smoothing_alpha', 0.3)
            if self._last_vr_pos is not None:
                target_pos = alpha * target_pos + (1 - alpha) * self._last_vr_pos
            self._last_vr_pos = target_pos.copy()
        
        # Calculate position and rotation deltas
        pos_delta = target_pos - self.robot_pos
        
        # Calculate rotation delta
        rot_delta = self.quat_diff(target_quat, self.robot_quat)
        rot_delta_euler = self.quat_to_euler(rot_delta)
        
        # Apply gains
        pos_vel = pos_delta * self.config['vr_control']['translation_gain']
        rot_vel = rot_delta_euler * self.config['vr_control']['rotation_gain']
        
        # Normalize and limit velocities
        pos_vel, rot_vel, gripper_vel = self._limit_velocity(pos_vel, rot_vel, gripper_val)
        
        # Combine into action vector
        action = np.concatenate([pos_vel, rot_vel, [gripper_vel]])
        
        return action, action_info
    
    def _limit_velocity(self, lin_vel, rot_vel, gripper_vel):
        """Apply velocity limits - preserved from original"""
        # Limit linear velocity
        lin_speed = np.linalg.norm(lin_vel)
        max_lin = self.config['vr_control']['max_lin_delta']
        
        if lin_speed > max_lin:
            lin_vel = lin_vel / lin_speed * max_lin
        
        # Limit rotational velocity
        rot_speed = np.linalg.norm(rot_vel)
        max_rot = self.config['vr_control']['max_rot_delta']
        
        if rot_speed > max_rot:
            rot_vel = rot_vel / rot_speed * max_rot
        
        # Normalize to [-1, 1] range for velocity command
        lin_vel_norm = lin_vel / max_lin if max_lin > 0 else lin_vel
        rot_vel_norm = rot_vel / max_rot if max_rot > 0 else rot_vel
        
        # Gripper is already 0-1 from trigger, convert to velocity
        # >0.5 means close (positive velocity), <0.5 means open (negative velocity)
        if gripper_vel > self.config['gripper']['trigger_threshold']:
            gripper_vel_norm = 1.0  # Close
        else:
            gripper_vel_norm = -1.0  # Open
        
        return lin_vel_norm, rot_vel_norm, gripper_vel_norm
    
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
    
    def _publish_vr_state(self):
        """Publish VR controller state for system monitoring"""
        msg = VRControllerState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Button states
        msg.button_a = self._state["buttons"].get("A", False) or self._state["buttons"].get("X", False)
        msg.button_b = self._state["buttons"].get("B", False) or self._state["buttons"].get("Y", False)
        msg.grip = self._state["buttons"].get(self.controller_id.upper() + "G", False)
        msg.joystick = self._state["buttons"].get(self.controller_id.upper() + "J", False)
        
        # Trigger value
        trigger_key = "rightTrig" if self.controller_id == "r" else "leftTrig"
        trigger_data = self._state["buttons"].get(trigger_key, [0.0])
        if isinstance(trigger_data, (tuple, list)) and len(trigger_data) > 0:
            msg.trigger = trigger_data[0]
        else:
            msg.trigger = 0.0
        
        # States
        msg.teleoperation_enabled = self.teleoperation_enabled
        msg.calibration_valid = self.calibration.calibrated
        msg.movement_enabled = self._state["movement_enabled"]
        
        self.vr_control_state_pub.publish(msg)
    
    # Service callbacks
    def calibrate_callback(self, request, response):
        """Handle calibration request"""
        self.get_logger().info("VR calibration requested via service")
        
        # The actual calibration is done via button presses
        # This service just provides status
        if self.calibration.calibrated:
            response.success = True
            response.message = "VR already calibrated. Use grip button to recalibrate."
        else:
            response.success = False
            response.message = "Press grip button to calibrate origin, joystick for forward direction"
        
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
                self.get_logger().info("✅ Teleoperation enabled")
        else:
            response.success = True
            response.message = "Teleoperation disabled"
            self.get_logger().info("⏸️  Teleoperation disabled")
        
        return response
    
    def save_calibration_callback(self, request, response):
        """Save calibration to file"""
        if self.save_calibration():
            response.success = True
            response.message = f"Calibration saved to {self.calibration_file}"
        else:
            response.success = False
            response.message = "Failed to save calibration"
        
        return response
    
    def save_calibration(self):
        """Save calibration to file"""
        try:
            # Convert numpy arrays to lists for JSON serialization
            data = {
                'vr_to_global_mat': self.vr_to_global_mat.tolist(),
                'global_to_env_mat': self.global_to_env_mat.tolist(),
                'vr_origin': self.vr_origin.tolist() if self.vr_origin is not None else None,
                'robot_origin': self.robot_origin.tolist() if self.robot_origin is not None else None,
                'vr_neutral_pose': self.vr_neutral_pose.tolist() if self.vr_neutral_pose is not None else None,
                'calibrated': self.calibration.calibrated
            }
            
            with open(self.calibration_file, 'w') as f:
                json.dump(data, f, indent=2)
            
            self.get_logger().info(f"✅ Saved calibration to {self.calibration_file}")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to save calibration: {e}")
            return False


def main(args=None):
    rclpy.init(args=args)
    
    node = VRTeleopNode()
    
    # Use multi-threaded executor for concurrent operations
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