#!/usr/bin/env python3
"""
System Manager Node for VR-based Franka Control

This node orchestrates the entire system:
- Subscribes to VR input from lbx_input_oculus
- Manages Franka robot control via MoveIt IK solver
- Handles data recording (start/stop)
- Provides user interface for system state
- Manages calibration sequences

The system follows the exact VR-to-robot pipeline from oculus_vr_server_moveit.py:
1. VR motion captured and transformed
2. Motion differences converted to velocity commands using gains
3. Velocities scaled to position deltas using max_delta parameters
4. Target poses sent to MoveIt IK solver for joint computation
5. Joint trajectories executed on robot
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.action import ActionClient

import numpy as np
import time
import threading
import queue
import yaml
import os
import re
import shutil
from typing import Dict, Optional, Tuple
from dataclasses import dataclass
import copy

# ROS 2 messages
from std_msgs.msg import String, Bool
from std_srvs.srv import Empty  # Services use std_srvs.srv.Empty, not std_msgs.msg.Empty
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Joy, JointState
from moveit_msgs.srv import GetPositionIK, GetPlanningScene, GetPositionFK
from moveit_msgs.msg import PositionIKRequest, RobotState as MoveitRobotState
from control_msgs.action import FollowJointTrajectory
from franka_msgs.action import Grasp
from lbx_interfaces.msg import VRControllerState, SystemStatus, RecordingStatus

# Import transformation utilities (preserved exactly from original)
from scipy.spatial.transform import Rotation as R

# Import FrankaController from separate module
from .franka_controller import FrankaController


@dataclass
class VRState:
    """Thread-safe VR controller state"""
    timestamp: float
    poses: Dict
    buttons: Dict
    movement_enabled: bool
    controller_on: bool
    
    def copy(self):
        """Deep copy for thread safety"""
        return VRState(
            timestamp=self.timestamp,
            poses=copy.deepcopy(self.poses),
            buttons=copy.deepcopy(self.buttons),
            movement_enabled=self.movement_enabled,
            controller_on=self.controller_on
        )


@dataclass
class RobotState:
    """Thread-safe robot state"""
    timestamp: float
    pos: np.ndarray
    quat: np.ndarray
    euler: np.ndarray
    gripper: float
    joint_positions: Optional[np.ndarray]
    
    def copy(self):
        """Deep copy for thread safety"""
        return RobotState(
            timestamp=self.timestamp,
            pos=self.pos.copy() if self.pos is not None else None,
            quat=self.quat.copy() if self.quat is not None else None,
            euler=self.euler.copy() if self.euler is not None else None,
            gripper=self.gripper,
            joint_positions=self.joint_positions.copy() if self.joint_positions is not None else None
        )


class SystemManager(Node):
    """Main system manager that orchestrates VR-based Franka control"""
    
    def __init__(self, config_path: str):
        super().__init__('lbx_system_manager')
        
        # Load configuration
        self.load_config(config_path)
        
        # Initialize state
        self.system_state = 'teleop'  # Default to teleop to match original behavior
        self.recording_active = False
        self.calibration_mode = None  # None, 'forward', 'origin'
        self.running = True
        
        # Thread-safe state storage
        self._vr_state_lock = threading.Lock()
        self._robot_state_lock = threading.Lock()
        self._latest_vr_state = None
        self._latest_robot_state = None
        
        # Calibration state
        self.reset_state()
        
        # Callback groups for async execution
        self.vr_callback_group = ReentrantCallbackGroup()
        self.robot_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer_callback_group = ReentrantCallbackGroup()
        
        # Create subscribers
        self.vr_pose_sub = self.create_subscription(
            PoseStamped,
            '/vr/controller_pose',
            self.vr_pose_callback,
            10,
            callback_group=self.vr_callback_group
        )
        
        self.vr_joy_sub = self.create_subscription(
            Joy,
            '/vr/controller_buttons',
            self.vr_joy_callback,
            10,
            callback_group=self.vr_callback_group
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.robot_callback_group
        )
        
        # Create publishers
        self.system_status_pub = self.create_publisher(SystemStatus, '/system_status', 1)
        self.recording_status_pub = self.create_publisher(RecordingStatus, '/recording_status', 1)
        self.vr_control_state_pub = self.create_publisher(VRControllerState, '/vr_control_state', 10)
        
        # Create services for user commands
        self.create_service(Empty, 'start_recording', self.start_recording_callback)
        self.create_service(Empty, 'stop_recording', self.stop_recording_callback)
        self.create_service(Empty, 'calibrate_forward', self.calibrate_forward_callback)
        self.create_service(Empty, 'reset_robot', self.reset_robot_callback)
        self.create_service(Empty, 'emergency_stop', self.emergency_stop_callback)
        
        # Create Franka control interface
        self.franka_controller = FrankaController(self, self.config)
        
        # Create data recorder interface
        if self.config['recording']['enabled']:
            self.data_recorder = DataRecorderInterface(self, self.config)
        else:
            self.data_recorder = None
        
        # Status update timer
        self.status_timer = self.create_timer(
            1.0,  # 1Hz
            self.publish_system_status,
            callback_group=self.timer_callback_group
        )
        
        # Control loop timer
        self.control_timer = self.create_timer(
            1.0 / self.config['vr_control']['control_hz'],
            self.control_loop,
            callback_group=self.robot_callback_group
        )
        
        self.get_logger().info('🎮 System Manager initialized')
        self.get_logger().info(f'   Config: {config_path}')
        self.get_logger().info(f'   Control rate: {self.config["vr_control"]["control_hz"]}Hz')
        self.get_logger().info(f'   Robot command rate: 45Hz')
        self.get_logger().info(f'   Recording: {"enabled" if self.config["recording"]["enabled"] else "disabled"}')
    
    def load_config(self, config_path: str):
        """Load configuration from YAML file"""
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # Store key parameters
        self.controller_id = "r" if self.config['vr_control']['use_right_controller'] else "l"
        self.control_hz = self.config['vr_control']['control_hz']
        
        # Initialize transformation matrices (preserved exactly)
        self.global_to_env_mat = self.vec_to_reorder_mat(self.config['vr_control']['coord_transform'])
        self.vr_to_global_mat = np.eye(4)
    
    def reset_state(self):
        """Reset internal state - exactly as in original"""
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
        self.robot_gripper = 0.0
        self.robot_joint_positions = None
        
        # Calibration state
        self.prev_joystick_state = False
        self.prev_grip_state = False
        self.calibrating_forward = False
        self.calibration_start_pose = None
        self.calibration_start_time = None
        self.vr_neutral_pose = None
        
        # Control state
        self.is_first_frame = True
        self._last_vr_pos = None
        self._last_action = np.zeros(7)
        self._last_command_time = 0.0
        
        # Recording state
        self.prev_a_button = False
    
    def __del__(self):
        """Destructor to ensure proper cleanup"""
        if hasattr(self, 'running'):
            self.running = False
    
    def initialize_robot(self):
        """Initialize robot on first frame - matches original behavior"""
        if self.is_first_frame:
            self.get_logger().info("🤖 Initializing robot on first frame...")
            init_pos, init_quat, init_joint_positions = self.franka_controller.reset_robot()
            
            # Set initial robot state
            self.robot_pos = init_pos
            self.robot_quat = init_quat
            self.robot_euler = self.quat_to_euler(init_quat)
            self.robot_gripper = 0.0
            self.robot_joint_positions = init_joint_positions
            self.is_first_frame = False
            
            # Initialize robot state
            with self._robot_state_lock:
                self._latest_robot_state = RobotState(
                    timestamp=time.time(),
                    pos=init_pos,
                    quat=init_quat,
                    euler=self.robot_euler,
                    gripper=self.robot_gripper,
                    joint_positions=init_joint_positions
                )
            
            self.get_logger().info("✅ Robot initialized and ready for teleoperation")
    
    # Preserved transformation functions exactly
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
            if self._latest_vr_state is None:
                self._latest_vr_state = VRState(
                    timestamp=time.time(),
                    poses={},
                    buttons={},
                    movement_enabled=False,
                    controller_on=True
                )
            
            self._latest_vr_state.poses[self.controller_id] = pose_mat
            self._latest_vr_state.timestamp = time.time()
    
    def vr_joy_callback(self, msg: Joy):
        """Handle VR controller button updates"""
        with self._vr_state_lock:
            if self._latest_vr_state is None:
                self._latest_vr_state = VRState(
                    timestamp=time.time(),
                    poses={},
                    buttons={},
                    movement_enabled=False,
                    controller_on=True
                )
            
            # Map Joy message to button states (following oculus_reader format)
            # This mapping must match the original oculus_reader output exactly
            buttons = {}
            
            # Standard button mapping for Oculus controllers
            # The original uses uppercase single letters for button names
            if len(msg.buttons) >= 7:
                # A/X button - button[0] on right, button[3] on left
                if self.controller_id == "r":
                    buttons["A"] = bool(msg.buttons[0])
                    buttons["B"] = bool(msg.buttons[1])
                else:
                    buttons["X"] = bool(msg.buttons[0])
                    buttons["Y"] = bool(msg.buttons[1])
                
                # Controller-specific buttons with uppercase prefix
                buttons[self.controller_id.upper() + "G"] = bool(msg.buttons[4])  # Grip
                buttons[self.controller_id.upper() + "J"] = bool(msg.buttons[6])  # Joystick press
                
                # Trigger as continuous value - exactly as in original
                if len(msg.axes) >= 3:
                    trigger_key = "rightTrig" if self.controller_id == "r" else "leftTrig"
                    # Ensure it's stored as a list with one element, matching original format
                    buttons[trigger_key] = [msg.axes[2]]  # Index trigger
            
            self._latest_vr_state.buttons = buttons
            self._latest_vr_state.movement_enabled = buttons.get(self.controller_id.upper() + "G", False)
    
    def joint_state_callback(self, msg: JointState):
        """Handle robot joint state updates"""
        # Extract FR3 joint positions
        positions = []
        for joint_name in self.config['robot']['joint_names']:
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                positions.append(msg.position[idx])
        
        if len(positions) == 7:
            # Get end-effector pose through FK
            ee_pos, ee_quat = self.franka_controller.get_current_end_effector_pose(positions)
            
            if ee_pos is not None and ee_quat is not None:
                # Get gripper state
                gripper_state = self.franka_controller.get_current_gripper_state(msg)
                
                # Update robot state
                with self._robot_state_lock:
                    self._latest_robot_state = RobotState(
                        timestamp=time.time(),
                        pos=ee_pos,
                        quat=ee_quat,
                        euler=self.quat_to_euler(ee_quat),
                        gripper=gripper_state,
                        joint_positions=np.array(positions)
                    )
    
    def control_loop(self):
        """Main control loop - runs at control_hz"""
        # Get latest states
        with self._vr_state_lock:
            vr_state = self._latest_vr_state.copy() if self._latest_vr_state else None
        
        with self._robot_state_lock:
            robot_state = self._latest_robot_state.copy() if self._latest_robot_state else None
        
        if vr_state is None or robot_state is None:
            return
        
        # Process VR state for control
        self._process_vr_state(vr_state, robot_state)
        
        # Handle recording if active
        if self.recording_active and self.data_recorder:
            self.data_recorder.record_timestep(vr_state, robot_state, self._last_action)
    
    def _process_vr_state(self, vr_state: VRState, robot_state: RobotState):
        """Process VR state and generate robot commands"""
        # Update internal state from VR
        self._state["poses"] = vr_state.poses
        self._state["buttons"] = vr_state.buttons
        self._state["movement_enabled"] = vr_state.movement_enabled
        self._state["controller_on"] = vr_state.controller_on
        
        # Update robot state
        self.robot_pos = robot_state.pos
        self.robot_quat = robot_state.quat
        self.robot_euler = robot_state.euler
        self.robot_gripper = robot_state.gripper
        self.robot_joint_positions = robot_state.joint_positions
        
        # Handle calibration (preserved exactly from original)
        self._handle_calibration()
        
        # Handle recording controls
        self._handle_recording_controls()
        
        # Calculate and execute control if enabled
        if self._state["movement_enabled"] and self.system_state == 'teleop':
            action, action_info = self._calculate_action()
            self._last_action = action.copy()
            
            # Get gripper state directly from trigger - matches original implementation
            trigger_key = "rightTrig" if self.controller_id == "r" else "leftTrig"
            trigger_data = self._state["buttons"].get(trigger_key, [0.0])
            
            # Handle both tuple and list formats
            if isinstance(trigger_data, (tuple, list)) and len(trigger_data) > 0:
                trigger_value = trigger_data[0]
            else:
                trigger_value = 0.0
            
            # Pass the raw trigger value in action_info for direct gripper control
            action_info['trigger_value'] = trigger_value
            
            # Send command to robot
            self.franka_controller.execute_command(action, action_info, robot_state)
        else:
            self._last_action = np.zeros(7)
            # Reset gripper tracking when movement is disabled - matches original
            if hasattr(self.franka_controller, '_last_gripper_command'):
                self.franka_controller._last_gripper_command = None
    
    def _handle_calibration(self):
        """Handle forward direction calibration - preserved exactly"""
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
            self.system_state = 'calibrating'
            self.calibration_mode = 'forward'
            self.get_logger().info("🎯 Forward calibration started - Move controller in desired forward direction")
        
        elif joystick_released and self.calibrating_forward:
            self.calibrating_forward = False
            
            if self.calibration_start_pose is not None:
                # Get movement vector
                start_pos = self.calibration_start_pose[:3, 3]
                end_pos = pose_matrix[:3, 3]
                movement_vec = end_pos - start_pos
                movement_distance = np.linalg.norm(movement_vec)
                
                if movement_distance > self.config['calibration']['movement_threshold']:
                    # Apply calibration (preserved exactly)
                    self._apply_forward_calibration(movement_vec, movement_distance)
                    self.system_state = 'teleop'
                    self.calibration_mode = None
                else:
                    self.get_logger().warn(f"Not enough movement detected ({movement_distance*1000:.1f}mm)")
                    self.reset_orientation = True
                    self.system_state = 'teleop'  # Stay in teleop mode, matching original
                    self.calibration_mode = None
        
        # Update previous button states
        self.prev_grip_state = current_grip
        self.prev_joystick_state = current_joystick
    
    def _apply_forward_calibration(self, movement_vec, movement_distance):
        """Apply forward direction calibration - preserved exactly"""
        # Normalize movement vector
        forward_vec = movement_vec / movement_distance
        
        self.get_logger().info(f"✅ Forward direction calibrated!")
        self.get_logger().info(f"   Movement distance: {movement_distance*1000:.1f}mm")
        self.get_logger().info(f"   Forward vector: [{forward_vec[0]:.3f}, {forward_vec[1]:.3f}, {forward_vec[2]:.3f}]")
        
        # Create rotation to align this vector with robot's forward
        temp_mat = np.eye(4)
        temp_mat[:3, 3] = forward_vec
        transformed_temp = self.global_to_env_mat @ temp_mat
        transformed_forward = transformed_temp[:3, 3]
        
        # Calculate rotation to align with robot's +X axis
        robot_forward = np.array([1.0, 0.0, 0.0])
        rotation_axis = np.cross(transformed_forward, robot_forward)
        rotation_angle = np.arccos(np.clip(np.dot(transformed_forward, robot_forward), -1.0, 1.0))
        
        if np.linalg.norm(rotation_axis) > 0.001:
            rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
            # Create rotation matrix using Rodrigues' formula
            K = np.array([[0, -rotation_axis[2], rotation_axis[1]],
                         [rotation_axis[2], 0, -rotation_axis[0]],
                         [-rotation_axis[1], rotation_axis[0], 0]])
            R_calibration = np.eye(3) + np.sin(rotation_angle) * K + (1 - np.cos(rotation_angle)) * K @ K
        else:
            # Movement is already aligned with robot forward or backward
            if transformed_forward[0] < 0:  # Moving backward
                R_calibration = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
            else:
                R_calibration = np.eye(3)
        
        # Update the VR to global transformation
        self.vr_to_global_mat = np.eye(4)
        self.vr_to_global_mat[:3, :3] = R_calibration
        
        try:
            self.vr_to_global_mat = np.linalg.inv(self.calibration_start_pose) @ self.vr_to_global_mat
        except:
            self.get_logger().warn("Could not invert calibration pose")
        
        self.reset_orientation = False
        self.vr_neutral_pose = np.asarray(self._state["poses"][self.controller_id]).copy()
        
        # Reset robot after calibration
        self.franka_controller.reset_robot()
    
    def _handle_recording_controls(self):
        """Handle recording start/stop via VR buttons"""
        if not self.config['recording']['enabled'] or not self.data_recorder:
            return
        
        # A button toggles recording
        current_a_button = self._state["buttons"].get("A" if self.controller_id == 'r' else "X", False)
        
        if current_a_button and not self.prev_a_button:
            if self.recording_active:
                self.get_logger().info("🛑 Stopping recording...")
                self.data_recorder.stop_recording(success=False)
                self.recording_active = False
            else:
                self.get_logger().info("▶️  Starting recording...")
                self.data_recorder.start_recording()
                self.recording_active = True
        
        self.prev_a_button = current_a_button
        
        # B button saves recording as successful
        if self._state["buttons"].get("B" if self.controller_id == 'r' else "Y", False) and self.recording_active:
            self.get_logger().info("✅ Marking recording as successful...")
            self.data_recorder.stop_recording(success=True)
            self.recording_active = False
    
    def _process_reading(self):
        """Apply coordinate transformations to VR controller pose - preserved exactly"""
        rot_mat = np.asarray(self._state["poses"][self.controller_id])
        
        # Apply position transformation
        transformed_mat = self.global_to_env_mat @ self.vr_to_global_mat @ rot_mat
        vr_pos = self.config['vr_control']['spatial_coeff'] * transformed_mat[:3, 3]
        
        # Apply position filtering to reduce noise/drift
        if self.config['vr_control']['use_position_filter'] and self._last_vr_pos is not None:
            pos_delta = vr_pos - self._last_vr_pos
            
            # Apply deadzone
            for i in range(3):
                if abs(pos_delta[i]) < self.config['vr_control']['translation_deadzone']:
                    pos_delta[i] = 0.0
            
            vr_pos = self._last_vr_pos + pos_delta
        
        self._last_vr_pos = vr_pos.copy()
        
        # Handle rotation - preserved exactly
        if hasattr(self, 'vr_neutral_pose') and self.vr_neutral_pose is not None:
            neutral_rot = R.from_matrix(self.vr_neutral_pose[:3, :3])
            current_rot = R.from_matrix(rot_mat[:3, :3])
            
            relative_rot = neutral_rot.inv() * current_rot
            rotvec = relative_rot.as_rotvec()
            angle = np.linalg.norm(rotvec)
            
            if angle > 0:
                axis = rotvec / angle
                transformed_axis = np.array([-axis[1], axis[0], axis[2]])
                transformed_rotvec = transformed_axis * angle
                transformed_rot = R.from_rotvec(transformed_rotvec)
                vr_quat = transformed_rot.as_quat()
            else:
                vr_quat = np.array([0, 0, 0, 1])
        else:
            transformed_rot_mat = self.global_to_env_mat[:3, :3] @ self.vr_to_global_mat[:3, :3] @ rot_mat[:3, :3]
            vr_quat = R.from_matrix(transformed_rot_mat).as_quat()
        
        # Get gripper state from trigger
        trigger_key = "rightTrig" if self.controller_id == "r" else "leftTrig"
        trigger_data = self._state["buttons"].get(trigger_key, [0.0])
        
        if isinstance(trigger_data, (tuple, list)) and len(trigger_data) > 0:
            vr_gripper = trigger_data[0]
        else:
            vr_gripper = 0.0
        
        self.vr_state = {"pos": vr_pos, "quat": vr_quat, "gripper": vr_gripper}
    
    def _calculate_action(self):
        """Calculate robot action from VR controller state - preserved exactly"""
        if self.update_sensor:
            self._process_reading()
            self.update_sensor = False
        
        if self.vr_state is None or self.robot_pos is None:
            return np.zeros(7), {}
        
        # Reset Origin On Release
        if self.reset_origin:
            self.robot_origin = {"pos": self.robot_pos, "quat": self.robot_quat}
            self.vr_origin = {"pos": self.vr_state["pos"], "quat": self.vr_state["quat"]}
            self.reset_origin = False
            self.get_logger().info("📍 Origin calibrated")
        
        # Calculate Positional Action - DROID exact
        robot_pos_offset = self.robot_pos - self.robot_origin["pos"]
        target_pos_offset = self.vr_state["pos"] - self.vr_origin["pos"]
        pos_action = target_pos_offset - robot_pos_offset
        
        # Calculate Rotation Action
        vr_relative_rot = R.from_quat(self.vr_origin["quat"]).inv() * R.from_quat(self.vr_state["quat"])
        target_rot = R.from_quat(self.robot_origin["quat"]) * vr_relative_rot
        target_quat = target_rot.as_quat()
        
        robot_quat_offset = self.quat_diff(self.robot_quat, self.robot_origin["quat"])
        target_quat_offset = self.quat_diff(self.vr_state["quat"], self.vr_origin["quat"])
        quat_action = self.quat_diff(target_quat_offset, robot_quat_offset)
        euler_action = self.quat_to_euler(quat_action)
        
        # Calculate Gripper Action
        gripper_action = (self.vr_state["gripper"] * 1.5) - self.robot_gripper
        
        # Calculate Desired Pose
        target_pos = pos_action + self.robot_pos
        target_euler = self.add_angles(euler_action, self.robot_euler)
        target_cartesian = np.concatenate([target_pos, target_euler])
        target_gripper = self.vr_state["gripper"]
        
        # Scale Appropriately
        pos_action *= self.config['vr_control']['pos_action_gain']
        euler_action *= self.config['vr_control']['rot_action_gain']
        gripper_action *= self.config['vr_control']['gripper_action_gain']
        
        # Apply velocity limits
        lin_vel, rot_vel, gripper_vel = self._limit_velocity(pos_action, euler_action, gripper_action)
        
        # Prepare Return Values
        info_dict = {
            "target_cartesian_position": target_cartesian,
            "target_gripper_position": target_gripper,
            "target_quaternion": target_quat
        }
        action = np.concatenate([lin_vel, rot_vel, [gripper_vel]])
        action = action.clip(-1, 1)
        
        return action, info_dict
    
    def _limit_velocity(self, lin_vel, rot_vel, gripper_vel):
        """Scales down the linear and angular magnitudes of the action - preserved exactly"""
        lin_vel_norm = np.linalg.norm(lin_vel)
        rot_vel_norm = np.linalg.norm(rot_vel)
        gripper_vel_norm = np.linalg.norm(gripper_vel)
        
        max_lin_vel = self.config['vr_control']['max_lin_vel']
        max_rot_vel = self.config['vr_control']['max_rot_vel']
        max_gripper_vel = self.config['vr_control']['max_gripper_vel']
        
        if lin_vel_norm > max_lin_vel:
            lin_vel = lin_vel * max_lin_vel / lin_vel_norm
        if rot_vel_norm > max_rot_vel:
            rot_vel = rot_vel * max_rot_vel / rot_vel_norm
        if gripper_vel_norm > max_gripper_vel:
            gripper_vel = gripper_vel * max_gripper_vel / gripper_vel_norm
        
        return lin_vel, rot_vel, gripper_vel
    
    def publish_system_status(self):
        """Publish system status at 1Hz"""
        status = SystemStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.system_state = self.system_state
        status.recording_active = self.recording_active
        status.calibration_mode = self.calibration_mode if self.calibration_mode else ""
        status.teleoperation_enabled = self._state.get("movement_enabled", False)
        status.controller_connected = self._state.get("controller_on", False)
        
        # Add VR calibration matrix if available
        if hasattr(self, 'vr_to_global_mat') and self.vr_to_global_mat is not None:
            # Flatten 4x4 matrix to 16-element array
            status.vr_calibration_matrix = self.vr_to_global_mat.flatten().tolist()
            status.vr_calibration_valid = True
        else:
            status.vr_calibration_matrix = [0.0] * 16  # Identity matrix
            status.vr_calibration_valid = False
        
        self.system_status_pub.publish(status)
        
        # Also publish VR control state
        if self._latest_vr_state and self.vr_state:
            vr_ctrl_state = VRControllerState()
            vr_ctrl_state.header.stamp = self.get_clock().now().to_msg()
            vr_ctrl_state.controller_id = self.controller_id
            vr_ctrl_state.grip_pressed = self._state.get("movement_enabled", False)
            vr_ctrl_state.trigger_value = self.vr_state.get("gripper", 0.0)
            vr_ctrl_state.a_button = self._state["buttons"].get("A" if self.controller_id == 'r' else "X", False)
            vr_ctrl_state.b_button = self._state["buttons"].get("B" if self.controller_id == 'r' else "Y", False)
            vr_ctrl_state.joystick_pressed = self._state["buttons"].get(self.controller_id.upper() + "J", False)
            
            self.vr_control_state_pub.publish(vr_ctrl_state)
    
    # Service callbacks
    def start_recording_callback(self, request, response):
        """Handle start recording request"""
        if self.config['recording']['enabled'] and self.data_recorder:
            if not self.recording_active:
                self.get_logger().info("▶️  Starting recording via service...")
                self.data_recorder.start_recording()
                self.recording_active = True
        return response
    
    def stop_recording_callback(self, request, response):
        """Handle stop recording request"""
        if self.recording_active and self.data_recorder:
            self.get_logger().info("🛑 Stopping recording via service...")
            self.data_recorder.stop_recording(success=True)
            self.recording_active = False
        return response
    
    def calibrate_forward_callback(self, request, response):
        """Handle forward calibration request"""
        self.get_logger().info("🎯 Forward calibration requested via service")
        # Calibration will be triggered by joystick button press
        return response
    
    def reset_robot_callback(self, request, response):
        """Handle robot reset request"""
        self.get_logger().info("🔄 Robot reset requested via service")
        self.franka_controller.reset_robot()
        self.reset_origin = True
        return response
    
    def emergency_stop_callback(self, request, response):
        """Handle emergency stop request"""
        self.get_logger().error("🛑 EMERGENCY STOP requested!")
        self.system_state = 'emergency_stop'
        self.franka_controller.emergency_stop()
        return response


class DataRecorderInterface:
    """Interface to data recording system"""
    
    def __init__(self, node: Node, config: Dict):
        self.node = node
        self.config = config
        
        # Import here to avoid circular dependency
        from std_srvs.srv import Trigger
        
        # Create service clients to lbx_data_recorder
        self.start_recording_client = node.create_client(Trigger, '/start_recording')
        self.stop_recording_client = node.create_client(Trigger, '/stop_recording')
        
        # Wait for services
        self.services_ready = False
        self._check_services()
    
    def _check_services(self):
        """Check if recording services are available"""
        start_ready = self.start_recording_client.wait_for_service(timeout_sec=2.0)
        stop_ready = self.stop_recording_client.wait_for_service(timeout_sec=2.0)
        
        if start_ready and stop_ready:
            self.services_ready = True
            self.node.get_logger().info("✅ Data recording services ready")
        else:
            self.node.get_logger().warn("⚠️  Data recording services not available")
    
    def start_recording(self):
        """Start data recording"""
        if not self.services_ready:
            self._check_services()
            
        if self.services_ready:
            from std_srvs.srv import Trigger
            request = Trigger.Request()
            future = self.start_recording_client.call_async(request)
            # Don't wait for response to maintain async operation
            future.add_done_callback(self._start_recording_done)
    
    def _start_recording_done(self, future):
        """Handle start recording response"""
        try:
            response = future.result()
            if response.success:
                self.node.get_logger().info(f"✅ {response.message}")
            else:
                self.node.get_logger().error(f"❌ {response.message}")
        except Exception as e:
            self.node.get_logger().error(f"Failed to start recording: {e}")
    
    def stop_recording(self, success: bool):
        """Stop data recording"""
        if not self.services_ready:
            self._check_services()
            
        if self.services_ready:
            from std_srvs.srv import Trigger
            request = Trigger.Request()
            future = self.stop_recording_client.call_async(request)
            # Add success marker in done callback
            future.add_done_callback(
                lambda f: self._stop_recording_done(f, success)
            )
    
    def _stop_recording_done(self, future, success: bool):
        """Handle stop recording response"""
        try:
            response = future.result()
            if response.success:
                self.node.get_logger().info(f"✅ {response.message}")
                if success:
                    # Move to success folder
                    self._mark_recording_successful(response.message)
            else:
                self.node.get_logger().error(f"❌ {response.message}")
        except Exception as e:
            self.node.get_logger().error(f"Failed to stop recording: {e}")
    
    def _mark_recording_successful(self, message: str):
        """Mark recording as successful by moving to success folder"""
        # Extract filename from message
        import re
        match = re.search(r'teleoperation_\d+_\d+\.mcap', message)
        if match:
            filename = match.group(0)
            src_path = os.path.join(os.path.expanduser('~/lbx_recordings'), filename)
            dst_dir = os.path.join(os.path.expanduser('~/lbx_recordings'), 'success')
            os.makedirs(dst_dir, exist_ok=True)
            dst_path = os.path.join(dst_dir, filename)
            
            try:
                import shutil
                shutil.move(src_path, dst_path)
                self.node.get_logger().info(f"✅ Recording moved to success folder")
            except Exception as e:
                self.node.get_logger().error(f"Failed to move recording: {e}")
    
    def record_timestep(self, vr_state, robot_state, action):
        """Record a single timestep - no longer needed with topic-based recording"""
        # The data recorder subscribes directly to topics
        pass


def main(args=None):
    rclpy.init(args=args)
    
    # Get config path from parameter or default
    config_path = os.path.join(
        os.path.dirname(__file__),
        '../../../configs/control/franka_vr_control_config.yaml'
    )
    
    # Create node
    system_manager = SystemManager(config_path)
    
    # Initialize robot on startup - matches original behavior
    system_manager.initialize_robot()
    
    # Use multi-threaded executor for async operation
    executor = MultiThreadedExecutor()
    executor.add_node(system_manager)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        system_manager.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        system_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 