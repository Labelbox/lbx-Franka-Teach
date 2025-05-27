#!/usr/bin/env python3
"""
Oculus VR Server - Implements VRPolicy-style teleoperation control
Based on droid/controllers/oculus_controller.py

Features:
- Velocity-based control with DROID-exact parameters
- Intuitive forward direction calibration (hold joystick + move forward)
- Origin calibration on grip press/release
- Coordinate transformation pipeline matching DROID
- Success/failure buttons (A/B or X/Y)
- 50Hz VR polling with internal state thread
- Safety limiting and workspace bounds
- Deoxys-compatible quaternion handling
- FR3 robot simulation mode
"""

import zmq
import time
import threading
import numpy as np
import signal
import sys
import argparse
import pickle
from scipy.spatial.transform import Rotation as R
from typing import Dict, Optional, Tuple
import os

# Import the Oculus Reader
from oculus_reader.reader import OculusReader

# Import robot control components
from frankateach.network import create_request_socket
from frankateach.constants import (
    HOST, CONTROL_PORT,
    GRIPPER_OPEN, GRIPPER_CLOSE,
    ROBOT_WORKSPACE_MIN, ROBOT_WORKSPACE_MAX,
)
from frankateach.messages import FrankaAction, FrankaState
from deoxys.utils import transform_utils

# Import simulation components
from simulation.fr3_sim_server import FR3SimServer


def vec_to_reorder_mat(vec):
    """Convert reordering vector to transformation matrix"""
    X = np.zeros((len(vec), len(vec)))
    for i in range(X.shape[0]):
        ind = int(abs(vec[i])) - 1
        X[i, ind] = np.sign(vec[i])
    return X


def rmat_to_quat(rot_mat):
    """Convert rotation matrix to quaternion (x,y,z,w)"""
    rotation = R.from_matrix(rot_mat)
    return rotation.as_quat()


def quat_to_rmat(quat):
    """Convert quaternion (x,y,z,w) to rotation matrix"""
    return R.from_quat(quat).as_matrix()


def quat_diff(target, source):
    """Calculate quaternion difference"""
    result = R.from_quat(target) * R.from_quat(source).inv()
    return result.as_quat()


def quat_to_euler(quat, degrees=False):
    """Convert quaternion to euler angles"""
    euler = R.from_quat(quat).as_euler("xyz", degrees=degrees)
    return euler


def euler_to_quat(euler, degrees=False):
    """Convert euler angles to quaternion"""
    return R.from_euler("xyz", euler, degrees=degrees).as_quat()


def add_angles(delta, source, degrees=False):
    """Add two sets of euler angles"""
    delta_rot = R.from_euler("xyz", delta, degrees=degrees)
    source_rot = R.from_euler("xyz", source, degrees=degrees)
    new_rot = delta_rot * source_rot
    return new_rot.as_euler("xyz", degrees=degrees)


class OculusVRServer:
    def __init__(self, 
                 debug=False, 
                 right_controller=True, 
                 ip_address=None,
                 simulation=False,
                 coord_transform=None):
        """
        Initialize the Oculus VR Server with DROID-exact VRPolicy control
        
        Args:
            debug: If True, only print data without controlling robot
            right_controller: If True, use right controller for robot control
            ip_address: IP address of Quest device (None for USB connection)
            simulation: If True, use simulated FR3 robot instead of real hardware
            coord_transform: Custom coordinate transformation vector (default: DROID's [-2, -1, -3, 4])
        """
        self.debug = debug
        self.right_controller = right_controller
        self.simulation = simulation
        self.running = True
        
        # DROID VRPolicy exact parameters - no customization
        self.max_lin_vel = 1.0
        self.max_rot_vel = 1.0
        self.max_gripper_vel = 1.0
        self.spatial_coeff = 1.0
        self.pos_action_gain = 5.0
        self.rot_action_gain = 2.0
        self.gripper_action_gain = 3.0
        self.control_hz = 15
        self.control_interval = 1.0 / self.control_hz
        
        # Coordinate transformation
        # Default: DROID exact from oculus_controller.py: rmat_reorder: list = [-2, -1, -3, 4]
        # But this might not be correct for all robot setups
        if coord_transform is None:
            # Try a more standard transformation that should work for most robots
            # This maps: VR +Z (forward) → Robot +X (forward)
            #           VR +X (right) → Robot -Y (right) 
            #           VR +Y (up) → Robot +Z (up)
            rmat_reorder = [-3, -1, 2, 4]  # More intuitive default
            print("\n⚠️  Using adjusted coordinate transformation for better compatibility")
            print("   If rotation is still incorrect, try --coord-transform with different values")
        else:
            rmat_reorder = coord_transform
            
        self.global_to_env_mat = vec_to_reorder_mat(rmat_reorder)
        
        if self.debug or coord_transform is not None:
            print("\n🔍 Coordinate Transformation:")
            print(f"   Reorder vector: {rmat_reorder}")
            print("   Transformation Matrix:")
            for i in range(4):
                row = self.global_to_env_mat[i]
                print(f"   [{row[0]:6.1f}, {row[1]:6.1f}, {row[2]:6.1f}, {row[3]:6.1f}]")
            
            # Show what this transformation does
            test_vecs = [
                ([1, 0, 0, 1], "VR right (+X)"),
                ([0, 1, 0, 1], "VR up (+Y)"),
                ([0, 0, 1, 1], "VR forward (+Z)"),
            ]
            print("\n   Mapping:")
            for vec, name in test_vecs:
                transformed = self.global_to_env_mat @ np.array(vec)
                direction = ""
                if abs(transformed[0]) > 0.5:
                    direction = f"Robot {'forward' if transformed[0] > 0 else 'backward'} (X)"
                elif abs(transformed[1]) > 0.5:
                    direction = f"Robot {'left' if transformed[1] > 0 else 'right'} (Y)"
                elif abs(transformed[2]) > 0.5:
                    direction = f"Robot {'up' if transformed[2] > 0 else 'down'} (Z)"
                print(f"   {name} → {direction}")
        
        # Initialize transformation matrices
        self.vr_to_global_mat = np.eye(4)
        
        # Controller ID
        self.controller_id = "r" if right_controller else "l"
        
        # Initialize state
        self.reset_state()
        
        # Initialize Oculus Reader
        print("🎮 Initializing Oculus Reader...")
        try:
            self.oculus_reader = OculusReader(
                ip_address=ip_address,
                print_FPS=False
            )
            print("✅ Oculus Reader initialized successfully")
        except Exception as e:
            print(f"❌ Failed to initialize Oculus Reader: {e}")
            sys.exit(1)
        
        # Simulation server (if in simulation mode)
        self.sim_server = None
        
        # Robot control components
        if self.simulation:
            print("🤖 Starting FR3 simulation server...")
            self.sim_server = FR3SimServer(visualize=True)
            self.sim_server.start()
            time.sleep(1.0)  # Give server time to start
            print("✅ Simulation server started")
        
        if not self.debug:
            print("🤖 Connecting to robot...")
            try:
                # Create robot control socket
                self.action_socket = create_request_socket(HOST, CONTROL_PORT)
                print("✅ Connected to robot server")
                
                # Create ZMQ context and publisher
                self.context = zmq.Context()
                self.controller_publisher = self.context.socket(zmq.PUB)
                self.controller_publisher.bind("tcp://192.168.1.54:5555")
                print("📡 Controller state publisher bound to tcp://192.168.1.54:5555")
            except Exception as e:
                print(f"❌ Failed to connect to robot: {e}")
                sys.exit(1)
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # Start state listening thread
        self._state_thread = threading.Thread(target=self._update_internal_state)
        self._state_thread.daemon = True
        self._state_thread.start()
        
        print("\n🎮 Oculus VR Server with DROID-exact VRPolicy Control")
        print(f"   Using {'RIGHT' if right_controller else 'LEFT'} controller")
        print(f"   Mode: {'DEBUG' if debug else 'LIVE ROBOT CONTROL'}")
        print(f"   Robot: {'SIMULATED FR3' if simulation else 'REAL HARDWARE'}")
        print(f"   Control frequency: {self.control_hz}Hz")
        print(f"   Position gain: {self.pos_action_gain}")
        print(f"   Rotation gain: {self.rot_action_gain}")
        print(f"   Gripper gain: {self.gripper_action_gain}")
        
        print("\n📋 Controls:")
        print("   - HOLD grip button: Enable teleoperation")
        print("   - RELEASE grip button: Pause teleoperation")
        print("   - PRESS trigger: Close gripper")
        print("   - RELEASE trigger: Open gripper")
        print("   - A/X button: Mark success and exit")
        print("   - B/Y button: Mark failure and exit")
        
        print("\n🧭 Forward Direction Calibration:")
        print("   - HOLD joystick button and MOVE controller forward")
        print("   - The direction you move defines 'forward' for the robot")
        print("   - Move at least 3mm in your desired forward direction")
        print("   - Release joystick button to complete calibration")
        
        print("\n💡 Tips:")
        print("   - Calibrate forward direction before starting teleoperation")
        print("   - Each grip press/release recalibrates the origin (starting position)")
        print("   - Robot movements are relative to the position when grip was pressed")
        
        if self.simulation:
            print("\n🎮 SIMULATION MODE:")
            print("   - 3D visualization window shows robot state")
            print("   - Green dashed line shows end-effector trajectory")
            print("   - Gray box shows robot workspace limits")
        
        print("\n⚠️  Note: Using Deoxys control interface (quaternion-based)")
        print("   Rotations are handled differently than DROID's Polymetis interface")
        
        print("\nPress Ctrl+C to exit gracefully\n")
        
    def reset_state(self):
        """Reset internal state - matches DROID exactly"""
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
        
        # Robot state - Deoxys uses quaternions directly
        self.robot_pos = None
        self.robot_quat = None
        self.robot_euler = None
        self.robot_gripper = 0.0
        
        # Button state tracking for edge detection
        self.prev_joystick_state = False
        self.prev_grip_state = False
        
        # Calibration state
        self.calibrating_forward = False
        self.calibration_start_pose = None
        self.calibration_start_time = None
        
        # First frame flag
        self.is_first_frame = True
        
        # Flag to reset robot after calibration
        self._reset_robot_after_calibration = False
        
    def signal_handler(self, signum, frame):
        """Handle Ctrl+C and other termination signals"""
        print(f"\n🛑 Received signal {signum}, shutting down gracefully...")
        self.stop_server()
        
    def _update_internal_state(self, num_wait_sec=5, hz=50):
        """Continuously poll VR controller state at 50Hz - with intuitive calibration"""
        last_read_time = time.time()
        
        while self.running:
            # Regulate Read Frequency
            time.sleep(1 / hz)
            
            # Read Controller
            time_since_read = time.time() - last_read_time
            poses, buttons = self.oculus_reader.get_transformations_and_buttons()
            self._state["controller_on"] = time_since_read < num_wait_sec
            
            if poses == {}:
                continue
            
            # Get current button states
            current_grip = buttons.get(self.controller_id.upper() + "G", False)
            current_joystick = buttons.get(self.controller_id.upper() + "J", False)
            
            # Detect edge transitions
            grip_toggled = self.prev_grip_state != current_grip
            joystick_pressed = current_joystick and not self.prev_joystick_state  # Rising edge
            joystick_released = not current_joystick and self.prev_joystick_state  # Falling edge
            
            # Update control flags
            self.update_sensor = self.update_sensor or current_grip
            self.reset_origin = self.reset_origin or grip_toggled
            
            # Save Info
            self._state["poses"] = poses
            self._state["buttons"] = buttons
            self._state["movement_enabled"] = current_grip
            self._state["controller_on"] = True
            last_read_time = time.time()
            
            # Handle Forward Direction Calibration
            if self.controller_id in self._state["poses"]:
                pose_matrix = self._state["poses"][self.controller_id]
                
                # Start calibration when joystick is pressed
                if joystick_pressed:
                    self.calibrating_forward = True
                    self.calibration_start_pose = pose_matrix.copy()
                    self.calibration_start_time = time.time()
                    print(f"\n🎯 Forward calibration started - Move controller in desired forward direction")
                    print(f"   Hold the joystick and move at least 3mm forward")
                
                # Complete calibration when joystick is released
                elif joystick_released and self.calibrating_forward:
                    self.calibrating_forward = False
                    
                    if self.calibration_start_pose is not None:
                        # Get movement vector
                        start_pos = self.calibration_start_pose[:3, 3]
                        end_pos = pose_matrix[:3, 3]
                        movement_vec = end_pos - start_pos
                        movement_distance = np.linalg.norm(movement_vec)
                        
                        if movement_distance > 0.003:  # 3mm threshold
                            # Normalize movement vector
                            forward_vec = movement_vec / movement_distance
                            
                            print(f"\n✅ Forward direction calibrated!")
                            print(f"   Movement distance: {movement_distance*1000:.1f}mm")
                            print(f"   Forward vector: [{forward_vec[0]:.3f}, {forward_vec[1]:.3f}, {forward_vec[2]:.3f}]")
                            
                            # Create rotation to align this vector with robot's forward
                            # First, apply the coordinate transform to see what this maps to
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
                            
                            # Also incorporate the starting orientation to match DROID behavior
                            # This makes the current controller orientation the "neutral" orientation
                            try:
                                self.vr_to_global_mat = np.linalg.inv(self.calibration_start_pose) @ self.vr_to_global_mat
                            except:
                                print("Warning: Could not invert calibration pose")
                            
                            self.reset_orientation = False  # Calibration complete
                            
                            # Reset robot to home position after calibration
                            if not self.debug and not self.reset_orientation:
                                print("🏠 Moving robot to reset position after calibration...")
                                self._reset_robot_after_calibration = True
                            elif self.debug:
                                print("\n✅ Calibration complete! Ready for teleoperation.")
                                print("   Hold grip button to start controlling the robot")
                        else:
                            print(f"\n⚠️  Not enough movement detected ({movement_distance*1000:.1f}mm)")
                            print(f"   Please move controller at least 3mm in your desired forward direction")
                            # Keep reset_orientation True to allow DROID-style calibration as fallback
                            self.reset_orientation = True
                
                # Show calibration progress
                elif self.calibrating_forward and current_joystick:
                    if time.time() - self.calibration_start_time > 0.5:  # Update every 0.5s
                        current_pos = pose_matrix[:3, 3]
                        start_pos = self.calibration_start_pose[:3, 3]
                        distance = np.linalg.norm(current_pos - start_pos) * 1000  # Convert to mm
                        print(f"   Current movement: {distance:.1f}mm", end='\r')
            
            # DROID-style calibration fallback (just pressing joystick without movement)
            if self.reset_orientation and not self.calibrating_forward:
                stop_updating = self._state["buttons"][self.controller_id.upper() + "J"] or self._state["movement_enabled"]
                if stop_updating:
                    rot_mat = np.asarray(self._state["poses"][self.controller_id])
                    self.reset_orientation = False
                    # try to invert the rotation matrix, if not possible, then just use the identity matrix
                    try:
                        rot_mat = np.linalg.inv(rot_mat)
                    except:
                        print(f"exception for rot mat: {rot_mat}")
                        rot_mat = np.eye(4)
                        self.reset_orientation = True
                    self.vr_to_global_mat = rot_mat
                    print("📐 Orientation reset (DROID-style)")
                    
                    # Reset robot to home position after calibration
                    if not self.debug and not self.reset_orientation:
                        print("🏠 Moving robot to reset position after calibration...")
                        self._reset_robot_after_calibration = True
            
            # Update previous button states for next iteration
            self.prev_grip_state = current_grip
            self.prev_joystick_state = current_joystick
    
    def _process_reading(self):
        """Apply coordinate transformations to VR controller pose - DROID exact"""
        rot_mat = np.asarray(self._state["poses"][self.controller_id])
        rot_mat = self.global_to_env_mat @ self.vr_to_global_mat @ rot_mat
        vr_pos = self.spatial_coeff * rot_mat[:3, 3]
        vr_quat = rmat_to_quat(rot_mat[:3, :3])
        vr_gripper = self._state["buttons"]["rightTrig" if self.controller_id == "r" else "leftTrig"][0]
        
        self.vr_state = {"pos": vr_pos, "quat": vr_quat, "gripper": vr_gripper}
    
    def _limit_velocity(self, lin_vel, rot_vel, gripper_vel):
        """Scales down the linear and angular magnitudes of the action - DROID exact"""
        lin_vel_norm = np.linalg.norm(lin_vel)
        rot_vel_norm = np.linalg.norm(rot_vel)
        gripper_vel_norm = np.linalg.norm(gripper_vel)
        
        if lin_vel_norm > self.max_lin_vel:
            lin_vel = lin_vel * self.max_lin_vel / lin_vel_norm
        if rot_vel_norm > self.max_rot_vel:
            rot_vel = rot_vel * self.max_rot_vel / rot_vel_norm
        if gripper_vel_norm > self.max_gripper_vel:
            gripper_vel = gripper_vel * self.max_gripper_vel / gripper_vel_norm
        
        return lin_vel, rot_vel, gripper_vel
    
    def _calculate_action(self):
        """Calculate robot action from VR controller state - DROID exact implementation"""
        # Read Sensor
        if self.update_sensor:
            self._process_reading()
            self.update_sensor = False
        
        # Check if we have valid data
        if self.vr_state is None or self.robot_pos is None:
            return np.zeros(7), {}
        
        # Reset Origin On Release
        if self.reset_origin:
            self.robot_origin = {"pos": self.robot_pos, "quat": self.robot_quat}
            self.vr_origin = {"pos": self.vr_state["pos"], "quat": self.vr_state["quat"]}
            self.reset_origin = False
            print("📍 Origin calibrated")
            if self.debug:
                print(f"   Robot origin: pos=[{self.robot_origin['pos'][0]:.3f}, {self.robot_origin['pos'][1]:.3f}, {self.robot_origin['pos'][2]:.3f}]")
                euler = quat_to_euler(self.robot_origin['quat'], degrees=True)
                print(f"   Robot origin rotation: [R:{euler[0]:6.1f}, P:{euler[1]:6.1f}, Y:{euler[2]:6.1f}]")
        
        # Calculate Positional Action - DROID exact
        robot_pos_offset = self.robot_pos - self.robot_origin["pos"]
        target_pos_offset = self.vr_state["pos"] - self.vr_origin["pos"]
        pos_action = target_pos_offset - robot_pos_offset
        
        # Calculate Rotation Action - CRITICAL FIX FOR DEOXYS
        # DROID calculates euler velocities because Polymetis expects euler angles
        # But Deoxys expects quaternions directly, so we need a different approach
        
        # Calculate the target quaternion directly
        # VR controller's relative rotation from its origin
        vr_relative_rot = R.from_quat(self.vr_origin["quat"]).inv() * R.from_quat(self.vr_state["quat"])
        # Apply this relative rotation to the robot's origin orientation
        target_rot = R.from_quat(self.robot_origin["quat"]) * vr_relative_rot
        target_quat = target_rot.as_quat()
        
        # For velocity-based control, we still need to calculate the rotation difference
        # But we'll use it differently than DROID
        robot_quat_offset = quat_diff(self.robot_quat, self.robot_origin["quat"])
        target_quat_offset = quat_diff(self.vr_state["quat"], self.vr_origin["quat"])
        quat_action = quat_diff(target_quat_offset, robot_quat_offset)
        euler_action = quat_to_euler(quat_action)
        
        # Calculate Gripper Action - DROID exact
        gripper_action = (self.vr_state["gripper"] * 1.5) - self.robot_gripper
        
        # Calculate Desired Pose
        target_pos = pos_action + self.robot_pos
        target_euler = add_angles(euler_action, self.robot_euler)
        target_cartesian = np.concatenate([target_pos, target_euler])
        target_gripper = self.vr_state["gripper"]
        
        # Scale Appropriately - DROID exact gains
        pos_action *= self.pos_action_gain
        euler_action *= self.rot_action_gain
        gripper_action *= self.gripper_action_gain
        
        # Apply velocity limits
        lin_vel, rot_vel, gripper_vel = self._limit_velocity(pos_action, euler_action, gripper_action)
        
        # Prepare Return Values
        # Store the target quaternion for Deoxys
        info_dict = {
            "target_cartesian_position": target_cartesian, 
            "target_gripper_position": target_gripper,
            "target_quaternion": target_quat  # Add this for Deoxys
        }
        action = np.concatenate([lin_vel, rot_vel, [gripper_vel]])
        action = action.clip(-1, 1)
        
        return action, info_dict
    
    def get_info(self):
        """Get controller state information - DROID exact"""
        info = {
            "success": self._state["buttons"]["A"] if self.controller_id == 'r' else self._state["buttons"]["X"],
            "failure": self._state["buttons"]["B"] if self.controller_id == 'r' else self._state["buttons"]["Y"],
            "movement_enabled": self._state["movement_enabled"],
            "controller_on": self._state["controller_on"],
        }
        
        # Add raw VR controller data for recording
        if self._state["poses"] and self._state["buttons"]:
            info["poses"] = self._state["poses"]
            info["buttons"] = self._state["buttons"]
        
        return info
        
    def reset_robot(self):
        """Reset robot to initial position"""
        if self.debug:
            print("🔄 [DEBUG] Would reset robot to initial position")
            # Return simulated values
            return np.array([0.4, 0.0, 0.3]), np.array([1.0, 0.0, 0.0, 0.0])
        
        print("🔄 Resetting robot to initial position...")
        action = FrankaAction(
            pos=np.zeros(3),
            quat=np.zeros(4),
            gripper=GRIPPER_OPEN,
            reset=True,
            timestamp=time.time(),
        )
        
        self.action_socket.send(bytes(pickle.dumps(action, protocol=-1)))
        robot_state = pickle.loads(self.action_socket.recv())
        
        print(f"✅ Robot reset complete")
        print(f"   Position: [{robot_state.pos[0]:.6f}, {robot_state.pos[1]:.6f}, {robot_state.pos[2]:.6f}]")
        print(f"   Quaternion: [{robot_state.quat[0]:.6f}, {robot_state.quat[1]:.6f}, {robot_state.quat[2]:.6f}, {robot_state.quat[3]:.6f}]")
        
        return robot_state.pos, robot_state.quat
    
    def velocity_to_position_target(self, velocity_action, current_pos, current_quat, action_info=None):
        """Convert velocity action to position target for deoxys control
        
        This is the key difference from DROID - Deoxys expects quaternions directly,
        while DROID's Polymetis converts euler angles to quaternions internally.
        """
        # Extract components
        lin_vel = velocity_action[:3]
        rot_vel = velocity_action[3:6]
        gripper_vel = velocity_action[6]
        
        # Calculate position delta (velocity * dt)
        pos_delta = lin_vel * self.control_interval
        rot_delta = rot_vel * self.control_interval
        gripper_delta = gripper_vel * self.control_interval
        
        # Calculate target position
        target_pos = current_pos + pos_delta
        
        # Calculate target orientation - CRITICAL DIFFERENCE FOR DEOXYS
        if action_info and "target_quaternion" in action_info:
            # Use the pre-calculated target quaternion for Deoxys
            target_quat = action_info["target_quaternion"]
        else:
            # Fallback: convert rotation velocity (euler angles) to quaternion
            # This is less accurate for Deoxys but maintains compatibility
            rot_delta_quat = euler_to_quat(rot_delta)
            current_rot = R.from_quat(current_quat)
            delta_rot = R.from_quat(rot_delta_quat)
            target_rot = delta_rot * current_rot
            target_quat = target_rot.as_quat()
        
        # Calculate target gripper
        target_gripper = np.clip(self.robot_gripper + gripper_delta, 0.0, 1.0)
        
        if self.debug:
            # Debug quaternion transformation
            current_euler = quat_to_euler(current_quat, degrees=True)
            target_euler = quat_to_euler(target_quat, degrees=True)
            print(f"\n🔄 Rotation Debug:")
            print(f"   Current Euler: [R:{current_euler[0]:6.1f}, P:{current_euler[1]:6.1f}, Y:{current_euler[2]:6.1f}]")
            print(f"   Rot Delta (deg): [R:{np.degrees(rot_delta[0]):6.1f}, P:{np.degrees(rot_delta[1]):6.1f}, Y:{np.degrees(rot_delta[2]):6.1f}]")
            print(f"   Target Euler: [R:{target_euler[0]:6.1f}, P:{target_euler[1]:6.1f}, Y:{target_euler[2]:6.1f}]")
            print(f"   Current Quat: [{current_quat[0]:.3f}, {current_quat[1]:.3f}, {current_quat[2]:.3f}, {current_quat[3]:.3f}]")
            print(f"   Target Quat: [{target_quat[0]:.3f}, {target_quat[1]:.3f}, {target_quat[2]:.3f}, {target_quat[3]:.3f}]")
        
        return target_pos, target_quat, target_gripper
    
    def control_loop(self):
        """Main control loop"""
        message_count = 0
        last_control_time = time.time()
        last_debug_time = time.time()
        
        while self.running:
            try:
                current_time = time.time()
                
                # Initialize robot on first frame
                if self.is_first_frame:
                    init_pos, init_quat = self.reset_robot()
                    self.robot_pos = init_pos
                    self.robot_quat = init_quat
                    self.robot_euler = quat_to_euler(init_quat)
                    self.robot_gripper = 0.0
                    self.is_first_frame = False
                
                # Handle robot reset after calibration
                if hasattr(self, '_reset_robot_after_calibration') and self._reset_robot_after_calibration:
                    self._reset_robot_after_calibration = False
                    print("🤖 Executing robot reset after calibration...")
                    reset_pos, reset_quat = self.reset_robot()
                    self.robot_pos = reset_pos
                    self.robot_quat = reset_quat
                    self.robot_euler = quat_to_euler(reset_quat)
                    self.robot_gripper = 0.0
                    # Clear any pending actions
                    self.reset_origin = True
                    print("✅ Robot is now at home position, ready for teleoperation")
                    print("   Hold grip button to start controlling the robot\n")
                    continue  # Skip this control cycle
                
                # Control at specified frequency
                if current_time - last_control_time >= self.control_interval:
                    # Get controller info
                    info = self.get_info()
                    
                    # Check for success/failure buttons
                    if info["success"]:
                        print("\n✅ Success button pressed!")
                        if not self.debug:
                            # Send termination signal
                            self.stop_server()
                            break
                    
                    if info["failure"]:
                        print("\n❌ Failure button pressed!")
                        if not self.debug:
                            # Send termination signal
                            self.stop_server()
                            break
                    
                    # Calculate action if movement is enabled
                    if info["movement_enabled"] and self._state["poses"]:
                        action, action_info = self._calculate_action()
                        
                        # Convert velocity to position target
                        target_pos, target_quat, target_gripper = self.velocity_to_position_target(
                            action, self.robot_pos, self.robot_quat, action_info
                        )
                        
                        # Apply workspace bounds
                        target_pos = np.clip(target_pos, ROBOT_WORKSPACE_MIN, ROBOT_WORKSPACE_MAX)
                        
                        # Handle gripper control - simple open/close based on trigger
                        trigger_value = self._state["buttons"].get("rightTrig" if self.right_controller else "leftTrig", [0.0])[0]
                        gripper_state = GRIPPER_CLOSE if trigger_value > 0.1 else GRIPPER_OPEN
                        # Update robot gripper state for tracking
                        self.robot_gripper = 1.0 if gripper_state == GRIPPER_CLOSE else 0.0
                        
                        if self.debug:
                            # Debug mode - print status for ACTIVE teleoperation
                            if current_time - last_debug_time > 0.5:
                                print(f"\n📊 Debug Data [{message_count:04d}]:")
                                print(f"   🟢 TELEOPERATION: ACTIVE")
                                print(f"   Action (vel): [{action[0]:.3f}, {action[1]:.3f}, {action[2]:.3f}, "
                                      f"{action[3]:.3f}, {action[4]:.3f}, {action[5]:.3f}]")
                                print(f"   Target Pos: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]")
                                target_euler_deg = np.degrees(quat_to_euler(target_quat))
                                print(f"   Target Rot (deg): [R:{target_euler_deg[0]:.1f}, P:{target_euler_deg[1]:.1f}, Y:{target_euler_deg[2]:.1f}]")
                                
                                # Show VR controller rotation for debugging
                                if hasattr(self, 'vr_state') and self.vr_state:
                                    vr_euler = quat_to_euler(self.vr_state['quat'], degrees=True)
                                    print(f"   VR Controller Rot (deg): [R:{vr_euler[0]:.1f}, P:{vr_euler[1]:.1f}, Y:{vr_euler[2]:.1f}]")
                                
                                # Show rotation difference
                                if np.any(np.abs(action[3:6]) > 0.01):
                                    print(f"   🔄 ROTATION ACTIVE - Rot velocities: [{action[3]:.3f}, {action[4]:.3f}, {action[5]:.3f}]")
                                
                                # Show color-coded gripper state
                                if gripper_state == GRIPPER_CLOSE:
                                    gripper_status = f"🔴 CLOSED"
                                else:
                                    gripper_status = f"🟢 OPEN"
                                print(f"   Gripper: {gripper_status} (trigger: {trigger_value:.3f})")
                                
                                # Show if velocities are being limited
                                if np.any(np.abs(action[:3]) >= 0.99):
                                    print(f"   ⚠️  Position velocity at limit!")
                                if np.any(np.abs(action[3:6]) >= 0.99):
                                    print(f"   ⚠️  Rotation velocity at limit!")
                                
                                last_debug_time = current_time
                            
                            # Simulate robot state update
                            self.robot_pos = target_pos
                            self.robot_quat = target_quat
                            self.robot_euler = quat_to_euler(target_quat)
                        else:
                            # Send action to robot - DEOXYS EXPECTS QUATERNIONS
                            robot_action = FrankaAction(
                                pos=target_pos.flatten().astype(np.float32),
                                quat=target_quat.flatten().astype(np.float32),  # Quaternion directly
                                gripper=gripper_state,
                                reset=False,
                                timestamp=time.time(),
                            )
                            
                            self.action_socket.send(bytes(pickle.dumps(robot_action, protocol=-1)))
                            robot_state = pickle.loads(self.action_socket.recv())
                            
                            # Update robot state
                            self.robot_pos = robot_state.pos
                            self.robot_quat = robot_state.quat
                            self.robot_euler = quat_to_euler(robot_state.quat)
                            self.robot_gripper = 1.0 if robot_state.gripper == GRIPPER_CLOSE else 0.0
                    else:
                        # Not moving - show status periodically
                        if current_time - last_debug_time > 0.5:
                            if self.debug:
                                print(f"\n📊 Debug Data [{message_count:04d}]:")
                            
                            if self.calibrating_forward:
                                if self.debug:
                                    print(f"   🎯 CALIBRATING FORWARD - Keep moving controller forward!")
                                # Status is printed in the state update thread
                            elif self.debug:
                                print(f"   🔴 TELEOPERATION: INACTIVE (Hold grip button to enable)")
                            
                            # Show gripper status even when not moving
                            if self.debug:
                                trigger_value = self._state["buttons"].get("rightTrig" if self.right_controller else "leftTrig", [0.0])[0]
                                if trigger_value > 0.1:
                                    gripper_status = f"🔴 CLOSED"
                                else:
                                    gripper_status = f"🟢 OPEN"
                                print(f"   Gripper: {gripper_status} (trigger: {trigger_value:.3f})")
                            
                            last_debug_time = current_time
                    
                    message_count += 1
                    last_control_time = current_time
                
                # Small sleep to prevent CPU spinning
                time.sleep(0.001)
                
            except Exception as e:
                if self.running:
                    print(f"❌ Error in control loop: {e}")
                    import traceback
                    traceback.print_exc()
                    time.sleep(1)  # Wait before retrying
    
    def start(self):
        """Start the server"""
        try:
            # Run control loop
            self.control_loop()
        except KeyboardInterrupt:
            print("\n🛑 Keyboard interrupt received")
            self.stop_server()
    
    def stop_server(self):
        """Gracefully stop the server"""
        if not self.running:
            return
            
        print("🛑 Stopping Oculus VR Server...")
        self.running = False
        
        # Stop state thread
        if hasattr(self, '_state_thread'):
            self._state_thread.join(timeout=1.0)
        
        # Stop Oculus Reader
        if hasattr(self, 'oculus_reader'):
            try:
                self.oculus_reader.stop()
                print("✅ Oculus Reader stopped")
            except Exception as e:
                print(f"⚠️  Error stopping Oculus Reader: {e}")
        
        # Stop simulation server if running
        if self.sim_server:
            self.sim_server.stop()
            print("✅ Simulation server stopped")
        
        # Close robot connections
        if not self.debug:
            if hasattr(self, 'action_socket'):
                self.action_socket.close()
            if hasattr(self, 'controller_publisher'):
                self.controller_publisher.close()
            if hasattr(self, 'context'):
                self.context.term()
            print("✅ Robot connections and ZMQ resources closed")
        
        print("✅ Server stopped gracefully")
        sys.exit(0)


def main():
    parser = argparse.ArgumentParser(
        description='Oculus VR Server with DROID-exact VRPolicy Control',
        epilog='''
This server implements the exact VRPolicy control from DROID with intuitive calibration.

Features:
  - DROID-exact control parameters (gains, velocities, transforms)
  - Intuitive forward direction calibration (hold joystick + move)
  - Deoxys-compatible quaternion handling
  - Origin recalibration on grip press/release

Calibration:
  - Hold joystick button and move controller forward (at least 3mm)
  - The direction you move defines "forward" for the robot
  - Release joystick to complete calibration
  - Falls back to DROID-style calibration if no movement detected

Note: This version is adapted for Deoxys control (quaternion-based) instead of
Polymetis (euler angle-based). The rotation handling has been adjusted accordingly.
        ''',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('--debug', action='store_true', 
                        help='Enable debug mode (no robot control)')
    parser.add_argument('--left-controller', action='store_true',
                        help='Use left controller instead of right (default: right)')
    parser.add_argument('--ip', type=str, default=None,
                        help='IP address of Quest device (default: USB connection)')
    parser.add_argument('--simulation', action='store_true',
                        help='Use simulated FR3 robot instead of real hardware')
    parser.add_argument('--coord-transform', nargs='+', type=float,
                        help='Custom coordinate transformation vector (format: x y z w)')
    parser.add_argument('--hot-reload', action='store_true',
                        help='Enable hot reload mode (auto-restart on file changes)')
    
    args = parser.parse_args()
    
    # If hot reload is requested, launch the hot reload wrapper instead
    if args.hot_reload:
        import subprocess
        import sys
        
        # Remove --hot-reload from args and pass the rest to the wrapper
        new_args = [arg for arg in sys.argv[1:] if arg != '--hot-reload']
        
        print("🔥 Launching in hot reload mode...")
        
        # Check if hot reload script exists
        if not os.path.exists('oculus_vr_server_hotreload.py'):
            print("❌ Hot reload script not found!")
            print("   Make sure oculus_vr_server_hotreload.py is in the same directory")
            sys.exit(1)
        
        # Launch the hot reload wrapper
        try:
            subprocess.run([sys.executable, 'oculus_vr_server_hotreload.py'] + new_args)
        except KeyboardInterrupt:
            print("\n✅ Hot reload stopped")
        sys.exit(0)
    
    # Normal execution (no hot reload)
    # Create and start server with DROID-exact parameters
    coord_transform = args.coord_transform
    server = OculusVRServer(
        debug=args.debug,
        right_controller=not args.left_controller,
        ip_address=args.ip,
        simulation=args.simulation,
        coord_transform=coord_transform
    )
    
    try:
        server.start()
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        server.stop_server()


if __name__ == "__main__":
    main() 