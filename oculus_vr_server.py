#!/usr/bin/env python3
"""
Oculus VR Server - Implements VRPolicy-style teleoperation control
Based on droid/controllers/oculus_controller.py

Features:
- Velocity-based control with configurable gains
- Forward direction calibration (joystick button)
- Origin calibration on grip press/release
- Coordinate transformation pipeline
- Success/failure buttons (A/B or X/Y)
- 50Hz VR polling with internal state thread
- Safety limiting and workspace bounds
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
                 max_lin_vel=1.0,
                 max_rot_vel=1.0,
                 max_gripper_vel=1.0,
                 spatial_coeff=1.0,
                 pos_action_gain=5.0,
                 rot_action_gain=2.0,
                 gripper_action_gain=3.0,
                 rmat_reorder=None,
                 control_hz=15,
                 use_simple_gripper=True):
        """
        Initialize the Oculus VR Server with VRPolicy-style control
        
        Args:
            debug: If True, only print data without controlling robot
            right_controller: If True, use right controller for robot control
            ip_address: IP address of Quest device (None for USB connection)
            max_lin_vel: Maximum linear velocity (m/s)
            max_rot_vel: Maximum rotational velocity (rad/s)
            max_gripper_vel: Maximum gripper velocity
            spatial_coeff: Spatial scaling factor
            pos_action_gain: Position action gain
            rot_action_gain: Rotation action gain
            gripper_action_gain: Gripper action gain
            rmat_reorder: Reordering vector for coordinate transformation
            control_hz: Control frequency (Hz)
            use_simple_gripper: If True, use simple open/close gripper control
        """
        self.debug = debug
        self.right_controller = right_controller
        self.running = True
        self.use_simple_gripper = use_simple_gripper
        
        # VRPolicy parameters
        self.max_lin_vel = max_lin_vel
        self.max_rot_vel = max_rot_vel
        self.max_gripper_vel = max_gripper_vel
        self.spatial_coeff = spatial_coeff
        self.pos_action_gain = pos_action_gain
        self.rot_action_gain = rot_action_gain
        self.gripper_action_gain = gripper_action_gain
        self.control_hz = control_hz
        self.control_interval = 1.0 / control_hz
        
        # VR to Robot coordinate mapping
        # VR: X=right, Y=up, Z=forward
        # Robot: X=forward, Y=left, Z=up
        # So: robot_X = -vr_Z, robot_Y = -vr_X, robot_Z = vr_Y
        if rmat_reorder is None:
            rmat_reorder = [-3, -1, 2, 4]  # Maps VR [X,Y,Z] to Robot [-Z,-X,Y]
        self.global_to_env_mat = vec_to_reorder_mat(rmat_reorder)
        
        if self.debug:
            print("\n🔍 DEBUG: VR to Robot Coordinate Mapping:")
            print("   VR Space:")
            print("   - X: Right")
            print("   - Y: Up")
            print("   - Z: Forward")
            print("\n   Robot Space:")
            print("   - X: Forward")
            print("   - Y: Left")
            print("   - Z: Up")
            print("\n   Transformation Matrix:")
            for i in range(4):
                row = self.global_to_env_mat[i]
                print(f"   [{row[0]:6.1f}, {row[1]:6.1f}, {row[2]:6.1f}, {row[3]:6.1f}]")
        
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
        
        # Robot control components (only if not in debug mode)
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
        
        print("\n🎮 Oculus VR Server with VRPolicy-style Control")
        print(f"   Using {'RIGHT' if right_controller else 'LEFT'} controller")
        print(f"   Mode: {'DEBUG' if debug else 'LIVE ROBOT CONTROL'}")
        print(f"   Control frequency: {control_hz}Hz")
        print(f"   Position gain: {pos_action_gain:.1f}")
        print(f"   Rotation gain: {rot_action_gain:.1f}")
        print(f"   Gripper: {'Simple open/close' if self.use_simple_gripper else f'Velocity-based (gain: {gripper_action_gain:.1f})'}")
        
        print("\n📋 Controls:")
        print("   - HOLD grip button: Enable teleoperation")
        print("   - RELEASE grip button: Pause teleoperation")
        print("   - PRESS trigger: Close gripper (like a real gripper)")
        print("   - RELEASE trigger: Open gripper")
        print("   - A/X button: Mark success and exit")
        print("   - B/Y button: Mark failure and exit")
        
        print("\n🧭 Forward Direction Calibration:")
        print("   - HOLD joystick button and MOVE controller forward")
        print("   - The direction you move defines 'forward' for the robot")
        print("   - Move at least 3mm in your desired forward direction")
        print("   - Release joystick button to complete calibration")
        print("   - The robot will move forward when you move the controller in that direction")
        
        print("\n💡 Tips:")
        print("   - Calibrate forward direction before starting teleoperation")
        print("   - Stand in a comfortable position facing your desired 'forward'")
        print("   - Make a clear forward motion while holding the joystick")
        print("   - Each grip press/release recalibrates the origin (starting position)")
        print("   - Robot movements are relative to the position when grip was pressed")
        
        print("\nPress Ctrl+C to exit gracefully\n")
        
        # VR position units appear to be in meters, scale for mm display
        self.vr_position_scale = 1.0  # Keep raw units
        self.distance_display_scale = 1000.0  # Convert to mm for display
        
    def reset_state(self):
        """Reset internal state"""
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
        
        # Button state tracking for edge detection
        self.prev_joystick_state = False
        self.prev_grip_state = False
        
        # Calibration state
        self.calibrating_forward = False
        self.calibration_start_pos = None
        self.calibration_positions = []
        self.calibration_max_distance = 0.0
        self.calibration_furthest_pos = None
        
        # Robot state
        self.robot_pos = None
        self.robot_quat = None
        self.robot_euler = None
        self.robot_gripper = 0.0
        
        # Current VR controller position (raw, before transformations)
        self.current_vr_pos = None
        
        # First frame flag
        self.is_first_frame = True
        
    def signal_handler(self, signum, frame):
        """Handle Ctrl+C and other termination signals"""
        print(f"\n🛑 Received signal {signum}, shutting down gracefully...")
        self.stop_server()
        
    def _transform_vr_position(self, pose_matrix):
        """Apply the same coordinate transformations as robot control"""
        transformed_matrix = self.global_to_env_mat @ self.vr_to_global_mat @ pose_matrix
        transformed_pos = self.spatial_coeff * transformed_matrix[:3, 3]
        return transformed_pos
    
    def _update_internal_state(self, num_wait_sec=5, hz=50):
        """Continuously poll VR controller state at 50Hz"""
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
            
            # Debug: Show raw poses data periodically
            if self.debug and len(poses) > 0:
                debug_counter = getattr(self, '_debug_counter', 0)
                self._debug_counter = debug_counter + 1
                if self._debug_counter % 50 == 0:  # Every 50 frames (1 second at 50Hz)
                    print(f"\n🔍 DEBUG: Raw VR poses data:")
                    for controller_id, pose_matrix in poses.items():
                        pos = pose_matrix[:3, 3]
                        print(f"   Controller '{controller_id}': [{pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f}]")
                    print(f"   Using controller: '{self.controller_id}'")
                    print(f"   Available controllers: {list(poses.keys())}")
            
            # Get current button states
            current_grip = buttons.get(self.controller_id.upper() + "G", False)
            current_joystick = buttons.get(self.controller_id.upper() + "J", False)
            
            # Debug: Show button states when they change
            if self.debug:
                if current_grip != self.prev_grip_state:
                    print(f"🎮 Grip button {'PRESSED' if current_grip else 'RELEASED'}")
                if current_joystick != self.prev_joystick_state:
                    print(f"🕹️  Joystick button {'PRESSED' if current_joystick else 'RELEASED'}")
            
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
                controller_pos = pose_matrix[:3, 3]
                
                # Store current position for consistent access
                self.current_vr_pos = controller_pos.copy()
            else:
                # Controller not found in poses
                if self.debug and self._state["poses"]:
                    print(f"⚠️  Controller '{self.controller_id}' not found in poses. Available: {list(self._state['poses'].keys())}")
                self.current_vr_pos = None
            
            # Start calibration when joystick is pressed
            if joystick_pressed:
                self.calibrating_forward = True
                # Capture initial position in RAW VR space (already in meters)
                pose_matrix = self._state["poses"][self.controller_id]
                raw_pos = pose_matrix[:3, 3]  # Keep raw units
                
                self.calibration_start_pos = raw_pos.copy()
                print(f"\n🎯 Forward calibration started - Move controller in desired forward direction")
                print(f"   Initial position (meters): [{raw_pos[0]:.6f}, {raw_pos[1]:.6f}, {raw_pos[2]:.6f}]")
                print(f"   Move the controller in the direction you want to be 'forward'")
                print(f"   The direction of your movement will define 'forward'")
                self.calibration_start_time = time.time()
                
                if self.debug:
                    print(f"\n🔍 Debug: Raw position data")
                    print(f"   Matrix from controller:")
                    for i in range(4):
                        print(f"   [{pose_matrix[i,0]:8.6f}, {pose_matrix[i,1]:8.6f}, {pose_matrix[i,2]:8.6f}, {pose_matrix[i,3]:8.6f}]")
            
            # Collect positions during calibration
            elif self.calibrating_forward and current_joystick:
                # Get current position in RAW VR space
                pose_matrix = self._state["poses"][self.controller_id]
                raw_pos = pose_matrix[:3, 3]  # Keep raw units
                
                # Calculate direct movement from start point (in meters)
                movement_vector = raw_pos - self.calibration_start_pos
                current_distance = np.linalg.norm(movement_vector)
                
                # Show progress periodically
                if len(self.calibration_positions) % 10 == 0:
                    # Calculate direction components
                    abs_movement = np.abs(movement_vector)
                    current_direction = movement_vector / (current_distance + 1e-6)
                    
                    print(f"\n   Current Movement Analysis:")
                    print(f"   Point-to-point distance: {current_distance*self.distance_display_scale:.1f}mm ({current_distance:.6f}m)")
                    print(f"   Current Direction: [{current_direction[0]:+.3f}, {current_direction[1]:+.3f}, {current_direction[2]:+.3f}]")
                    print(f"   Movement by axis:")
                    print(f"   - X: {movement_vector[0]*self.distance_display_scale:+.1f}mm ({abs_movement[0]/current_distance*100:.1f}%)")
                    print(f"   - Y: {movement_vector[1]*self.distance_display_scale:+.1f}mm ({abs_movement[1]/current_distance*100:.1f}%)")
                    print(f"   - Z: {movement_vector[2]*self.distance_display_scale:+.1f}mm ({abs_movement[2]/current_distance*100:.1f}%)")
                    
                    if current_distance < 0.003:  # 3mm in meters
                        print(f"   ⚠️  Need more movement! ({(current_distance/0.003)*100:.0f}% of 3mm goal)")
                    else:
                        print(f"   ✅ Movement sufficient! ({current_distance*self.distance_display_scale:.1f}mm)")
            
            # Complete calibration when joystick is released
            elif joystick_released and self.calibrating_forward:
                self.calibrating_forward = False
                
                # Get final position
                pose_matrix = self._state["poses"][self.controller_id]
                end_pos = pose_matrix[:3, 3]  # Keep raw units
                
                # Calculate movement vector from start to end (in meters)
                start_pos = self.calibration_start_pos
                forward_vec = end_pos - start_pos
                movement_distance = np.linalg.norm(forward_vec)
                
                print(f"\n🔍 Calibration Analysis:")
                print(f"   Start position (m): [{start_pos[0]:.6f}, {start_pos[1]:.6f}, {start_pos[2]:.6f}]")
                print(f"   End position (m):   [{end_pos[0]:.6f}, {end_pos[1]:.6f}, {end_pos[2]:.6f}]")
                print(f"   Movement vector (m): [{forward_vec[0]:+.6f}, {forward_vec[1]:+.6f}, {forward_vec[2]:+.6f}]")
                print(f"   Total 3D distance: {movement_distance*self.distance_display_scale:.1f}mm ({movement_distance:.6f}m)")
                
                # Calculate direction components
                if movement_distance > 0:
                    direction = forward_vec / movement_distance
                    abs_components = np.abs(direction)
                    dominant_axis = np.argmax(abs_components)
                    axis_names = ['X', 'Y', 'Z']
                    
                    print(f"\n   Movement Direction Analysis:")
                    print(f"   Normalized direction: [{direction[0]:+.3f}, {direction[1]:+.3f}, {direction[2]:+.3f}]")
                    print(f"   Direction components:")
                    print(f"   - X: {abs_components[0]*100:4.1f}%{' (dominant)' if dominant_axis == 0 else ''}")
                    print(f"   - Y: {abs_components[1]*100:4.1f}%{' (dominant)' if dominant_axis == 1 else ''}")
                    print(f"   - Z: {abs_components[2]*100:4.1f}%{' (dominant)' if dominant_axis == 2 else ''}")
                
                if movement_distance > 0.003:  # 3mm in meters
                    # Normalize the movement vector to get forward direction
                    forward_vec_normalized = forward_vec / movement_distance  # Already in meters
                    
                    print(f"\n✅ Forward direction calibrated!")
                    print(f"   Point-to-point distance: {movement_distance*self.distance_display_scale:.1f}mm ({movement_distance:.6f}m)")
                    print(f"   This movement direction will be mapped to robot's forward axis")
                    
                    # Create rotation matrix to align this vector with robot's forward (X) axis
                    temp_mat = np.eye(4)
                    temp_mat[:3, 3] = forward_vec_normalized
                    transformed_temp = self.global_to_env_mat @ temp_mat
                    transformed_forward = transformed_temp[:3, 3]
                    
                    print(f"\n   After coordinate reordering:")
                    print(f"   [{transformed_forward[0]:+.3f}, {transformed_forward[1]:+.3f}, {transformed_forward[2]:+.3f}]")
                    
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
                    
                    # Test the calibration
                    test_mat = np.eye(4)
                    test_mat[:3, 3] = forward_vec_normalized
                    test_result = self.global_to_env_mat @ self.vr_to_global_mat @ test_mat
                    test_forward = test_result[:3, 3]
                    
                    print(f"\n   Final Calibration Test:")
                    print(f"   Your movement direction will map to:")
                    print(f"   [{test_forward[0]:+.3f}, {test_forward[1]:+.3f}, {test_forward[2]:+.3f}]")
                    
                    if abs(test_forward[0] - 1.0) < 0.01:
                        print(f"   ✅ Perfect alignment with robot's forward axis!")
                    else:
                        print(f"   ℹ️  Movement will be projected onto robot's forward axis")
                        print(f"       This is normal if you moved diagonally")
                else:
                    print(f"\n⚠️  Not enough movement detected ({movement_distance:.1f}mm)")
                    print(f"   Please move controller at least 3mm in your desired forward direction")
                    print(f"   💡 Tips:")
                    print(f"   - Make a clear, straight movement in your desired forward direction")
                    print(f"   - Move in a single, deliberate motion")
                    print(f"   - Current point-to-point distance: {movement_distance:.1f}mm")
            
            # Update previous button states for next iteration
            self.prev_grip_state = current_grip
            self.prev_joystick_state = current_joystick
    
    def _process_reading(self):
        """Apply coordinate transformations to VR controller pose"""
        # Get full pose matrix from controller
        pose_matrix = np.asarray(self._state["poses"].get(self.controller_id, np.eye(4)))
        
        # Debug: Print raw rotation matrix periodically
        if self.debug and hasattr(self, '_raw_rot_debug_counter'):
            self._raw_rot_debug_counter += 1
            if self._raw_rot_debug_counter % 50 == 0:  # Every 50 frames
                print(f"\n🎮 Raw VR Controller Matrix:")
                for i in range(4):
                    print(f"   [{pose_matrix[i,0]:7.3f}, {pose_matrix[i,1]:7.3f}, {pose_matrix[i,2]:7.3f}, {pose_matrix[i,3]:7.3f}]")
                # Check if rotation part is identity
                rot_part = pose_matrix[:3, :3]
                if np.allclose(rot_part, np.eye(3), atol=0.01):
                    print("   ⚠️  WARNING: Rotation matrix is nearly identity! Controller may not be sending rotation data.")
        elif self.debug and not hasattr(self, '_raw_rot_debug_counter'):
            self._raw_rot_debug_counter = 0
        
        # Apply coordinate transformations
        # First apply calibration (vr_to_global)
        transformed = self.vr_to_global_mat @ pose_matrix
        # Then apply coordinate system reordering
        transformed = self.global_to_env_mat @ transformed
        
        # Extract position and rotation
        vr_pos = self.spatial_coeff * transformed[:3, 3]  # Position vector
        vr_rot_mat = transformed[:3, :3]  # Rotation matrix
        vr_quat = rmat_to_quat(vr_rot_mat)  # Convert to quaternion
        
        # Get trigger value for gripper
        if self.right_controller:
            vr_gripper = self._state["buttons"].get("rightTrig", [0.0])[0]
        else:
            vr_gripper = self._state["buttons"].get("leftTrig", [0.0])[0]
        
        # Store transformed state
        self.vr_state = {
            "pos": vr_pos,
            "rot_mat": vr_rot_mat,
            "quat": vr_quat,
            "gripper": vr_gripper
        }
        
        if self.debug and hasattr(self, '_rot_debug_counter'):
            self._rot_debug_counter += 1
            if self._rot_debug_counter % 50 == 0:
                euler = quat_to_euler(vr_quat, degrees=True)
                print(f"\n🔄 Controller Rotation:")
                print(f"   Euler (deg): [R:{euler[0]:6.1f}, P:{euler[1]:6.1f}, Y:{euler[2]:6.1f}]")
                print(f"   Quaternion: [{vr_quat[0]:6.3f}, {vr_quat[1]:6.3f}, {vr_quat[2]:6.3f}, {vr_quat[3]:6.3f}]")
        elif self.debug and not hasattr(self, '_rot_debug_counter'):
            self._rot_debug_counter = 0
    
    def _limit_velocity(self, lin_vel, rot_vel, gripper_vel):
        """Scales down the linear and angular magnitudes of the action"""
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
        """Calculate robot action from VR controller state"""
        # Read Sensor
        if self.update_sensor:
            self._process_reading()
            self.update_sensor = False
        
        # Check if we have valid data
        if self.vr_state is None or self.robot_pos is None:
            return np.zeros(7), {}
        
        # Reset Origin On Release
        if self.reset_origin:
            self.robot_origin = {
                "pos": self.robot_pos.copy(),
                "quat": self.robot_quat.copy(),
                "rot_mat": quat_to_rmat(self.robot_quat)
            }
            self.vr_origin = {
                "pos": self.vr_state["pos"].copy(),
                "quat": self.vr_state["quat"].copy(),
                "rot_mat": self.vr_state["rot_mat"].copy()
            }
            self.reset_origin = False
            print("📍 Origin calibrated")
            if self.debug:
                print(f"   Robot origin: pos=[{self.robot_origin['pos'][0]:.3f}, {self.robot_origin['pos'][1]:.3f}, {self.robot_origin['pos'][2]:.3f}]")
                euler = quat_to_euler(self.robot_origin['quat'], degrees=True)
                print(f"   Robot origin rotation: [R:{euler[0]:6.1f}, P:{euler[1]:6.1f}, Y:{euler[2]:6.1f}]")
        
        # Calculate Positional Action
        robot_pos_offset = self.robot_pos - self.robot_origin["pos"]
        target_pos_offset = self.vr_state["pos"] - self.vr_origin["pos"]
        pos_action = target_pos_offset - robot_pos_offset
        
        # Calculate Rotation Action
        # Get relative rotations from origin
        robot_rot_offset = quat_diff(self.robot_quat, self.robot_origin["quat"])
        target_rot_offset = quat_diff(self.vr_state["quat"], self.vr_origin["quat"])
        # Calculate difference between target and current rotation
        rot_action_quat = quat_diff(target_rot_offset, robot_rot_offset)
        # Convert to euler angles for velocity control
        rot_action = quat_to_euler(rot_action_quat)
        
        # Calculate Gripper Action
        gripper_action = (self.vr_state["gripper"] * 1.5) - self.robot_gripper
        
        # Calculate Desired Pose
        target_pos = pos_action + self.robot_pos
        target_euler = add_angles(rot_action, self.robot_euler)
        target_cartesian = np.concatenate([target_pos, target_euler])
        target_gripper = self.vr_state["gripper"]
        
        # Scale Appropriately
        pos_action *= self.pos_action_gain
        rot_action *= self.rot_action_gain
        gripper_action *= self.gripper_action_gain
        
        # Apply velocity limits
        lin_vel, rot_vel, gripper_vel = self._limit_velocity(pos_action, rot_action, gripper_action)
        
        # Prepare Return Values
        info_dict = {
            "target_cartesian_position": target_cartesian,
            "target_gripper_position": target_gripper,
            "rot_action_quat": rot_action_quat
        }
        action = np.concatenate([lin_vel, rot_vel, [gripper_vel]])
        action = action.clip(-1, 1)
        
        return action, info_dict
    
    def get_info(self):
        """Get controller state information"""
        info = {
            "success": self._state["buttons"].get("A" if self.controller_id == 'r' else "X", False),
            "failure": self._state["buttons"].get("B" if self.controller_id == 'r' else "Y", False),
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
    
    def velocity_to_position_target(self, velocity_action, current_pos, current_quat):
        """Convert velocity action to position target for deoxys control"""
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
        
        # Calculate target orientation
        current_euler = quat_to_euler(current_quat)
        target_euler = add_angles(rot_delta, current_euler)
        target_quat = euler_to_quat(target_euler)
        
        # Calculate target gripper
        target_gripper = np.clip(self.robot_gripper + gripper_delta, 0.0, 1.0)
        
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
                            action, self.robot_pos, self.robot_quat
                        )
                        
                        # Apply workspace bounds
                        target_pos = np.clip(target_pos, ROBOT_WORKSPACE_MIN, ROBOT_WORKSPACE_MAX)
                        
                        # Handle gripper control
                        if self.use_simple_gripper:
                            # Simple open/close based on trigger threshold
                            trigger_value = self._state["buttons"].get("rightTrig" if self.right_controller else "leftTrig", [0.0])[0]
                            gripper_state = GRIPPER_CLOSE if trigger_value > 0.1 else GRIPPER_OPEN
                            # Update robot gripper state for tracking
                            self.robot_gripper = 1.0 if gripper_state == GRIPPER_CLOSE else 0.0
                        else:
                            # Velocity-based gripper control
                            gripper_state = GRIPPER_CLOSE if target_gripper > 0.5 else GRIPPER_OPEN
                            self.robot_gripper = target_gripper
                        
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
                                
                                # Show color-coded gripper state
                                trigger_value = self._state["buttons"].get("rightTrig" if self.right_controller else "leftTrig", [0.0])[0]
                                if gripper_state == GRIPPER_CLOSE:
                                    gripper_status = f"🔴 CLOSED"
                                else:
                                    gripper_status = f"🟢 OPEN"
                                print(f"   Gripper: {gripper_status} (trigger: {trigger_value:.3f})")
                                
                                # Show if velocities are being limited
                                if np.any(np.abs(action[:3]) >= 0.99):
                                    print(f"   ⚠️  Position velocity at limit! Consider increasing --pos-action-gain or --max-lin-vel")
                                if np.any(np.abs(action[3:6]) >= 0.99):
                                    print(f"   ⚠️  Rotation velocity at limit! Consider increasing --rot-action-gain or --max-rot-vel")
                                
                                last_debug_time = current_time
                            
                            # Simulate robot state update
                            self.robot_pos = target_pos
                            self.robot_quat = target_quat
                            self.robot_euler = quat_to_euler(target_quat)
                        else:
                            # Send action to robot
                            robot_action = FrankaAction(
                                pos=target_pos.flatten().astype(np.float32),
                                quat=target_quat.flatten().astype(np.float32),
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
                                if not self.debug:
                                    print(f"\n🎯 CALIBRATING FORWARD - Keep moving controller forward!")
                                else:
                                    print(f"   🎯 CALIBRATING FORWARD - Keep moving controller forward!")
                                # Use real-time position data directly from state
                                if self.controller_id in self._state["poses"]:
                                    raw_pos = self._state["poses"][self.controller_id][:3, 3]
                                    pose_matrix = self._state["poses"][self.controller_id]
                                    transformed_pos = self._transform_vr_position(pose_matrix)
                                    prefix = "   " if self.debug else ""
                                    print(f"{prefix}   Current pos (raw): [{raw_pos[0]:.6f}, {raw_pos[1]:.6f}, {raw_pos[2]:.6f}]")
                                    print(f"{prefix}   Current pos (transformed): [{transformed_pos[0]:.6f}, {transformed_pos[1]:.6f}, {transformed_pos[2]:.6f}]")
                                    if len(self.calibration_positions) > 0:
                                        start_pos = self.calibration_start_pos
                                        movement = np.linalg.norm(raw_pos - start_pos)
                                        max_movement = self.calibration_max_distance
                                        print(f"{prefix}   Start pos: [{start_pos[0]:.6f}, {start_pos[1]:.6f}, {start_pos[2]:.6f}]")
                                        print(f"{prefix}   Movement: {movement:.6f}m (Max: {max_movement:.6f}m), Samples: {len(self.calibration_positions)}")
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
        description='Oculus VR Server with VRPolicy-style Control',
        epilog='''
Performance Tuning Tips:
  For faster motion, increase gains:
    --pos-action-gain 10.0 --rot-action-gain 5.0
  
  For smoother motion, increase control frequency:
    --control-hz 30
  
  For larger movements, increase velocity limits:
    --max-lin-vel 2.0 --max-rot-vel 2.0
  
  Example for responsive control:
    python oculus_vr_server.py --debug --pos-action-gain 10 --rot-action-gain 5 --control-hz 30
        ''',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('--debug', action='store_true', 
                        help='Enable debug mode (no robot control)')
    parser.add_argument('--left-controller', action='store_true',
                        help='Use left controller instead of right (default: right)')
    parser.add_argument('--ip', type=str, default=None,
                        help='IP address of Quest device (default: USB connection)')
    
    # VRPolicy parameters with better defaults
    parser.add_argument('--max-lin-vel', type=float, default=1.5,
                        help='Maximum linear velocity in m/s (default: 1.5)')
    parser.add_argument('--max-rot-vel', type=float, default=1.5,
                        help='Maximum rotational velocity in rad/s (default: 1.5)')
    parser.add_argument('--max-gripper-vel', type=float, default=1.0,
                        help='Maximum gripper velocity (default: 1.0)')
    parser.add_argument('--spatial-coeff', type=float, default=1.0,
                        help='Spatial coefficient for scaling VR movements (default: 1.0)')
    parser.add_argument('--pos-action-gain', type=float, default=8.0,
                        help='Position action gain - higher = faster movement (default: 8.0)')
    parser.add_argument('--rot-action-gain', type=float, default=4.0,
                        help='Rotation action gain - higher = faster rotation (default: 4.0)')
    parser.add_argument('--gripper-action-gain', type=float, default=3.0,
                        help='Gripper action gain (default: 3.0)')
    parser.add_argument('--control-hz', type=float, default=20.0,
                        help='Control frequency in Hz - higher = smoother (default: 20.0)')
    parser.add_argument('--velocity-gripper', action='store_true',
                        help='Use velocity-based gripper control instead of simple open/close (default: simple)')
    
    args = parser.parse_args()
    
    # Create and start server
    server = OculusVRServer(
        debug=args.debug,
        right_controller=not args.left_controller,
        ip_address=args.ip,
        max_lin_vel=args.max_lin_vel,
        max_rot_vel=args.max_rot_vel,
        max_gripper_vel=args.max_gripper_vel,
        spatial_coeff=args.spatial_coeff,
        pos_action_gain=args.pos_action_gain,
        rot_action_gain=args.rot_action_gain,
        gripper_action_gain=args.gripper_action_gain,
        control_hz=args.control_hz,
        use_simple_gripper=not args.velocity_gripper
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