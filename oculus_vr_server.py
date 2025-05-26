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
        
        # Default reordering matrix (VR to robot coordinate transformation)
        if rmat_reorder is None:
            rmat_reorder = [-2, -1, -3, 4]  # Default from VRPolicy
        self.global_to_env_mat = vec_to_reorder_mat(rmat_reorder)
        
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
        print("   - Move at least 5cm in your desired forward direction")
        print("   - Release joystick button to complete calibration")
        print("   - The robot will move forward when you move the controller in that direction")
        
        print("\n💡 Tips:")
        print("   - Calibrate forward direction before starting teleoperation")
        print("   - Stand in a comfortable position facing your desired 'forward'")
        print("   - Make a clear forward motion while holding the joystick")
        print("   - Each grip press/release recalibrates the origin (starting position)")
        print("   - Robot movements are relative to the position when grip was pressed")
        
        print("\nPress Ctrl+C to exit gracefully\n")
        
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
        
        # Robot state
        self.robot_pos = None
        self.robot_quat = None
        self.robot_euler = None
        self.robot_gripper = 0.0
        
        # First frame flag
        self.is_first_frame = True
        
    def signal_handler(self, signum, frame):
        """Handle Ctrl+C and other termination signals"""
        print(f"\n🛑 Received signal {signum}, shutting down gracefully...")
        self.stop_server()
        
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
                controller_pos = pose_matrix[:3, 3]
                
                # Start calibration when joystick is pressed
                if joystick_pressed:
                    self.calibrating_forward = True
                    self.calibration_start_pos = controller_pos.copy()
                    self.calibration_positions = [controller_pos.copy()]
                    if self.debug:
                        print(f"\n🎯 Forward calibration started - Move controller forward while holding joystick")
                        print(f"   Start position: [{controller_pos[0]:.3f}, {controller_pos[1]:.3f}, {controller_pos[2]:.3f}]")
                
                # Collect positions during calibration
                elif self.calibrating_forward and current_joystick:
                    # Always append the current position during calibration
                    self.calibration_positions.append(controller_pos.copy())
                    
                    # Show progress periodically
                    if self.debug and len(self.calibration_positions) % 10 == 0:
                        movement = np.linalg.norm(controller_pos - self.calibration_start_pos)
                        print(f"   Tracking movement: {movement:.3f}m (samples: {len(self.calibration_positions)})")
                
                # Complete calibration when joystick is released
                elif joystick_released and self.calibrating_forward:
                    self.calibrating_forward = False
                    
                    if self.debug:
                        print(f"\n   Calibration ended with {len(self.calibration_positions)} samples")
                    
                    # Filter out duplicate positions
                    unique_positions = [self.calibration_positions[0]]
                    for pos in self.calibration_positions[1:]:
                        if not np.allclose(pos, unique_positions[-1], atol=0.02):  # 2cm tolerance
                            unique_positions.append(pos)
                    
                    if self.debug:
                        print(f"   Unique positions: {len(unique_positions)}")
                        print(f"   Total samples collected: {len(self.calibration_positions)}")
                        if len(self.calibration_positions) >= 2:
                            first_pos = self.calibration_positions[0]
                            last_pos = self.calibration_positions[-1]
                            total_movement = np.linalg.norm(last_pos - first_pos)
                            print(f"   Raw first: [{first_pos[0]:.3f}, {first_pos[1]:.3f}, {first_pos[2]:.3f}]")
                            print(f"   Raw last: [{last_pos[0]:.3f}, {last_pos[1]:.3f}, {last_pos[2]:.3f}]")
                            print(f"   Raw movement: {total_movement:.3f}m")
                        if len(unique_positions) >= 2:
                            print(f"   Filtered first: [{unique_positions[0][0]:.3f}, {unique_positions[0][1]:.3f}, {unique_positions[0][2]:.3f}]")
                            print(f"   Filtered last: [{unique_positions[-1][0]:.3f}, {unique_positions[-1][1]:.3f}, {unique_positions[-1][2]:.3f}]")
                        else:
                            print(f"   All positions within 2cm of each other")
                    
                    if len(unique_positions) >= 2:  # Need at least start and end position
                        # Calculate forward direction from movement
                        start_pos = np.array(unique_positions[0])
                        end_pos = np.array(unique_positions[-1])
                        forward_vec = end_pos - start_pos
                        
                        if self.debug:
                            print(f"   Start: [{start_pos[0]:.3f}, {start_pos[1]:.3f}, {start_pos[2]:.3f}]")
                            print(f"   End: [{end_pos[0]:.3f}, {end_pos[1]:.3f}, {end_pos[2]:.3f}]")
                            print(f"   Movement: [{forward_vec[0]:.3f}, {forward_vec[1]:.3f}, {forward_vec[2]:.3f}]")
                        
                        # Check if there was enough movement
                        movement_distance = np.linalg.norm(forward_vec)
                        if movement_distance > 0.05:  # 5cm minimum movement
                            forward_vec_normalized = forward_vec / movement_distance  # Normalize
                            
                            if self.debug:
                                print(f"\n✅ Forward direction calibrated!")
                                print(f"   Movement direction: [{forward_vec[0]:.3f}, {forward_vec[1]:.3f}, {forward_vec[2]:.3f}]")
                                print(f"   Normalized: [{forward_vec_normalized[0]:.3f}, {forward_vec_normalized[1]:.3f}, {forward_vec_normalized[2]:.3f}]")
                                print(f"   Total movement: {movement_distance:.3f}m")
                            
                            # Determine which axis had the most movement
                            abs_movement = np.abs(forward_vec)
                            dominant_axis = np.argmax(abs_movement)
                            axis_names = ['X', 'Y', 'Z']
                            
                            if self.debug:
                                print(f"   Dominant axis: {axis_names[dominant_axis]} ({forward_vec[dominant_axis]:.3f}m)")
                            
                            # Create a calibration matrix that maps the movement direction to robot forward
                            # The goal is to make the user's forward motion map to robot's forward (X) motion
                            # We need to create a rotation that aligns the movement vector with [1, 0, 0] (robot forward)
                            
                            # First, let's create the target forward vector in robot space
                            robot_forward = np.array([1.0, 0.0, 0.0])  # Robot forward is +X
                            
                            # Calculate the rotation needed to align movement with robot forward
                            # Using Rodrigues' rotation formula
                            rotation_axis = np.cross(forward_vec_normalized, robot_forward)
                            rotation_angle = np.arccos(np.clip(np.dot(forward_vec_normalized, robot_forward), -1.0, 1.0))
                            
                            if np.linalg.norm(rotation_axis) > 0.001:
                                rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
                                # Create rotation matrix
                                K = np.array([[0, -rotation_axis[2], rotation_axis[1]],
                                             [rotation_axis[2], 0, -rotation_axis[0]],
                                             [-rotation_axis[1], rotation_axis[0], 0]])
                                R_calibration = np.eye(3) + np.sin(rotation_angle) * K + (1 - np.cos(rotation_angle)) * K @ K
                            else:
                                # Movement is already aligned with robot forward or backward
                                if forward_vec_normalized[0] < 0:  # Moving backward
                                    # Rotate 180 degrees around Z axis
                                    R_calibration = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
                                else:
                                    R_calibration = np.eye(3)
                            
                            # Update the VR to global transformation
                            # This matrix will transform VR coordinates so that the user's forward motion
                            # becomes robot forward motion
                            self.vr_to_global_mat = np.eye(4)
                            self.vr_to_global_mat[:3, :3] = R_calibration
                            
                            if self.debug:
                                print(f"   Calibration matrix applied")
                                # Test the calibration
                                test_forward = R_calibration @ forward_vec_normalized
                                print(f"   Test: Your forward [{forward_vec_normalized[0]:.2f}, {forward_vec_normalized[1]:.2f}, {forward_vec_normalized[2]:.2f}]")
                                print(f"         maps to robot [{test_forward[0]:.2f}, {test_forward[1]:.2f}, {test_forward[2]:.2f}]")
                        else:
                            if self.debug:
                                print(f"\n⚠️  Not enough movement detected ({movement_distance:.3f}m). Please move controller at least 5cm forward.")
                    else:
                        if self.debug:
                            print(f"\n⚠️  No movement detected. Please move the controller while holding the joystick button.")
            
            # Update previous button states for next iteration
            self.prev_grip_state = current_grip
            self.prev_joystick_state = current_joystick
    
    def _process_reading(self):
        """Apply coordinate transformations to VR controller pose"""
        rot_mat = np.asarray(self._state["poses"].get(self.controller_id, np.eye(4)))
        rot_mat = self.global_to_env_mat @ self.vr_to_global_mat @ rot_mat
        vr_pos = self.spatial_coeff * rot_mat[:3, 3]
        vr_quat = rmat_to_quat(rot_mat[:3, :3])
        
        # Get trigger value
        if self.right_controller:
            vr_gripper = self._state["buttons"].get("rightTrig", [0.0])[0]
        else:
            vr_gripper = self._state["buttons"].get("leftTrig", [0.0])[0]
        
        self.vr_state = {"pos": vr_pos, "quat": vr_quat, "gripper": vr_gripper}
    
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
            self.robot_origin = {"pos": self.robot_pos.copy(), "quat": self.robot_quat.copy()}
            self.vr_origin = {"pos": self.vr_state["pos"].copy(), "quat": self.vr_state["quat"].copy()}
            self.reset_origin = False
            if self.debug:
                print("📍 Origin calibrated")
        
        # Calculate Positional Action
        robot_pos_offset = self.robot_pos - self.robot_origin["pos"]
        target_pos_offset = self.vr_state["pos"] - self.vr_origin["pos"]
        pos_action = target_pos_offset - robot_pos_offset
        
        # Calculate Euler Action
        robot_quat_offset = quat_diff(self.robot_quat, self.robot_origin["quat"])
        target_quat_offset = quat_diff(self.vr_state["quat"], self.vr_origin["quat"])
        quat_action = quat_diff(target_quat_offset, robot_quat_offset)
        euler_action = quat_to_euler(quat_action)
        
        # Calculate Gripper Action (with 1.5x scaling from VRPolicy)
        gripper_action = (self.vr_state["gripper"] * 1.5) - self.robot_gripper
        
        # Calculate Desired Pose
        target_pos = pos_action + self.robot_pos
        target_euler = add_angles(euler_action, self.robot_euler)
        target_cartesian = np.concatenate([target_pos, target_euler])
        target_gripper = self.vr_state["gripper"]
        
        # Scale Appropriately
        pos_action *= self.pos_action_gain
        euler_action *= self.rot_action_gain
        gripper_action *= self.gripper_action_gain
        lin_vel, rot_vel, gripper_vel = self._limit_velocity(pos_action, euler_action, gripper_action)
        
        # Prepare Return Values
        info_dict = {"target_cartesian_position": target_cartesian, "target_gripper_position": target_gripper}
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
                        # Not moving - just print status in debug mode
                        if self.debug and current_time - last_debug_time > 0.5:
                            print(f"\n📊 Debug Data [{message_count:04d}]:")
                            if self.calibrating_forward:
                                print(f"   🎯 CALIBRATING FORWARD - Keep moving controller forward!")
                                if self.controller_id in self._state["poses"]:
                                    current_pos = self._state["poses"][self.controller_id][:3, 3]
                                    print(f"   Current pos: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")
                                    if len(self.calibration_positions) > 0:
                                        start_pos = self.calibration_positions[0]
                                        movement = np.linalg.norm(current_pos - start_pos)
                                        print(f"   Start pos: [{start_pos[0]:.3f}, {start_pos[1]:.3f}, {start_pos[2]:.3f}]")
                                        print(f"   Movement: {movement:.3f}m, Samples: {len(self.calibration_positions)}")
                            else:
                                print(f"   🔴 TELEOPERATION: INACTIVE (Hold grip button to enable)")
                            
                            # Show gripper status even when not moving
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