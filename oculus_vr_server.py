#!/usr/bin/env python3
"""
Oculus VR Server - Reads data from Meta Quest headset via Oculus Reader
and directly controls the robot via teleoperation

Features:
- Controller represents the position and orientation of the gripper
- Grip button controls teleoperation (hold to move, release to stop)
- Trigger controls gripper (hold to close, release to open)
- Direct robot control without intermediate teleoperator
- Configurable scaling for position and rotation
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
from numpy.linalg import pinv

# Import the Oculus Reader
from oculus_reader.reader import OculusReader

# Import robot control components
from frankateach.network import create_request_socket
from frankateach.constants import (
    HOST, CONTROL_PORT,
    GRIPPER_OPEN, GRIPPER_CLOSE,
    ROBOT_WORKSPACE_MIN, ROBOT_WORKSPACE_MAX,
    H_R_V, H_R_V_star
)
from frankateach.messages import FrankaAction, FrankaState
from deoxys.utils import transform_utils


def get_relative_affine(init_affine, current_affine):
    """Calculate relative transformation between two affine matrices"""
    # Just get the relative motion in controller space
    relative_affine = pinv(init_affine) @ current_affine
    return relative_affine


class OculusVRServer:
    def __init__(self, debug=False, use_right_controller=True, ip_address=None, 
                 home_offset=[0, 0, 0], position_scale=1.0, rotation_scale=1.0):
        """
        Initialize the Oculus VR Server with direct robot control
        
        Args:
            debug: If True, only print data without controlling robot
            use_right_controller: If True, use right controller for robot control
            ip_address: IP address of Quest device (None for USB connection)
            home_offset: Offset from robot's reset position
            position_scale: Scale factor for position movements (default 1.0)
            rotation_scale: Scale factor for rotation movements (default 1.0)
        """
        self.debug = debug
        self.use_right_controller = use_right_controller
        self.running = True
        self.home_offset = np.array(home_offset, dtype=np.float64)
        self.position_scale = position_scale
        self.rotation_scale = rotation_scale
        
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
        
        # Controller selection
        self.controller_id = "r" if use_right_controller else "l"
        
        # ========== TELEOPERATION STATE ==========
        
        # Teleoperation control
        self.is_first_frame = True
        self.teleoperation_active = False
        self.init_vr_affine = None
        self.home_pos = None
        self.home_rot = None
        
        # Gripper state
        self.gripper_state = GRIPPER_OPEN
        
        # Button states
        self.grip_pressed = False
        self.grip_was_pressed = False
        
        # Default pose when controller is not tracked
        self.default_position = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.default_quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)  # w,x,y,z
        
        print("\n🎮 Oculus VR Server with Direct Robot Control")
        print(f"   Using {'RIGHT' if use_right_controller else 'LEFT'} controller")
        print(f"   Mode: {'DEBUG' if debug else 'LIVE ROBOT CONTROL'}")
        print(f"   Position scale: {position_scale:.2f}x")
        print(f"   Rotation scale: {rotation_scale:.2f}x")
        print("\n   Controls:")
        print("   - HOLD grip button: Enable teleoperation (robot follows controller)")
        print("   - RELEASE grip button: Disable teleoperation (robot stops)")
        print("   - Hold trigger: Close gripper")
        print("   - Release trigger: Open gripper")
        print("\n   Coordinate system:")
        print("   - Using raw controller coordinates:")
        print("     • Controller +X: Right")
        print("     • Controller +Y: Up")
        print("     • Controller +Z: Backward (toward user)")
        print("   - Movements are relative to initial grip position")
        print("\n   ⚠️  IMPORTANT: Robot ONLY moves while grip is HELD")
        print("\nPress Ctrl+C to exit gracefully\n")
        
    def signal_handler(self, signum, frame):
        """Handle Ctrl+C and other termination signals"""
        print(f"\n🛑 Received signal {signum}, shutting down gracefully...")
        self.stop_server()
        
    def reset_robot(self):
        """Reset robot to initial position"""
        if self.debug:
            print("🔄 [DEBUG] Would reset robot to initial position")
            return np.array([0.4, 0.0, 0.3]), np.array([1.0, 0.0, 0.0, 0.0])
        
        print("🔄 Resetting robot to initial position...")
        action = FrankaAction(
            pos=np.zeros(3),
            quat=np.zeros(4),
            gripper=self.gripper_state,
            reset=True,
            timestamp=time.time(),
        )
        
        self.action_socket.send(bytes(pickle.dumps(action, protocol=-1)))
        robot_state = pickle.loads(self.action_socket.recv())
        
        # Move to offset position
        target_pos = robot_state.pos + self.home_offset
        target_quat = robot_state.quat
        action = FrankaAction(
            pos=target_pos.flatten().astype(np.float32),
            quat=target_quat.flatten().astype(np.float32),
            gripper=self.gripper_state,
            reset=False,
            timestamp=time.time(),
        )
        
        self.action_socket.send(bytes(pickle.dumps(action, protocol=-1)))
        robot_state = pickle.loads(self.action_socket.recv())
        
        print(f"✅ Robot reset complete")
        print(f"   Position: [{robot_state.pos[0]:.6f}, {robot_state.pos[1]:.6f}, {robot_state.pos[2]:.6f}]")
        print(f"   Quaternion: [{robot_state.quat[0]:.6f}, {robot_state.quat[1]:.6f}, {robot_state.quat[2]:.6f}, {robot_state.quat[3]:.6f}]")
        
        return robot_state.pos, robot_state.quat
    
    def get_controller_affine(self, position, rotation_matrix):
        """Convert controller position and rotation to 4x4 affine matrix"""
        affine = np.eye(4, dtype=np.float64)
        affine[:3, :3] = rotation_matrix
        affine[:3, 3] = position
        return affine
    
    def create_controller_message(self, controller_position, controller_quaternion, grip_pressed, trigger_value, buttons):
        """Create controller message in the format expected by the system"""
        # Format position and rotation
        position_str = f"{controller_position[0]:.6f},{controller_position[1]:.6f},{controller_position[2]:.6f}"
        # Convert quaternion from w,x,y,z to x,y,z,w format
        rotation_str = f"{controller_quaternion[1]:.6f},{controller_quaternion[2]:.6f},{controller_quaternion[3]:.6f},{controller_quaternion[0]:.6f}"
        
        if self.use_right_controller:
            # Left controller (inactive)
            left_part = (
                f"left;"
                f"x:false;"
                f"y:false;"
                f"menu:false;"
                f"thumbstick:false;"
                f"index_trigger:0.000000;"
                f"hand_trigger:0.000000;"
                f"thumbstick_axes:0.000000,0.000000;"
                f"position:0.000000,0.000000,0.000000;"
                f"rotation:0.000000,0.000000,0.000000,1.000000;"
            )
            
            # Right controller (active)
            right_part = (
                f"right;"
                f"a:{str(buttons.get('A', False)).lower()};"
                f"b:{str(buttons.get('B', False)).lower()};"
                f"menu:false;"
                f"thumbstick:{str(buttons.get('RJ', False)).lower()};"
                f"index_trigger:{trigger_value:.6f};"
                f"hand_trigger:{1.000000 if grip_pressed else 0.000000:.6f};"
                f"thumbstick_axes:0.000000,0.000000;"
                f"position:{position_str};"
                f"rotation:{rotation_str};"
            )
        else:
            # Left controller (active)
            left_part = (
                f"left;"
                f"x:{str(buttons.get('X', False)).lower()};"
                f"y:{str(buttons.get('Y', False)).lower()};"
                f"menu:false;"
                f"thumbstick:{str(buttons.get('LJ', False)).lower()};"
                f"index_trigger:{trigger_value:.6f};"
                f"hand_trigger:{1.000000 if grip_pressed else 0.000000:.6f};"
                f"thumbstick_axes:0.000000,0.000000;"
                f"position:{position_str};"
                f"rotation:{rotation_str};"
            )
            
            # Right controller (inactive)
            right_part = (
                f"right;"
                f"a:false;"
                f"b:false;"
                f"menu:false;"
                f"thumbstick:false;"
                f"index_trigger:0.000000;"
                f"hand_trigger:0.000000;"
                f"thumbstick_axes:0.000000,0.000000;"
                f"position:0.000000,0.000000,0.000000;"
                f"rotation:0.000000,0.000000,0.000000,1.000000;"
            )
        
        return f"{left_part}|{right_part}"
    
    def process_controller_data(self, poses, buttons):
        """Process controller data and control robot"""
        
        # Get button states
        if self.use_right_controller:
            self.grip_pressed = buttons.get("RG", False)
            trigger_value = buttons.get("rightTrig", [0.0])[0]
        else:
            self.grip_pressed = buttons.get("LG", False)
            trigger_value = buttons.get("leftTrig", [0.0])[0]
        
        # Track grip state changes
        grip_just_pressed = self.grip_pressed and not self.grip_was_pressed
        grip_just_released = not self.grip_pressed and self.grip_was_pressed
        
        # Get current controller pose
        controller_position = self.default_position.copy()
        controller_quaternion = self.default_quaternion.copy()
        controller_affine = None
        
        if self.controller_id in poses:
            pose_matrix = poses[self.controller_id]
            if not np.allclose(pose_matrix, 0):
                controller_position = pose_matrix[:3, 3]
                controller_rot_matrix = pose_matrix[:3, :3]
                # Extract quaternion from rotation matrix
                rotation = R.from_matrix(controller_rot_matrix)
                quat_xyzw = rotation.as_quat()
                controller_quaternion = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])  # w,x,y,z
                controller_affine = self.get_controller_affine(controller_position, controller_rot_matrix)
        
        # Initialize robot on first frame
        if self.is_first_frame:
            self.home_pos, home_quat = self.reset_robot()
            self.home_rot = transform_utils.quat2mat(home_quat)
            self.is_first_frame = False
        
        # Handle teleoperation state changes
        if grip_just_pressed and controller_affine is not None:
            # Start teleoperation
            self.teleoperation_active = True
            self.init_vr_affine = controller_affine.copy()
            if self.debug:
                print(f"\n➡️  Teleoperation STARTED - Robot will follow controller")
                print(f"   Reference VR position: [{controller_position[0]:.3f}, {controller_position[1]:.3f}, {controller_position[2]:.3f}]")
        
        elif grip_just_released:
            # Stop teleoperation
            self.teleoperation_active = False
            self.init_vr_affine = None
            if self.debug:
                print(f"\n⏸️  Teleoperation STOPPED - Robot holds position")
        
        # Calculate target position and orientation
        if self.teleoperation_active and self.init_vr_affine is not None and controller_affine is not None:
            # Calculate relative motion using the original method
            relative_affine = get_relative_affine(self.init_vr_affine, controller_affine)
            relative_pos = relative_affine[:3, 3]
            relative_rot = relative_affine[:3, :3]
            
            # Apply scaling
            relative_pos *= self.position_scale
            
            # Scale rotation if needed
            if self.rotation_scale != 1.0:
                try:
                    # Convert to axis-angle, scale, and convert back
                    rotation = R.from_matrix(relative_rot)
                    axis_angle = rotation.as_rotvec()
                    # Scale the rotation angle
                    angle = np.linalg.norm(axis_angle)
                    if angle > 0:
                        axis = axis_angle / angle
                        scaled_angle = angle * self.rotation_scale
                        scaled_axis_angle = axis * scaled_angle
                        scaled_rotation = R.from_rotvec(scaled_axis_angle)
                        relative_rot = scaled_rotation.as_matrix()
                except Exception as e:
                    if self.debug:
                        print(f"⚠️  Error scaling rotation: {e}")
            
            # Apply relative motion to home position
            target_pos = self.home_pos + relative_pos
            target_rot = self.home_rot @ relative_rot
            target_quat = transform_utils.mat2quat(target_rot)
            
            # Apply workspace bounds
            target_pos = np.clip(target_pos, ROBOT_WORKSPACE_MIN, ROBOT_WORKSPACE_MAX)
        else:
            # Not in teleoperation - maintain current position
            target_pos = self.home_pos + self.home_offset
            target_quat = transform_utils.mat2quat(self.home_rot)
        
        # Handle gripper control
        new_gripper_state = GRIPPER_CLOSE if trigger_value > 0.1 else GRIPPER_OPEN
        if new_gripper_state != self.gripper_state:
            self.gripper_state = new_gripper_state
            if self.debug:
                print(f"\n{'🟢 Gripper CLOSED' if self.gripper_state == GRIPPER_CLOSE else '🔵 Gripper OPEN'}")
        
        # Send action to robot (or print in debug mode)
        if self.debug:
            return {
                'teleoperation_active': self.teleoperation_active,
                'target_pos': target_pos,
                'target_quat': target_quat,
                'gripper_state': self.gripper_state,
                'controller_pos': controller_position,
                'grip_pressed': self.grip_pressed,
                'trigger_value': trigger_value,
                'grip_just_pressed': grip_just_pressed,
                'grip_just_released': grip_just_released
            }
        else:
            # Send action to robot
            action = FrankaAction(
                pos=target_pos.flatten().astype(np.float32),
                quat=target_quat.flatten().astype(np.float32),
                gripper=self.gripper_state,
                reset=False,
                timestamp=time.time(),
            )
            
            self.action_socket.send(bytes(pickle.dumps(action, protocol=-1)))
            robot_state = pickle.loads(self.action_socket.recv())
            
            # Publish controller state for compatibility with existing system
            controller_msg = self.create_controller_message(
                controller_position, controller_quaternion, 
                self.grip_pressed, trigger_value, buttons
            )
            self.controller_publisher.send_string("oculus_controller")
            self.controller_publisher.send_string(controller_msg)
            
            return {
                'teleoperation_active': self.teleoperation_active,
                'robot_state': robot_state,
                'action': action,
                'controller_msg': controller_msg
            }
    
    def control_loop(self):
        """Main control loop"""
        message_count = 0
        last_debug_time = time.time()
        
        while self.running:
            try:
                # Get data from Oculus Reader
                poses, buttons = self.oculus_reader.get_transformations_and_buttons()
                
                if poses and buttons:
                    # Process controller data and control robot
                    result = self.process_controller_data(poses, buttons)
                    
                    if self.debug:
                        # Debug mode - print status
                        current_time = time.time()
                        if current_time - last_debug_time > 0.5:  # Print every 0.5 seconds
                            print(f"\n📊 Debug Data [{message_count:04d}]:")
                            print(f"   Timestamp: {current_time:.3f}")
                            
                            # Teleoperation state
                            if result['teleoperation_active']:
                                print(f"   🟢 TELEOPERATION: ACTIVE (Grip button HELD)")
                            else:
                                print(f"   🔴 TELEOPERATION: INACTIVE (Grip button RELEASED)")
                            
                            # Gripper state
                            if result['gripper_state'] == GRIPPER_CLOSE:
                                print(f"   🟡 GRIPPER: CLOSED (Trigger HELD)")
                            else:
                                print(f"   🔵 GRIPPER: OPEN (Trigger RELEASED)")
                            
                            # Controller position
                            pos = result['controller_pos']
                            print(f"   Controller Position: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
                            
                            # Target robot position (only if teleoperation active)
                            if result['teleoperation_active']:
                                tpos = result['target_pos']
                                print(f"   Target Robot Position: [{tpos[0]:.3f}, {tpos[1]:.3f}, {tpos[2]:.3f}]")
                            
                            # Button states
                            print(f"   Grip: {result['grip_pressed']}, Trigger: {result['trigger_value']:.3f}")
                            
                            last_debug_time = current_time
                    
                    message_count += 1
                
                # Always update grip state for next iteration
                self.grip_was_pressed = self.grip_pressed
                
                time.sleep(0.05)  # 20 Hz
                
            except Exception as e:
                if self.running:
                    print(f"❌ Error in control loop: {e}")
                    import traceback
                    traceback.print_exc()
                    time.sleep(1)  # Wait before retrying
    
    def start(self):
        """Start the server"""
        try:
            # Run control loop directly (no separate thread needed)
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
    parser = argparse.ArgumentParser(description='Oculus VR Server for Direct Robot Control')
    parser.add_argument('--debug', action='store_true', 
                        help='Enable debug mode (no robot control)')
    parser.add_argument('--left-controller', action='store_true',
                        help='Use left controller instead of right (default: right)')
    parser.add_argument('--ip', type=str, default=None,
                        help='IP address of Quest device (default: USB connection)')
    parser.add_argument('--home-offset', type=float, nargs=3, default=[0, 0, 0],
                        help='Home offset from reset position (x y z in meters)')
    parser.add_argument('--position-scale', type=float, default=1.0,
                        help='Scale factor for position movements (default: 1.0)')
    parser.add_argument('--rotation-scale', type=float, default=1.0,
                        help='Scale factor for rotation movements (default: 1.0)')
    
    args = parser.parse_args()
    
    # Create and start server
    server = OculusVRServer(
        debug=args.debug,
        use_right_controller=not args.left_controller,
        ip_address=args.ip,
        home_offset=args.home_offset,
        position_scale=args.position_scale,
        rotation_scale=args.rotation_scale
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