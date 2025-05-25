#!/usr/bin/env python3
"""
Oculus VR Server - Reads data from Meta Quest headset via Oculus Reader
and publishes it for robot teleoperation
"""

import zmq
import time
import threading
import numpy as np
import signal
import sys
import argparse
from scipy.spatial.transform import Rotation as R

# Import the Oculus Reader
from oculus_reader.reader import OculusReader


class OculusVRServer:
    def __init__(self, debug=False, use_right_controller=True, ip_address=None):
        """
        Initialize the Oculus VR Server
        
        Args:
            debug: If True, only print data without publishing to ZMQ
            use_right_controller: If True, use right controller for robot control
            ip_address: IP address of Quest device (None for USB connection)
        """
        self.debug = debug
        self.use_right_controller = use_right_controller
        self.running = True
        
        # Initialize Oculus Reader
        print("🎮 Initializing Oculus Reader...")
        try:
            self.oculus_reader = OculusReader(
                ip_address=ip_address,
                print_FPS=debug
            )
            print("✅ Oculus Reader initialized successfully")
        except Exception as e:
            print(f"❌ Failed to initialize Oculus Reader: {e}")
            sys.exit(1)
        
        # ZMQ publisher for teleop system (only if not in debug mode)
        if not self.debug:
            self.context = zmq.Context()
            self.zmq_publisher = self.context.socket(zmq.PUB)
            # Bind to the specific IP that oculus_stick.py expects
            self.zmq_publisher.bind("tcp://192.168.1.54:5555")
            print("📡 ZMQ publisher bound to tcp://192.168.1.54:5555")
        else:
            print("🐛 Debug mode enabled - not publishing to ZMQ")
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # Controller selection
        self.controller_id = "r" if use_right_controller else "l"
        
        # Relative motion tracking
        self.grip_pressed = False
        self.grip_was_pressed = False
        self.home_pose = None
        self.home_position = np.array([0.0, 0.0, 0.3])  # Default position when not gripping
        self.home_quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # Default orientation
        
        # Gripper state - default is open
        self.default_gripper_value = 0.0  # 0.0 = open, 1.0 = closed
        
        print("\n🎮 Oculus VR Server for Robot Control started")
        print(f"   Using {'RIGHT' if use_right_controller else 'LEFT'} controller")
        print("   Controls:")
        print("   - Hold grip button: Enable robot control (sets home position)")
        print("   - Move controller while gripping: Control robot position (relative to grip point)")
        print("   - Hold trigger: Close gripper (release to open)")
        print("   - A/X button: Mark success and stop")
        print("   - B/Y button: Mark failure and stop")
        print("   - Joystick click: Reset orientation")
        print("\nPress Ctrl+C to exit gracefully\n")
        
    def signal_handler(self, signum, frame):
        """Handle Ctrl+C and other termination signals"""
        print(f"\n🛑 Received signal {signum}, shutting down gracefully...")
        self.stop_server()
        
    def quaternion_from_matrix(self, matrix):
        """Convert rotation matrix to quaternion (w, x, y, z)"""
        rotation = R.from_matrix(matrix[:3, :3])
        # Returns quaternion in scalar-last format (x, y, z, w)
        quat_xyzw = rotation.as_quat()
        # Convert to scalar-first format (w, x, y, z)
        return np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])
    
    def create_controller_message(self, poses, buttons):
        """
        Create controller message in the format expected by teleop.py
        
        Format: "left;button1:val;button2:val;...;position:x,y,z;rotation:w,x,y,z;|right;..."
        """
        
        # Get grip button state
        if self.use_right_controller:
            self.grip_pressed = buttons.get("RG", False)
            trigger_value = buttons.get("rightTrig", [0.0])[0]
        else:
            self.grip_pressed = buttons.get("LG", False)
            trigger_value = buttons.get("leftTrig", [0.0])[0]
        
        # Binary gripper control - trigger must be held to close gripper
        # Gripper is open (0.0) by default, closed (1.0) only while trigger is pressed
        gripper_value = 1.0 if trigger_value > 0.1 else 0.0  # Small threshold to avoid noise
        
        # Get current controller pose
        if self.controller_id in poses:
            current_pose_matrix = poses[self.controller_id]
            current_position = current_pose_matrix[:3, 3]
            current_quaternion = self.quaternion_from_matrix(current_pose_matrix)
        else:
            # Default pose if controller not tracked
            current_position = self.home_position.copy()
            current_quaternion = self.home_quaternion.copy()
        
        # Handle grip press/release for relative motion
        if self.grip_pressed and not self.grip_was_pressed:
            # Grip just pressed - save current pose as home
            self.home_pose = current_pose_matrix.copy() if self.controller_id in poses else None
            if self.debug:
                print(f"\n🏠 Home position set at: [{current_position[0]:.3f}, {current_position[1]:.3f}, {current_position[2]:.3f}]")
        
        # Calculate position and rotation to send
        if self.grip_pressed and self.home_pose is not None:
            # Grip is held - calculate relative motion from home pose
            if self.controller_id in poses:
                # Calculate relative transformation
                relative_transform = np.linalg.inv(self.home_pose) @ current_pose_matrix
                
                # Use relative position offset added to default home position
                position = self.home_position + relative_transform[:3, 3]
                quaternion = self.quaternion_from_matrix(relative_transform)
            else:
                position = self.home_position.copy()
                quaternion = self.home_quaternion.copy()
        else:
            # Grip not pressed - return to default home position
            position = self.home_position.copy()
            quaternion = self.home_quaternion.copy()
        
        # Update grip state for next iteration
        self.grip_was_pressed = self.grip_pressed
        
        # Get button states
        if self.use_right_controller:
            # Right controller buttons
            a_button = buttons.get("A", False)
            b_button = buttons.get("B", False)
            menu_button = False  # Not available in Oculus Reader
            thumbstick_button = buttons.get("RJ", False)
            index_trigger = gripper_value  # Binary gripper control - always active
            hand_trigger = 1.0  # Always 1.0 so teleop processes our commands
            thumbstick_axes = "0.0,0.0"  # Not available in Oculus Reader
        else:
            # Left controller buttons
            a_button = buttons.get("X", False)
            b_button = buttons.get("Y", False)
            menu_button = False  # Not available in Oculus Reader
            thumbstick_button = buttons.get("LJ", False)
            index_trigger = gripper_value  # Binary gripper control - always active
            hand_trigger = 1.0  # Always 1.0 so teleop processes our commands
            thumbstick_axes = "0.0,0.0"  # Not available in Oculus Reader
        
        # Format position and rotation
        position_str = f"{position[0]:.6f},{position[1]:.6f},{position[2]:.6f}"
        rotation_str = f"{quaternion[0]:.6f},{quaternion[1]:.6f},{quaternion[2]:.6f},{quaternion[3]:.6f}"
        
        # Create message for both controllers (inactive controller has default values)
        if self.use_right_controller:
            # Left controller (inactive)
            left_part = (
                f"left;"
                f"x:false;"
                f"y:false;"
                f"menu:false;"
                f"thumbstick:false;"
                f"index_trigger:0.0;"
                f"hand_trigger:0.0;"
                f"thumbstick_axes:0.0,0.0;"
                f"position:0.0,0.0,0.0;"
                f"rotation:1.0,0.0,0.0,0.0;"
            )
            
            # Right controller (active)
            right_part = (
                f"right;"
                f"a:{str(a_button).lower()};"
                f"b:{str(b_button).lower()};"
                f"menu:{str(menu_button).lower()};"
                f"thumbstick:{str(thumbstick_button).lower()};"
                f"index_trigger:{index_trigger:.6f};"
                f"hand_trigger:{hand_trigger:.6f};"
                f"thumbstick_axes:{thumbstick_axes};"
                f"position:{position_str};"
                f"rotation:{rotation_str};"
            )
        else:
            # Left controller (active)
            left_part = (
                f"left;"
                f"x:{str(a_button).lower()};"
                f"y:{str(b_button).lower()};"
                f"menu:{str(menu_button).lower()};"
                f"thumbstick:{str(thumbstick_button).lower()};"
                f"index_trigger:{index_trigger:.6f};"
                f"hand_trigger:{hand_trigger:.6f};"
                f"thumbstick_axes:{thumbstick_axes};"
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
                f"index_trigger:0.0;"
                f"hand_trigger:0.0;"
                f"thumbstick_axes:0.0,0.0;"
                f"position:0.0,0.0,0.0;"
                f"rotation:1.0,0.0,0.0,0.0;"
            )
        
        return f"{left_part}|{right_part}"
    
    def publish_loop(self):
        """Continuously read from Oculus and publish/debug data"""
        message_count = 0
        last_debug_time = time.time()
        
        while self.running:
            try:
                # Get data from Oculus Reader
                poses, buttons = self.oculus_reader.get_transformations_and_buttons()
                
                if poses and buttons:
                    # Create controller message
                    controller_text = self.create_controller_message(poses, buttons)
                    
                    if self.debug:
                        # Debug mode - print data
                        current_time = time.time()
                        if current_time - last_debug_time > 0.5:  # Print every 0.5 seconds
                            # Get trigger value for debug display
                            if self.use_right_controller:
                                trigger_value = buttons.get("rightTrig", [0.0])[0]
                            else:
                                trigger_value = buttons.get("leftTrig", [0.0])[0]
                            gripper_state = "CLOSED" if trigger_value > 0.1 else "OPEN"
                            
                            print(f"\n📊 Debug Data [{message_count:04d}]:")
                            print(f"   Timestamp: {current_time:.3f}")
                            
                            # Print grip state
                            print(f"   Grip: {'PRESSED (Robot Active)' if self.grip_pressed else 'Released (Robot Idle)'}")
                            
                            # Print controller poses
                            for controller_id, pose in poses.items():
                                pos = pose[:3, 3]
                                print(f"   {controller_id.upper()} Controller (raw):")
                                print(f"     Position: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
                            
                            # Print relative position if grip is pressed
                            if self.grip_pressed and self.home_pose is not None:
                                print(f"   Robot command position: {controller_text.split('position:')[1].split(';')[0]}")
                            
                            # Print button states
                            print(f"   Buttons:")
                            print(f"     A: {buttons.get('A', False)}, B: {buttons.get('B', False)}")
                            print(f"     X: {buttons.get('X', False)}, Y: {buttons.get('Y', False)}")
                            print(f"     RG: {buttons.get('RG', False)}, LG: {buttons.get('LG', False)}")
                            print(f"     RJ: {buttons.get('RJ', False)}, LJ: {buttons.get('LJ', False)}")
                            print(f"     Right Trigger (Gripper): {buttons.get('rightTrig', [0.0])[0]:.3f} -> {gripper_state}")
                            print(f"     Left Trigger: {buttons.get('leftTrig', [0.0])[0]:.3f}")
                            
                            # Print formatted message snippet
                            if len(controller_text) > 150:
                                snippet = controller_text[:150] + "..."
                            else:
                                snippet = controller_text
                            print(f"   Message: {snippet}")
                            
                            # Print the active controller's gripper value from the message
                            if self.use_right_controller:
                                # Extract right controller's index_trigger value
                                right_part = controller_text.split('|')[1]
                                if 'index_trigger:' in right_part:
                                    trigger_in_msg = right_part.split('index_trigger:')[1].split(';')[0]
                                    print(f"   Active controller gripper in message: {trigger_in_msg}")
                                if 'hand_trigger:' in right_part:
                                    hand_trigger_in_msg = right_part.split('hand_trigger:')[1].split(';')[0]
                                    print(f"   Active controller hand_trigger in message: {hand_trigger_in_msg}")
                            else:
                                # Extract left controller's index_trigger value
                                left_part = controller_text.split('|')[0]
                                if 'index_trigger:' in left_part:
                                    trigger_in_msg = left_part.split('index_trigger:')[1].split(';')[0]
                                    print(f"   Active controller gripper in message: {trigger_in_msg}")
                                if 'hand_trigger:' in left_part:
                                    hand_trigger_in_msg = left_part.split('hand_trigger:')[1].split(';')[0]
                                    print(f"   Active controller hand_trigger in message: {hand_trigger_in_msg}")
                            
                            last_debug_time = current_time
                    else:
                        # Normal mode - publish to ZMQ
                        # First send topic
                        self.zmq_publisher.send_string("oculus_controller")
                        # Then send controller data
                        self.zmq_publisher.send_string(controller_text)
                    
                    message_count += 1
                
                time.sleep(0.05)  # 20 Hz
                
            except Exception as e:
                if self.running:
                    print(f"❌ Error in publish loop: {e}")
                    time.sleep(1)  # Wait before retrying
    
    def start(self):
        """Start the server"""
        try:
            # Start publishing thread
            self.publish_thread = threading.Thread(target=self.publish_loop, daemon=True)
            self.publish_thread.start()
            
            # Keep main thread alive
            while self.running:
                time.sleep(1)
                
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
        
        # Close ZMQ resources
        if not self.debug and hasattr(self, 'zmq_publisher'):
            try:
                self.zmq_publisher.close()
                self.context.term()
                print("✅ ZMQ resources closed")
            except Exception as e:
                print(f"⚠️  Error closing ZMQ: {e}")
        
        print("✅ Server stopped gracefully")
        sys.exit(0)


def main():
    parser = argparse.ArgumentParser(description='Oculus VR Server for Robot Control')
    parser.add_argument('--debug', action='store_true', 
                        help='Enable debug mode (print data without publishing)')
    parser.add_argument('--left-controller', action='store_true',
                        help='Use left controller instead of right (default: right)')
    parser.add_argument('--ip', type=str, default=None,
                        help='IP address of Quest device (default: USB connection)')
    
    args = parser.parse_args()
    
    # Create and start server
    server = OculusVRServer(
        debug=args.debug,
        use_right_controller=not args.left_controller,
        ip_address=args.ip
    )
    
    try:
        server.start()
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        server.stop_server()


if __name__ == "__main__":
    main() 