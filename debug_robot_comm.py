#!/usr/bin/env python3
"""
Debug robot communication and command flow with detailed logging.
"""

import time
import pickle
import numpy as np
import sys
import threading
from datetime import datetime
from frankateach.network import create_request_socket
from frankateach.constants import HOST, CONTROL_PORT
from frankateach.messages import FrankaAction, FrankaState


class RobotDebugger:
    def __init__(self):
        self.log_file = open("robot_comm_debug.log", "w")
        self.log(f"🔍 ROBOT COMMUNICATION DEBUGGER STARTED")
        self.log(f"Connecting to {HOST}:{CONTROL_PORT}")
        
        try:
            self.action_socket = create_request_socket(HOST, CONTROL_PORT)
            self.log(f"✅ Connected to robot server")
            self.connected = True
        except Exception as e:
            self.log(f"❌ Failed to connect: {e}")
            self.connected = False
            
    def log(self, message):
        """Log message with timestamp to both console and file."""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        log_entry = f"[{timestamp}] {message}"
        print(log_entry)
        self.log_file.write(log_entry + "\n")
        self.log_file.flush()
    
    def monitor_state(self):
        """Continuously monitor robot state in background."""
        if not self.connected:
            return
            
        self.log("Starting state monitoring thread...")
        
        def state_monitor():
            last_pos = None
            while self.monitoring:
                try:
                    self.action_socket.send(b"get_state")
                    response = self.action_socket.recv()
                    state = pickle.loads(response)
                    
                    if state != b"state_error":
                        if last_pos is not None:
                            movement = np.linalg.norm(state.pos - last_pos)
                            if movement > 0.0001:  # 0.1mm threshold
                                self.log(f"🚨 ROBOT MOVED! Distance: {movement*1000:.2f}mm")
                        
                        last_pos = state.pos.copy()
                        
                except Exception as e:
                    self.log(f"❌ State monitor error: {e}")
                    
                time.sleep(0.1)  # 10Hz monitoring
        
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=state_monitor)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
    
    def send_test_command(self, movement_type="small"):
        """Send a test movement command and log everything."""
        if not self.connected:
            self.log("❌ Not connected, cannot send command")
            return
            
        try:
            # Get current state
            self.log("📊 Getting current robot state...")
            self.action_socket.send(b"get_state")
            response = self.action_socket.recv()
            current_state = pickle.loads(response)
            
            if current_state == b"state_error":
                self.log("❌ Could not get robot state")
                return
                
            self.log(f"   Current position: {current_state.pos}")
            self.log(f"   Current gripper: {current_state.gripper}")
            
            # Prepare movement
            if movement_type == "small":
                offset = np.array([0.005, 0.0, 0.0])  # 5mm in X
            elif movement_type == "medium":
                offset = np.array([0.02, 0.0, 0.0])   # 20mm in X
            else:
                offset = np.array([0.0, 0.0, 0.0])    # No movement
                
            target_pos = current_state.pos + offset
            
            # Create action
            action = FrankaAction(
                pos=target_pos.flatten().astype(np.float32),
                quat=current_state.quat.flatten().astype(np.float32),
                gripper=current_state.gripper,
                reset=False,
                timestamp=time.time(),
            )
            
            self.log(f"📤 SENDING COMMAND:")
            self.log(f"   Target position: {target_pos}")
            self.log(f"   Expected movement: {np.linalg.norm(offset)*1000:.1f}mm")
            self.log(f"   Command timestamp: {action.timestamp}")
            
            # Send command
            start_time = time.time()
            self.action_socket.send(bytes(pickle.dumps(action, protocol=-1)))
            
            # Get response
            response = self.action_socket.recv()
            end_time = time.time()
            
            result_state = pickle.loads(response)
            
            self.log(f"📥 RECEIVED RESPONSE:")
            self.log(f"   Round trip time: {(end_time - start_time)*1000:.1f}ms")
            self.log(f"   New position: {result_state.pos}")
            
            # Check movement
            actual_movement = np.linalg.norm(result_state.pos - current_state.pos)
            self.log(f"   Actual movement: {actual_movement*1000:.3f}mm")
            
            if actual_movement > 0.001:  # 1mm threshold
                self.log(f"✅ ROBOT MOVED SUCCESSFULLY!")
            else:
                self.log(f"⚠️  NO SIGNIFICANT MOVEMENT DETECTED")
                
        except Exception as e:
            self.log(f"❌ ERROR sending command: {e}")
            import traceback
            self.log(f"   Traceback: {traceback.format_exc()}")
    
    def test_communication_loop(self, count=5):
        """Test communication with multiple commands."""
        self.log(f"\n🔄 TESTING COMMUNICATION LOOP ({count} iterations)")
        self.log("=" * 60)
        
        for i in range(count):
            self.log(f"\n--- Iteration {i+1}/{count} ---")
            self.send_test_command("small")
            time.sleep(1)
    
    def interactive_debug(self):
        """Interactive debugging session."""
        self.log("\n🎮 INTERACTIVE DEBUG MODE")
        self.log("Commands: 's' (small move), 'm' (medium move), 'g' (get state), 'q' (quit)")
        
        self.monitor_state()  # Start background monitoring
        
        while True:
            try:
                cmd = input("\nCommand> ").strip().lower()
                
                if cmd == 'q':
                    break
                elif cmd == 's':
                    self.send_test_command("small")
                elif cmd == 'm':
                    self.send_test_command("medium")
                elif cmd == 'g':
                    self.action_socket.send(b"get_state")
                    response = self.action_socket.recv()
                    state = pickle.loads(response)
                    if state != b"state_error":
                        self.log(f"Current state: pos={state.pos}, gripper={state.gripper}")
                    else:
                        self.log("State error")
                else:
                    self.log("Unknown command")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.log(f"Error: {e}")
    
    def close(self):
        """Clean up resources."""
        self.monitoring = False
        if hasattr(self, 'monitor_thread'):
            self.monitor_thread.join(timeout=1)
        if self.connected:
            self.action_socket.close()
        self.log_file.close()


def main():
    """Main function."""
    print("🔍 ROBOT COMMUNICATION DEBUGGER")
    print("This tool helps debug robot command flow and logging")
    print("=" * 60)
    
    debugger = RobotDebugger()
    
    if not debugger.connected:
        print("\n❌ Cannot connect to robot server")
        print("Make sure franka_server.py is running!")
        return
    
    # Check command line arguments
    if len(sys.argv) > 1 and sys.argv[1] == "--auto":
        # Automatic test mode
        debugger.test_communication_loop(count=3)
    else:
        # Interactive mode
        debugger.interactive_debug()
    
    debugger.close()
    print("\n📋 Debug log saved to: robot_comm_debug.log")


if __name__ == "__main__":
    main() 