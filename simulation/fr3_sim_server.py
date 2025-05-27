"""
FR3 Simulated Robot Server
Provides the same socket interface as the real robot server
"""

import zmq
import pickle
import time
import threading
import numpy as np
from typing import Optional

from frankateach.messages import FrankaAction, FrankaState
from frankateach.constants import (
    HOST, CONTROL_PORT, STATE_PORT,
    GRIPPER_OPEN, GRIPPER_CLOSE,
    ROBOT_WORKSPACE_MIN, ROBOT_WORKSPACE_MAX
)
from .fr3_sim_controller import FR3SimController


class FR3SimServer:
    """
    Simulated robot server that mimics the real Franka server interface
    """
    
    def __init__(self, visualize: bool = True):
        """
        Initialize simulated robot server
        
        Args:
            visualize: Whether to show 3D visualization
        """
        self.visualize = visualize
        self.running = False
        
        # Initialize simulated robot controller
        self.sim_controller = FR3SimController(visualize=visualize)
        
        # ZMQ context and sockets
        self.context = zmq.Context()
        self.control_socket = None
        self.state_socket = None
        
        # Server threads
        self.control_thread = None
        self.state_thread = None
        
        # Current robot state
        self.current_pos = np.zeros(3)
        self.current_quat = np.array([0, 0, 0, 1])
        self.current_gripper = GRIPPER_OPEN
        
        print("🤖 FR3 Simulation Server initialized")
        
    def start(self):
        """Start the simulation server"""
        if self.running:
            print("⚠️  Server already running")
            return
            
        self.running = True
        
        # Start simulation controller
        self.sim_controller.start()
        
        # Create sockets
        self.control_socket = self.context.socket(zmq.REP)
        self.control_socket.bind(f"tcp://{HOST}:{CONTROL_PORT}")
        print(f"📡 Control socket bound to tcp://{HOST}:{CONTROL_PORT}")
        
        self.state_socket = self.context.socket(zmq.PUB)
        self.state_socket.bind(f"tcp://{HOST}:{STATE_PORT}")
        print(f"📡 State publisher bound to tcp://{HOST}:{STATE_PORT}")
        
        # Start server threads
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        self.state_thread = threading.Thread(target=self._state_publisher_loop)
        self.state_thread.daemon = True
        self.state_thread.start()
        
        print("✅ FR3 Simulation Server started")
        print("   - Control commands on port", CONTROL_PORT)
        print("   - State publishing on port", STATE_PORT)
        print("   - Visualization:", "ENABLED" if self.visualize else "DISABLED")
        
    def stop(self):
        """Stop the simulation server"""
        if not self.running:
            return
            
        print("🛑 Stopping FR3 Simulation Server...")
        self.running = False
        
        # Stop threads
        if self.control_thread:
            self.control_thread.join(timeout=1.0)
        if self.state_thread:
            self.state_thread.join(timeout=1.0)
            
        # Close sockets
        if self.control_socket:
            self.control_socket.close()
        if self.state_socket:
            self.state_socket.close()
            
        # Stop simulation controller
        self.sim_controller.stop()
        
        # Terminate context
        self.context.term()
        
        print("✅ Server stopped")
        
    def _control_loop(self):
        """Handle control commands from clients"""
        while self.running:
            try:
                # Set timeout to allow checking running flag
                if self.control_socket.poll(timeout=100):
                    # Receive control command
                    message = self.control_socket.recv()
                    action = pickle.loads(message)
                    
                    if isinstance(action, FrankaAction):
                        # Process action
                        state = self._process_action(action)
                        
                        # Send response
                        response = pickle.dumps(state, protocol=-1)
                        self.control_socket.send(response)
                    else:
                        print(f"⚠️  Received unknown action type: {type(action)}")
                        # Send current state as response
                        state = self._get_current_state()
                        response = pickle.dumps(state, protocol=-1)
                        self.control_socket.send(response)
                        
            except Exception as e:
                if self.running:
                    print(f"❌ Error in control loop: {e}")
                    import traceback
                    traceback.print_exc()
                    
    def _state_publisher_loop(self):
        """Publish robot state at regular intervals"""
        publish_rate = 100  # Hz
        publish_interval = 1.0 / publish_rate
        last_publish_time = time.time()
        
        while self.running:
            current_time = time.time()
            
            if current_time - last_publish_time >= publish_interval:
                # Get current state
                state = self._get_current_state()
                
                # Publish state
                try:
                    state_bytes = pickle.dumps(state, protocol=-1)
                    self.state_socket.send(state_bytes)
                except Exception as e:
                    if self.running:
                        print(f"❌ Error publishing state: {e}")
                
                last_publish_time = current_time
                
            # Small sleep to prevent CPU spinning
            time.sleep(0.001)
            
    def _process_action(self, action: FrankaAction) -> FrankaState:
        """
        Process control action and return current state
        
        Args:
            action: FrankaAction command
            
        Returns:
            Current robot state
        """
        if action.reset:
            # Reset to home position
            print("🏠 Resetting robot to home position")
            pos, quat = self.sim_controller.reset_to_home()
            self.current_pos = pos
            self.current_quat = quat
            self.current_gripper = GRIPPER_OPEN
        else:
            # Apply workspace limits
            target_pos = np.clip(action.pos, ROBOT_WORKSPACE_MIN, ROBOT_WORKSPACE_MAX)
            
            # Convert gripper command to 0-1 range
            target_gripper = 1.0 if action.gripper == GRIPPER_CLOSE else 0.0
            
            # Send target to simulation controller
            self.sim_controller.set_target_pose(
                target_pos,
                action.quat,
                target_gripper
            )
            
            # Get current state from simulation
            pos, quat, gripper = self.sim_controller.get_state()
            self.current_pos = pos
            self.current_quat = quat
            self.current_gripper = GRIPPER_CLOSE if gripper > 0.5 else GRIPPER_OPEN
            
        return self._get_current_state()
        
    def _get_current_state(self) -> FrankaState:
        """Get current robot state"""
        # Get state from simulation controller
        pos, quat, gripper = self.sim_controller.get_state()
        
        # Convert gripper state to discrete open/close
        gripper_state = GRIPPER_CLOSE if gripper > 0.5 else GRIPPER_OPEN
        
        return FrankaState(
            pos=pos,
            quat=quat,
            gripper=np.array([gripper_state]),
            timestamp=time.time(),
            start_teleop=False
        )
        
    def run_forever(self):
        """Run server until interrupted"""
        try:
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n⌨️  Keyboard interrupt received")
        finally:
            self.stop()


def main():
    """Main entry point for standalone server"""
    import argparse
    
    parser = argparse.ArgumentParser(description='FR3 Simulated Robot Server')
    parser.add_argument('--no-viz', action='store_true',
                        help='Disable 3D visualization')
    args = parser.parse_args()
    
    # Create and start server
    server = FR3SimServer(visualize=not args.no_viz)
    server.start()
    
    print("\n🎮 FR3 Simulation Server is running")
    print("Press Ctrl+C to stop\n")
    
    # Run until interrupted
    server.run_forever()


if __name__ == "__main__":
    main() 