#!/usr/bin/env python3
"""
Debug Teleop - Shows exactly what's happening in the teleop loop
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import time
import pickle
from frankateach.utils import notify_component_start
from frankateach.network import (
    ZMQKeypointSubscriber,
    create_request_socket,
    ZMQKeypointPublisher,
)
from frankateach.constants import (
    COMMANDED_STATE_PORT,
    CONTROL_PORT,
    HOST,
    STATE_PORT,
    VR_CONTROLLER_STATE_PORT,
    GRIPPER_OPEN,
    GRIPPER_CLOSE,
)
from frankateach.messages import FrankaAction, FrankaState

class DebugFrankaOperator:
    def __init__(self, teleop_mode="human"):
        print("🔧 Initializing DebugFrankaOperator...")
        
        # Subscribe controller state
        print("🔧 Creating controller state subscriber...")
        self._controller_state_subscriber = ZMQKeypointSubscriber(
            host=HOST, port=VR_CONTROLLER_STATE_PORT, topic="controller_state"
        )
        print("✅ Controller state subscriber created")

        print("🔧 Creating action socket...")
        self.action_socket = create_request_socket(HOST, CONTROL_PORT)
        print("✅ Action socket created")
        
        print("🔧 Creating state publishers...")
        self.state_socket = ZMQKeypointPublisher(HOST, STATE_PORT)
        self.commanded_state_socket = ZMQKeypointPublisher(HOST, COMMANDED_STATE_PORT)
        print("✅ State publishers created")

        # Class variables
        self.is_first_frame = True
        self.gripper_state = GRIPPER_OPEN
        self.start_teleop = False
        self.init_affine = None
        self.teleop_mode = teleop_mode
        self.home_offset = [-0.22, 0.0, 0.1] if teleop_mode == "human" else [0, 0, 0]
        
        print(f"✅ DebugFrankaOperator initialized for {teleop_mode} mode")

    def debug_apply_retargeted_angles(self):
        print(f"\n🔄 [Frame] Starting _apply_retargeted_angles (first_frame: {self.is_first_frame})")
        
        # Try to receive controller state
        print("📡 Receiving controller state...")
        try:
            self.controller_state = self._controller_state_subscriber.recv_keypoints()
            print(f"✅ Received controller state:")
            print(f"   Right A: {self.controller_state.right_a}")
            print(f"   Right B: {self.controller_state.right_b}")
            print(f"   Right position: {self.controller_state.right_local_position}")
        except Exception as e:
            print(f"❌ Error receiving controller state: {e}")
            return

        if self.is_first_frame:
            print("🏠 First frame - performing reset sequence...")
            
            # Reset robot
            print("🔄 Sending reset action...")
            action = FrankaAction(
                pos=np.zeros(3),
                quat=np.zeros(4),
                gripper=self.gripper_state,
                reset=True,
                timestamp=time.time(),
            )
            self.action_socket.send(bytes(pickle.dumps(action, protocol=-1)))
            robot_state = pickle.loads(self.action_socket.recv())
            print(f"✅ Reset complete: {robot_state}")

            # Move to offset position
            print("🎯 Moving to offset position...")
            import numpy as np
            target_pos = robot_state.pos + np.array(self.home_offset)
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
            print(f"✅ Moved to home position: {robot_state}")

            # Store home position
            from deoxys.utils import transform_utils
            self.home_rot, self.home_pos = (
                transform_utils.quat2mat(robot_state.quat),
                robot_state.pos,
            )
            print(f"🏠 Home position stored: {self.home_pos}")
            
            self.is_first_frame = False
            print("✅ First frame complete, entering main loop...")

        # Check button states
        print(f"🎮 Button states - A: {self.controller_state.right_a}, B: {self.controller_state.right_b}")
        
        if self.controller_state.right_a:
            print("🟢 A button pressed - Starting teleop!")
            self.start_teleop = True
            self.init_affine = self.controller_state.right_affine
            
        if self.controller_state.right_b:
            print("🔴 B button pressed - Stopping teleop!")
            self.start_teleop = False
            self.init_affine = None
            
            # Get current robot state
            self.action_socket.send(b"get_state")
            robot_state = pickle.loads(self.action_socket.recv())
            if robot_state != b"state_error":
                from deoxys.utils import transform_utils
                self.home_rot, self.home_pos = (
                    transform_utils.quat2mat(robot_state.quat),
                    robot_state.pos,
                )

        print(f"📊 Teleop status: {self.start_teleop}")

        # In human mode, always send get_state
        if self.teleop_mode == "human":
            print("👤 Human mode - sending get_state request...")
            self.action_socket.send(b"get_state")
        else:
            print("🤖 Robot mode - would send movement commands...")
            # Would send actual movement commands in robot mode

        # Receive robot state
        print("📥 Receiving robot state...")
        robot_state = self.action_socket.recv()
        robot_state = pickle.loads(robot_state)
        robot_state.start_teleop = self.start_teleop
        
        print(f"✅ Robot state received: start_teleop={robot_state.start_teleop}")
        
        # Publish states
        print("📤 Publishing states...")
        self.state_socket.pub_keypoints(robot_state, "robot_state")
        
        # Create dummy action for commanded state
        import numpy as np
        from deoxys.utils import transform_utils
        dummy_action = FrankaAction(
            pos=self.home_pos.flatten().astype(np.float32),
            quat=transform_utils.mat2quat(self.home_rot).flatten().astype(np.float32),
            gripper=self.gripper_state,
            reset=False,
            timestamp=time.time(),
        )
        self.commanded_state_socket.pub_keypoints(dummy_action, "commanded_robot_state")
        print("✅ States published")

    def debug_stream(self):
        notify_component_start("Debug Franka teleoperator control")
        print("🚀 Starting debug teleop stream...")
        print("🎮 Use mouse VR server to control:")
        print("   - Right click: A button (start/stop recording)")
        print("   - Left click + move: Hand tracking")
        print("   - Press Ctrl+C to stop")
        print()

        loop_count = 0
        try:
            while True:
                loop_count += 1
                print(f"\n{'='*60}")
                print(f"🔄 LOOP {loop_count}")
                print(f"{'='*60}")
                
                # Call the debug version
                self.debug_apply_retargeted_angles()
                
                print(f"✅ Loop {loop_count} complete")
                time.sleep(0.1)  # Small delay to make output readable
                
        except KeyboardInterrupt:
            print("\n🛑 Debug teleop stopped by user")
        except Exception as e:
            print(f"\n❌ Error in debug teleop: {e}")
            import traceback
            traceback.print_exc()
        finally:
            print("🧹 Cleaning up...")
            self._controller_state_subscriber.stop()
            self.action_socket.close()
            print("✅ Cleanup complete")

def main():
    import numpy as np  # Import here to avoid issues
    
    print("🐛 Starting Debug Teleop for Human Mode")
    print("=" * 50)
    
    operator = DebugFrankaOperator(teleop_mode="human")
    operator.debug_stream()

if __name__ == "__main__":
    main() 