#!/usr/bin/env python3
"""
Check detailed robot state and control status.
"""

import time
import pickle
import numpy as np
from frankateach.network import create_request_socket
from frankateach.constants import HOST, CONTROL_PORT
from frankateach.messages import FrankaAction, FrankaState


def get_robot_state(socket):
    """Get current robot state."""
    try:
        socket.send(b"get_state")
        response = socket.recv()
        return pickle.loads(response)
    except Exception as e:
        print(f"Error: {e}")
        return None


def main():
    print("🤖 ROBOT STATE CHECKER")
    print("=" * 60)
    
    # Connect to robot
    try:
        action_socket = create_request_socket(HOST, CONTROL_PORT)
        print(f"✅ Connected to robot server at {HOST}:{CONTROL_PORT}")
    except Exception as e:
        print(f"❌ Failed to connect: {e}")
        return
    
    # Get multiple state readings
    print("\n📊 Robot State Information:")
    print("-" * 40)
    
    states = []
    for i in range(3):
        state = get_robot_state(action_socket)
        if state and state != b"state_error":
            states.append(state)
            print(f"\nReading {i+1}:")
            print(f"  Position: {state.pos}")
            print(f"  Quaternion: {state.quat}")
            print(f"  Gripper: {state.gripper} ({'open' if state.gripper < 0 else 'closed'})")
            time.sleep(0.5)
    
    if not states:
        print("❌ Could not get robot state")
        return
    
    # Check for movement capability
    print("\n🔍 Movement Capability Test:")
    print("-" * 40)
    
    # Try a minimal movement
    current_state = states[-1]
    test_offset = np.array([0.001, 0.0, 0.0])  # 1mm test
    target_pos = current_state.pos + test_offset
    
    action = FrankaAction(
        pos=target_pos.flatten().astype(np.float32),
        quat=current_state.quat.flatten().astype(np.float32),
        gripper=current_state.gripper,
        reset=False,
        timestamp=time.time(),
    )
    
    print("Sending 1mm test movement...")
    action_socket.send(bytes(pickle.dumps(action, protocol=-1)))
    response = action_socket.recv()
    new_state = pickle.loads(response)
    
    if new_state != b"state_error":
        movement = np.linalg.norm(new_state.pos - current_state.pos)
        print(f"  Movement detected: {movement*1000:.3f}mm")
        
        if movement < 0.0001:
            print("  ❌ No movement - Robot is LOCKED")
        else:
            print("  ✅ Robot can move")
    
    # Check position variance (is robot vibrating/moving at all?)
    print("\n📈 Position Stability Check:")
    print("-" * 40)
    
    positions = []
    print("Monitoring position for 2 seconds...")
    start_time = time.time()
    
    while time.time() - start_time < 2:
        state = get_robot_state(action_socket)
        if state and state != b"state_error":
            positions.append(state.pos)
        time.sleep(0.1)
    
    if len(positions) > 1:
        # Calculate variance
        pos_array = np.array(positions)
        variance = np.std(pos_array, axis=0) * 1000  # Convert to mm
        max_variance = np.max(variance)
        
        print(f"  Position variance (mm): X={variance[0]:.4f}, Y={variance[1]:.4f}, Z={variance[2]:.4f}")
        print(f"  Max variance: {max_variance:.4f}mm")
        
        if max_variance < 0.01:
            print("  ✅ Robot position is stable")
        else:
            print("  ⚠️  Robot position is unstable/vibrating")
    
    # Diagnosis
    print("\n🔧 DIAGNOSIS:")
    print("=" * 60)
    
    print("✅ Robot communication is working")
    print("✅ Can read robot state")
    
    if movement < 0.0001:
        print("❌ Robot is NOT moving when commanded")
        print("\n📌 POSSIBLE CAUSES:")
        print("1. Robot is in LOCKED state")
        print("2. User doesn't have control permission")
        print("3. Safety system is engaged")
        print("4. Robot is in 'Desk' control mode (not FCI)")
        print("\n🔧 TO FIX:")
        print("1. Check the robot's physical status:")
        print("   - Look at robot lights (should be white, not yellow/red)")
        print("   - Check if emergency stop is pressed")
        print("2. Try accessing Franka Desk from another computer:")
        print("   - http://192.168.1.59")
        print("   - Or use the teach pendant if available")
        print("3. Check if someone else has control of the robot")
        print("4. Try power cycling the robot (turn off/on)")
    else:
        print("✅ Robot can move!")
    
    action_socket.close()


if __name__ == "__main__":
    main() 