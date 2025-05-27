#!/usr/bin/env python3
"""
Debug deoxys command handling to understand why it crashes.
"""

import time
import pickle
import numpy as np
from frankateach.network import create_request_socket
from frankateach.constants import HOST, CONTROL_PORT, GRIPPER_OPEN
from frankateach.messages import FrankaAction, FrankaState


def test_minimal_command():
    """Test the most minimal command possible."""
    print("🔍 DEOXYS COMMAND DEBUG")
    print("=" * 60)
    print("This will send minimal commands to test deoxys stability")
    print()
    
    # Wait for deoxys to be ready
    print("⏱️  Make sure deoxys is running and shows:")
    print("   'Initialize Pose interpolator!'")
    print("   Then press Enter to continue...")
    input()
    
    # Connect
    print("Connecting to robot server...")
    try:
        socket = create_request_socket(HOST, CONTROL_PORT)
        print(f"✅ Connected to {HOST}:{CONTROL_PORT}")
    except Exception as e:
        print(f"❌ Failed to connect: {e}")
        return
    
    # Test 1: Get state only
    print("\n" + "="*40)
    print("TEST 1: Get State Only")
    print("="*40)
    print("Sending get_state command...")
    
    try:
        socket.send(b"get_state")
        response = socket.recv()
        state = pickle.loads(response)
        
        if state != b"state_error":
            print(f"✅ Got state successfully!")
            print(f"   Position: {state.pos}")
            print(f"   Quaternion: {state.quat}")
            print(f"   Gripper: {state.gripper}")
        else:
            print("❌ State error")
            return
    except Exception as e:
        print(f"❌ Error getting state: {e}")
        return
    
    # Wait before next test
    print("\n⏸️  Waiting 2 seconds before movement test...")
    time.sleep(2)
    
    # Test 2: Send reset command
    print("\n" + "="*40)
    print("TEST 2: Reset Command")
    print("="*40)
    print("Sending reset command (should move to default joints)...")
    print("Watch deoxys output for crash!")
    
    reset_action = FrankaAction(
        pos=np.zeros(3),
        quat=np.zeros(4),
        gripper=GRIPPER_OPEN,
        reset=True,
        timestamp=time.time(),
    )
    
    try:
        socket.send(bytes(pickle.dumps(reset_action, protocol=-1)))
        print("✅ Reset command sent, waiting for response...")
        
        response = socket.recv()
        result = pickle.loads(response)
        
        if result != b"state_error":
            print(f"✅ Reset successful!")
            print(f"   New position: {result.pos}")
        else:
            print("❌ Reset failed with state_error")
    except Exception as e:
        print(f"❌ Error during reset: {e}")
        print("   Deoxys likely crashed - check terminal!")
        return
    
    # If we got here, deoxys didn't crash on reset
    print("\n⏸️  Waiting 3 seconds...")
    time.sleep(3)
    
    # Test 3: Send movement command with current position
    print("\n" + "="*40)
    print("TEST 3: No-Op Movement")
    print("="*40)
    print("Sending command to stay at current position...")
    
    # Get current state first
    socket.send(b"get_state")
    response = socket.recv()
    current_state = pickle.loads(response)
    
    if current_state != b"state_error":
        # Send command to stay at same position
        noop_action = FrankaAction(
            pos=current_state.pos,
            quat=current_state.quat,
            gripper=current_state.gripper,
            reset=False,
            timestamp=time.time(),
        )
        
        try:
            socket.send(bytes(pickle.dumps(noop_action, protocol=-1)))
            print("✅ No-op command sent, waiting for response...")
            
            response = socket.recv()
            result = pickle.loads(response)
            
            if result != b"state_error":
                print(f"✅ No-op successful!")
            else:
                print("❌ No-op failed")
        except Exception as e:
            print(f"❌ Error during no-op: {e}")
            print("   Deoxys likely crashed!")
    
    print("\n" + "="*60)
    print("DIAGNOSIS:")
    print("="*60)
    print("Check which test caused deoxys to crash:")
    print("- If crashed on TEST 1: Basic communication issue")
    print("- If crashed on TEST 2: Reset command issue")  
    print("- If crashed on TEST 3: Movement command issue")
    print("\nAlso check deoxys terminal for specific error messages!")
    
    socket.close()


def test_command_format():
    """Test if command format is correct."""
    print("\n🔍 COMMAND FORMAT TEST")
    print("=" * 60)
    
    # Create a sample action
    action = FrankaAction(
        pos=np.array([0.4, 0.0, 0.3]),
        quat=np.array([0.7071, 0.7071, 0.0, 0.0]),
        gripper=-1.0,
        reset=False,
        timestamp=time.time(),
    )
    
    print("Sample FrankaAction:")
    print(f"  pos shape: {action.pos.shape}, dtype: {action.pos.dtype}")
    print(f"  quat shape: {action.quat.shape}, dtype: {action.quat.dtype}")
    print(f"  gripper: {action.gripper} (type: {type(action.gripper)})")
    print(f"  reset: {action.reset}")
    print(f"  timestamp: {action.timestamp}")
    
    # Test serialization
    try:
        serialized = pickle.dumps(action, protocol=-1)
        print(f"\n✅ Serialization successful, size: {len(serialized)} bytes")
        
        # Test deserialization
        deserialized = pickle.loads(serialized)
        print(f"✅ Deserialization successful")
        print(f"   Matches original: {np.allclose(deserialized.pos, action.pos)}")
    except Exception as e:
        print(f"❌ Serialization error: {e}")


def main():
    print("Choose test:")
    print("1. Minimal command test (interactive)")
    print("2. Command format test")
    
    choice = input("\nEnter choice (1 or 2): ").strip()
    
    if choice == "1":
        test_minimal_command()
    elif choice == "2":
        test_command_format()
    else:
        print("Invalid choice")


if __name__ == "__main__":
    main() 