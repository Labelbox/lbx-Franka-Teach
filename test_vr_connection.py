#!/usr/bin/env python3
"""
Test VR Connection - Verify that VR data is being received properly
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from frankateach.oculus_stick import OculusVRStickDetector
from frankateach.constants import HOST, VR_CONTROLLER_STATE_PORT
import time

def test_vr_connection():
    print("Testing VR Connection...")
    print("=" * 50)
    print("1. Make sure mouse_vr_server.py is running")
    print("2. This will test if oculus_stick can receive VR data")
    print("=" * 50)
    
    try:
        # Create the VR detector (same as teleop does)
        detector = OculusVRStickDetector(HOST, VR_CONTROLLER_STATE_PORT)
        
        print("✅ VR detector created successfully")
        print("🔍 Listening for VR controller data...")
        print("   Try moving mouse and clicking buttons in mouse_vr_server")
        print("   Press Ctrl+C to stop")
        print()
        
        message_count = 0
        last_heartbeat = time.time()
        
        while True:
            try:
                # This is the same loop as in oculus_stick.py
                message = detector.stick_socket.recv_string()
                
                if message == "oculus_controller":
                    continue
                
                message_count += 1
                current_time = time.time()
                
                # Parse the controller state (same as oculus_stick does)
                from frankateach.utils import parse_controller_state
                controller_state = parse_controller_state(message)
                
                print(f"📥 [{message_count:03d}] Received VR data:")
                print(f"   Right A (recording): {controller_state.right_a}")
                print(f"   Right B: {controller_state.right_b}")
                print(f"   Right position: {controller_state.right_local_position}")
                print(f"   Right rotation: {controller_state.right_local_rotation}")
                
                # Show affine matrix calculation
                right_affine = controller_state.right_affine
                print(f"   Right affine shape: {right_affine.shape}")
                print(f"   Right affine position: {right_affine[:3, 3]}")
                print()
                
                # Heartbeat every 5 seconds
                if current_time - last_heartbeat > 5:
                    print(f"💓 Heartbeat - {message_count} messages received")
                    last_heartbeat = current_time
                
            except Exception as e:
                print(f"❌ Error receiving VR data: {e}")
                break
                
    except KeyboardInterrupt:
        print("\n🛑 Test stopped by user")
    except Exception as e:
        print(f"❌ Error creating VR detector: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_vr_connection() 