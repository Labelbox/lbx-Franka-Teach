#!/usr/bin/env python3
"""
Test script to verify Oculus Reader connection and data
"""

import sys
import time
import numpy as np

try:
    from oculus_reader.reader import OculusReader
except ImportError as e:
    print(f"❌ Failed to import OculusReader: {e}")
    print("Make sure you're in the project root directory")
    sys.exit(1)


def test_oculus_connection(ip_address=None):
    """Test Oculus Reader connection and display data"""
    
    print("🎮 Testing Oculus Reader Connection...")
    print(f"   Connection: {'USB' if ip_address is None else f'WiFi ({ip_address})'}")
    print("\nInitializing...")
    
    try:
        # Initialize reader
        reader = OculusReader(
            ip_address=ip_address,
            print_FPS=False
        )
        print("✅ Oculus Reader initialized successfully!")
        
    except Exception as e:
        print(f"❌ Failed to initialize Oculus Reader: {e}")
        print("\nTroubleshooting:")
        print("1. Check USB cable is connected")
        print("2. Put on headset and accept 'Allow USB Debugging'")
        print("3. Run: adb devices")
        return
    
    print("\n📊 Reading controller data...")
    print("   Move controllers and press buttons to see data")
    print("   Press Ctrl+C to exit\n")
    
    # Data tracking
    data_received = False
    last_print_time = time.time()
    
    try:
        while True:
            # Get data
            poses, buttons = reader.get_transformations_and_buttons()
            
            # Check if we have data
            if poses and buttons:
                if not data_received:
                    print("✅ Receiving data from Quest!\n")
                    data_received = True
                
                # Print data every 0.5 seconds
                current_time = time.time()
                if current_time - last_print_time > 0.5:
                    print(f"[{current_time:.1f}] Controller Data:")
                    
                    # Controller positions
                    for controller_id, pose in poses.items():
                        pos = pose[:3, 3]
                        print(f"  {controller_id.upper()}: pos=[{pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f}]")
                    
                    # Active buttons
                    active_buttons = []
                    for button, state in buttons.items():
                        if button in ['rightTrig', 'leftTrig']:
                            if state[0] > 0.1:  # Trigger threshold
                                active_buttons.append(f"{button}={state[0]:.2f}")
                        elif state:  # Boolean buttons
                            active_buttons.append(button)
                    
                    if active_buttons:
                        print(f"  Active: {', '.join(active_buttons)}")
                    else:
                        print("  Active: None")
                    
                    print()  # Empty line
                    last_print_time = current_time
                    
            else:
                # No data yet
                if not data_received and time.time() - last_print_time > 2:
                    print("⏳ Waiting for controller data...")
                    print("   Make sure the Quest app is running")
                    print("   Try moving the controllers\n")
                    last_print_time = time.time()
            
            time.sleep(0.05)  # 20 Hz
            
    except KeyboardInterrupt:
        print("\n\n🛑 Test stopped by user")
        
    finally:
        # Cleanup
        try:
            reader.stop()
            print("✅ Oculus Reader stopped cleanly")
        except:
            pass
        
        # Summary
        if data_received:
            print("\n✅ Test PASSED - Oculus Reader is working!")
            print("   You can now use: python3 oculus_vr_server.py")
        else:
            print("\n❌ Test FAILED - No data received")
            print("\nTroubleshooting:")
            print("1. Put on the Quest headset")
            print("2. Make sure controllers are on")
            print("3. Check if the teleop app is installed")
            print("4. Try: python3 oculus_reader_app/oculus_reader/reader.py")


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Test Oculus Reader Connection')
    parser.add_argument('--ip', type=str, default=None,
                        help='IP address for WiFi connection (default: USB)')
    
    args = parser.parse_args()
    
    test_oculus_connection(args.ip)


if __name__ == "__main__":
    main() 