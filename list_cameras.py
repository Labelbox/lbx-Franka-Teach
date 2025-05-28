#!/usr/bin/env python3
"""
Simple camera listing utility - shows connected cameras without warnings
"""

import os
import sys

# Suppress all warnings
os.environ['OPENCV_LOG_LEVEL'] = 'FATAL'

# Add project to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from frankateach.camera_utils import get_realsense_cameras, get_usb_port_for_device

def main():
    print("📷 Connected Cameras:")
    print("=" * 60)
    
    # List RealSense cameras
    print("\nIntel RealSense Cameras:")
    print("-" * 40)
    
    try:
        import pyrealsense2 as rs
        ctx = rs.context()
        devices = ctx.query_devices()
        
        if len(devices) == 0:
            print("  No RealSense cameras found")
        else:
            for i, device in enumerate(devices):
                serial = device.get_info(rs.camera_info.serial_number)
                name = device.get_info(rs.camera_info.name)
                fw = device.get_info(rs.camera_info.firmware_version)
                
                # Try to get USB port
                usb_port = get_usb_port_for_device(serial)
                
                print(f"\n  Camera {i+1}:")
                print(f"    Model: {name}")
                print(f"    Serial: {serial}")
                print(f"    Firmware: {fw}")
                if usb_port:
                    print(f"    USB Port: {usb_port}")
                    
    except ImportError:
        print("  ⚠️  Intel RealSense SDK not installed")
    except Exception as e:
        print(f"  ❌ Error: {e}")
    
    # List USB cameras using v4l2
    print("\n\nOther USB Cameras:")
    print("-" * 40)
    
    try:
        import subprocess
        
        # Get list of video devices
        result = subprocess.run(['ls', '/dev/video*'], 
                              capture_output=True, 
                              text=True, 
                              shell=True)
        
        if result.returncode == 0:
            devices = result.stdout.strip().split()
            
            # Filter out RealSense devices
            non_rs_devices = []
            for device in devices:
                # Check device info
                info_result = subprocess.run(
                    ['v4l2-ctl', '--device', device, '--info'],
                    capture_output=True,
                    text=True,
                    timeout=1
                )
                
                if info_result.returncode == 0:
                    output = info_result.stdout
                    # Skip if it's a RealSense device
                    if 'realsense' not in output.lower():
                        # Extract device name
                        import re
                        card_match = re.search(r'Card type\s*:\s*(.+)', output)
                        if card_match:
                            name = card_match.group(1).strip()
                            # Skip metadata devices
                            if 'metadata' not in name.lower():
                                non_rs_devices.append((device, name))
            
            if non_rs_devices:
                for device, name in non_rs_devices:
                    print(f"\n  {device}: {name}")
            else:
                print("  No other USB cameras found")
                
    except Exception as e:
        print(f"  ❌ Error listing USB devices: {e}")
    
    print("\n" + "=" * 60)
    print("\n💡 To configure cameras for recording:")
    print("   1. Edit configs/cameras.yaml")
    print("   2. Run: ./run_server.sh")

if __name__ == "__main__":
    main() 