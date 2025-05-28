#!/usr/bin/env python3
"""
Simple camera discovery script for Labelbox Franka Teach System
Run this to find all connected cameras and their USB ports
"""

from frankateach.camera_utils import discover_all_cameras, print_camera_info, generate_camera_config
import sys
import os

# Suppress OpenCV warnings
os.environ['OPENCV_LOG_LEVEL'] = 'FATAL'

def main():
    print("🎥 Discovering cameras...")
    print("=" * 60)
    
    # Discover all cameras
    cameras = discover_all_cameras()
    
    # Print detailed information
    print_camera_info(cameras)
    
    # If cameras found, offer to generate config
    if cameras:
        print("\n" + "=" * 60)
        print("\n💡 Configuration Options:")
        print("1. Generate manufacturer-specific configs (recommended)")
        print("2. Generate single combined config file")
        print("3. Exit")
        
        choice = input("\nSelect option (1-3): ")
        
        if choice == "1":
            # Generate manufacturer-specific configs
            configs = generate_camera_config(cameras, split_by_manufacturer=True)
            print(f"\n✅ Created {len(configs)} configuration file(s):")
            for config in configs:
                print(f"   - {config}")
            print("\nNext steps:")
            print("1. Review and edit the configuration files in configs/")
            print("2. Run the server with one of:")
            print("   python3 oculus_vr_server.py --enable-cameras --camera-config configs/cameras_intel.yaml")
            print("   python3 oculus_vr_server.py --enable-cameras --camera-config configs/cameras_usb.yaml")
            
        elif choice == "2":
            # Ask for custom filename
            filename = input("Enter config filename (default: configs/cameras_all.yaml): ").strip()
            if not filename:
                filename = "configs/cameras_all.yaml"
                
            configs = generate_camera_config(cameras, output_path=filename, split_by_manufacturer=False)
            print(f"\n✅ Configuration saved to: {filename}")
            print("\nNext steps:")
            print("1. Review and edit the configuration file")
            print("2. Run the server with: python3 oculus_vr_server.py --enable-cameras --camera-config", filename)
    else:
        print("\n❌ No cameras found!")
        print("\nTroubleshooting:")
        print("- Check USB connections")
        print("- Install camera drivers (pyrealsense2 for RealSense)")
        print("- Run with sudo if permission issues")
        
if __name__ == "__main__":
    main() 