#!/usr/bin/env python3
"""
Standalone camera test script for Labelbox Franka Teach System
Tests camera functionality before running the main server
"""

import argparse
import sys
from frankateach.camera_test import test_cameras
import yaml


def main():
    parser = argparse.ArgumentParser(
        description='Test camera functionality for Labelbox Franka Teach System',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  # Test Intel RealSense cameras
  python3 test_cameras.py --config configs/cameras_intel.yaml
  
  # Test USB cameras
  python3 test_cameras.py --config configs/cameras_usb.yaml
  
  # Test all cameras
  python3 test_cameras.py --config configs/cameras.yaml

The test will:
  - Connect to each enabled camera
  - Capture RGB frames (and depth for RealSense/ZED)
  - Measure actual FPS
  - Check data quality
  - Report any issues
        '''
    )
    
    parser.add_argument('--config', type=str, required=True,
                       help='Path to camera configuration YAML file')
    parser.add_argument('--verbose', action='store_true',
                       help='Show detailed test output')
    
    args = parser.parse_args()
    
    print("🎥 Labelbox Franka Teach Camera Test")
    print("=" * 60)
    
    try:
        # Load configuration
        with open(args.config, 'r') as f:
            camera_configs = yaml.safe_load(f)
            
        print(f"📄 Loaded configuration from: {args.config}")
        
        # Check manufacturer
        manufacturer = camera_configs.get('manufacturer', 'Unknown')
        print(f"🏭 Manufacturer: {manufacturer}")
        
        # Count enabled cameras
        enabled_count = sum(1 for cam in camera_configs.get('cameras', {}).values() 
                          if cam.get('enabled', False))
        total_count = len(camera_configs.get('cameras', {}))
        
        print(f"📷 Cameras: {enabled_count} enabled out of {total_count} configured")
        
        if enabled_count == 0:
            print("\n⚠️  No cameras are enabled in the configuration!")
            print("   Edit the config file and set 'enabled: true' for cameras to test")
            return 1
            
        # Run tests
        all_passed, results = test_cameras(camera_configs)
        
        # Exit with appropriate code
        return 0 if all_passed else 1
        
    except FileNotFoundError:
        print(f"\n❌ Configuration file not found: {args.config}")
        print("   Run 'python3 discover_cameras.py' to generate a configuration")
        return 1
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        if args.verbose:
            traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main()) 