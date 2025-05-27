#!/usr/bin/env python3
"""
Test different hand rotation values in the URDF
"""

import os
import sys
import math
import shutil
from pathlib import Path

def set_hand_rotation(rotation_deg):
    """Set the hand rotation in the URDF file"""
    urdf_file = Path("robot_urdf_models/fr3_franka_hand_snug.urdf")
    
    if not urdf_file.exists():
        print(f"❌ URDF file not found: {urdf_file}")
        return False
    
    # Read the file
    with open(urdf_file, 'r') as f:
        content = f.read()
    
    # Convert degrees to radians
    rotation_rad = math.radians(rotation_deg)
    
    # Find and replace the hand joint rotation
    import re
    pattern = r'(<joint name="fr3_hand_joint"[^>]*>.*?<origin[^>]*rpy=")([^"]+)("[^>]*/>)'
    
    match = re.search(pattern, content, re.DOTALL)
    if match:
        old_rpy = match.group(2)
        # Keep roll and pitch, only change yaw
        rpy_values = old_rpy.split()
        if len(rpy_values) >= 3:
            new_rpy = f"{rpy_values[0]} {rpy_values[1]} {rotation_rad}"
            new_content = content.replace(
                match.group(0),
                match.group(1) + new_rpy + match.group(3)
            )
            
            # Write back
            with open(urdf_file, 'w') as f:
                f.write(new_content)
            
            print(f"✅ Set hand rotation to {rotation_deg}° ({rotation_rad:.6f} rad)")
            return True
    
    print("❌ Could not find hand joint in URDF")
    return False

def main():
    if len(sys.argv) < 2:
        print("Usage: ./test_hand_rotation.py <rotation_degrees>")
        print("\nExamples:")
        print("  ./test_hand_rotation.py 0      # No rotation")
        print("  ./test_hand_rotation.py -45    # Default rotation")
        print("  ./test_hand_rotation.py 45     # 45° clockwise")
        print("  ./test_hand_rotation.py -90    # 90° counter-clockwise")
        print("\nCurrent common values:")
        print("  -45° : Default Franka hand orientation")
        print("    0° : Straight/aligned with arm")
        print("   45° : 45° clockwise from straight")
        sys.exit(1)
    
    try:
        rotation = float(sys.argv[1])
    except ValueError:
        print(f"❌ Invalid rotation value: {sys.argv[1]}")
        sys.exit(1)
    
    print(f"\nSetting hand rotation to {rotation}°...")
    
    if set_hand_rotation(rotation):
        print(f"\n✅ Successfully set hand rotation to {rotation}°")
        print("\nTo test this in your visualization:")
        print("1. Restart any running visualization")
        print("2. The MCAP recorder will use the updated URDF")
        print("\nNote: The rotation is applied around the Z-axis (yaw)")
        print("      Positive values = clockwise (when looking down)")
        print("      Negative values = counter-clockwise")
    else:
        print("\n❌ Failed to set rotation")

if __name__ == '__main__':
    main() 