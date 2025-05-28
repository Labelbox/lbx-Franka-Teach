#!/usr/bin/env python3
"""
Patch the existing URDF file used by MCAP recorder to update the hand offset and rotation.
This directly modifies the URDF without needing xacro.
"""

import re
import shutil
import math
from pathlib import Path

def patch_urdf_hand_offset(offset_z=-0.110, rotation_z_deg=45):
    """Patch the URDF file to update hand joint offset and rotation"""
    
    # Path to the URDF file used by MCAP recorder
    urdf_file = Path(__file__).parent / "franka_description" / "urdfs" / "fr3_franka_hand.urdf"
    
    if not urdf_file.exists():
        print(f"❌ URDF file not found: {urdf_file}")
        return False
    
    # Create backup
    backup_file = urdf_file.with_suffix('.urdf.backup')
    shutil.copy2(urdf_file, backup_file)
    print(f"✅ Created backup: {backup_file}")
    
    # Read the URDF
    with open(urdf_file, 'r') as f:
        urdf_content = f.read()
    
    print("🔧 Patching URDF hand offset and rotation...")
    
    # First, update joint8 position (offset)
    joint8_pattern = r'(<joint name="fr3_joint8"[^>]*>.*?<origin[^>]*xyz=")([^"]+)("[^>]*/>)'
    
    match = re.search(joint8_pattern, urdf_content, re.DOTALL)
    
    if match:
        original_xyz = match.group(2)
        xyz_values = original_xyz.split()
        
        if len(xyz_values) >= 3:
            try:
                # Parse current values
                x = float(xyz_values[0])
                y = float(xyz_values[1])
                z = float(xyz_values[2])
                
                # Calculate new Z position (0.107 is original, add our offset)
                new_z = 0.107 + offset_z
                new_xyz = f"{x} {y} {new_z}"
                
                # Replace in content
                old_text = match.group(0)
                new_text = match.group(1) + new_xyz + match.group(3)
                urdf_content = urdf_content.replace(old_text, new_text)
                
                print(f"✅ Updated joint8 (link8) position:")
                print(f"   Original: {original_xyz}")
                print(f"   New:      {new_xyz}")
                print(f"   Hand moved {abs(offset_z)*1000:.0f}mm closer to arm")
                
            except ValueError:
                print(f"⚠️  Could not parse XYZ values: {original_xyz}")
                return False
    else:
        print("❌ Could not find joint8 in URDF")
        return False
    
    # Second, update hand_joint rotation
    # The default rotation is -45 degrees (-pi/4), we want to set it relative to that
    hand_joint_pattern = r'(<joint name="fr3_hand_joint"[^>]*>.*?<origin[^>]*rpy=")([^"]+)("[^>]*/>)'
    
    match = re.search(hand_joint_pattern, urdf_content, re.DOTALL)
    
    if match:
        original_rpy = match.group(2)
        rpy_values = original_rpy.split()
        
        if len(rpy_values) >= 3:
            try:
                # Parse current values
                roll = float(rpy_values[0])
                pitch = float(rpy_values[1])
                yaw = float(rpy_values[2])
                
                # Calculate new yaw: default (-pi/4) + rotation
                default_rotation = -math.pi/4  # -45 degrees
                rotation_z_rad = math.radians(rotation_z_deg)
                new_yaw = default_rotation + rotation_z_rad
                new_rpy = f"{roll} {pitch} {new_yaw}"
                
                # Replace in content
                old_text = match.group(0)
                new_text = match.group(1) + new_rpy + match.group(3)
                urdf_content = urdf_content.replace(old_text, new_text)
                
                print(f"✅ Updated hand joint rotation:")
                print(f"   Original RPY: {original_rpy}")
                print(f"   New RPY:      {new_rpy}")
                print(f"   Set to {rotation_z_deg}° from default orientation")
                print(f"   Total rotation: {math.degrees(new_yaw):.1f}° ({math.degrees(default_rotation):.0f}° default + {rotation_z_deg}° adjustment)")
                
            except ValueError:
                print(f"⚠️  Could not parse RPY values: {original_rpy}")
                return False
    else:
        print("⚠️  Could not find hand_joint rotation in URDF")
    
    # Write the modified URDF
    with open(urdf_file, 'w') as f:
        f.write(urdf_content)
    
    print(f"✅ Updated URDF saved to: {urdf_file}")
    
    # Verify the changes
    verify_urdf_changes(urdf_file, offset_z, rotation_z_deg)
    
    return True

def verify_urdf_changes(urdf_file, expected_offset, expected_rotation_deg):
    """Verify the hand offset and rotation in the URDF file"""
    with open(urdf_file, 'r') as f:
        content = f.read()
    
    # Check joint8 position
    match = re.search(r'<joint name="fr3_joint8".*?<origin[^>]*xyz="([^"]+)"', content, re.DOTALL)
    if match:
        xyz = match.group(1).split()
        if len(xyz) >= 3:
            z_value = float(xyz[2])
            expected_z = 0.107 + expected_offset
            print(f"\n📍 Joint8 (link8) Z position in URDF: {z_value}m")
            if abs(z_value - expected_z) < 0.001:
                print(f"✅ Hand offset correctly applied!")
                print(f"   Expected: {expected_z:.3f}m, Actual: {z_value:.3f}m")
            else:
                print(f"⚠️  Unexpected Z value. Expected: {expected_z:.3f}m, Actual: {z_value:.3f}m")
    
    # Check hand joint rotation
    match = re.search(r'<joint name="fr3_hand_joint".*?<origin[^>]*rpy="([^"]+)"', content, re.DOTALL)
    if match:
        rpy = match.group(1).split()
        if len(rpy) >= 3:
            yaw_value = float(rpy[2])
            expected_yaw = -math.pi/4 + math.radians(expected_rotation_deg)
            print(f"\n📍 Hand joint rotation (yaw) in URDF: {math.degrees(yaw_value):.1f}°")
            print(f"   Expected total: {math.degrees(expected_yaw):.1f}° (-45° default + {expected_rotation_deg}° adjustment)")

def main():
    print("MCAP URDF Hand Offset & Rotation Patcher")
    print("=========================================")
    print("This tool directly patches the URDF file used by MCAP recorder")
    print("to include the hand offset of -0.110m (110mm closer)")
    print("and rotation of 45° from default orientation.\n")
    
    success = patch_urdf_hand_offset(-0.110, 45)
    
    if success:
        print("\n✅ MCAP URDF successfully patched!")
        print("\nThe MCAP recorder will now use the updated robot model with:")
        print("- Hand moved 110mm closer to the arm")
        print("- Hand rotated 45° from default orientation (total: 0°)")
        print("- Link8 position changed from 0.107m to -0.003m")
        print("\nAll future MCAP recordings will include this configuration.")
        print("\nNote: This is a direct patch. To restore original, use the .backup file.")
    else:
        print("\n❌ Failed to patch MCAP URDF")
        print("\nThe URDF structure may be different than expected.")
        print("You may need to manually edit the file.")

if __name__ == '__main__':
    main() 