#!/usr/bin/env python3
"""
Apply hand offset adjustment to the Franka hand configuration.
This script modifies the configuration files to set a snug fit.
"""

import os
import shutil
import argparse
import sys
import math

def backup_file(filepath):
    """Create a backup of the original file"""
    backup_path = filepath + '.backup'
    if not os.path.exists(backup_path):
        shutil.copy2(filepath, backup_path)
        print(f"Created backup: {backup_path}")
    return backup_path

def modify_hand_arguments(offset_z=-0.105, rotation_z_deg=15):
    """Modify the franka_hand_arguments.xacro file"""
    base_path = os.path.expanduser('~/projects/lbx-Franka-Teach/franka_description/end_effectors/franka_hand')
    args_file = os.path.join(base_path, 'franka_hand_arguments.xacro')
    
    if not os.path.exists(args_file):
        print(f"Error: File not found: {args_file}")
        return False
    
    # Backup original file
    backup_file(args_file)
    
    # Read the file
    with open(args_file, 'r') as f:
        content = f.read()
    
    # Find and replace the xyz_ee line
    import re
    pattern = r'(<xacro:arg name="xyz_ee" default=")([^"]+)(" />)'
    replacement = f'\\g<1>0 0 {offset_z}\\g<3>'
    
    new_content = re.sub(pattern, replacement, content)
    
    # Find and replace the rpy_ee line for rotation
    # Convert degrees to radians and add to existing rotation
    rotation_z_rad = math.radians(rotation_z_deg)
    # The default rotation is -pi/4, we add our rotation to it
    total_rotation = f"0 0 ${{-math.pi/4 + {rotation_z_rad:.6f}}}"
    
    rpy_pattern = r'(<xacro:arg name="rpy_ee" default=")([^"]+)(" />)'
    rpy_replacement = f'\\g<1>{total_rotation}\\g<3>'
    
    new_content = re.sub(rpy_pattern, rpy_replacement, new_content)
    
    if new_content != content:
        # Write the modified content
        with open(args_file, 'w') as f:
            f.write(new_content)
        print(f"Modified {args_file}")
        print(f"Set hand offset to: 0 0 {offset_z}")
        print(f"Set hand rotation to: {total_rotation} (includes {rotation_z_deg}° CCW)")
        return True
    else:
        print("No changes needed - pattern not found or already set")
        return False

def create_config_override(offset_z=-0.105, rotation_z_deg=15):
    """Create a configuration override file"""
    config_content = f"""# Franka Hand Offset Configuration
# This file contains the adjusted hand offset for a snug fit

# Hand offset in meters (negative Z brings hand closer to arm)
hand_z_offset: {offset_z}

# Hand rotation in degrees (from default orientation)
hand_z_rotation_deg: {rotation_z_deg}

# To apply this offset, use one of these methods:
# 1. Launch file: roslaunch franka_description fr3_with_snug_hand.launch hand_z_offset:={offset_z}
# 2. URDF parameter: xyz_ee:='0 0 {offset_z}' rpy_ee:='0 0 ${{-pi/4 + {math.radians(rotation_z_deg):.6f}}}'
# 3. Modify franka_hand_arguments.xacro default value

# Current configuration:
#  Offset: {abs(offset_z)*1000:.0f}mm closer to arm
#  Rotation: {rotation_z_deg}° from default orientation (total: {-45 + rotation_z_deg}°)
"""
    
    config_file = 'franka_hand_offset_config.yaml'
    with open(config_file, 'w') as f:
        f.write(config_content)
    print(f"Created configuration file: {config_file}")
    return config_file

def show_current_config():
    """Display the current hand offset configuration"""
    args_file = os.path.expanduser('~/projects/lbx-Franka-Teach/franka_description/end_effectors/franka_hand/franka_hand_arguments.xacro')
    
    if os.path.exists(args_file):
        with open(args_file, 'r') as f:
            content = f.read()
        
        import re
        xyz_match = re.search(r'<xacro:arg name="xyz_ee" default="([^"]+)"', content)
        rpy_match = re.search(r'<xacro:arg name="rpy_ee" default="([^"]+)"', content)
        
        if xyz_match:
            print(f"Current hand offset (xyz_ee): {xyz_match.group(1)}")
        if rpy_match:
            print(f"Current hand rotation (rpy_ee): {rpy_match.group(1)}")
        
        if not xyz_match and not rpy_match:
            print("Could not find xyz_ee or rpy_ee parameters in configuration")
    else:
        print(f"Configuration file not found: {args_file}")

def restore_backup():
    """Restore the original configuration from backup"""
    args_file = os.path.expanduser('~/projects/lbx-Franka-Teach/franka_description/end_effectors/franka_hand/franka_hand_arguments.xacro')
    backup_file = args_file + '.backup'
    
    if os.path.exists(backup_file):
        shutil.copy2(backup_file, args_file)
        print(f"Restored original configuration from {backup_file}")
        return True
    else:
        print("No backup file found")
        return False

def main():
    parser = argparse.ArgumentParser(
        description='Apply hand offset adjustment for Franka FR3',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Apply new offset (105mm closer, 15° CCW)
  ./apply_hand_offset.py
  
  # Apply custom offset
  ./apply_hand_offset.py --offset -0.12 --rotation 20
  
  # Show current configuration
  ./apply_hand_offset.py --show
  
  # Restore original configuration
  ./apply_hand_offset.py --restore
  
  # Create config file only
  ./apply_hand_offset.py --config-only
""")
    
    parser.add_argument('--offset', type=float, default=-0.105,
                        help='Z offset in meters (default: -0.105)')
    parser.add_argument('--rotation', type=float, default=15,
                        help='Z rotation in degrees CCW (default: 15)')
    parser.add_argument('--show', action='store_true',
                        help='Show current configuration')
    parser.add_argument('--restore', action='store_true',
                        help='Restore original configuration from backup')
    parser.add_argument('--config-only', action='store_true',
                        help='Only create config file without modifying URDF')
    
    args = parser.parse_args()
    
    print("Franka Hand Offset Adjustment Tool")
    print("==================================\n")
    
    if args.show:
        show_current_config()
    elif args.restore:
        if restore_backup():
            show_current_config()
    elif args.config_only:
        create_config_override(args.offset, args.rotation)
    else:
        print(f"Applying hand offset: {args.offset}m ({abs(args.offset)*1000:.0f}mm closer)")
        print(f"Applying hand rotation: {args.rotation}° CCW")
        
        # Create config file
        create_config_override(args.offset, args.rotation)
        
        # Modify the URDF arguments
        if modify_hand_arguments(args.offset, args.rotation):
            print("\nSuccessfully applied hand offset and rotation!")
            show_current_config()
            
            print("\nNext steps:")
            print("1. Test with your robot visualization tools")
            print("2. Use the launch file to verify: roslaunch franka_description fr3_with_snug_hand.launch")
            print("3. If needed, restore original with: ./apply_hand_offset.py --restore")
        else:
            print("\nFailed to apply changes")

if __name__ == '__main__':
    main() 