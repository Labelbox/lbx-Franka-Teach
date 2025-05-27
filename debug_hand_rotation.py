#!/usr/bin/env python3
"""
Debug hand rotation in URDF
"""

import re
from pathlib import Path

def check_rotations():
    """Check all rotation values in the URDF"""
    urdf_file = Path("robot_urdf_models/fr3_franka_hand_snug.urdf")
    
    if not urdf_file.exists():
        print(f"❌ URDF file not found: {urdf_file}")
        return
    
    with open(urdf_file, 'r') as f:
        content = f.read()
    
    print("🔍 Checking rotation values in URDF:\n")
    
    # Check joint7 (last arm joint)
    match = re.search(r'<joint name="fr3_joint7".*?<origin[^>]*rpy="([^"]+)"', content, re.DOTALL)
    if match:
        print(f"fr3_joint7 rotation: {match.group(1)}")
    
    # Check joint8 (connects link7 to link8)
    match = re.search(r'<joint name="fr3_joint8".*?<origin[^>]*rpy="([^"]+)"', content, re.DOTALL)
    if match:
        print(f"fr3_joint8 rotation: {match.group(1)}")
        rpy = match.group(1).split()
        if len(rpy) >= 3:
            import math
            print(f"  → Yaw: {rpy[2]} rad = {math.degrees(float(rpy[2])):.1f}°")
    
    # Check hand_joint (connects link8 to hand)
    match = re.search(r'<joint name="fr3_hand_joint".*?<origin[^>]*rpy="([^"]+)"', content, re.DOTALL)
    if match:
        print(f"fr3_hand_joint rotation: {match.group(1)}")
        rpy = match.group(1).split()
        if len(rpy) >= 3:
            import math
            print(f"  → Yaw: {rpy[2]} rad = {math.degrees(float(rpy[2])):.1f}°")
    
    # Check hand_tcp_joint
    match = re.search(r'<joint name="fr3_hand_tcp_joint".*?<origin[^>]*rpy="([^"]+)"', content, re.DOTALL)
    if match:
        print(f"fr3_hand_tcp_joint rotation: {match.group(1)}")
    
    # Check finger joints
    match = re.search(r'<joint name="fr3_finger_joint1".*?<origin[^>]*rpy="([^"]+)"', content, re.DOTALL)
    if match:
        print(f"fr3_finger_joint1 rotation: {match.group(1)}")
    
    match = re.search(r'<joint name="fr3_finger_joint2".*?<origin[^>]*rpy="([^"]+)"', content, re.DOTALL)
    if match:
        print(f"fr3_finger_joint2 rotation: {match.group(1)}")
    
    print("\n💡 Notes:")
    print("- Rotations are in RPY (roll, pitch, yaw) format")
    print("- The hand rotation is typically controlled by joint8 or hand_joint")
    print("- If viewing in Foxglove, you need to record a new MCAP file")
    print("- If viewing in RViz, you may need to restart or reload the robot model")

if __name__ == '__main__':
    check_rotations() 