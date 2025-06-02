#!/usr/bin/env python3

import sys
import os
sys.path.insert(0, '/home/lbx-robot-1/projects/lbx-Franka-Teach/lbx_robotics/src/lbx_vision_camera')

from lbx_vision_camera.camera_utilities.camera_utils import discover_all_cameras, get_realsense_cameras
import logging

# Set up logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
handler = logging.StreamHandler()
handler.setFormatter(logging.Formatter('%(levelname)s: %(message)s'))
logger.addHandler(handler)

print("🔍 Testing camera discovery function...")
print("=" * 50)

print("\n1. Testing get_realsense_cameras() directly:")
realsense_cams = get_realsense_cameras()
print(f"Found {len(realsense_cams)} RealSense cameras:")
for i, cam in enumerate(realsense_cams):
    print(f"  Camera {i+1}:")
    for key, value in cam.items():
        print(f"    {key}: {value}")

print("\n2. Testing discover_all_cameras():")
all_cams = discover_all_cameras(logger)
print(f"Discovery result: {all_cams}")

print("\n3. Checking discovered serial numbers:")
discovered_sns = []
if 'realsense' in all_cams:
    for cam in all_cams['realsense']:
        sn = cam.get('serial_number', cam.get('serial', ''))
        discovered_sns.append(sn)
        print(f"  Discovered SN: '{sn}'")

print(f"\nDiscovered RealSense SNs list: {discovered_sns}")
print(f"Expected SN '218622277093' in list: {'218622277093' in discovered_sns}") 