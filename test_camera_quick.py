#!/usr/bin/env python3
import sys
sys.path.append('/home/lbx-robot-1/projects/lbx-Franka-Teach/lbx_robotics/install/lbx_vision_camera/lib/python3.10/site-packages')

from lbx_vision_camera.camera_utilities.camera_test import test_cameras
import yaml

# Load the camera config
with open('/home/lbx-robot-1/projects/lbx-Franka-Teach/lbx_robotics/configs/sensors/realsense_cameras.yaml', 'r') as f:
    config = yaml.safe_load(f)

# Run the test
print("🔍 Testing camera with updated timeout...")
all_passed, results = test_cameras(config)
print(f'All tests passed: {all_passed}')
for result in results:
    print(f'Result: {result}') 