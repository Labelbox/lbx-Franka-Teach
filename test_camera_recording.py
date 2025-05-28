#!/usr/bin/env python3
"""
Test camera recording with MCAP
"""

import time
import sys
from pathlib import Path

# Add parent directory to path
sys.path.append(str(Path(__file__).parent))

from frankateach.camera_manager import CameraManager
from frankateach.mcap_data_recorder import MCAPDataRecorder
from frankateach.mcap_verifier import MCAPVerifier


def test_camera_recording():
    """Test that cameras are being recorded to MCAP files"""
    print("🧪 Testing camera recording with MCAP...")
    print("=" * 60)
    
    # Check for camera config
    config_path = "configs/cameras_intel.yaml"
    if not Path(config_path).exists():
        print(f"❌ Camera config not found: {config_path}")
        return False
        
    # Initialize camera manager
    print("\n📷 Initializing camera manager...")
    try:
        camera_manager = CameraManager(config_path)
        camera_manager.start()
        print("✅ Camera manager started")
    except Exception as e:
        print(f"❌ Failed to start camera manager: {e}")
        return False
        
    # Initialize MCAP recorder
    print("\n📹 Initializing MCAP recorder...")
    try:
        recorder = MCAPDataRecorder(
            save_images=True,
            save_depth=True,
            camera_manager=camera_manager
        )
        print("✅ MCAP recorder initialized")
    except Exception as e:
        print(f"❌ Failed to initialize recorder: {e}")
        camera_manager.stop()
        return False
        
    # Start recording
    print("\n▶️  Starting recording...")
    recorder.start_recording("camera_test")
    
    # Record for a few seconds
    print("📹 Recording camera data for 5 seconds...")
    start_time = time.time()
    frame_count = 0
    
    while time.time() - start_time < 5.0:
        # Get camera frames
        frames = camera_manager.get_all_frames(timeout=0.1)
        if frames:
            frame_count += len(frames)
            print(f"   Captured {frame_count} frames...", end='\r')
        
        # Also write some dummy robot data to make the file valid
        timestep = {
            "observation": {
                "timestamp": {
                    "robot_state": {
                        "read_start": int(time.time() * 1e9),
                        "read_end": int(time.time() * 1e9)
                    }
                },
                "robot_state": {
                    "joint_positions": [0.0] * 7,
                    "cartesian_position": [0.4, 0.0, 0.3, 0.0, 0.0, 0.0],
                    "gripper_position": 0.0
                },
                "controller_info": {
                    "poses": {},
                    "buttons": {},
                    "movement_enabled": False,
                    "controller_on": True
                }
            },
            "action": [0.0] * 7
        }
        recorder.write_timestep(timestep)
        
        time.sleep(0.1)
    
    print(f"\n✅ Captured {frame_count} total frames")
    
    # Stop recording
    print("\n⏹️  Stopping recording...")
    filepath = recorder.stop_recording(success=True)
    
    # Stop camera manager
    camera_manager.stop()
    
    if not filepath:
        print("❌ No file was saved")
        return False
        
    print(f"✅ Recording saved to: {filepath}")
    
    # Verify the recording
    print("\n🔍 Verifying recorded data...")
    verifier = MCAPVerifier(filepath)
    results = verifier.verify(verbose=True)
    
    # Check if camera data was recorded
    has_cameras = results["summary"]["has_camera_data"]
    camera_count = results["summary"]["camera_count"]
    
    print("\n" + "=" * 60)
    print("📊 Test Results:")
    print(f"   Camera data recorded: {'✅ Yes' if has_cameras else '❌ No'}")
    print(f"   Number of cameras: {camera_count}")
    print(f"   Total frames captured: {frame_count}")
    
    if has_cameras and camera_count > 0:
        print("\n✅ Camera recording test PASSED!")
        return True
    else:
        print("\n❌ Camera recording test FAILED!")
        print("   No camera data found in MCAP file")
        return False


if __name__ == "__main__":
    success = test_camera_recording()
    sys.exit(0 if success else 1) 