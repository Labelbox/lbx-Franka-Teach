#!/usr/bin/env python3
"""
Verify camera recording in MCAP files
"""

import sys
import time
from pathlib import Path
from frankateach.mcap_verifier import MCAPVerifier

def find_latest_mcap():
    """Find the most recently created MCAP file"""
    recordings_dir = Path.home() / "recordings"
    mcap_files = []
    
    # Check both success and failure directories
    for subdir in ["success", "failure"]:
        dir_path = recordings_dir / subdir
        if dir_path.exists():
            mcap_files.extend(dir_path.glob("*.mcap"))
    
    if not mcap_files:
        return None
        
    # Return the most recent file
    return max(mcap_files, key=lambda p: p.stat().st_mtime)

def main():
    print("🔍 Checking for camera data in MCAP recordings...")
    print("=" * 60)
    
    # Wait a moment for any ongoing recording to capture some data
    print("\n⏳ Waiting for recording to capture data...")
    time.sleep(3)
    
    # Find the latest MCAP file
    latest_file = find_latest_mcap()
    
    if not latest_file:
        print("❌ No MCAP files found in ~/recordings/")
        return 1
        
    print(f"\n📁 Found latest MCAP file: {latest_file.name}")
    print(f"   Path: {latest_file}")
    
    # Verify the file
    verifier = MCAPVerifier(str(latest_file))
    results = verifier.verify(verbose=False)
    
    # Check camera data
    camera_streams = results.get("camera_streams", {})
    summary = results.get("summary", {})
    
    print("\n📷 Camera Data Analysis:")
    print(f"   Has camera data: {'✅ Yes' if summary.get('has_camera_data', False) else '❌ No'}")
    print(f"   Number of cameras: {summary.get('camera_count', 0)}")
    print(f"   Has depth data: {'✅ Yes' if summary.get('has_depth_data', False) else '❌ No'}")
    print(f"   Total camera messages: {summary.get('camera_messages', 0)}")
    
    if camera_streams:
        print("\n📊 Camera Stream Details:")
        
        # Group by camera ID
        cameras = {}
        for topic, info in camera_streams.items():
            cam_id = info.get('camera_id', 'unknown')
            if cam_id not in cameras:
                cameras[cam_id] = {'rgb': None, 'depth': None, 'calibration': None}
            
            if 'compressed' in topic:
                cameras[cam_id]['rgb'] = info
            elif 'depth' in topic:
                cameras[cam_id]['depth'] = info
            elif 'camera_info' in topic:
                cameras[cam_id]['calibration'] = info
        
        # Print info for each camera
        for cam_id, cam_data in cameras.items():
            print(f"\n   Camera: {cam_id}")
            
            if cam_data['rgb']:
                rgb = cam_data['rgb']
                print(f"      RGB Stream:")
                print(f"         Messages: {rgb['message_count']}")
                print(f"         Frequency: {rgb['average_frequency_hz']:.1f} Hz")
                print(f"         Format: {rgb.get('format', 'unknown')}")
                
            if cam_data['depth']:
                depth = cam_data['depth']
                print(f"      Depth Stream:")
                print(f"         Messages: {depth['message_count']}")
                print(f"         Frequency: {depth['average_frequency_hz']:.1f} Hz")
                print(f"         Resolution: {depth.get('resolution', 'unknown')}")
                print(f"         Format: {depth.get('format', 'unknown')}")
                
            if cam_data['calibration']:
                calib = cam_data['calibration']
                print(f"      Calibration:")
                print(f"         Messages: {calib['message_count']}")
                print(f"         Topic: /camera/{cam_id}/camera_info")
    
    # Check synchronization
    if camera_streams:
        print("\n⏱️  Synchronization Check:")
        
        # Get all camera topics
        rgb_topics = [t for t in camera_streams if 'compressed' in t]
        depth_topics = [t for t in camera_streams if 'depth' in t]
        
        if rgb_topics and depth_topics:
            # Compare message counts
            for rgb_topic in rgb_topics:
                cam_id = camera_streams[rgb_topic].get('camera_id', 'unknown')
                depth_topic = f"/camera/{cam_id}/depth"
                
                if depth_topic in camera_streams:
                    rgb_count = camera_streams[rgb_topic]['message_count']
                    depth_count = camera_streams[depth_topic]['message_count']
                    
                    sync_ratio = min(rgb_count, depth_count) / max(rgb_count, depth_count) if max(rgb_count, depth_count) > 0 else 0
                    
                    print(f"   Camera {cam_id}:")
                    print(f"      RGB messages: {rgb_count}")
                    print(f"      Depth messages: {depth_count}")
                    print(f"      Sync ratio: {sync_ratio:.2%}")
                    
                    if sync_ratio < 0.95:
                        print(f"      ⚠️  Possible synchronization issue")
                    else:
                        print(f"      ✅ Well synchronized")
    
    # Overall verdict
    print("\n" + "=" * 60)
    if summary.get('has_camera_data', False) and summary.get('camera_count', 0) > 0:
        print("✅ Camera recording is working properly!")
        print(f"   - {summary.get('camera_count', 0)} camera(s) detected")
        print(f"   - {summary.get('camera_messages', 0)} total camera messages")
        if summary.get('has_depth_data', False):
            print("   - Depth data is being recorded")
        return 0
    else:
        print("❌ No camera data found in recording!")
        print("   Please ensure:")
        print("   - Cameras are connected and working")
        print("   - Recording is active (press A button)")
        print("   - Camera recording is enabled in the server")
        return 1

if __name__ == "__main__":
    sys.exit(main()) 