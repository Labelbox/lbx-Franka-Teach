#!/usr/bin/env python3
"""Diagnose joint position data in MCAP recordings"""

import json
import mcap
from mcap.reader import make_reader
from pathlib import Path
import numpy as np

def diagnose_joint_data(filepath):
    """Diagnose joint position issues in MCAP file"""
    
    print(f"\n🔍 Diagnosing joint data in: {filepath}")
    
    joint_data_found = False
    sample_count = 0
    joint_positions_samples = []
    robot_state_samples = []
    
    with open(filepath, "rb") as f:
        reader = make_reader(f)
        
        for schema, channel, message in reader.iter_messages():
            if channel.topic == "/joint_states":
                joint_data_found = True
                sample_count += 1
                
                if sample_count <= 5:  # Sample first 5 messages
                    data = json.loads(message.data)
                    joint_positions_samples.append(data)
                    
                    print(f"\n📊 Joint State Message #{sample_count}:")
                    print(f"   Timestamp: {message.log_time}")
                    
                    # Check joint names
                    joint_names = data.get("name", [])
                    print(f"   Joint names ({len(joint_names)}): {joint_names}")
                    
                    # Check joint positions
                    positions = data.get("position", [])
                    if positions:
                        print(f"   Joint positions ({len(positions)}):")
                        for i, (name, pos) in enumerate(zip(joint_names, positions)):
                            print(f"      {name}: {pos:.4f} rad ({np.degrees(pos):.1f}°)")
                    else:
                        print("   ❌ No position data!")
                    
                    # Check if positions are all zeros
                    if positions and all(abs(p) < 0.001 for p in positions[:7]):
                        print("   ⚠️  WARNING: All joint positions are near zero!")
                    
            elif channel.topic == "/robot_state":
                if sample_count <= 5:
                    data = json.loads(message.data)
                    robot_state_samples.append(data)
                    
                    joint_pos = data.get("joint_positions", [])
                    if sample_count == 1:
                        print(f"\n🤖 Robot State Data:")
                        print(f"   Joint positions available: {'Yes' if joint_pos else 'No'}")
                        if joint_pos:
                            print(f"   Number of joints: {len(joint_pos)}")
                            print(f"   Sample values: {[f'{p:.3f}' for p in joint_pos[:3]]}...")
    
    # Analysis
    print("\n🩺 Diagnosis:")
    
    if not joint_data_found:
        print("❌ No /joint_states messages found!")
        print("   The robot model won't move without joint state data")
        return
    
    # Check if joint positions are being updated
    if len(joint_positions_samples) >= 2:
        first_pos = joint_positions_samples[0].get("position", [])
        last_pos = joint_positions_samples[-1].get("position", [])
        
        if first_pos and last_pos:
            max_change = max(abs(f - l) for f, l in zip(first_pos[:7], last_pos[:7]))
            if max_change < 0.01:
                print("⚠️  Joint positions are not changing significantly")
                print(f"   Maximum change: {max_change:.4f} rad ({np.degrees(max_change):.1f}°)")
            else:
                print("✅ Joint positions are changing")
                print(f"   Maximum change: {max_change:.4f} rad ({np.degrees(max_change):.1f}°)")
    
    # Check data source
    print("\n📍 Data Source Analysis:")
    if robot_state_samples:
        # Check if joint_positions are populated in robot_state
        has_joint_data = any(rs.get("joint_positions") for rs in robot_state_samples)
        if has_joint_data:
            print("✅ Robot state contains joint position data")
        else:
            print("❌ Robot state does NOT contain joint position data")
            print("   This is likely why the robot doesn't move in Foxglove")
    
    # Check if we're in debug mode
    if all(all(abs(p) < 0.001 for p in sample.get("position", [])[:7]) 
           for sample in joint_positions_samples):
        print("\n⚠️  All joint positions are zero!")
        print("   Possible causes:")
        print("   1. Running in debug mode (no real robot connection)")
        print("   2. Robot state not being received from franka_server.py")
        print("   3. Joint data not being forwarded from robot")

if __name__ == "__main__":
    # Find most recent MCAP file
    recordings_dir = Path.home() / "recordings"
    mcap_files = list(recordings_dir.glob("**/*.mcap"))
    
    if mcap_files:
        latest_file = max(mcap_files, key=lambda p: p.stat().st_mtime)
        print(f"Using most recent MCAP file: {latest_file.name}")
        diagnose_joint_data(latest_file)
    else:
        print("❌ No MCAP files found in ~/recordings/") 