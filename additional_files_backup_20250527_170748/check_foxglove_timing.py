#!/usr/bin/env python3
"""Check timing and frequency of messages in MCAP for Foxglove issues"""

import json
import mcap
from mcap.reader import make_reader
from pathlib import Path
import sys
from collections import defaultdict

def check_foxglove_timing(filepath):
    """Check timing issues that might affect Foxglove visualization"""
    
    print(f"\n🔍 Checking timing in: {filepath}")
    
    # Track message timings by topic
    message_times = defaultdict(list)
    first_timestamp = None
    last_timestamp = None
    
    with open(filepath, "rb") as f:
        reader = make_reader(f)
        
        # Collect all message timestamps
        for schema, channel, message in reader.iter_messages():
            if first_timestamp is None:
                first_timestamp = message.log_time
            last_timestamp = message.log_time
            
            message_times[channel.topic].append(message.log_time)
    
    # Analyze timing
    print(f"\n⏱️  Recording Duration: {(last_timestamp - first_timestamp) / 1e9:.2f} seconds")
    
    print(f"\n📊 Message Frequencies:")
    for topic, times in sorted(message_times.items()):
        if len(times) > 1:
            # Calculate average frequency
            time_diffs = [(times[i+1] - times[i]) / 1e9 for i in range(len(times)-1)]
            avg_period = sum(time_diffs) / len(time_diffs)
            avg_freq = 1.0 / avg_period if avg_period > 0 else 0
            
            print(f"\n   {topic}:")
            print(f"     Messages: {len(times)}")
            print(f"     Avg frequency: {avg_freq:.1f} Hz")
            print(f"     First msg: {(times[0] - first_timestamp) / 1e6:.1f} ms from start")
            
            # Check for gaps
            max_gap = max(time_diffs) if time_diffs else 0
            if max_gap > 0.5:  # More than 500ms gap
                print(f"     ⚠️  Max gap: {max_gap:.2f} seconds")
        else:
            print(f"\n   {topic}:")
            print(f"     Messages: {len(times)} (single message)")
    
    # Check specific timing issues
    print(f"\n🩺 Timing Analysis:")
    
    # Check if transforms are published before joint states
    if "/tf" in message_times and "/joint_states" in message_times:
        tf_first = message_times["/tf"][0]
        joint_first = message_times["/joint_states"][0]
        
        if tf_first > joint_first:
            print("⚠️  First transform published AFTER first joint state")
            print(f"   Delay: {(tf_first - joint_first) / 1e6:.1f} ms")
            print("   This might cause initial visualization issues")
        else:
            print("✅ Transforms published before joint states")
    
    # Check if static transforms exist
    if "/tf_static" in message_times:
        print(f"✅ Static transforms present ({len(message_times['/tf_static'])} messages)")
    else:
        print("⚠️  No static transforms (/tf_static)")
        print("   Consider publishing static transforms for fixed frames")
    
    # Check message synchronization
    if "/joint_states" in message_times and "/robot_state" in message_times:
        joint_times = message_times["/joint_states"]
        robot_times = message_times["/robot_state"]
        
        if len(joint_times) == len(robot_times):
            # Check if they're synchronized
            max_offset = max(abs(j - r) / 1e6 for j, r in zip(joint_times, robot_times))
            if max_offset < 1.0:  # Less than 1ms
                print(f"✅ Joint states and robot states are synchronized (max offset: {max_offset:.2f} ms)")
            else:
                print(f"⚠️  Joint states and robot states have timing offset: {max_offset:.2f} ms")
        else:
            print(f"⚠️  Different number of joint_states ({len(joint_times)}) and robot_state ({len(robot_times)}) messages")
    
    # Check for Foxglove-specific requirements
    print(f"\n💡 Foxglove Tips:")
    
    if "/joint_states" in message_times:
        freq = len(message_times["/joint_states"]) / ((last_timestamp - first_timestamp) / 1e9)
        if freq < 10:
            print(f"⚠️  Low joint state frequency ({freq:.1f} Hz)")
            print("   Foxglove prefers at least 10 Hz for smooth animation")
        else:
            print(f"✅ Good joint state frequency ({freq:.1f} Hz)")
    
    # Check transform tree completeness
    print(f"\n🌳 Transform Tree:")
    tf_frames = set()
    
    with open(filepath, "rb") as f:
        reader = make_reader(f)
        
        # Sample first few transform messages
        tf_count = 0
        for schema, channel, message in reader.iter_messages():
            if channel.topic in ["/tf", "/tf_static"] and tf_count < 5:
                data = json.loads(message.data)
                transforms = data.get("transforms", [])
                
                for tf in transforms:
                    parent = tf.get("header", {}).get("frame_id", "?")
                    child = tf.get("child_frame_id", "?")
                    tf_frames.add(parent)
                    tf_frames.add(child)
                    
                    if tf_count == 0:
                        print(f"   {parent} → {child}")
                
                tf_count += 1
    
    print(f"\n   All frames found: {sorted(tf_frames)}")
    
    # Check if all joint frames are present
    expected_frames = ["world", "base", "fr3_link0", "fr3_link1", "fr3_link2", 
                      "fr3_link3", "fr3_link4", "fr3_link5", "fr3_link6", 
                      "fr3_link7", "fr3_hand"]
    
    missing_frames = [f for f in expected_frames if f not in tf_frames]
    if missing_frames:
        print(f"\n   ⚠️  Missing expected frames: {missing_frames}")
        print("   Foxglove needs transforms for all robot links")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        filepath = sys.argv[1]
    else:
        # Find most recent MCAP file
        recordings_dir = Path.home() / "recordings"
        mcap_files = list(recordings_dir.glob("**/*.mcap"))
        
        if mcap_files:
            filepath = max(mcap_files, key=lambda p: p.stat().st_mtime)
            print(f"Using most recent MCAP file: {filepath.name}")
        else:
            print("❌ No MCAP files found in ~/recordings/")
            sys.exit(1)
    
    check_foxglove_timing(filepath) 