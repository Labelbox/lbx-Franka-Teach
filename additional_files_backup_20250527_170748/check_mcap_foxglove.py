#!/usr/bin/env python3
"""Check MCAP file for Foxglove compatibility"""

import json
import mcap
from mcap.reader import make_reader
from pathlib import Path
import sys

def check_mcap_foxglove(filepath):
    """Check MCAP file for Foxglove compatibility"""
    
    print(f"\n🔍 Checking MCAP file for Foxglove: {filepath}")
    print(f"   File size: {Path(filepath).stat().st_size / 1024:.1f} KB")
    
    # Track what we find
    has_joint_states = False
    has_tf_messages = False
    has_robot_description = False
    has_urdf_attachment = False
    
    topics_found = set()
    schemas_found = {}
    attachments_found = []
    
    with open(filepath, "rb") as f:
        reader = make_reader(f)
        
        # Check summary
        summary = reader.get_summary()
        if summary:
            print(f"\n📊 MCAP Summary:")
            print(f"   Message count: {summary.statistics.message_count}")
            print(f"   Schema count: {summary.statistics.schema_count}")
            print(f"   Channel count: {summary.statistics.channel_count}")
            print(f"   Attachment count: {summary.statistics.attachment_count}")
        
        # Check schemas
        print(f"\n📋 Schemas:")
        for schema_id, schema in reader.get_summary().schemas.items():
            schemas_found[schema.name] = schema.encoding
            print(f"   - {schema.name} (encoding: {schema.encoding})")
        
        # Check channels/topics
        print(f"\n📡 Topics:")
        for channel_id, channel in reader.get_summary().channels.items():
            topics_found.add(channel.topic)
            # Get schema name from schema_id
            schema_name = "None"
            if channel.schema_id and channel.schema_id in reader.get_summary().schemas:
                schema_name = reader.get_summary().schemas[channel.schema_id].name
            print(f"   - {channel.topic} (schema: {schema_name})")
            
            if channel.topic == "/joint_states":
                has_joint_states = True
            elif channel.topic == "/tf":
                has_tf_messages = True
            elif channel.topic == "/robot_description":
                has_robot_description = True
        
        # Check attachments
        print(f"\n📎 Attachments:")
        for attachment in reader.iter_attachments():
            attachments_found.append(attachment.name)
            print(f"   - {attachment.name} ({len(attachment.data)} bytes)")
            if attachment.name == "robot_description":
                has_urdf_attachment = True
                # Check if it's valid URDF
                try:
                    urdf_content = attachment.data.decode('utf-8')
                    if '<robot' in urdf_content and '</robot>' in urdf_content:
                        print(f"     ✅ Valid URDF detected")
                    else:
                        print(f"     ❌ Invalid URDF content")
                except:
                    print(f"     ❌ Failed to decode URDF")
        
        # Sample some messages
        print(f"\n📨 Sample Messages:")
        message_count = 0
        for schema, channel, message in reader.iter_messages():
            if message_count < 3:
                print(f"\n   Message #{message_count + 1} on {channel.topic}:")
                print(f"   Timestamp: {message.log_time}")
                
                if channel.topic == "/joint_states":
                    data = json.loads(message.data)
                    joint_names = data.get("name", [])
                    positions = data.get("position", [])
                    print(f"   Joints: {len(joint_names)}")
                    if positions:
                        print(f"   Has positions: Yes ({len(positions)} values)")
                    else:
                        print(f"   Has positions: No")
                        
                elif channel.topic == "/tf":
                    data = json.loads(message.data)
                    transforms = data.get("transforms", [])
                    print(f"   Transforms: {len(transforms)}")
                    for tf in transforms[:2]:
                        parent = tf.get("header", {}).get("frame_id", "?")
                        child = tf.get("child_frame_id", "?")
                        print(f"     - {parent} → {child}")
                
                message_count += 1
    
    # Diagnosis
    print(f"\n🩺 Foxglove Compatibility Check:")
    
    if has_joint_states:
        print("✅ Joint states topic found (/joint_states)")
    else:
        print("❌ No /joint_states topic - robot won't animate")
    
    if has_tf_messages:
        print("✅ Transform messages found (/tf)")
    else:
        print("❌ No /tf topic - coordinate frames missing")
    
    if has_robot_description or has_urdf_attachment:
        print("✅ Robot description found")
        if has_urdf_attachment:
            print("   - As attachment (preferred)")
        if has_robot_description:
            print("   - As topic")
    else:
        print("❌ No robot description - robot model won't load")
    
    # Check for common issues
    print(f"\n⚠️  Potential Issues:")
    
    if "sensor_msgs/msg/JointState" not in schemas_found:
        print("- Missing sensor_msgs/msg/JointState schema")
    
    if "tf2_msgs/msg/TFMessage" not in schemas_found:
        print("- Missing tf2_msgs/msg/TFMessage schema")
    
    if not has_urdf_attachment:
        print("- No URDF attachment (robot_description)")
        print("  Foxglove needs this to visualize the robot")
    
    # Foxglove requirements summary
    print(f"\n📋 Foxglove Requirements Summary:")
    print(f"   1. URDF: {'✅' if (has_robot_description or has_urdf_attachment) else '❌'}")
    print(f"   2. Joint States: {'✅' if has_joint_states else '❌'}")
    print(f"   3. Transforms: {'✅' if has_tf_messages else '❌'}")
    
    return has_joint_states and (has_robot_description or has_urdf_attachment) and has_tf_messages

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
    
    is_compatible = check_mcap_foxglove(filepath)
    
    if is_compatible:
        print("\n✅ MCAP file should work in Foxglove!")
    else:
        print("\n❌ MCAP file may have issues in Foxglove") 