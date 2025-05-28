#!/usr/bin/env python3
"""
Test MCAP file for Foxglove compatibility
"""

import sys
import json
from pathlib import Path
import mcap
from mcap.reader import make_reader


def check_mcap_foxglove_compatibility(filepath: str):
    """Check MCAP file for Foxglove compatibility issues"""
    print(f"🔍 Checking MCAP file for Foxglove compatibility...")
    print(f"   File: {filepath}")
    print("=" * 60)
    
    issues = []
    warnings = []
    
    with open(filepath, "rb") as f:
        reader = make_reader(f)
        
        # Check schemas
        print("\n📋 Checking schemas...")
        for schema_id, schema in reader.get_summary().schemas.items():
            print(f"   Schema: {schema.name}")
            
            # Check for camera-related schemas
            if "Camera" in schema.name or "Image" in schema.name:
                try:
                    schema_data = json.loads(schema.data.decode('utf-8'))
                    print(f"      Encoding: {schema.encoding}")
                    
                    # Check for distortion model in calibration
                    if schema.name == "foxglove.CameraCalibration":
                        if 'properties' in schema_data:
                            props = schema_data['properties']
                            if 'distortion_model' in props:
                                print("      ✅ Has distortion_model field")
                except Exception as e:
                    issues.append(f"Failed to parse schema {schema.name}: {e}")
        
        # Check messages
        print("\n📊 Checking messages...")
        camera_messages = 0
        transform_messages = 0
        distortion_models = set()
        camera_frames = set()
        
        for schema, channel, msg in reader.iter_messages():
            # Check camera info messages
            if "camera_info" in channel.topic:
                camera_messages += 1
                try:
                    data = json.loads(msg.data)
                    model = data.get('distortion_model', 'unknown')
                    distortion_models.add(model)
                    frame_id = data.get('frame_id', 'unknown')
                    camera_frames.add(frame_id)
                except:
                    pass
                    
            # Check transform messages
            elif channel.topic in ["/tf", "/tf_static"]:
                transform_messages += 1
                try:
                    data = json.loads(msg.data)
                    for transform in data.get('transforms', []):
                        child_frame = transform.get('child_frame_id', '')
                        if 'camera' in child_frame:
                            camera_frames.add(child_frame)
                except:
                    pass
                    
            # Check depth images
            elif "depth" in channel.topic:
                try:
                    data = json.loads(msg.data)
                    encoding = data.get('encoding', 'unknown')
                    if encoding != '16UC1':
                        warnings.append(f"Depth image has encoding '{encoding}', expected '16UC1'")
                    
                    # Check if data is properly base64 encoded
                    import base64
                    try:
                        decoded = base64.b64decode(data.get('data', ''))
                        expected_size = data.get('width', 0) * data.get('height', 0) * 2
                        if len(decoded) != expected_size:
                            issues.append(f"Depth data size mismatch: got {len(decoded)}, expected {expected_size}")
                    except:
                        issues.append("Failed to decode depth image data")
                except Exception as e:
                    issues.append(f"Failed to parse depth message: {e}")
        
        # Report findings
        print(f"\n📷 Camera Info Messages: {camera_messages}")
        print(f"🔄 Transform Messages: {transform_messages}")
        
        if distortion_models:
            print(f"\n🔍 Distortion Models Found:")
            for model in distortion_models:
                if 'brown_conrady' in model.lower():
                    print(f"   ⚠️  {model} - Not standard, should be converted to 'plumb_bob'")
                else:
                    print(f"   ✅ {model}")
        
        if camera_frames:
            print(f"\n🖼️  Camera Frames:")
            for frame in sorted(camera_frames):
                print(f"   {frame}")
        
        # Check for missing transforms
        print(f"\n🔗 Transform Check:")
        camera_frame_ids = {f for f in camera_frames if f.startswith('camera_')}
        for frame_id in camera_frame_ids:
            if frame_id not in camera_frames:
                issues.append(f"Missing transform for frame '{frame_id}'")
            else:
                print(f"   ✅ Transform exists for {frame_id}")
    
    # Summary
    print("\n" + "=" * 60)
    if issues:
        print("❌ ISSUES FOUND:")
        for issue in issues:
            print(f"   - {issue}")
    else:
        print("✅ No critical issues found")
        
    if warnings:
        print("\n⚠️  WARNINGS:")
        for warning in warnings:
            print(f"   - {warning}")
    
    print("\n💡 Foxglove Compatibility Tips:")
    print("   - Use standard distortion models (plumb_bob, rational_polynomial)")
    print("   - Ensure all camera frames have transforms to world/base")
    print("   - Use proper encoding for depth images (16UC1)")
    print("   - Base64 encode binary data in JSON messages")
    
    return len(issues) == 0


def main():
    if len(sys.argv) < 2:
        # Find latest MCAP file
        recordings_dir = Path.home() / "recordings"
        mcap_files = []
        
        for subdir in ["success", "failure"]:
            dir_path = recordings_dir / subdir
            if dir_path.exists():
                mcap_files.extend(dir_path.glob("*.mcap"))
        
        if not mcap_files:
            print("❌ No MCAP files found")
            return 1
            
        filepath = max(mcap_files, key=lambda p: p.stat().st_mtime)
    else:
        filepath = sys.argv[1]
    
    if check_mcap_foxglove_compatibility(str(filepath)):
        return 0
    else:
        return 1


if __name__ == "__main__":
    sys.exit(main()) 