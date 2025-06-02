#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import time

print("🔍 Testing RealSense Camera with Robust Initialization...")

# Create pipeline
pipeline = rs.pipeline()
config = rs.config()
success = False

try:
    # First try - simple color stream only
    print("Trying simple color stream configuration...")
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    # Start streaming
    print("Starting RealSense pipeline...")
    profile = pipeline.start(config)
    device = profile.get_device()
    print(f"Connected to: {device.get_info(rs.camera_info.name)}")
    print(f"Serial Number: {device.get_info(rs.camera_info.serial_number)}")
    
    # Give camera time to initialize
    print("Waiting for camera initialization...")
    time.sleep(2.0)
    
    # Try to get frames with increasing timeout
    print("Attempting to capture frames...")
    frame_count = 0
    start_time = time.time()
    
    for attempt in range(5):
        timeout_ms = 2000 + (attempt * 1000)  # Start with 2s, increase by 1s each attempt
        print(f"Attempt {attempt + 1}: Using {timeout_ms}ms timeout...")
        
        try:
            frames = pipeline.wait_for_frames(timeout_ms=timeout_ms)
            if frames:
                color_frame = frames.get_color_frame()
                if color_frame:
                    frame_count += 1
                    print(f"✅ SUCCESS! Got frame {frame_count} (size: {color_frame.get_width()}x{color_frame.get_height()})")
                    success = True
                    break
                else:
                    print("❌ Got frameset but no color frame")
            else:
                print("❌ No frameset received")
                
        except RuntimeError as e:
            print(f"❌ Timeout after {timeout_ms}ms: {e}")
        except Exception as e:
            print(f"❌ Error: {e}")
            
        time.sleep(0.5)  # Wait between attempts

    elapsed = time.time() - start_time
    
    print(f"\n📊 Results:")
    print(f"Total frames captured: {frame_count}")
    print(f"Time elapsed: {elapsed:.2f} seconds")
    
    if frame_count > 0:
        print("🎉 Camera is working!")
    else:
        print("😞 Camera failed to provide frames")

except Exception as e:
    print(f"💥 Error during camera test: {e}")
finally:
    try:
        pipeline.stop()
        print("🛑 Pipeline stopped")
    except:
        pass

if success:
    print("\n✅ Overall result: Camera test PASSED")
else:
    print("\n❌ Overall result: Camera test FAILED") 