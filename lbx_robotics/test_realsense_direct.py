#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import time

print("Testing RealSense Camera Direct Access...")

# Create pipeline
pipeline = rs.pipeline()
config = rs.config()

# Enable streams
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

try:
    # Start streaming
    print("Starting RealSense pipeline...")
    profile = pipeline.start(config)
    device = profile.get_device()
    print(f"Connected to: {device.get_info(rs.camera_info.name)}")
    print(f"Serial Number: {device.get_info(rs.camera_info.serial_number)}")
    print(f"Firmware: {device.get_info(rs.camera_info.firmware_version)}")
    
    # Try to get some frames
    print("Attempting to capture frames...")
    frame_count = 0
    start_time = time.time()
    
    for i in range(10):  # Try 10 frames to test quickly
        try:
            frames = pipeline.wait_for_frames(timeout_ms=1000)  # 1 second timeout
            if frames:
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                
                if color_frame and depth_frame:
                    frame_count += 1
                    print(f"Frame {frame_count}: Got both color and depth ✅")
                elif color_frame:
                    frame_count += 1
                    print(f"Frame {frame_count}: Got color only ⚠️")
                
        except RuntimeError as e:
            print(f"Frame {i}: Timeout - {e}")
            break
        except Exception as e:
            print(f"Frame {i}: Error - {e}")
            break
            
        time.sleep(0.1)  # Short delay

    elapsed = time.time() - start_time
    fps = frame_count / elapsed if elapsed > 0 else 0
    
    print(f"\nResults:")
    print(f"Total frames captured: {frame_count}")
    print(f"Time elapsed: {elapsed:.2f} seconds") 
    print(f"Average FPS: {fps:.2f}")
    
    if frame_count > 0:
        print("✅ Camera is working!")
    else:
        print("❌ No frames captured")

except Exception as e:
    print(f"Error: {e}")
finally:
    try:
        pipeline.stop()
        print("Pipeline stopped")
    except:
        pass 