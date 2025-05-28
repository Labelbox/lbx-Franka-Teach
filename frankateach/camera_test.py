#!/usr/bin/env python3
"""
Camera Test Module for Labelbox Franka Teach System
Tests actual camera functionality during server startup
"""

import time
import numpy as np
from typing import Dict, List, Optional, Tuple
import logging
import sys

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Camera-specific imports with availability checks
try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False
    logger.warning("Intel RealSense SDK not available - pyrealsense2 not installed")

try:
    import pyzed.sl as sl
    ZED_AVAILABLE = True
except ImportError:
    ZED_AVAILABLE = False
    logger.warning("ZED SDK not available - pyzed not installed")

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    logger.warning("OpenCV not available - opencv-python not installed")


class CameraTestResult:
    """Results from camera testing"""
    def __init__(self, camera_id: str, camera_type: str):
        self.camera_id = camera_id
        self.camera_type = camera_type
        self.connection_ok = False
        self.rgb_capture_ok = False
        self.depth_capture_ok = False
        self.fps_achieved = 0.0
        self.resolution = (0, 0)
        self.error_message = ""
        self.warnings = []
        
    def is_success(self) -> bool:
        """Check if all tests passed"""
        if self.camera_type == "realsense" or self.camera_type == "zed":
            return self.connection_ok and self.rgb_capture_ok and self.depth_capture_ok
        else:
            return self.connection_ok and self.rgb_capture_ok
            
    def __str__(self) -> str:
        status = "✅ PASS" if self.is_success() else "❌ FAIL"
        msg = f"{status} {self.camera_id} ({self.camera_type})"
        if self.connection_ok:
            msg += f" - {self.resolution[0]}x{self.resolution[1]} @ {self.fps_achieved:.1f}fps"
        if self.error_message:
            msg += f" - Error: {self.error_message}"
        return msg


def test_realsense_camera(serial_number: str, config: Dict) -> CameraTestResult:
    """Test Intel RealSense camera functionality"""
    result = CameraTestResult(f"realsense_{serial_number}", "realsense")
    
    if not REALSENSE_AVAILABLE:
        result.error_message = "RealSense SDK not available"
        return result
        
    pipeline = None
    try:
        # Create pipeline
        pipeline = rs.pipeline()
        config_rs = rs.config()
        
        # Enable device by serial number
        config_rs.enable_device(serial_number)
        
        # Configure streams
        width = config.get('width', 640)
        height = config.get('height', 480)
        fps = config.get('fps', 30)
        
        config_rs.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        if config.get('enable_depth', True):
            config_rs.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        
        # Start pipeline
        profile = pipeline.start(config_rs)
        result.connection_ok = True
        
        # Get device info
        device = profile.get_device()
        logger.info(f"Connected to {device.get_info(rs.camera_info.name)} "
                   f"(FW: {device.get_info(rs.camera_info.firmware_version)})")
        
        # Test frame capture
        frame_count = 0
        start_time = time.time()
        test_duration = 2.0  # Test for 2 seconds
        
        while time.time() - start_time < test_duration:
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            
            # Test color frame
            color_frame = frames.get_color_frame()
            if color_frame:
                color_data = np.asanyarray(color_frame.get_data())
                if color_data.size > 0:
                    result.rgb_capture_ok = True
                    result.resolution = (color_frame.get_width(), color_frame.get_height())
            
            # Test depth frame
            if config.get('enable_depth', True):
                depth_frame = frames.get_depth_frame()
                if depth_frame:
                    depth_data = np.asanyarray(depth_frame.get_data())
                    if depth_data.size > 0:
                        result.depth_capture_ok = True
                        
                        # Check depth quality
                        valid_depth = np.count_nonzero(depth_data)
                        total_pixels = depth_data.size
                        depth_coverage = valid_depth / total_pixels
                        
                        if depth_coverage < 0.5:
                            result.warnings.append(f"Low depth coverage: {depth_coverage*100:.1f}%")
            
            frame_count += 1
        
        # Calculate actual FPS
        elapsed = time.time() - start_time
        result.fps_achieved = frame_count / elapsed
        
        if result.fps_achieved < fps * 0.8:  # Less than 80% of target
            result.warnings.append(f"Low FPS: {result.fps_achieved:.1f} (target: {fps})")
            
    except Exception as e:
        result.error_message = str(e)
        logger.error(f"RealSense test failed: {e}")
        
    finally:
        if pipeline:
            try:
                pipeline.stop()
            except:
                pass
                
    return result


def test_zed_camera(serial_number: str, config: Dict) -> CameraTestResult:
    """Test ZED camera functionality"""
    result = CameraTestResult(f"zed_{serial_number}", "zed")
    
    if not ZED_AVAILABLE:
        result.error_message = "ZED SDK not available"
        return result
        
    zed = None
    try:
        # Create ZED camera object
        zed = sl.Camera()
        
        # Set initialization parameters
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720  # Default
        init_params.camera_fps = config.get('fps', 30)
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        init_params.coordinate_units = sl.UNIT.METER
        
        # Set specific camera by serial number
        init_params.set_from_serial_number(int(serial_number))
        
        # Open camera
        err = zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            result.error_message = f"Failed to open ZED camera: {err}"
            return result
            
        result.connection_ok = True
        
        # Get camera info
        cam_info = zed.get_camera_information()
        logger.info(f"Connected to ZED {cam_info.camera_model} "
                   f"(SN: {cam_info.serial_number}, FW: {cam_info.camera_firmware_version})")
        
        # Prepare runtime parameters
        runtime_params = sl.RuntimeParameters()
        runtime_params.confidence_threshold = 100
        runtime_params.texture_confidence_threshold = 100
        
        # Prepare data containers
        image = sl.Mat()
        depth = sl.Mat()
        
        # Test frame capture
        frame_count = 0
        start_time = time.time()
        test_duration = 2.0
        
        while time.time() - start_time < test_duration:
            # Grab frame
            if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
                # Test RGB capture
                zed.retrieve_image(image, sl.VIEW.LEFT)
                if image.get_data() is not None:
                    result.rgb_capture_ok = True
                    result.resolution = (image.get_width(), image.get_height())
                
                # Test depth capture
                if config.get('enable_depth', True):
                    zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
                    if depth.get_data() is not None:
                        result.depth_capture_ok = True
                        
                        # Check depth quality
                        depth_data = depth.get_data()
                        valid_depth = np.count_nonzero(~np.isnan(depth_data))
                        total_pixels = depth_data.size
                        depth_coverage = valid_depth / total_pixels
                        
                        if depth_coverage < 0.5:
                            result.warnings.append(f"Low depth coverage: {depth_coverage*100:.1f}%")
                
                frame_count += 1
            else:
                result.warnings.append("Frame grab failed")
        
        # Calculate actual FPS
        elapsed = time.time() - start_time
        result.fps_achieved = frame_count / elapsed
        
        target_fps = config.get('fps', 30)
        if result.fps_achieved < target_fps * 0.8:
            result.warnings.append(f"Low FPS: {result.fps_achieved:.1f} (target: {target_fps})")
            
    except Exception as e:
        result.error_message = str(e)
        logger.error(f"ZED test failed: {e}")
        
    finally:
        if zed:
            zed.close()
            
    return result


def test_usb_camera(device_id: int, config: Dict) -> CameraTestResult:
    """Test generic USB camera functionality"""
    result = CameraTestResult(f"usb_{device_id}", "usb")
    
    if not CV2_AVAILABLE:
        result.error_message = "OpenCV not available"
        return result
        
    cap = None
    try:
        # Open camera
        cap = cv2.VideoCapture(device_id)
        if not cap.isOpened():
            result.error_message = f"Failed to open camera device {device_id}"
            return result
            
        result.connection_ok = True
        
        # Set camera properties
        width = config.get('width', 640)
        height = config.get('height', 480)
        fps = config.get('fps', 30)
        
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        
        # Get actual properties
        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        result.resolution = (actual_width, actual_height)
        
        # Test frame capture
        frame_count = 0
        start_time = time.time()
        test_duration = 2.0
        
        while time.time() - start_time < test_duration:
            ret, frame = cap.read()
            if ret and frame is not None:
                result.rgb_capture_ok = True
                frame_count += 1
            else:
                result.warnings.append("Frame read failed")
        
        # Calculate actual FPS
        elapsed = time.time() - start_time
        result.fps_achieved = frame_count / elapsed
        
        if result.fps_achieved < fps * 0.8:
            result.warnings.append(f"Low FPS: {result.fps_achieved:.1f} (target: {fps})")
            
    except Exception as e:
        result.error_message = str(e)
        logger.error(f"USB camera test failed: {e}")
        
    finally:
        if cap:
            cap.release()
            
    return result


def test_cameras(camera_configs: Dict) -> Tuple[bool, List[CameraTestResult]]:
    """
    Test all configured cameras
    
    Args:
        camera_configs: Camera configuration dictionary
        
    Returns:
        Tuple of (all_passed, results_list)
    """
    results = []
    
    if not camera_configs or 'cameras' not in camera_configs:
        logger.warning("No cameras configured")
        return True, results
    
    logger.info("🔍 Starting camera tests...")
    logger.info("=" * 60)
    
    # Test each configured camera
    for camera_id, camera_config in camera_configs['cameras'].items():
        if not camera_config.get('enabled', False):
            logger.info(f"⏭️  Skipping disabled camera: {camera_id}")
            continue
            
        camera_type = camera_config.get('type', 'unknown')
        logger.info(f"\n📷 Testing {camera_id} ({camera_type})...")
        
        if camera_type == 'realsense':
            serial = camera_config.get('serial_number')
            if not serial:
                logger.error(f"No serial number for {camera_id}")
                continue
            result = test_realsense_camera(serial, camera_config.get('config', {}))
            
        elif camera_type == 'zed':
            serial = camera_config.get('serial_number')
            if not serial:
                logger.error(f"No serial number for {camera_id}")
                continue
            result = test_zed_camera(serial, camera_config.get('config', {}))
            
        elif camera_type == 'usb' or camera_type == 'fisheye':
            device_id = camera_config.get('device_id', 0)
            result = test_usb_camera(device_id, camera_config.get('config', {}))
            
        else:
            logger.error(f"Unknown camera type: {camera_type}")
            continue
            
        results.append(result)
        logger.info(f"   {result}")
        
        if result.warnings:
            for warning in result.warnings:
                logger.warning(f"   ⚠️  {warning}")
    
    # Summary
    logger.info("\n" + "=" * 60)
    logger.info("📊 Camera Test Summary:")
    
    passed = sum(1 for r in results if r.is_success())
    total = len(results)
    all_passed = passed == total and total > 0
    
    logger.info(f"   Total cameras tested: {total}")
    logger.info(f"   Passed: {passed}")
    logger.info(f"   Failed: {total - passed}")
    
    if all_passed:
        logger.info("\n✅ All camera tests PASSED!")
    else:
        logger.error("\n❌ Some camera tests FAILED!")
        logger.error("   Please check camera connections and configurations")
        
    return all_passed, results


def main():
    """Test function for standalone execution"""
    import yaml
    import argparse
    
    parser = argparse.ArgumentParser(description='Test camera functionality')
    parser.add_argument('--config', type=str, required=True,
                       help='Path to camera configuration YAML file')
    args = parser.parse_args()
    
    # Load config
    with open(args.config, 'r') as f:
        camera_configs = yaml.safe_load(f)
    
    # Run tests
    all_passed, results = test_cameras(camera_configs)
    
    # Exit with appropriate code
    sys.exit(0 if all_passed else 1)


if __name__ == "__main__":
    main() 