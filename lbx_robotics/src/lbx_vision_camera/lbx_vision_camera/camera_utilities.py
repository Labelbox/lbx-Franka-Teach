import rclpy
import time
import yaml
import numpy as np
import cv2 # For ZED format conversion if needed
from typing import Dict, List, Optional, Tuple, Any
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from .camera_realsense import RealSenseCamera, REALSENSE_AVAILABLE
from .camera_zed import ZEDCamera, ZED_AVAILABLE # Assuming you have a ZED wrapper

# Global logger instance, can be set by the node that uses these utilities
utility_logger = rclpy.logging.get_logger('camera_utilities')

def set_camera_utils_logger(logger_instance):
    global utility_logger
    utility_logger = logger_instance

class FrameData:
    """Simple container for camera frame data."""
    def __init__(self, camera_id: str, color_image=None, depth_image=None, intrinsics=None, header=None, point_cloud=None):
        self.camera_id = camera_id
        self.color_image = color_image
        self.depth_image = depth_image
        self.intrinsics = intrinsics # Should contain fx, fy, cx, cy, model, coeffs, width, height
        self.header = header # ROS Header for timestamping
        self.point_cloud = point_cloud # Optional point cloud data

class CameraManager:
    def __init__(self, config_path: str, node_logger: rclpy.logging.Logger, 
                 discovered_realsense_sns: Optional[List[str]] = None, 
                 discovered_zed_sns: Optional[List[str]] = None):
        self.logger = node_logger
        set_camera_utils_logger(node_logger) # Ensure utility functions also use the node's logger
        self.camera_configs = self._load_config(config_path)
        self.cameras: Dict[str, Any] = {}
        self.unavailable_cameras: Dict[str, Dict] = {} # Store info about cameras that failed to init

        if not self.camera_configs or 'cameras' not in self.camera_configs:
            self.logger.warn("No cameras found in configuration or config is empty.")
            return

        for cam_id, cam_cfg in self.camera_configs.get('cameras', {}).items():
            if not cam_cfg.get('enabled', False):
                self.logger.info(f"Skipping disabled camera: {cam_id}")
                self.unavailable_cameras[cam_id] = {'config': cam_cfg, 'status': 'Disabled in config'}
                continue
            
            cam_type = cam_cfg.get('type', '').lower()
            serial_num = str(cam_cfg.get('serial_number', '')) # Ensure serial is string
            device_id_zed = cam_cfg.get('device_id') # For ZED, device_id might be an int

            try:
                if cam_type == 'realsense':
                    if not REALSENSE_AVAILABLE:
                        self.logger.warn(f"RealSense SDK not available, cannot initialize {cam_id}")
                        self.unavailable_cameras[cam_id] = {'config': cam_cfg, 'status': 'RealSense SDK Missing'}
                        continue
                    # If specific serials were discovered AND the list is not empty, only init if this camera is in the list
                    # If discovered_realsense_sns is None or empty, we bypass this check and attempt to init.
                    if discovered_realsense_sns and serial_num and serial_num not in discovered_realsense_sns:
                        self.logger.warn(f"RealSense camera {cam_id} (SN: {serial_num}) in config but not in the auto-detected list of {len(discovered_realsense_sns)} cameras. Skipping.")
                        self.unavailable_cameras[cam_id] = {'config': cam_cfg, 'status': 'Not in auto-detected list (RealSense)'}
                        continue
                    self.cameras[cam_id] = RealSenseCamera(cam_cfg, self.logger, serial_number_override=serial_num)
                    self.logger.info(f"RealSense camera {cam_id} (SN: {serial_num}) initialized.")
                elif cam_type == 'zed':
                    if not ZED_AVAILABLE:
                        self.logger.warn(f"ZED SDK not available, cannot initialize {cam_id}")
                        self.unavailable_cameras[cam_id] = {'config': cam_cfg, 'status': 'ZED SDK Missing'}
                        continue
                    # Similar auto-detection check for ZED if discovered_zed_sns is used
                    # This part might need adjustment based on how ZED cameras are uniquely identified by discovery
                    # For now, let's assume serial_number is the key for ZED too, or adapt if device_id is better
                    zed_identifier = serial_num if serial_num else str(device_id_zed)
                    if discovered_zed_sns is not None and zed_identifier and zed_identifier not in discovered_zed_sns:
                        self.logger.warn(f"ZED camera {cam_id} (ID: {zed_identifier}) in config but not auto-detected. Skipping.")
                        self.unavailable_cameras[cam_id] = {'config': cam_cfg, 'status': 'Not auto-detected (ZED)'}
                        continue
                    self.cameras[cam_id] = ZEDCamera(cam_cfg, self.logger, device_id_override=device_id_zed)
                    self.logger.info(f"ZED camera {cam_id} (ID: {zed_identifier}) initialized.")
                else:
                    self.logger.warn(f"Unsupported camera type '{cam_type}' for {cam_id}")
                    self.unavailable_cameras[cam_id] = {'config': cam_cfg, 'status': f'Unsupported type: {cam_type}'}
            except Exception as e:
                self.logger.error(f"Failed to initialize camera {cam_id}: {e}")
                self.unavailable_cameras[cam_id] = {'config': cam_cfg, 'status': f'Initialization error: {e}'}

    def _load_config(self, config_path: str) -> Dict:
        try:
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.logger.error(f"Failed to load camera config {config_path}: {e}")
            return {}

    def start(self):
        """Starts all initialized cameras."""
        self.logger.info("Starting CameraManager streams...")
        try:
            # Attempt to start even if some are unavailable, log issues
            active_cameras_copy = dict(self.cameras) # Create a copy for safe iteration and modification
            for cam_id, cam_obj in active_cameras_copy.items():
                try:
                    cam_obj.start_stream()
                    self.logger.info(f"Stream started for camera: {cam_id}")
                except Exception as e:
                    self.logger.error(f"Failed to start stream for {cam_id}: {e}")
                    # Add to unavailable cameras
                    self.unavailable_cameras[cam_id] = {'config': self.camera_configs.get('cameras', {}).get(cam_id), 'status': f"Stream start failed: {e}"}
                    # Remove from active cameras if it's there, as it failed to start
                    if cam_id in self.cameras:
                        del self.cameras[cam_id]
                        self.logger.info(f"Moved {cam_id} from active to unavailable due to stream start failure.")
            
            self.logger.info("CameraManager started.")
        except Exception as e: 
            self.logger.error(f"Error during CameraManager start: {e}")

    def stop(self):
        self.logger.info("Stopping all camera streams...")
        for cam_id, cam_obj in self.cameras.items():
            try:
                cam_obj.stop_stream()
                self.logger.info(f"Stream stopped for camera: {cam_id}")
            except Exception as e:
                self.logger.error(f"Error stopping camera {cam_id}: {e}")
        self.logger.info("All camera streams stopped.")

    def get_all_frames(self, timeout_ms: int = 10) -> Dict[str, FrameData]:
        frames_dict = {}
        for cam_id, cam_obj in self.cameras.items():
            try:
                frames_dict[cam_id] = cam_obj.get_frames(timeout_ms=timeout_ms)
            except Exception as e:
                self.logger.warn(f"Could not get frame from {cam_id}: {e}")
                # Return an empty FrameData or specific error indicator if needed
                frames_dict[cam_id] = FrameData(camera_id=cam_id, header=Header(stamp=rclpy.clock.Clock().now().to_msg()))
        return frames_dict

    def get_frame(self, cam_id: str, timeout_ms: int = 10) -> Optional[FrameData]:
        if cam_id in self.cameras:
            try:
                return self.cameras[cam_id].get_frames(timeout_ms=timeout_ms)
            except Exception as e:
                self.logger.warn(f"Could not get frame from {cam_id}: {e}")
        else:
            self.logger.warn(f"Camera {cam_id} not found or not active.")
        return None

def test_cameras(camera_configs_yaml: Dict, logger: rclpy.logging.Logger, frame_timeout_ms: int = 500, test_duration_sec: float = 1.5) -> Tuple[bool, List[Dict]]:
    logger.info("🔍 Starting camera functionality tests (RealSense & ZED only)...")
    logger.info("="*60)
    results = []
    all_passed = True

    if not (REALSENSE_AVAILABLE or ZED_AVAILABLE):
        logger.warn("Neither RealSense nor ZED SDK available, skipping tests.")
        return True, []

    global_test_settings = camera_configs_yaml.get('global_settings', {}).get('test_settings', {})
    
    # Use YAML config if present, otherwise use function defaults
    effective_test_duration = global_test_settings.get('test_duration_sec', test_duration_sec) 
    effective_frame_timeout = global_test_settings.get('frame_timeout_ms', frame_timeout_ms) # New: allow YAML override
    # Log what effective settings are being used for the test session
    logger.info(f"[Test Params] Effective Duration: {effective_test_duration}s, Effective Frame Timeout: {effective_frame_timeout}ms")

    min_depth_coverage_pct = global_test_settings.get('min_depth_coverage_pct', 20.0)

    active_cameras_to_test = []
    if 'cameras' in camera_configs_yaml:
        for cam_id, cam_cfg in camera_configs_yaml['cameras'].items():
            if cam_cfg.get('enabled', False) and cam_cfg.get('type', '').lower() in ['realsense', 'zed']:
                active_cameras_to_test.append((cam_id, cam_cfg))
            elif cam_cfg.get('enabled', False):
                logger.info(f"⏩ Skipping test for non-RealSense/ZED camera: {cam_id}")
            else:
                logger.info(f"⏭️  Skipping disabled camera: {cam_id}")
    
    if not active_cameras_to_test:
        logger.info("No RealSense or ZED cameras enabled for testing.")
        return True, []

    for cam_id, cam_cfg in active_cameras_to_test:
        cam_type = cam_cfg.get('type', '').lower()
        serial_num = str(cam_cfg.get('serial_number', 'N/A'))
        test_result = {'id': cam_id, 'serial': serial_num, 'type': cam_type, 'status': 'SKIPPED', 'details': 'Not RealSense or ZED'}
        
        logger.info("\n📷 Testing {}: {} (SN: {})...".format(cam_type.capitalize(), cam_id, serial_num))
        time.sleep(0.2) # Small delay for camera to settle before test

        cam_instance = None
        try:
            if cam_type == 'realsense' and REALSENSE_AVAILABLE:
                cam_instance = RealSenseCamera(cam_cfg, logger, serial_number_override=serial_num) # Pass serial
            elif cam_type == 'zed' and ZED_AVAILABLE:
                cam_instance = ZEDCamera(cam_cfg, logger, device_id_override=cam_cfg.get('device_id')) # Pass device_id for ZED
            else:
                results.append(test_result)
                logger.info(f"Unsupported or unavailable SDK for {cam_id}, skipping detailed test.")
                continue

            cam_instance.start_stream(test_mode=True) # Use a test_mode flag if available
            
            # Check connection and basic info
            connected_info = cam_instance.get_connected_camera_info()
            if connected_info:
                logger.info(f"Connected RS: {connected_info}")
            else:
                raise Exception("Failed to get connected camera info.")

            # Frame acquisition test
            start_time = time.monotonic()
            frames_received = 0
            depth_frames_valid = 0
            last_fps_calc_time = start_time
            current_fps = 0.0

            while time.monotonic() - start_time < effective_test_duration:
                frame_data = cam_instance.get_frames(timeout_ms=effective_frame_timeout) # Use effective timeout
                if frame_data and frame_data.color_image is not None:
                    frames_received += 1
                    if frame_data.depth_image is not None:
                        # Simple depth coverage check (optional)
                        # coverage = np.count_nonzero(frame_data.depth_image) / frame_data.depth_image.size * 100
                        # if coverage >= min_depth_coverage_pct:
                        depth_frames_valid +=1
                # else:
                    # logger.warn(f"   ⚠️  Frame timeout: Frame didn't arrive within {effective_frame_timeout}ms")
                    # Removed break to allow test to continue and potentially recover

                # Calculate FPS roughly every 0.5s during test
                if time.monotonic() - last_fps_calc_time > 0.5:
                    elapsed_chunk = time.monotonic() - last_fps_calc_time
                    current_fps = (frames_received - (test_result.get('_last_frames_counted',0)) ) / elapsed_chunk if elapsed_chunk > 0 else 0
                    test_result['_last_frames_counted'] = frames_received
                    last_fps_calc_time = time.monotonic()
                
                time.sleep(0.01) # Brief pause to prevent tight loop burn
            
            # Final FPS calculation
            total_test_time = time.monotonic() - start_time
            final_avg_fps = frames_received / total_test_time if total_test_time > 0 else 0

            if frames_received > 0:
                test_result['status'] = 'PASS'
                test_result['details'] = f"{frames_received} frames @ {final_avg_fps:.1f}fps (Depth valid: {depth_frames_valid})"
                logger.info(f"   ✅ PASS {cam_id} ({cam_type}) - {test_result['details']}")
            else:
                all_passed = False
                test_result['status'] = 'FAIL'
                test_result['details'] = f"0 frames @ {final_avg_fps:.1f}fps"
                logger.info(f"   ❌ FAIL {cam_id} ({cam_type}) - {test_result['details']}")
                if final_avg_fps < 1.0: # Arbitrary low FPS threshold
                     logger.warn(f"   ⚠️  Low FPS: {final_avg_fps:.1f}")

        except Exception as e:
            all_passed = False
            test_result['status'] = 'ERROR'
            test_result['details'] = str(e)
            logger.error(f"   ❌ ERROR testing {cam_id} ({cam_type}): {e}")
        finally:
            if cam_instance:
                try: cam_instance.stop_stream()
                except Exception as e: logger.warn(f"Error stopping test stream for {cam_id}: {e}")
            results.append(test_result)
    
    logger.info("\n" + "="*60)
    logger.info("📊 Camera Test Summary (RealSense & ZED):")
    total_tested_hw = len([res for res in results if res['type'] in ['realsense', 'zed']])
    passed_count = len([res for res in results if res['status'] == 'PASS'])
    failed_count = total_tested_hw - passed_count

    logger.info(f"   Total RealSense/ZED cameras tested: {total_tested_hw}")
    logger.info(f"   Passed: {passed_count}")
    logger.info(f"   Failed: {failed_count}")

    if not all_passed and total_tested_hw > 0:
        logger.error("\n❌ Some RealSense/ZED camera tests FAILED! Please check logs, connections, and configurations.")
    elif total_tested_hw == 0 and camera_configs_yaml.get('cameras'):
        logger.info("No RealSense or ZED cameras were configured and enabled for testing.")
    elif passed_count == total_tested_hw and total_tested_hw > 0:
        logger.info("\n✅ All RealSense/ZED camera tests passed!")
        
    return all_passed, results 