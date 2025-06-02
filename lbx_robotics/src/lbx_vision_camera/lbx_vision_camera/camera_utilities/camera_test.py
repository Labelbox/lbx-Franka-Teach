#!/usr/bin/env python3
"""
Camera Test Module for Labelbox Robotics System
Tests actual camera functionality for RealSense or ZED cameras during server startup.
"""

import time
import numpy as np
from typing import Dict, List, Optional, Tuple
import logging
import sys

# Camera-specific imports
try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False

try:
    import pyzed.sl as sl
    ZED_AVAILABLE = True
except ImportError:
    ZED_AVAILABLE = False

# CV2 is still needed by ZED camera for color conversion
try:
    import cv2 # Keep for ZED if it uses it internally for data conversion
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

class CameraTestResult:
    """Results from camera testing"""
    def __init__(self, camera_id: str, camera_type: str, config_data: Optional[Dict] = None):
        self.camera_id = camera_id
        self.camera_type = camera_type
        self.config = config_data if config_data else {}
        self.connection_ok = False
        self.rgb_capture_ok = False
        self.depth_capture_ok = False 
        self.fps_achieved = 0.0
        self.resolution = (0, 0)
        self.error_message = ""
        self.warnings = []
        
    def expects_depth(self) -> bool:
        if self.camera_type in ["realsense", "zed"]:
            return self.config.get("depth", {}).get("enabled", True)
        return False

    def is_success(self) -> bool:
        if self.expects_depth():
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

def test_realsense_camera(serial_number: str, cam_specific_config: Dict, global_config: Dict, logger_instance=None) -> CameraTestResult:
    logger = logger_instance if logger_instance else logging.getLogger(f"test_realsense_{serial_number}")
    result = CameraTestResult(f"realsense_{serial_number}", "realsense", cam_specific_config)
    if not REALSENSE_AVAILABLE: result.error_message = "RealSense SDK missing"; return result
    pipeline = None
    pipeline_started = False  # Track if pipeline was successfully started
    try:
        pipeline = rs.pipeline(); rs_conf = rs.config()
        if serial_number: rs_conf.enable_device(serial_number)
        color_cfg = cam_specific_config.get('color', {})
        depth_cfg = cam_specific_config.get('depth', {})
        expects_depth = result.expects_depth()
        w, h, fps = color_cfg.get('width',640), color_cfg.get('height',480), color_cfg.get('fps',30)
        rs_conf.enable_stream(rs.stream.color, w, h, getattr(rs.format, color_cfg.get('format','bgr8').lower(), rs.format.bgr8), fps)
        if expects_depth:
            dw, dh,dfps = depth_cfg.get('width',w), depth_cfg.get('height',h), depth_cfg.get('fps',fps)
            rs_conf.enable_stream(rs.stream.depth, dw, dh, getattr(rs.format, depth_cfg.get('format','z16').lower(), rs.format.z16), dfps)
        profile = pipeline.start(rs_conf)
        pipeline_started = True  # Mark that pipeline was successfully started
        result.connection_ok = True
        dev = profile.get_device(); logger.info(f"Connected RS: {dev.get_info(rs.camera_info.name)} SN: {dev.get_info(rs.camera_info.serial_number)}")
        fc, t_start = 0, time.time(); dur = global_config.get('test_duration_sec',2.0); min_depth_cov = global_config.get('min_depth_coverage_pct',30.)/100.
        for _ in range(int(fps*dur)):
            try:
                frames = pipeline.wait_for_frames(1000)
                if not frames: continue
                cf = frames.get_color_frame()
                if cf: result.rgb_capture_ok=True; result.resolution=(cf.get_width(),cf.get_height())
                if expects_depth:
                    df = frames.get_depth_frame()
                    if df: 
                        dd = np.asanyarray(df.get_data())
                        if dd.size > 0: result.depth_capture_ok=True
                        if np.count_nonzero(dd)/dd.size < min_depth_cov: result.warnings.append(f"Low depth coverage: {np.count_nonzero(dd)/dd.size*100:.1f}%")
                fc += 1
            except RuntimeError as e: result.warnings.append(f"Frame timeout: {e}"); break
            if time.time()-t_start > dur+1: break
        elapsed = time.time()-t_start
        if elapsed > 0: result.fps_achieved = fc/elapsed
        if result.fps_achieved < fps*global_config.get('fps_tolerance_factor',0.8): result.warnings.append(f"Low FPS: {result.fps_achieved:.1f}")
    except Exception as e: result.error_message=str(e); logger.error(f"RS test {serial_number} error: {e}")
    finally: 
        if pipeline and pipeline_started:  # Only stop if pipeline was started
            try:
                pipeline.stop()
            except Exception as e:
                logger.warning(f"Error stopping pipeline: {e}")
    return result

def test_zed_camera(serial_number: str, cam_specific_config: Dict, global_config: Dict, logger_instance=None) -> CameraTestResult:
    logger = logger_instance if logger_instance else logging.getLogger(f"test_zed_{serial_number}")
    result = CameraTestResult(f"zed_{serial_number}", "zed", cam_specific_config)
    if not ZED_AVAILABLE: result.error_message = "ZED SDK missing"; return result
    if not CV2_AVAILABLE: result.warnings.append("OpenCV not found, ZED color conversion might be an issue in main node.")
    zed=None
    try:
        zed = sl.Camera(); init_params = sl.InitParameters()
        color_cfg = cam_specific_config.get('color', {})
        depth_cfg = cam_specific_config.get('depth', {})
        expects_depth = result.expects_depth()
        init_params.camera_resolution = getattr(sl.RESOLUTION, color_cfg.get('resolution','HD720').upper(), sl.RESOLUTION.HD720)
        init_params.camera_fps = color_cfg.get('fps',30)
        if expects_depth:
            init_params.depth_mode = getattr(sl.DEPTH_MODE, depth_cfg.get('mode','PERFORMANCE').upper(), sl.DEPTH_MODE.PERFORMANCE)
            init_params.depth_minimum_distance = depth_cfg.get('min_distance_m',0.3)
        else: init_params.depth_mode = sl.DEPTH_MODE.NONE
        init_params.coordinate_units = sl.UNIT.METER
        init_params.set_from_serial_number(int(serial_number))
        if zed.open(init_params) != sl.ERROR_CODE.SUCCESS: result.error_message = f"Failed to open ZED SN {serial_number}"; return result
        result.connection_ok=True; cam_info=zed.get_camera_information(); logger.info(f"Connected ZED: {cam_info.camera_model} SN: {cam_info.serial_number}")
        rt_params = sl.RuntimeParameters(); img_mat, dep_mat = sl.Mat(), sl.Mat()
        fc,t_start = 0,time.time(); dur=global_config.get('test_duration_sec',2.0); min_depth_cov=global_config.get('min_depth_coverage_pct',30.)/100.
        target_fps = color_cfg.get('fps',30)
        for _ in range(int(target_fps*dur)):
            if zed.grab(rt_params) == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_image(img_mat, sl.VIEW.LEFT)
                if img_mat.get_data() is not None: result.rgb_capture_ok=True; result.resolution=(img_mat.get_width(),img_mat.get_height())
                if expects_depth:
                    zed.retrieve_measure(dep_mat, sl.MEASURE.DEPTH); dd = dep_mat.get_data()
                    if dd is not None: result.depth_capture_ok=True
                    if np.count_nonzero(~np.isnan(dd) & ~np.isinf(dd))/dd.size < min_depth_cov: result.warnings.append(f"Low ZED depth coverage")
                fc+=1
            else: result.warnings.append("ZED frame grab failed"); break
            if time.time()-t_start > dur+1: break
        elapsed=time.time()-t_start
        if elapsed>0: result.fps_achieved=fc/elapsed
        if result.fps_achieved < target_fps*global_config.get('fps_tolerance_factor',0.8): result.warnings.append(f"Low ZED FPS: {result.fps_achieved:.1f}")
    except Exception as e: result.error_message=str(e); logger.error(f"ZED test {serial_number} error: {e}")
    finally: 
        if zed and zed.is_opened(): zed.close()
    return result

def test_cameras(camera_configs_yaml: Dict, node_logger=None) -> Tuple[bool, List[CameraTestResult]]:
    results = []
    logger = node_logger if node_logger else logging.getLogger("camera_tests")
    
    if not camera_configs_yaml or 'cameras' not in camera_configs_yaml:
        logger.warning("No cameras found in the provided configuration YAML.")
        return True, results 
    
    logger.info("🔍 Starting camera functionality tests (RealSense & ZED only)...")
    logger.info("=" * 60)
    
    global_test_config = camera_configs_yaml.get('global_settings', {}).get('test_settings', {
        'test_duration_sec': 2.0,
        'fps_tolerance_factor': 0.8, 
        'min_depth_coverage_pct': 30.0 
    })

    for cam_id_cfg, cam_config_data in camera_configs_yaml.get('cameras', {}).items():
        if not cam_config_data.get('enabled', False):
            logger.info(f"⏭️  Skipping disabled camera: {cam_id_cfg}")
            continue
            
        cam_type = cam_config_data.get('type', 'unknown').lower()
        test_result_item = None # Renamed from test_result_obj

        if cam_type == 'realsense':
            serial = cam_config_data.get('serial_number')
            if not serial or serial == "": 
                logger.error(f"Serial number missing for RealSense camera {cam_id_cfg}. Skipping test.")
                temp_res = CameraTestResult(cam_id_cfg, cam_type, cam_config_data); temp_res.error_message = "Missing serial in config."; results.append(temp_res); continue
            logger.info(f"\n📷 Testing RealSense: {cam_id_cfg} (SN: {serial})...")
            test_result_item = test_realsense_camera(serial, cam_config_data, global_test_config, logger)
        elif cam_type == 'zed':
            serial = cam_config_data.get('serial_number')
            if not serial: 
                logger.error(f"Serial number missing for ZED camera {cam_id_cfg}. Skipping test.")
                temp_res = CameraTestResult(cam_id_cfg, cam_type, cam_config_data); temp_res.error_message = "Missing serial in config."; results.append(temp_res); continue
            logger.info(f"\n📷 Testing ZED: {cam_id_cfg} (SN: {serial})...")
            test_result_item = test_zed_camera(serial, cam_config_data, global_test_config, logger)
        else:
            logger.info(f"⏭️  Skipping unsupported camera type for testing: {cam_id_cfg} (type: {cam_type})")
            # Optionally create a minimal test result indicating skipped status if needed for diagnostics
            # temp_res = CameraTestResult(cam_id_cfg, cam_type, cam_config_data)
            # temp_res.error_message = f"Unsupported type for automated test: {cam_type}"
            # results.append(temp_res)
            continue # Skip to next camera if not RealSense or ZED
        
        if test_result_item:
            results.append(test_result_item)
            logger.info(f"   {test_result_item}") # Log individual result summary
            if test_result_item.warnings:
                for warning in test_result_item.warnings:
                    logger.warning(f"   ⚠️  {warning}")
    
    logger.info("\n" + "=" * 60)
    logger.info("📊 Camera Test Summary (RealSense & ZED):")
    
    active_results = [r for r in results if r.camera_type in ['realsense', 'zed'] and (r.connection_ok or r.error_message)]
    passed_count = sum(1 for r in active_results if r.is_success())
    total_tested = len(active_results)
    all_passed = (passed_count == total_tested) if total_tested > 0 else True 
    
    logger.info(f"   Total RealSense/ZED cameras tested: {total_tested}")
    logger.info(f"   Passed: {passed_count}")
    logger.info(f"   Failed: {total_tested - passed_count}")
    
    if all_passed:
        logger.info("\n✅ All attempted RealSense/ZED camera tests PASSED or were skipped due to missing SDKs!")
    else:
        logger.error("\n❌ Some RealSense/ZED camera tests FAILED! Please check logs, connections, and configurations.")
    return all_passed, results

if __name__ == "__main__":
    import yaml
    import argparse
    
    parser = argparse.ArgumentParser(description='Test camera functionality based on a YAML configuration file.')
    parser.add_argument('--config', type=str, required=True,
                       help='Path to camera configuration YAML file (e.g., lbx_robotics/configs/sensors/cameras_setup.yaml)')
    cli_args = parser.parse_args()
    
    standalone_logger = logging.getLogger("StandaloneCameraTest")
    standalone_logger.setLevel(logging.INFO)
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(logging.Formatter('%(levelname)s: %(name)s - %(message)s'))
    standalone_logger.addHandler(handler)

    try:
        with open(cli_args.config, 'r') as f:
            camera_configs = yaml.safe_load(f)
    except Exception as e:
        standalone_logger.error(f"Failed to load camera config file {cli_args.config}: {e}")
        sys.exit(1)
    
    all_passed_main, _ = test_cameras(camera_configs, standalone_logger)
    sys.exit(0 if all_passed_main else 1) 