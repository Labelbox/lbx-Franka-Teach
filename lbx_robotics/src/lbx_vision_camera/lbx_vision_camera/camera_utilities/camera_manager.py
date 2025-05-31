#!/usr/bin/env python3
"""
Camera Manager for Labelbox Robotics System
Handles multiple camera types (RealSense, ZED) with an asynchronous
architecture. It continuously captures frames from configured and enabled cameras
and makes them available via thread-safe queues. This manager is designed to be
used by a ROS 2 node (e.g., camera_node) that will then publish these frames.

Features:
- Support for Intel RealSense and ZED cameras.
- Asynchronous frame capture threads, one per camera.
- Thread-safe queues for providing frames to a consuming process/thread.
- Configuration via YAML file.
- Designed for decoupling hardware interaction from data publishing/processing logic.
"""

import threading
import queue
import time
import numpy as np
from typing import Dict, Optional, Tuple, Any
from dataclasses import dataclass
from pathlib import Path
import yaml
import logging

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
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


@dataclass
class CameraFrame:
    """Container for camera frame data"""
    timestamp: float # System time when CameraFrame object is created
    camera_id: str
    color_image: Optional[np.ndarray] = None
    depth_image: Optional[np.ndarray] = None
    intrinsics: Optional[Dict] = None
    metadata: Optional[Dict] = None # Includes backend_timestamp_us if available


class RealsenseCamera:
    """Intel RealSense camera implementation using pyrealsense2"""
    
    def __init__(self, camera_id: str, config: Dict, logger_instance=None):
        self.camera_id = camera_id
        self.config = config
        self.serial_number = config.get('serial_number') # Use .get for safety
        self.pipeline = None
        self.align = None
        self.running = False
        self.logger = logger_instance if logger_instance else logging.getLogger(f"realsense_cam_{camera_id}")
        
    def start(self):
        if not REALSENSE_AVAILABLE:
            self.logger.error("Intel RealSense SDK (pyrealsense2) not available.")
            raise RuntimeError("Intel RealSense SDK not available")
            
        try:
            self.pipeline = rs.pipeline()
            rs_config = rs.config()
            
            if self.serial_number and self.serial_number != "":
                self.logger.info(f"Attempting to enable RealSense device with SN: {self.serial_number}")
                rs_config.enable_device(self.serial_number)
            else:
                self.logger.info("No serial_number in config for RealSense, will use first available device.")
            
            cam_specific_cfg = self.config # Main config for this camera (e.g., cameras.realsense_hand)
            color_cfg = cam_specific_cfg.get('color', {})
            depth_cfg = cam_specific_cfg.get('depth', {})

            color_width = color_cfg.get('width', 640)
            color_height = color_cfg.get('height', 480)
            color_fps = color_cfg.get('fps', 30)
            color_format_str = color_cfg.get('format', 'bgr8').lower() # pyrealsense uses lowercase for formats
            rs_color_format = getattr(rs.format, color_format_str, rs.format.bgr8)

            self.logger.info(f"Configuring RealSense color: {color_width}x{color_height} @ {color_fps}fps, Format: {color_format_str}")
            rs_config.enable_stream(rs.stream.color, color_width, color_height, rs_color_format, color_fps)
            
            if depth_cfg.get('enabled', True):
                depth_width = depth_cfg.get('width', color_width) # Default to color width/height
                depth_height = depth_cfg.get('height', color_height)
                depth_fps = depth_cfg.get('fps', color_fps)
                rs_depth_format_str = depth_cfg.get('format', 'z16').lower()
                rs_depth_format = getattr(rs.format, rs_depth_format_str, rs.format.z16)
                self.logger.info(f"Configuring RealSense depth: {depth_width}x{depth_height} @ {depth_fps}fps, Format: {rs_depth_format_str}")
                rs_config.enable_stream(rs.stream.depth, depth_width, depth_height, rs_depth_format, depth_fps)
                
            profile = self.pipeline.start(rs_config)
            device = profile.get_device()
            actual_sn = device.get_info(rs.camera_info.serial_number)
            if self.serial_number and self.serial_number != "" and self.serial_number != actual_sn:
                self.logger.warn(f"Connected RealSense SN {actual_sn} but expected {self.serial_number}.")
            elif not self.serial_number:
                 self.serial_number = actual_sn # Store actual SN if it was auto-detected
                 self.logger.info(f"Auto-detected RealSense SN: {self.serial_number}")

            self.logger.info(f"Started {self.camera_id}: {device.get_info(rs.camera_info.name)} (SN: {self.serial_number})")
            time.sleep(0.5) # Allow sensor to settle
            
            global_settings = self.config.get('global_settings', {})
            if depth_cfg.get('enabled', True) and global_settings.get('align_depth_to_color', True):
                self.align = rs.align(rs.stream.color)
                self.logger.info("RealSense depth-to-color alignment enabled.")
                            
            self.running = True
        except Exception as e:
            self.logger.error(f"Failed to start RealSense camera {self.camera_id}: {e}")
            if self.pipeline: 
                try: self.pipeline.stop() 
                except Exception: pass # Fixed: Changed catch to except Exception
            raise
            
    def capture_frame(self) -> Optional[CameraFrame]:
        if not self.running or not self.pipeline: return None
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000) 
            if not frames: self.logger.warn(f"Timeout for {self.camera_id}"); return None
            if self.align and frames.get_depth_frame(): frames = self.align.process(frames)
            color_frame = frames.get_color_frame()
            if not color_frame: self.logger.warn(f"No color frame {self.camera_id}"); return None
            color_image = np.asanyarray(color_frame.get_data())
            depth_image, depth_frame_obj = None, frames.get_depth_frame()
            if depth_frame_obj:
                # Apply filters if configured and available
                # Example: if hasattr(self, 'decimation'): depth_frame_obj = self.decimation.process(depth_frame_obj)
                depth_image = np.asanyarray(depth_frame_obj.get_data())
            
            color_intr_obj = color_frame.profile.as_video_stream_profile().intrinsics
            intr_data = {
                'width': color_intr_obj.width, 'height': color_intr_obj.height,
                'fx': color_intr_obj.fx, 'fy': color_intr_obj.fy, 'cx': color_intr_obj.ppx, 'cy': color_intr_obj.ppy,
                'model': str(color_intr_obj.model).lower(), 'coeffs': list(color_intr_obj.coeffs)
            }
            if depth_frame_obj:
                depth_intr_obj = depth_frame_obj.profile.as_video_stream_profile().intrinsics
                intr_data['depth_intrinsics'] = {
                    'width': depth_intr_obj.width, 'height': depth_intr_obj.height,
                    'fx': depth_intr_obj.fx, 'fy': depth_intr_obj.fy, 'cx': depth_intr_obj.ppx, 'cy': depth_intr_obj.ppy,
                    'model': str(depth_intr_obj.model).lower(), 'coeffs': list(depth_intr_obj.coeffs)
                }
            metadata = {
                'frame_number': color_frame.get_frame_number(),
                'timestamp_domain': str(color_frame.get_frame_timestamp_domain()),
                'backend_timestamp_us': color_frame.get_timestamp()
            }
            return CameraFrame(time.time(), self.camera_id, color_image, depth_image, intr_data, metadata)
        except RuntimeError as e:
            if "Frame didn't arrive within" in str(e) or "Timed out" in str(e): self.logger.warn(f"Timeout {self.camera_id}: {e}")
            else: self.logger.error(f"Runtime error {self.camera_id}: {e}")
            return None
        except Exception as e: self.logger.error(f"Capture error {self.camera_id}: {e}"); return None
            
    def stop(self):
        self.running = False
        if self.pipeline: 
            try: self.pipeline.stop()
            except Exception as e: self.logger.error(f"Error stopping {self.camera_id}: {e}")
            self.pipeline = None
        self.logger.info(f"Stopped RealSense camera {self.camera_id}")

class ZEDCamera:
    def __init__(self, camera_id: str, config: Dict, logger_instance=None):
        self.camera_id = camera_id
        self.config = config
        self.serial_number = config.get('serial_number')
        self.zed = None
        self.runtime_params = None
        self.running = False
        self.logger = logger_instance if logger_instance else logging.getLogger(f"zed_cam_{camera_id}")
        if not CV2_AVAILABLE: self.logger.warn("OpenCV not available, ZED color conversion might fail.")
        
    def start(self):
        if not ZED_AVAILABLE:
            self.logger.error("ZED SDK (pyzed) not available."); raise RuntimeError("ZED SDK not available")
        if not self.serial_number: 
             self.logger.error(f"Serial number missing for ZED camera {self.camera_id}."); raise ValueError("ZED SN Missing")
        try:
            self.zed = sl.Camera()
            init_params = sl.InitParameters()
            color_cfg = self.config.get('color', {})
            depth_cfg = self.config.get('depth', {})
            resolution_str = color_cfg.get('resolution', 'HD720').upper()
            init_params.camera_resolution = getattr(sl.RESOLUTION, resolution_str, sl.RESOLUTION.HD720)
            init_params.camera_fps = color_cfg.get('fps', 30)
            if depth_cfg.get('enabled', True):
                depth_mode_str = depth_cfg.get('mode', 'PERFORMANCE').upper() # Default to PERFORMANCE for ZED
                init_params.depth_mode = getattr(sl.DEPTH_MODE, depth_mode_str, sl.DEPTH_MODE.PERFORMANCE)
                init_params.depth_minimum_distance = depth_cfg.get('min_distance_m', 0.3)
            else: init_params.depth_mode = sl.DEPTH_MODE.NONE
            init_params.coordinate_units = sl.UNIT.METER
            init_params.set_from_serial_number(int(self.serial_number))
            
            err = self.zed.open(init_params)
            if err != sl.ERROR_CODE.SUCCESS:
                raise RuntimeError(f"Failed to open ZED {self.serial_number}: {err}")
            cam_info = self.zed.get_camera_information()
            self.logger.info(f"Started {self.camera_id}: ZED {cam_info.camera_model} (SN: {cam_info.serial_number})")
            self.runtime_params = sl.RuntimeParameters()
            # self.runtime_params.confidence_threshold = depth_cfg.get('confidence_threshold', 80) # Example
            self.image_mat, self.depth_mat = sl.Mat(), sl.Mat()
            self.running = True
        except Exception as e:
            self.logger.error(f"Failed to start ZED {self.camera_id}: {e}")
            if self.zed and self.zed.is_opened(): self.zed.close()
            raise
            
    def capture_frame(self) -> Optional[CameraFrame]:
        if not self.running or not self.zed or not self.zed.is_opened(): return None
        try:
            if self.zed.grab(self.runtime_params) != sl.ERROR_CODE.SUCCESS:
                self.logger.warn(f"Grab failed {self.camera_id}"); return None
            self.zed.retrieve_image(self.image_mat, sl.VIEW.LEFT)
            color_image_rgba = self.image_mat.get_data()
            if color_image_rgba is None: self.logger.warn(f"No color data {self.camera_id}"); return None
            if not CV2_AVAILABLE: self.logger.error("OpenCV needed for ZED BGR conversion."); return None
            color_image = cv2.cvtColor(color_image_rgba, cv2.COLOR_BGRA2BGR)
            
            depth_image, depth_cfg = None, self.config.get('depth', {})
            if depth_cfg.get('enabled', True):
                self.zed.retrieve_measure(self.depth_mat, sl.MEASURE.DEPTH)
                depth_data_m = self.depth_mat.get_data()
                if depth_data_m is not None: depth_image = (depth_data_m * 1000.0).astype(np.uint16)
            
            cam_calib = self.zed.get_camera_information().camera_configuration.calibration_parameters.left_cam
            intrinsics = {
                'width': self.image_mat.get_width(), 'height': self.image_mat.get_height(),
                'fx': cam_calib.fx, 'fy': cam_calib.fy, 'cx': cam_calib.cx, 'cy': cam_calib.cy,
                'model': 'plumb_bob', 'coeffs': [cam_calib.k1, cam_calib.k2, cam_calib.p1, cam_calib.p2, cam_calib.k3]
            }
            metadata = {'timestamp_ns': self.zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()}
            return CameraFrame(time.time(), self.camera_id, color_image, depth_image, intrinsics, metadata)
        except Exception as e: self.logger.error(f"Capture error ZED {self.camera_id}: {e}"); return None
            
    def stop(self):
        self.running = False
        if self.zed and self.zed.is_opened():
            try: self.zed.close()
            except Exception as e: self.logger.error(f"Error stopping ZED {self.camera_id}: {e}")
            self.zed = None
        self.logger.info(f"Stopped ZED camera {self.camera_id}")

class CameraManager:
    """Camera Manager for Labelbox Robotics System"""
    def __init__(self, config_path: str, node_logger=None):
        self.config_path = config_path
        self.config_data = self._load_config_data()
        self.cameras: Dict[str, Any] = {}
        self.capture_threads: Dict[str, threading.Thread] = {}
        self.frame_queues: Dict[str, queue.Queue] = {}
        self.running = False
        self.logger = node_logger if node_logger else logging.getLogger("CameraManager")
        
    def _load_config_data(self) -> Dict:
        try:
            with open(self.config_path, 'r') as f: return yaml.safe_load(f)
        except Exception as e:
            self.logger.error(f"Failed to load camera config {self.config_path}: {e}")
            return {"cameras": {}} 
            
    def _init_camera(self, camera_id: str, camera_conf_item: Dict):
        cam_type = camera_conf_item.get('type', 'unknown').lower()
        self.logger.info(f"Initializing camera {camera_id} of type: {cam_type}")
        if cam_type == 'realsense':
            if not REALSENSE_AVAILABLE: self.logger.error("RealSense SDK not found for RealSense camera."); return None
            return RealsenseCamera(camera_id, camera_conf_item, self.logger)
        elif cam_type == 'zed':
            if not ZED_AVAILABLE: self.logger.error("ZED SDK not found for ZED camera."); return None
            return ZEDCamera(camera_id, camera_conf_item, self.logger)
        else:
            self.logger.error(f"Unsupported camera type for {camera_id}: {cam_type}. Only 'realsense' or 'zed' are supported.")
            return None # Removed raise, will just skip this camera
            
    def _capture_worker(self, camera_id: str):
        camera = self.cameras[camera_id]
        frame_q = self.frame_queues[camera_id]
        self.logger.info(f"Capture thread started for {camera_id} ({camera.config.get('type')})")
        while self.running:
            try:
                frame = camera.capture_frame()
                if frame:
                    try: frame_q.put_nowait(frame)
                    except queue.Full:
                        try: frame_q.get_nowait(); frame_q.put_nowait(frame)
                        except queue.Empty: pass 
                        except queue.Full: self.logger.warn(f"Queue full for {camera_id}, frame dropped.")
            except Exception as e:
                self.logger.error(f"Error in capture for {camera_id}: {e}")
                time.sleep(0.1) # Brief pause on error
        self.logger.info(f"Capture thread stopped for {camera_id}")
        
    def start(self):
        if self.running: self.logger.info("CameraManager already running."); return
        self.running = True
        global_settings = self.config_data.get('global_settings', {})
        buffer_size = global_settings.get('buffer_size', 5)

        for camera_id, cam_config_item in self.config_data.get('cameras', {}).items():
            if not cam_config_item.get('enabled', False):
                self.logger.info(f"Skipping disabled camera: {camera_id}"); continue
            try:
                camera_instance = self._init_camera(camera_id, cam_config_item)
                if camera_instance: # Only proceed if camera was successfully initialized
                    camera_instance.start()
                    self.cameras[camera_id] = camera_instance
                    self.frame_queues[camera_id] = queue.Queue(maxsize=buffer_size)
                    thread = threading.Thread(target=self._capture_worker, args=(camera_id,), daemon=True, name=f"CamCap-{camera_id}")
                    thread.start()
                    self.capture_threads[camera_id] = thread
                else:
                    self.logger.warn(f"Could not initialize camera {camera_id} due to missing SDK or unsupported type.")
            except Exception as e:
                self.logger.error(f"Failed to start camera {camera_id}: {e}. Skipped.")
        self.logger.info(f"CameraManager started with {len(self.cameras)} active camera(s).")
        
    def get_frame(self, camera_id: str, timeout: float = 0.01) -> Optional[CameraFrame]:
        if camera_id not in self.frame_queues: 
            # self.logger.warn(f"No queue for camera {camera_id}") # Can be too verbose
            return None
        try: return self.frame_queues[camera_id].get(timeout=timeout)
        except queue.Empty: return None
            
    def get_all_frames(self, timeout: float = 0.01) -> Dict[str, CameraFrame]:
        frames = {}
        for camera_id in list(self.frame_queues.keys()):
            if camera_id in self.cameras and hasattr(self.cameras[camera_id], 'running') and self.cameras[camera_id].running:
                 frame = self.get_frame(camera_id, timeout)
                 if frame: frames[camera_id] = frame
        return frames
        
    def stop(self):
        if not self.running: self.logger.info("CameraManager not running or already stopped."); return
        self.logger.info("Stopping CameraManager...")
        self.running = False
        active_threads = [t for t in self.capture_threads.values() if t.is_alive()]
        for thread in active_threads:
             self.logger.info(f"Joining thread {thread.name}...")
             thread.join(timeout=1.0) # Shorter timeout for joining
             if thread.is_alive(): self.logger.warn(f"Thread {thread.name} did not stop in time.")
        for camera_id, camera_obj in self.cameras.items():
            try:
                if hasattr(camera_obj, 'running') and camera_obj.running: camera_obj.stop()
            except Exception as e: self.logger.error(f"Error stopping camera {camera_id}: {e}")
        self.cameras.clear(); self.capture_threads.clear(); self.frame_queues.clear()
        self.logger.info("CameraManager stopped.") 