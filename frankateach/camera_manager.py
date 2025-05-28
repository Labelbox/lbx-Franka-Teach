#!/usr/bin/env python3
"""
Camera Manager for Labelbox Franka Teach System
Handles multiple camera types with async architecture for high-performance recording

Features:
- Supports Intel RealSense, ZED, and generic USB cameras
- Asynchronous frame capture and processing
- Thread-safe queue-based architecture
- Direct integration with MCAP recording
- Minimal latency and CPU overhead
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
    print("⚠️  Intel RealSense SDK not available")

try:
    import pyzed.sl as sl
    ZED_AVAILABLE = True
except ImportError:
    ZED_AVAILABLE = False
    print("⚠️  ZED SDK not available")

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    print("⚠️  OpenCV not available")

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class CameraFrame:
    """Container for camera frame data"""
    timestamp: float
    camera_id: str
    color_image: Optional[np.ndarray] = None
    depth_image: Optional[np.ndarray] = None
    intrinsics: Optional[Dict] = None
    metadata: Optional[Dict] = None


class RealsenseCamera:
    """Intel RealSense camera implementation using pyrealsense2"""
    
    def __init__(self, camera_id: str, config: Dict):
        self.camera_id = camera_id
        self.config = config
        self.serial_number = config['serial_number']
        self.pipeline = None
        self.align = None
        self.running = False
        
    def start(self):
        """Start the RealSense camera pipeline"""
        if not REALSENSE_AVAILABLE:
            raise RuntimeError("Intel RealSense SDK not available")
            
        try:
            # Create pipeline
            self.pipeline = rs.pipeline()
            config = rs.config()
            
            # Enable device by serial number
            config.enable_device(self.serial_number)
            
            # Configure streams
            cam_config = self.config.get('config', {})
            width = cam_config.get('width', 640)
            height = cam_config.get('height', 480)
            fps = cam_config.get('fps', 30)
            
            # Enable color stream
            config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
            
            # Enable depth stream if requested
            if cam_config.get('enable_depth', True):
                config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
                
            # Start pipeline
            profile = self.pipeline.start(config)
            
            # Get device info
            device = profile.get_device()
            logger.info(f"Started {self.camera_id}: {device.get_info(rs.camera_info.name)} "
                       f"(SN: {self.serial_number})")
            
            # Give camera time to initialize
            time.sleep(0.5)
            
            # Create align object if depth is enabled
            if cam_config.get('enable_depth', True) and cam_config.get('align_depth_to_color', True):
                self.align = rs.align(rs.stream.color)
                
            # Configure post-processing filters if depth is enabled
            if cam_config.get('enable_depth', True):
                self.decimation = rs.decimation_filter()
                self.decimation.set_option(rs.option.filter_magnitude, 
                                         cam_config.get('decimation_filter', 2))
                
                if cam_config.get('spatial_filter', True):
                    self.spatial = rs.spatial_filter()
                    self.spatial.set_option(rs.option.filter_magnitude, 2)
                    self.spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
                    self.spatial.set_option(rs.option.filter_smooth_delta, 20)
                    
                if cam_config.get('temporal_filter', True):
                    self.temporal = rs.temporal_filter()
                    
                if cam_config.get('hole_filling_filter', 1) > 0:
                    self.hole_filling = rs.hole_filling_filter(cam_config.get('hole_filling_filter', 1))
                    
            self.running = True
            
        except Exception as e:
            logger.error(f"Failed to start {self.camera_id}: {e}")
            raise
            
    def capture_frame(self) -> Optional[CameraFrame]:
        """Capture a frame from the camera"""
        if not self.running or not self.pipeline:
            return None
            
        try:
            # Wait for frames with timeout
            frames = self.pipeline.wait_for_frames(timeout_ms=100)
            
            # Align depth to color if enabled
            if self.align and frames.get_depth_frame():
                frames = self.align.process(frames)
                
            # Get color frame
            color_frame = frames.get_color_frame()
            if not color_frame:
                return None
                
            # Convert to numpy array
            color_image = np.asanyarray(color_frame.get_data())
            
            # Get depth frame if available
            depth_image = None
            depth_frame = frames.get_depth_frame()
            if depth_frame:
                # Apply filters
                if hasattr(self, 'decimation'):
                    depth_frame = self.decimation.process(depth_frame)
                if hasattr(self, 'spatial'):
                    depth_frame = self.spatial.process(depth_frame)
                if hasattr(self, 'temporal'):
                    depth_frame = self.temporal.process(depth_frame)
                if hasattr(self, 'hole_filling'):
                    depth_frame = self.hole_filling.process(depth_frame)
                    
                depth_image = np.asanyarray(depth_frame.get_data())
                
            # Get intrinsics
            intrinsics = None
            color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
            
            # Get depth intrinsics if we have depth
            depth_intrinsics_data = None
            if depth_frame:
                depth_profile = depth_frame.profile.as_video_stream_profile()
                depth_intrinsics_obj = depth_profile.intrinsics
                depth_intrinsics_data = {
                    'width': depth_intrinsics_obj.width,
                    'height': depth_intrinsics_obj.height,
                    'fx': depth_intrinsics_obj.fx,
                    'fy': depth_intrinsics_obj.fy,
                    'cx': depth_intrinsics_obj.ppx,
                    'cy': depth_intrinsics_obj.ppy,
                    'model': str(depth_intrinsics_obj.model),
                    'coeffs': list(depth_intrinsics_obj.coeffs)
                }
            
            # Store both color and depth intrinsics
            intrinsics = {
                'width': color_intrinsics.width,
                'height': color_intrinsics.height,
                'fx': color_intrinsics.fx,
                'fy': color_intrinsics.fy,
                'cx': color_intrinsics.ppx,
                'cy': color_intrinsics.ppy,
                'model': str(color_intrinsics.model),
                'coeffs': list(color_intrinsics.coeffs),
                # Add depth intrinsics if available
                'depth_intrinsics': depth_intrinsics_data,
                # Store decimation factor for reference
                'decimation_factor': self.config.get('config', {}).get('decimation_filter', 2) if depth_intrinsics_data else None
            }
            
            # Get metadata
            metadata = {
                'frame_number': color_frame.get_frame_number(),
                'timestamp_ms': color_frame.get_timestamp()
            }
            
            return CameraFrame(
                timestamp=time.time(),
                camera_id=self.camera_id,
                color_image=color_image,
                depth_image=depth_image,
                intrinsics=intrinsics,
                metadata=metadata
            )
            
        except Exception as e:
            logger.error(f"Error capturing frame from {self.camera_id}: {e}")
            return None
            
    def stop(self):
        """Stop the camera"""
        self.running = False
        if self.pipeline:
            try:
                self.pipeline.stop()
            except:
                pass
            self.pipeline = None
            

class ZEDCamera:
    """ZED camera implementation using pyzed"""
    
    def __init__(self, camera_id: str, config: Dict):
        self.camera_id = camera_id
        self.config = config
        self.serial_number = config['serial_number']
        self.zed = None
        self.runtime_params = None
        self.running = False
        
    def start(self):
        """Start the ZED camera"""
        if not ZED_AVAILABLE:
            raise RuntimeError("ZED SDK not available")
            
        try:
            # Create camera object
            self.zed = sl.Camera()
            
            # Set initialization parameters
            init_params = sl.InitParameters()
            cam_config = self.config.get('config', {})
            
            # Set resolution
            resolution = cam_config.get('resolution', 'HD720')
            if hasattr(sl.RESOLUTION, resolution):
                init_params.camera_resolution = getattr(sl.RESOLUTION, resolution)
            else:
                init_params.camera_resolution = sl.RESOLUTION.HD720
                
            # Set FPS
            init_params.camera_fps = cam_config.get('fps', 30)
            
            # Set depth mode
            if cam_config.get('enable_depth', True):
                depth_mode = cam_config.get('depth_mode', 'ULTRA')
                if hasattr(sl.DEPTH_MODE, depth_mode):
                    init_params.depth_mode = getattr(sl.DEPTH_MODE, depth_mode)
                else:
                    init_params.depth_mode = sl.DEPTH_MODE.ULTRA
            else:
                init_params.depth_mode = sl.DEPTH_MODE.NONE
                
            # Set coordinate units
            init_params.coordinate_units = sl.UNIT.METER
            
            # Set minimum depth
            init_params.depth_minimum_distance = cam_config.get('depth_minimum_distance', 0.3)
            
            # Set specific camera by serial number
            init_params.set_from_serial_number(int(self.serial_number))
            
            # Open camera
            err = self.zed.open(init_params)
            if err != sl.ERROR_CODE.SUCCESS:
                raise RuntimeError(f"Failed to open ZED camera: {err}")
                
            # Get camera info
            cam_info = self.zed.get_camera_information()
            logger.info(f"Started {self.camera_id}: ZED {cam_info.camera_model} "
                       f"(SN: {cam_info.serial_number})")
            
            # Set runtime parameters
            self.runtime_params = sl.RuntimeParameters()
            self.runtime_params.confidence_threshold = cam_config.get('confidence_threshold', 100)
            self.runtime_params.texture_confidence_threshold = cam_config.get('texture_confidence_threshold', 100)
            
            # Prepare data containers
            self.image = sl.Mat()
            self.depth = sl.Mat()
            
            self.running = True
            
        except Exception as e:
            logger.error(f"Failed to start {self.camera_id}: {e}")
            if self.zed:
                self.zed.close()
            raise
            
    def capture_frame(self) -> Optional[CameraFrame]:
        """Capture a frame from the camera"""
        if not self.running or not self.zed:
            return None
            
        try:
            # Grab frame
            if self.zed.grab(self.runtime_params) != sl.ERROR_CODE.SUCCESS:
                return None
                
            # Get color image
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
            color_image = self.image.get_data()
            if color_image is None:
                return None
                
            # Convert BGRA to BGR
            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGRA2BGR)
            
            # Get depth if enabled
            depth_image = None
            if self.config.get('config', {}).get('enable_depth', True):
                self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH)
                depth_data = self.depth.get_data()
                if depth_data is not None:
                    # Convert to millimeters (uint16)
                    depth_scale = self.config.get('config', {}).get('depth_scale', 0.001)
                    depth_image = (depth_data / depth_scale).astype(np.uint16)
                    
            # Get camera intrinsics
            cam_info = self.zed.get_camera_information()
            calibration = cam_info.camera_configuration.calibration_parameters.left_cam
            intrinsics = {
                'width': self.image.get_width(),
                'height': self.image.get_height(),
                'fx': calibration.fx,
                'fy': calibration.fy,
                'cx': calibration.cx,
                'cy': calibration.cy,
                'model': 'plumb_bob',
                'coeffs': [calibration.k1, calibration.k2, calibration.p1, calibration.p2, calibration.k3]
            }
            
            # Get metadata
            metadata = {
                'timestamp_ns': self.zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds(),
                'frame_dropped': self.zed.get_frame_dropped_count()
            }
            
            return CameraFrame(
                timestamp=time.time(),
                camera_id=self.camera_id,
                color_image=color_image,
                depth_image=depth_image,
                intrinsics=intrinsics,
                metadata=metadata
            )
            
        except Exception as e:
            logger.error(f"Error capturing frame from {self.camera_id}: {e}")
            return None
            
    def stop(self):
        """Stop the camera"""
        self.running = False
        if self.zed:
            try:
                self.zed.close()
            except:
                pass
            self.zed = None


class FisheyeCamera:
    """Generic USB/Fisheye camera implementation using OpenCV"""
    
    def __init__(self, camera_id: str, config: Dict):
        self.camera_id = camera_id
        self.config = config
        self.device_id = config.get('device_id', 0)
        self.cap = None
        self.running = False
        
    def start(self):
        """Start the camera"""
        if not CV2_AVAILABLE:
            raise RuntimeError("OpenCV not available")
            
        try:
            # Open camera
            self.cap = cv2.VideoCapture(self.device_id)
            if not self.cap.isOpened():
                raise RuntimeError(f"Failed to open camera device {self.device_id}")
                
            # Set camera properties
            cam_config = self.config.get('config', {})
            width = cam_config.get('width', 640)
            height = cam_config.get('height', 480)
            fps = cam_config.get('fps', 30)
            
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, fps)
            
            # Set FOURCC if specified
            fourcc = cam_config.get('fourcc', 'MJPG')
            if fourcc:
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
                
            # Get actual properties
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            
            logger.info(f"Started {self.camera_id}: USB camera on /dev/video{self.device_id} "
                       f"({actual_width}x{actual_height} @ {actual_fps}fps)")
            
            self.running = True
            
        except Exception as e:
            logger.error(f"Failed to start {self.camera_id}: {e}")
            if self.cap:
                self.cap.release()
            raise
            
    def capture_frame(self) -> Optional[CameraFrame]:
        """Capture a frame from the camera"""
        if not self.running or not self.cap:
            return None
            
        try:
            ret, frame = self.cap.read()
            if not ret or frame is None:
                return None
                
            # No depth for USB cameras
            # Basic intrinsics (approximate)
            height, width = frame.shape[:2]
            intrinsics = {
                'width': width,
                'height': height,
                'fx': width,  # Approximate
                'fy': width,  # Approximate
                'cx': width / 2,
                'cy': height / 2,
                'model': 'pinhole',
                'coeffs': [0, 0, 0, 0, 0]  # No distortion info
            }
            
            return CameraFrame(
                timestamp=time.time(),
                camera_id=self.camera_id,
                color_image=frame,
                depth_image=None,
                intrinsics=intrinsics,
                metadata={}
            )
            
        except Exception as e:
            logger.error(f"Error capturing frame from {self.camera_id}: {e}")
            return None
            
    def stop(self):
        """Stop the camera"""
        self.running = False
        if self.cap:
            try:
                self.cap.release()
            except:
                pass
            self.cap = None


class CameraManager:
    """Manages multiple cameras with async capture"""
    
    def __init__(self, config_path: str):
        self.config_path = config_path
        self.config = self._load_config()
        self.cameras = {}
        self.capture_threads = {}
        self.frame_queues = {}
        self.running = False
        
    def _load_config(self) -> Dict:
        """Load camera configuration from YAML file"""
        with open(self.config_path, 'r') as f:
            return yaml.safe_load(f)
            
    def _init_camera(self, camera_id: str, camera_config: Dict):
        """Initialize a camera based on its type"""
        cam_type = camera_config.get('type', 'unknown')
        
        if cam_type == 'realsense':
            return RealsenseCamera(camera_id, camera_config)
        elif cam_type == 'zed':
            return ZEDCamera(camera_id, camera_config)
        elif cam_type in ['fisheye', 'usb']:
            return FisheyeCamera(camera_id, camera_config)
        else:
            raise ValueError(f"Unknown camera type: {cam_type}")
            
    def _capture_worker(self, camera_id: str):
        """Worker thread for capturing frames from a camera"""
        camera = self.cameras[camera_id]
        frame_queue = self.frame_queues[camera_id]
        
        logger.info(f"Started capture thread for {camera_id}")
        
        while self.running:
            try:
                frame = camera.capture_frame()
                if frame:
                    # Try to put frame in queue (non-blocking)
                    try:
                        frame_queue.put_nowait(frame)
                    except queue.Full:
                        # Drop oldest frame if queue is full
                        try:
                            frame_queue.get_nowait()
                            frame_queue.put_nowait(frame)
                        except:
                            pass
                            
            except Exception as e:
                logger.error(f"Error in capture thread for {camera_id}: {e}")
                time.sleep(0.1)  # Brief pause before retry
                
        logger.info(f"Stopped capture thread for {camera_id}")
        
    def start(self):
        """Start all configured cameras"""
        if self.running:
            return
            
        self.running = True
        
        # Initialize cameras
        for camera_id, camera_config in self.config.get('cameras', {}).items():
            if not camera_config.get('enabled', False):
                logger.info(f"Skipping disabled camera: {camera_id}")
                continue
                
            try:
                # Initialize camera
                camera = self._init_camera(camera_id, camera_config)
                camera.start()
                self.cameras[camera_id] = camera
                
                # Create frame queue
                buffer_size = self.config.get('camera_settings', {}).get('buffer_size', 5)
                self.frame_queues[camera_id] = queue.Queue(maxsize=buffer_size)
                
                # Start capture thread
                thread = threading.Thread(
                    target=self._capture_worker,
                    args=(camera_id,),
                    daemon=True,
                    name=f"Camera-{camera_id}"
                )
                thread.start()
                self.capture_threads[camera_id] = thread
                
            except Exception as e:
                logger.error(f"Failed to start camera {camera_id}: {e}")
                
        logger.info(f"Camera manager started with {len(self.cameras)} camera(s)")
        
    def get_frame(self, camera_id: str, timeout: float = 0.1) -> Optional[CameraFrame]:
        """Get the latest frame from a camera"""
        if camera_id not in self.frame_queues:
            return None
            
        try:
            return self.frame_queues[camera_id].get(timeout=timeout)
        except queue.Empty:
            return None
            
    def get_all_frames(self, timeout: float = 0.1) -> Dict[str, CameraFrame]:
        """Get latest frames from all cameras"""
        frames = {}
        for camera_id in self.frame_queues:
            frame = self.get_frame(camera_id, timeout)
            if frame:
                frames[camera_id] = frame
        return frames
        
    def stop(self):
        """Stop all cameras"""
        if not self.running:
            return
            
        self.running = False
        
        # Wait for capture threads to stop
        for thread in self.capture_threads.values():
            thread.join(timeout=2.0)
            
        # Stop cameras
        for camera in self.cameras.values():
            try:
                camera.stop()
            except Exception as e:
                logger.error(f"Error stopping camera: {e}")
                
        self.cameras.clear()
        self.capture_threads.clear()
        self.frame_queues.clear()
        
        logger.info("Camera manager stopped") 