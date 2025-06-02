import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2 # Assuming you might add PointCloud2 later
from geometry_msgs.msg import TransformStamped
import tf2_ros
from cv_bridge import CvBridge
import yaml
import numpy as np
import time
import os
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from ament_index_python.packages import get_package_share_directory

from diagnostic_updater import Updater, DiagnosticTask, DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

from .camera_utilities import (
    CameraManager,
    test_cameras,
    set_camera_utils_logger,
    discover_all_cameras, # For auto-detection
    REALSENSE_AVAILABLE,
    ZED_AVAILABLE,
    CV2_AVAILABLE # Still needed by ZEDCamera for format conversion
)

def get_configs_sensors_dir():
    """Helper function to locate the configs/sensors directory"""
    # Try to get from environment variable first
    workspace_root = os.environ.get('COLCON_WS', None)
    if workspace_root:
        configs_path = os.path.join(workspace_root, 'lbx_robotics', 'configs', 'sensors')
        if os.path.exists(configs_path):
            return configs_path
    
    # Fallback: try to detect based on current package location
    try:
        lbx_vision_camera_share = get_package_share_directory('lbx_vision_camera')
        # Navigate from install/lbx_vision_camera/share/lbx_vision_camera to workspace root
        workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(lbx_vision_camera_share))))
        configs_path = os.path.join(workspace_root, 'lbx_robotics', 'configs', 'sensors')
        if os.path.exists(configs_path):
            return configs_path
    except Exception:
        pass
        
    # Try current working directory structure
    try:
        current_dir = os.getcwd()
        configs_path = os.path.join(current_dir, 'lbx_robotics', 'configs', 'sensors')
        if os.path.exists(configs_path):
            return configs_path
    except Exception:
        pass
        
    # Try relative to this script's location
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # Navigate up from src/lbx_vision_camera/lbx_vision_camera/camera_node.py
        workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(script_dir))))
        configs_path = os.path.join(workspace_root, 'lbx_robotics', 'configs', 'sensors')
        if os.path.exists(configs_path):
            return configs_path
    except Exception:
        pass
        
    # Create a fallback directory if nothing else works
    fallback_dir = os.path.expanduser('~/.lbx_camera_configs')
    try:
        os.makedirs(fallback_dir, exist_ok=True)
        return fallback_dir
    except Exception:
        # Last resort
        return '/tmp/camera_configs'

class CameraNode(Node):
    def __init__(self):
        super().__init__('vision_camera_node')  # Changed from 'camera_node' to match launch file
        set_camera_utils_logger(self.get_logger())
        self.initialization_ok = True

        if not (REALSENSE_AVAILABLE or ZED_AVAILABLE):
            self.get_logger().error("Neither RealSense nor ZED SDK is available. This node cannot function.")
            self.initialization_ok = False
            return

        self.declare_parameter('camera_config_file', '') # Default to empty to trigger auto-detection logic
        self.declare_parameter('enable_publishing', True)
        self.declare_parameter('run_startup_tests', True)
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('enable_pointcloud', True)
        self.declare_parameter('tf_publish_rate_hz', 10.0)
        self.declare_parameter('diagnostics_publish_rate_hz', 0.2)
        self.declare_parameter('auto_config_generation_dir', get_configs_sensors_dir())

        try:
            self.camera_config_path_param = self.get_parameter('camera_config_file').get_parameter_value().string_value
            self.enable_publishing = self.get_parameter('enable_publishing').get_parameter_value().bool_value
            self.run_startup_tests = self.get_parameter('run_startup_tests').get_parameter_value().bool_value
            self.publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
            self.enable_pointcloud = self.get_parameter('enable_pointcloud').get_parameter_value().bool_value
            tf_publish_rate = self.get_parameter('tf_publish_rate_hz').get_parameter_value().double_value
            diagnostics_publish_rate = self.get_parameter('diagnostics_publish_rate_hz').get_parameter_value().double_value
            self.auto_config_dir = self.get_parameter('auto_config_generation_dir').get_parameter_value().string_value
        except Exception as e:
            self.get_logger().error(f"Failed to get parameters: {e}")
            self.initialization_ok = False
            return
        
        self.camera_config_path = ""
        self.camera_configs_yaml = None
        self.discovered_realsense_sns = []
        self.discovered_zed_sns = [] # Assuming ZED might have serial numbers or unique IDs
        self._determine_camera_config()

        if not self.camera_configs_yaml:
            self.get_logger().error("Failed to load or determine a valid camera configuration. Node cannot start.")
            self.initialization_ok = False
            return

        self.camera_test_results = []
        self.all_tests_passed = True 
        if self.run_startup_tests:
            self.get_logger().info("Running camera startup tests...")
            try:
                self.all_tests_passed, self.camera_test_results = test_cameras(self.camera_configs_yaml, self.get_logger())
                if not self.all_tests_passed:
                    self.get_logger().warn("One or more camera tests failed. Check logs.")
                else: 
                    self.get_logger().info("All configured and enabled camera tests passed.")
            except Exception as e:
                self.get_logger().error(f"Camera tests failed with exception: {e}")
                self.all_tests_passed = False
        else: 
            self.get_logger().info("Startup tests are disabled.")

        try:
            self.cv_bridge = CvBridge()
            self.camera_manager = CameraManager(
                config_path=self.camera_config_path, 
                node_logger=self.get_logger(),
                discovered_realsense_sns=self.discovered_realsense_sns,
                discovered_zed_sns=self.discovered_zed_sns
            )
            self.camera_publishers = {}
            self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
            self._tf_dynamic_broadcaster = tf2_ros.TransformBroadcaster(self) 
            self._published_static_tfs = set()
            self._initialize_publishers_and_tfs()
            self.camera_manager.start()
        except Exception as e:
            self.get_logger().error(f"Failed to initialize camera manager: {e}")
            self.initialization_ok = False
            return

        self.frames_published_count = {cam_id: {'color': 0, 'depth': 0, 'pc': 0} for cam_id in self.camera_manager.cameras.keys()}
        self.actual_publish_rates = {cam_id: {'color': 0.0, 'depth': 0.0, 'pc': 0.0} for cam_id in self.camera_manager.cameras.keys()}
        self._publish_interval_start_time = time.monotonic()

        if self.enable_publishing:
            self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.publish_camera_data)
            self.get_logger().info(f"Camera node started. Publishing enabled at ~{self.publish_rate} Hz.")
        else: 
            self.get_logger().info("Camera node started. Publishing is disabled.")
        
        if tf_publish_rate > 0: 
            self.tf_timer = self.create_timer(1.0 / tf_publish_rate, self.publish_dynamic_transforms)

        # Initialize diagnostics with error handling
        try:
            self.diagnostic_updater = Updater(self, period=(1.0/diagnostics_publish_rate) if diagnostics_publish_rate > 0 else 5.0)
            self.diagnostic_updater.setHardwareID("vision_cameras_lbx")
            self.diagnostic_updater.add(CameraNodeOverallStatusTask("Camera System Status", self))

            # Add individual diagnostic tasks for each active and unavailable camera
            if self.camera_manager:
                for cam_id, cam_obj in self.camera_manager.cameras.items():
                    if cam_cfg_data := self.camera_configs_yaml.get('cameras', {}).get(cam_id):
                        task_name = f"Camera {cam_id}"
                        self.diagnostic_updater.add(IndividualCameraDiagnosticTask(task_name, self, cam_id, cam_cfg_data, is_active=True))
                        self.get_logger().info(f"Added IndividualCameraDiagnosticTask for active camera: {task_name}")
                
                for cam_id, unavailable_cam_info in self.camera_manager.unavailable_cameras.items():
                    if cam_cfg_data := unavailable_cam_info.get('config'):
                        task_name = f"Camera {cam_id} (Unavailable)"
                        self.diagnostic_updater.add(IndividualCameraDiagnosticTask(task_name, self, cam_id, cam_cfg_data, is_active=False, unavailable_status=unavailable_cam_info.get('status')))
                        self.get_logger().info(f"Added IndividualCameraDiagnosticTask for unavailable camera: {task_name}")

            self.get_logger().info(f"Diagnostics publishing every {self.diagnostic_updater.period:.2f}s.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize diagnostics: {e}")
            # Don't fail the whole node for diagnostics issues
            self.diagnostic_updater = None

    def _determine_camera_config(self):
        if self.camera_config_path_param and os.path.exists(self.camera_config_path_param):
            self.camera_config_path = self.camera_config_path_param
            self.get_logger().info(f"Using provided camera config: {self.camera_config_path}")
        else:
            if self.camera_config_path_param: # Path was given but not found
                 self.get_logger().warn(f"Provided camera_config_file '{self.camera_config_path_param}' not found. Attempting auto-detection.")
            else: # No path given
                 self.get_logger().info("No camera_config_file provided. Attempting auto-detection and loading default configs.")

            try:
                discovered_devices_info = discover_all_cameras(self.get_logger())
                self.get_logger().info(f"DEBUG: Raw discovery result from discover_all_cameras: {discovered_devices_info}")
                
                # Robust extraction of RealSense serial numbers
                self.discovered_realsense_sns = []
                for cam in discovered_devices_info.get('realsense', []):
                    # Handle different possible data structures
                    if isinstance(cam, dict):
                        serial = cam.get('serial') or cam.get('serial_number') or cam.get('device_serial') or str(cam.get('device_id', 'unknown'))
                    else:
                        # If cam is not a dict, convert to string
                        serial = str(cam)
                    
                    if serial and serial != 'unknown':
                        self.discovered_realsense_sns.append(serial)
                        self.get_logger().info(f"Found RealSense camera with serial: {serial}")
                
                self.get_logger().info(f"DEBUG: Populated self.discovered_realsense_sns: {self.discovered_realsense_sns}")
                # Robust extraction of ZED serial numbers
                self.discovered_zed_sns = []
                for cam in discovered_devices_info.get('zed', []):
                    if isinstance(cam, dict):
                        serial = cam.get('serial_number') or cam.get('serial') or cam.get('id') or str(cam.get('device_id', 'unknown_zed'))
                    else:
                        serial = str(cam)
                    
                    if serial and 'unknown' not in serial:
                        self.discovered_zed_sns.append(serial)
                        self.get_logger().info(f"Found ZED camera with serial: {serial}")
                
            except Exception as e:
                self.get_logger().error(f"Camera discovery failed: {e}")
                self.discovered_realsense_sns = []
                self.discovered_zed_sns = []

            num_realsense = len(self.discovered_realsense_sns)
            num_zed = len(self.discovered_zed_sns)

            config_to_load = None
            if num_realsense > 0 and num_zed == 0:
                self.get_logger().info(f"Auto-detected {num_realsense} RealSense camera(s). Attempting to load 'realsense_cameras.yaml'.")
                config_to_load = os.path.join(self.auto_config_dir, 'realsense_cameras.yaml')
            elif num_zed > 0 and num_realsense == 0:
                self.get_logger().info(f"Auto-detected {num_zed} ZED camera(s). Attempting to load 'zed_camera.yaml'.")
                config_to_load = os.path.join(self.auto_config_dir, 'zed_camera.yaml')
            elif num_realsense > 0 and num_zed > 0:
                self.get_logger().error("Both RealSense and ZED cameras detected. Please specify a camera_config_file.")
                return # Stop initialization
            else:
                self.get_logger().warn("No RealSense or ZED cameras auto-detected. Will try to load default 'cameras_setup.yaml'.")
                config_to_load = os.path.join(self.auto_config_dir, 'cameras_setup.yaml')

            if config_to_load and os.path.exists(config_to_load):
                self.camera_config_path = config_to_load
                self.get_logger().info(f"Using auto-selected camera config: {self.camera_config_path}")
            elif config_to_load:
                self.get_logger().warn(f"Auto-selected config '{config_to_load}' not found. Please ensure it exists or provide one.")
                # As a last resort, we can try to generate one if discover_all_cameras found something
                if discovered_devices_info:
                    from .camera_utilities import generate_camera_config # Local import
                    self.get_logger().info(f"Attempting to generate a default config in {self.auto_config_dir} based on discovered cameras.")
                    # Pass the full discovered_devices_info to generate_camera_config
                    generated_path = generate_camera_config(discovered_devices_info, config_dir=self.auto_config_dir)
                    if generated_path and os.path.exists(generated_path):
                        self.camera_config_path = generated_path
                        self.get_logger().info(f"Using newly generated camera config: {self.camera_config_path}")
                    else:
                        self.get_logger().error("Failed to generate or find a suitable camera configuration.")
                        return
                else:
                    self.get_logger().error(f"No cameras discovered to generate a config from, and default config '{config_to_load}' not found.")
                    return
            else: # This case should ideally not be reached if logic above is correct
                self.get_logger().error("Could not determine a camera configuration file to load.")
                return

        try:
            with open(self.camera_config_path, 'r') as f:
                self.camera_configs_yaml = yaml.safe_load(f)
            self.get_logger().info(f"Successfully loaded camera configuration from {self.camera_config_path}")
        except Exception as e:
            self.get_logger().error(f"Error loading YAML from {self.camera_config_path}: {e}")
            self.camera_configs_yaml = None # Ensure it's None on failure
            self.initialization_ok = False

    def _initialize_publishers_and_tfs(self):
        qos_sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        qos_info = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        if not self.camera_configs_yaml or 'cameras' not in self.camera_configs_yaml:
            self.get_logger().warn("No cameras defined in config."); return

        for cam_id, cam_config in self.camera_configs_yaml.get('cameras', {}).items():
            if not cam_config.get('enabled', False): continue
            self.camera_publishers[cam_id] = {}
            topics_config = cam_config.get('topics', {})
            base_topic = topics_config.get('base', f"~/cam_{cam_id}")
            
            # Use standard ROS camera topic structure:
            # - base_topic/image_raw for main RGB stream
            # - base_topic/camera_info for RGB camera info
            # - base_topic/depth/image_raw for depth stream
            # - base_topic/depth/camera_info for depth camera info
            self.camera_publishers[cam_id]['color_image'] = self.create_publisher(Image, f"{base_topic}/image_raw", qos_sensor)
            self.camera_publishers[cam_id]['color_info'] = self.create_publisher(CameraInfo, f"{base_topic}/camera_info", qos_info)
            
            depth_config = cam_config.get('depth', {})
            if depth_config.get('enabled', True): 
                self.camera_publishers[cam_id]['depth_image'] = self.create_publisher(Image, f"{base_topic}/depth/image_raw", qos_sensor)
                self.camera_publishers[cam_id]['depth_info'] = self.create_publisher(CameraInfo, f"{base_topic}/depth/camera_info", qos_info)
                pointcloud_config = cam_config.get('pointcloud', {})
                if self.enable_pointcloud and pointcloud_config.get('enabled', True):
                     self.camera_publishers[cam_id]['pointcloud'] = self.create_publisher(PointCloud2, f"{base_topic}/depth/color/points", qos_sensor)
            self.get_logger().info(f"Publishers for {cam_id} on {base_topic}")
            self._publish_static_camera_tf(cam_id, cam_config.get('transforms', {}))

    def _publish_static_camera_tf(self, cam_id, transform_config):
        if cam_id in self._published_static_tfs: return
        parent = transform_config.get('parent_frame', 'base_link')
        camera_link = transform_config.get('camera_frame', f"{cam_id}_link")
        optical_frame = transform_config.get('optical_frame', f"{cam_id}_optical_frame")
        stamp = self.get_clock().now().to_msg()
        
        # Create transform from parent to camera link
        tpl = TransformStamped()
        tpl.header.stamp = stamp
        tpl.header.frame_id = parent
        tpl.child_frame_id = camera_link
        
        trans_pl = transform_config.get('translation', [0.,0.,0.])
        rot_pl_deg = transform_config.get('rotation_deg', [0.,0.,0.])
        
        # Convert Euler angles to quaternion using numpy
        from scipy.spatial.transform import Rotation as R
        q_pl = R.from_euler('xyz', np.deg2rad(rot_pl_deg)).as_quat()
        
        tpl.transform.translation.x = float(trans_pl[0])
        tpl.transform.translation.y = float(trans_pl[1])
        tpl.transform.translation.z = float(trans_pl[2])
        tpl.transform.rotation.x = float(q_pl[0])
        tpl.transform.rotation.y = float(q_pl[1])
        tpl.transform.rotation.z = float(q_pl[2])
        tpl.transform.rotation.w = float(q_pl[3])
        
        # Create transform from camera link to optical frame (standard camera convention)
        tlo = TransformStamped()
        tlo.header.stamp = stamp
        tlo.header.frame_id = camera_link
        tlo.child_frame_id = optical_frame
        
        # Standard camera optical frame transformation: -90° around X, then -90° around Z
        q_lo = R.from_euler('xyz', [-np.pi/2, 0, -np.pi/2]).as_quat()
        tlo.transform.translation.x = 0.0
        tlo.transform.translation.y = 0.0
        tlo.transform.translation.z = 0.0
        tlo.transform.rotation.x = float(q_lo[0])
        tlo.transform.rotation.y = float(q_lo[1])
        tlo.transform.rotation.z = float(q_lo[2])
        tlo.transform.rotation.w = float(q_lo[3])
        
        self.tf_broadcaster.sendTransform([tpl, tlo])
        self._published_static_tfs.add(cam_id)
        self.get_logger().info(f"Static TF for {cam_id}: {parent} -> {camera_link} -> {optical_frame}")

    def publish_dynamic_transforms(self): pass

    def _create_camera_info_msg(self, frame_data, stamp, optical_frame_id):
        msg = CameraInfo(); msg.header.stamp = stamp; msg.header.frame_id = optical_frame_id
        intr = frame_data.intrinsics
        msg.width = intr.get('width', 0); msg.height = intr.get('height', 0)
        model = intr.get('model', 'plumb_bob').lower()
        if 'brown_conrady' in model: msg.distortion_model = 'plumb_bob'
        elif 'kannala_brandt4' in model or 'fisheye' in model: msg.distortion_model = 'equidistant'
        else: msg.distortion_model = 'plumb_bob'
        msg.d = list(map(float, intr.get('coeffs', [0.]*5)[:5]))
        fx,fy,cx,cy = intr.get('fx',0.), intr.get('fy',0.), intr.get('cx',0.), intr.get('cy',0.)
        msg.k = [fx,0.,cx, 0.,fy,cy, 0.,0.,1.]
        msg.r = [1.,0.,0., 0.,1.,0., 0.,0.,1.]
        msg.p = [fx,0.,cx,0., 0.,fy,cy,0., 0.,0.,1.,0.]
        return msg

    def publish_camera_data(self):
        if not self.enable_publishing or not self.camera_manager: 
            return
            
        try:
            all_frames = self.camera_manager.get_all_frames(timeout=0.001)
            now_msg_time = self.get_clock().now().to_msg()
            
            for cam_id, frame in all_frames.items():
                if cam_id not in self.camera_publishers: 
                    continue
                    
                cam_config = self.camera_configs_yaml['cameras'].get(cam_id, {})
                optical_frame_id = cam_config.get('transforms', {}).get('optical_frame', f"{cam_id}_optical_frame")
                frame_stamp = now_msg_time 
                
                if frame.color_image is not None and 'color_image' in self.camera_publishers[cam_id]:
                    try:
                        img_msg = self.cv_bridge.cv2_to_imgmsg(frame.color_image, encoding="bgr8")
                        img_msg.header.stamp = frame_stamp
                        img_msg.header.frame_id = optical_frame_id
                        self.camera_publishers[cam_id]['color_image'].publish(img_msg)
                        self.frames_published_count[cam_id]['color'] += 1
                        
                        if frame.intrinsics and 'color_info' in self.camera_publishers[cam_id]:
                            self.camera_publishers[cam_id]['color_info'].publish(self._create_camera_info_msg(frame, frame_stamp, optical_frame_id))
                    except Exception as e: 
                        self.get_logger().error(f"Pub color {cam_id} error: {e}")
                
                depth_config = cam_config.get('depth', {})
                if frame.depth_image is not None and 'depth_image' in self.camera_publishers[cam_id] and depth_config.get('enabled', True):
                    try:
                        depth_msg = self.cv_bridge.cv2_to_imgmsg(frame.depth_image, encoding="16UC1")
                        depth_msg.header.stamp = frame_stamp
                        depth_msg.header.frame_id = optical_frame_id
                        self.camera_publishers[cam_id]['depth_image'].publish(depth_msg)
                        self.frames_published_count[cam_id]['depth'] += 1
                        
                        if frame.intrinsics and 'depth_info' in self.camera_publishers[cam_id]:
                            depth_intr = frame.intrinsics.get('depth_intrinsics', frame.intrinsics)
                            class TempDepthFrameData: pass
                            temp_depth_data = TempDepthFrameData()
                            temp_depth_data.intrinsics = depth_intr
                            temp_depth_data.camera_id = cam_id
                            self.camera_publishers[cam_id]['depth_info'].publish(self._create_camera_info_msg(temp_depth_data, frame_stamp, optical_frame_id))
                    except Exception as e: 
                        self.get_logger().error(f"Pub depth {cam_id} error: {e}")
                        
            # Update publish rates
            current_time = time.monotonic()
            elapsed_interval = current_time - self._publish_interval_start_time
            if elapsed_interval >= 1.0:
                for cam_id_key in list(self.frames_published_count.keys()):
                    if cam_id_key in self.actual_publish_rates:
                        for stream_type in ['color', 'depth', 'pc']:
                            self.actual_publish_rates[cam_id_key][stream_type] = self.frames_published_count[cam_id_key][stream_type] / elapsed_interval
                            self.frames_published_count[cam_id_key][stream_type] = 0
                self._publish_interval_start_time = current_time
                
        except Exception as e:
            self.get_logger().error(f"Error in publish_camera_data: {e}")

    def destroy_node(self):
        self.get_logger().info("Shutting down Camera Node...")
        try:
            if hasattr(self, 'publish_timer') and self.publish_timer: 
                self.publish_timer.cancel()
        except Exception as e:
            self.get_logger().warn(f"Error canceling publish timer: {e}")
            
        try:
            if hasattr(self, 'tf_timer') and self.tf_timer: 
                self.tf_timer.cancel()
        except Exception as e:
            self.get_logger().warn(f"Error canceling tf timer: {e}")
            
        try:
            if hasattr(self, 'camera_manager') and self.camera_manager: 
                self.camera_manager.stop()
        except Exception as e:
            self.get_logger().warn(f"Error stopping camera manager: {e}")
            
        super().destroy_node()
        self.get_logger().info("Camera Node shutdown complete.")

class CameraNodeOverallStatusTask(DiagnosticTask):
    def __init__(self, name, node_instance):
        super().__init__(name)
        self.node = node_instance
        
    def run(self, stat: DiagnosticStatus):
        if not self.node.initialization_ok: 
            stat.summary(DiagnosticStatus.ERROR, "Node initialization failed.")
            return stat
        
        try:
            active_camera_count = len(self.node.camera_manager.cameras) if self.node.camera_manager else 0
            unavailable_camera_count = len(self.node.camera_manager.unavailable_cameras) if self.node.camera_manager else 0
            configured_enabled_cameras = [cid for cid, cconf in self.node.camera_configs_yaml.get('cameras', {}).items() if cconf.get('enabled', False)]
            total_configured_enabled = len(configured_enabled_cameras)

            if active_camera_count == total_configured_enabled and total_configured_enabled > 0:
                stat.summary(DiagnosticStatus.OK, f"{active_camera_count}/{total_configured_enabled} configured cameras active.")
            elif active_camera_count > 0:
                stat.summary(DiagnosticStatus.WARN, f"{active_camera_count}/{total_configured_enabled} configured cameras active. {unavailable_camera_count} unavailable.")
            elif total_configured_enabled > 0:
                stat.summary(DiagnosticStatus.ERROR, f"0/{total_configured_enabled} configured cameras active. All unavailable or failed.")
            else:
                stat.summary(DiagnosticStatus.WARN, "No cameras configured or enabled.")

            stat.add("Config File Used", str(self.node.camera_config_path) if self.node.camera_config_path else "None/Error")
            stat.add("Run Startup Tests", str(self.node.run_startup_tests))
            stat.add("Active Cameras Count", str(active_camera_count))
            stat.add("Unavailable Configured Cameras Count", str(unavailable_camera_count))
            
            # Brief summary of test results if run
            if self.node.run_startup_tests and hasattr(self.node, 'camera_test_results'):
                passed_tests = sum(1 for res in self.node.camera_test_results if hasattr(res, 'is_success') and res.is_success())
                total_tests = len(self.node.camera_test_results)
                if total_tests > 0:
                     stat.add("Startup Test Results", f"{passed_tests}/{total_tests} passed")
        except Exception as e:
            stat.summary(DiagnosticStatus.ERROR, f"Diagnostic error: {e}")
            
        return stat

class IndividualCameraDiagnosticTask(DiagnosticTask):
    def __init__(self, name: str, node_instance: 'CameraNode', camera_id: str, camera_config: Dict, is_active: bool, unavailable_status: Optional[str] = None):
        super().__init__(name) # Name will be like "Camera realsense_123"
        self.node = node_instance
        self.camera_id = camera_id
        self.camera_config = camera_config # This is the specific config for this camera
        self.is_active = is_active
        self.unavailable_status = unavailable_status

    def run(self, stat: DiagnosticStatus):
        try:
            # Hardware ID for this specific camera, e.g., "camera_realsense_123"
            stat.hardware_id = f"camera_{self.camera_id}"
            
            cam_type = self.camera_config.get('type', "N/A")
            configured_sn = str(self.camera_config.get('serial_number') or self.camera_config.get('device_id', "N/A"))
            position = self.camera_config.get('metadata', {}).get('position', 'Unknown')

            stat.add("Camera ID", self.camera_id)
            stat.add("Type", cam_type)
            stat.add("Configured SN/Idx", configured_sn)
            stat.add("Position", position)

            if self.is_active and hasattr(self.node, 'camera_manager') and self.node.camera_manager:
                cam_obj = self.node.camera_manager.cameras.get(self.camera_id)
                actual_sn = cam_obj.serial_number if cam_obj and hasattr(cam_obj, 'serial_number') else "Unknown"
                
                stat.summary(DiagnosticStatus.OK, f"Running. SN: {actual_sn}")
                stat.add("Status", "Running")
                stat.add("Actual SN", actual_sn)

                if hasattr(self.node, 'actual_publish_rates') and self.camera_id in self.node.actual_publish_rates:
                    color_cfg = self.camera_config.get('color',{})
                    depth_cfg = self.camera_config.get('depth',{})

                    target_color_fps = str(color_cfg.get('fps','N/A'))
                    actual_color_fps_val = self.node.actual_publish_rates[self.camera_id]['color']
                    stat.add("Target Color FPS", target_color_fps)
                    stat.add("Actual Color FPS", f"{actual_color_fps_val:.2f}")
                    
                    # Calculate color efficiency
                    if target_color_fps != 'N/A':
                        try:
                            eff = (actual_color_fps_val / float(target_color_fps)) * 100 if float(target_color_fps) > 0 else 0
                            stat.add("Color Efficiency (%)", f"{eff:.1f}")
                        except ValueError: 
                            pass

                    if depth_cfg.get('enabled', True):
                        target_depth_fps = str(depth_cfg.get('fps','N/A'))
                        actual_depth_fps_val = self.node.actual_publish_rates[self.camera_id]['depth']
                        stat.add("Target Depth FPS", target_depth_fps)
                        stat.add("Actual Depth FPS", f"{actual_depth_fps_val:.2f}")
                        if target_depth_fps != 'N/A':
                            try:
                                eff = (actual_depth_fps_val / float(target_depth_fps)) * 100 if float(target_depth_fps) > 0 else 0
                                stat.add("Depth Efficiency (%)", f"{eff:.1f}")
                            except ValueError: 
                                pass
                    
                    if hasattr(self.node, 'frames_published_count') and self.camera_id in self.node.frames_published_count:
                        stat.add("Color Frames Published", str(self.node.frames_published_count[self.camera_id]['color']))
                        stat.add("Depth Frames Published", str(self.node.frames_published_count[self.camera_id]['depth']))
                else:
                    stat.add("Actual Color FPS", "N/A")
                    if self.camera_config.get('depth',{}).get('enabled', True):
                        stat.add("Actual Depth FPS", "N/A")
            else:
                status_msg = self.unavailable_status if self.unavailable_status else "Not Active"
                stat.summary(DiagnosticStatus.WARN if "Not Detected" in status_msg else DiagnosticStatus.ERROR, status_msg)
                stat.add("Status", status_msg)
                
        except Exception as e:
            stat.summary(DiagnosticStatus.ERROR, f"Diagnostic error: {e}")
            
        return stat

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        if not (REALSENSE_AVAILABLE or ZED_AVAILABLE or CV2_AVAILABLE): 
            print("No usable camera SDKs (RealSense, ZED, OpenCV) found. CameraNode cannot start.", file=sys.stderr)
            if rclpy.ok(): 
                rclpy.shutdown()
            return
            
        node = CameraNode()
        if not hasattr(node, 'initialization_ok') or not node.initialization_ok:
            if node: 
                node.get_logger().error("CameraNode initialization failed. Shutting down.")
            else: 
                print("Critical error: CameraNode object could not be created.", file=sys.stderr)
            if rclpy.ok(): 
                rclpy.shutdown()
            return
            
        node.get_logger().info("Camera node initialized successfully, starting spin...")
        if rclpy.ok(): 
            rclpy.spin(node)
            
    except KeyboardInterrupt:
        if node: 
            node.get_logger().info('Keyboard interrupt, shutting down.')
    except Exception as e:
        if node and hasattr(node, 'get_logger'): 
            node.get_logger().error(f"Unhandled exception: {e}")
        else: 
            print(f"Unhandled exception: {e}", file=sys.stderr)
            import traceback
            traceback.print_exc()
    finally:
        try:
            if node and rclpy.ok() and hasattr(node, 'destroy_node'): 
                node.destroy_node()
        except Exception as e:
            print(f"Error during node destruction: {e}", file=sys.stderr)
        try:
            if rclpy.ok(): 
                rclpy.shutdown()
        except Exception as e:
            print(f"Error during rclpy shutdown: {e}", file=sys.stderr)

if __name__ == '__main__':
    main() 