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
        return os.path.join(workspace_root, 'lbx_robotics', 'configs', 'sensors')
    
    # Fallback: try to detect based on current package location
    try:
        lbx_vision_camera_share = get_package_share_directory('lbx_vision_camera')
        # Navigate from install/lbx_vision_camera/share/lbx_vision_camera to workspace root
        workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(lbx_vision_camera_share))))
        return os.path.join(workspace_root, 'lbx_robotics', 'configs', 'sensors')
    except:
        # Last resort: hardcoded fallback
        return '/tmp/camera_configs'

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
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

        self.camera_config_path_param = self.get_parameter('camera_config_file').get_parameter_value().string_value
        self.enable_publishing = self.get_parameter('enable_publishing').get_parameter_value().bool_value
        self.run_startup_tests = self.get_parameter('run_startup_tests').get_parameter_value().bool_value
        self.publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.enable_pointcloud = self.get_parameter('enable_pointcloud').get_parameter_value().bool_value
        tf_publish_rate = self.get_parameter('tf_publish_rate_hz').get_parameter_value().double_value
        diagnostics_publish_rate = self.get_parameter('diagnostics_publish_rate_hz').get_parameter_value().double_value
        self.auto_config_dir = self.get_parameter('auto_config_generation_dir').get_parameter_value().string_value
        
        self.camera_config_path = ""
        self.camera_configs_yaml = None
        self._determine_camera_config()

        if not self.camera_configs_yaml:
            self.get_logger().error("Failed to load or determine a valid camera configuration. Node cannot start.")
            self.initialization_ok = False
            return

        self.camera_test_results = []
        self.all_tests_passed = True 
        if self.run_startup_tests:
            self.get_logger().info("Running camera startup tests...")
            self.all_tests_passed, self.camera_test_results = test_cameras(self.camera_configs_yaml, self.get_logger())
            if not self.all_tests_passed:
                self.get_logger().warn("One or more camera tests failed. Check logs.")
            else: self.get_logger().info("All configured and enabled camera tests passed.")
        else: self.get_logger().info("Startup tests are disabled.")

        self.cv_bridge = CvBridge()
        self.camera_manager = CameraManager(config_path=self.camera_config_path, node_logger=self.get_logger())
        self.publishers = {}
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self._tf_dynamic_broadcaster = tf2_ros.TransformBroadcaster(self) 
        self._published_static_tfs = set()
        self._initialize_publishers_and_tfs()
        self.camera_manager.start()

        self.frames_published_count = {cam_id: {'color': 0, 'depth': 0, 'pc': 0} for cam_id in self.camera_manager.cameras.keys()}
        self.actual_publish_rates = {cam_id: {'color': 0.0, 'depth': 0.0, 'pc': 0.0} for cam_id in self.camera_manager.cameras.keys()}
        self._publish_interval_start_time = time.monotonic()

        if self.enable_publishing:
            self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.publish_camera_data)
            self.get_logger().info(f"Camera node started. Publishing enabled at ~{self.publish_rate} Hz.")
        else: self.get_logger().info("Camera node started. Publishing is disabled.")
        
        if tf_publish_rate > 0: self.tf_timer = self.create_timer(1.0 / tf_publish_rate, self.publish_dynamic_transforms)

        self.diagnostic_updater = Updater(self, period=(1.0/diagnostics_publish_rate) if diagnostics_publish_rate > 0 else 5.0)
        self.diagnostic_updater.setHardwareID("vision_cameras_lbx")
        self.diagnostic_updater.add(CameraNodeStatusTask("Camera System Status", self))
        self.get_logger().info(f"Diagnostics publishing every {self.diagnostic_updater.period:.2f}s.")

    def _determine_camera_config(self):
        if self.camera_config_path_param and os.path.exists(self.camera_config_path_param):
            self.camera_config_path = self.camera_config_path_param
            self.get_logger().info(f"Using provided camera config: {self.camera_config_path}")
        else:
            if self.camera_config_path_param: # Path was given but not found
                 self.get_logger().warn(f"Provided camera_config_file '{self.camera_config_path_param}' not found. Attempting auto-detection.")
            else: # No path given
                 self.get_logger().info("No camera_config_file provided. Attempting auto-detection and loading default configs.")

            discovered_cameras = discover_all_cameras(self.get_logger())
            num_realsense = len(discovered_cameras.get('realsense', []))
            num_zed = len(discovered_cameras.get('zed', []))

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
                if discovered_cameras:
                    from .camera_utilities import generate_camera_config # Local import
                    self.get_logger().info(f"Attempting to generate a default config in {self.auto_config_dir} based on discovered cameras.")
                    generated_path = generate_camera_config(discovered_cameras, config_dir=self.auto_config_dir)
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
            self.publishers[cam_id] = {}
            topics_config = cam_config.get('topics', {})
            base_topic = topics_config.get('base', f"~/cam_{cam_id}")
            
            self.publishers[cam_id]['color_image'] = self.create_publisher(Image, f"{base_topic}/color/image_raw", qos_sensor)
            self.publishers[cam_id]['color_info'] = self.create_publisher(CameraInfo, f"{base_topic}/color/camera_info", qos_info)
            
            depth_config = cam_config.get('depth', {})
            if depth_config.get('enabled', True): 
                self.publishers[cam_id]['depth_image'] = self.create_publisher(Image, f"{base_topic}/depth/image_raw", qos_sensor)
                self.publishers[cam_id]['depth_info'] = self.create_publisher(CameraInfo, f"{base_topic}/depth/camera_info", qos_info)
                pointcloud_config = cam_config.get('pointcloud', {})
                if self.enable_pointcloud and pointcloud_config.get('enabled', True):
                     self.publishers[cam_id]['pointcloud'] = self.create_publisher(PointCloud2, f"{base_topic}/depth/color/points", qos_sensor)
            self.get_logger().info(f"Publishers for {cam_id} on {base_topic}")
            self._publish_static_camera_tf(cam_id, cam_config.get('transforms', {}))

    def _publish_static_camera_tf(self, cam_id, transform_config):
        if cam_id in self._published_static_tfs: return
        parent = transform_config.get('parent_frame', 'base_link')
        camera_link = transform_config.get('camera_frame', f"{cam_id}_link")
        optical_frame = transform_config.get('optical_frame', f"{cam_id}_optical_frame")
        stamp = self.get_clock().now().to_msg()
        from tf_transformations import quaternion_from_euler
        tpl = TransformStamped(); tpl.header.stamp = stamp; tpl.header.frame_id = parent; tpl.child_frame_id = camera_link
        trans_pl = transform_config.get('translation', [0.,0.,0.]); rot_pl_deg = transform_config.get('rotation_deg', [0.,0.,0.])
        q_pl = quaternion_from_euler(*np.deg2rad(rot_pl_deg))
        tpl.transform.translation.x, tpl.transform.translation.y, tpl.transform.translation.z = float(trans_pl[0]), float(trans_pl[1]), float(trans_pl[2])
        tpl.transform.rotation.x, tpl.transform.rotation.y, tpl.transform.rotation.z, tpl.transform.rotation.w = map(float, q_pl)
        tlo = TransformStamped(); tlo.header.stamp = stamp; tlo.header.frame_id = link; tlo.child_frame_id = optical
        q_lo = quaternion_from_euler(-np.pi/2, 0, -np.pi/2)
        tlo.transform.rotation.x, tlo.transform.rotation.y, tlo.transform.rotation.z, tlo.transform.rotation.w = map(float, q_lo)
        self.tf_broadcaster.sendTransform([tpl, tlo])
        self._published_static_tfs.add(cam_id)
        self.get_logger().info(f"Static TF for {cam_id}: {parent} -> {link} -> {optical}")

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
        if not self.enable_publishing or not self.camera_manager: return
        all_frames = self.camera_manager.get_all_frames(timeout=0.005)
        now_msg_time = self.get_clock().now().to_msg()
        for cam_id, frame in all_frames.items():
            if cam_id not in self.publishers: continue
            cam_config = self.camera_configs_yaml['cameras'].get(cam_id, {})
            optical_frame_id = cam_config.get('transforms', {}).get('optical_frame', f"{cam_id}_optical_frame")
            frame_stamp = now_msg_time 
            if frame.color_image is not None and 'color_image' in self.publishers[cam_id]:
                try:
                    img_msg = self.cv_bridge.cv2_to_imgmsg(frame.color_image, encoding="bgr8")
                    img_msg.header.stamp = frame_stamp; img_msg.header.frame_id = optical_frame_id
                    self.publishers[cam_id]['color_image'].publish(img_msg)
                    self.frames_published_count[cam_id]['color'] += 1
                    if frame.intrinsics and 'color_info' in self.publishers[cam_id]:
                        self.publishers[cam_id]['color_info'].publish(self._create_camera_info_msg(frame, frame_stamp, optical_frame_id))
                except Exception as e: self.get_logger().error(f"Pub color {cam_id} error: {e}")
            depth_config = cam_config.get('depth', {})
            if frame.depth_image is not None and 'depth_image' in self.publishers[cam_id] and depth_config.get('enabled', True):
                try:
                    depth_msg = self.cv_bridge.cv2_to_imgmsg(frame.depth_image, encoding="16UC1")
                    depth_msg.header.stamp = frame_stamp; depth_msg.header.frame_id = optical_frame_id
                    self.publishers[cam_id]['depth_image'].publish(depth_msg)
                    self.frames_published_count[cam_id]['depth'] += 1
                    if frame.intrinsics and 'depth_info' in self.publishers[cam_id]:
                        depth_intr = frame.intrinsics.get('depth_intrinsics', frame.intrinsics)
                        class TempDepthFrameData: pass
                        temp_depth_data = TempDepthFrameData(); temp_depth_data.intrinsics = depth_intr; temp_depth_data.camera_id = cam_id
                        self.publishers[cam_id]['depth_info'].publish(self._create_camera_info_msg(temp_depth_data, frame_stamp, optical_frame_id))
                except Exception as e: self.get_logger().error(f"Pub depth {cam_id} error: {e}")
        current_time = time.monotonic()
        elapsed_interval = current_time - self._publish_interval_start_time
        if elapsed_interval >= 1.0:
            for cam_id_key in list(self.frames_published_count.keys()):
                if cam_id_key in self.actual_publish_rates:
                    for stream_type in ['color', 'depth', 'pc']:
                        self.actual_publish_rates[cam_id_key][stream_type] = self.frames_published_count[cam_id_key][stream_type] / elapsed_interval
                        self.frames_published_count[cam_id_key][stream_type] = 0
            self._publish_interval_start_time = current_time

    def destroy_node(self):
        self.get_logger().info("Shutting down Camera Node...")
        if hasattr(self, 'publish_timer') and self.publish_timer: self.publish_timer.cancel()
        if hasattr(self, 'tf_timer') and self.tf_timer: self.tf_timer.cancel()
        if hasattr(self, 'diagnostic_updater'): self.diagnostic_updater.destroy()
        if hasattr(self, 'camera_manager') and self.camera_manager: self.camera_manager.stop()
        super().destroy_node()
        self.get_logger().info("Camera Node shutdown complete.")

class CameraNodeStatusTask(DiagnosticTask):
    def __init__(self, name, node_instance):
        super().__init__(name)
        self.node = node_instance
    def run(self, stat: DiagnosticStatus):
        if not self.node.initialization_ok: stat.summary(DiagnosticStatus.ERROR, "Node initialization failed."); return stat
        active_camera_count = len(self.node.camera_manager.cameras) if self.node.camera_manager else 0
        if self.node.all_tests_passed and active_camera_count > 0:
            stat.summary(DiagnosticStatus.OK, f"{active_camera_count} camera(s) active and passed tests.")
        elif active_camera_count > 0:
            stat.summary(DiagnosticStatus.WARN, f"{active_camera_count} camera(s) active; startup tests failed/skipped for some.")
        elif self.node.run_startup_tests and not self.node.all_tests_passed:
             stat.summary(DiagnosticStatus.ERROR, "Camera startup tests failed for configured/enabled cameras.")
        else: stat.summary(DiagnosticStatus.WARN, "No active cameras or tests were not run.")
        stat.add("Config File Used", str(self.node.camera_config_path) if self.node.camera_config_path else "None/Error")
        stat.add("Run Startup Tests", str(self.node.run_startup_tests))
        stat.add("All Configured Tests Passed", str(self.node.all_tests_passed))
        stat.add("Active Cameras (Mgr)", str(active_camera_count))
        for cam_id_cfg, cam_cfg_data in self.node.camera_configs_yaml.get('cameras', {}).items():
            if not cam_cfg_data.get('enabled', False): continue
            cam_obj = self.node.camera_manager.cameras.get(cam_id_cfg)
            stat.add(f"Cam [{cam_id_cfg}] Type", cam_cfg_data.get('type', "N/A"))
            stat.add(f"Cam [{cam_id_cfg}] SN/Idx", str(cam_cfg_data.get('serial_number') or cam_cfg_data.get('device_id', "N/A")))
            stat.add(f"Cam [{cam_id_cfg}] Status", "Running" if cam_obj and cam_obj.running else "Not Running/Error")
            if cam_id_cfg in self.node.actual_publish_rates:
                stat.add(f"Cam [{cam_id_cfg}] Target FPS (Color)", str(cam_cfg_data.get('color',{}).get('fps','N/A')))
                stat.add(f"Cam [{cam_id_cfg}] Actual Color FPS", f"{self.node.actual_publish_rates[cam_id_cfg]['color']:.2f}")
                if cam_cfg_data.get('depth',{}).get('enabled', True):
                    stat.add(f"Cam [{cam_id_cfg}] Target FPS (Depth)", str(cam_cfg_data.get('depth',{}).get('fps','N/A')))
                    stat.add(f"Cam [{cam_id_cfg}] Actual Depth FPS", f"{self.node.actual_publish_rates[cam_id_cfg]['depth']:.2f}")
        if self.node.run_startup_tests and self.node.camera_test_results:
            for i, test_res in enumerate(self.node.camera_test_results):
                cam_conf = self.node.camera_configs_yaml.get('cameras',{}).get(test_res.camera_id, {})
                expects_depth = cam_conf.get('depth',{}).get('enabled', True) if test_res.camera_type in ["realsense","zed"] else False
                res_stat = "OK" if test_res.is_success(expects_depth=expects_depth) else "FAIL"
                stat.add(f"Test {i} ID", test_res.camera_id); stat.add(f"Test {i} Result", res_stat)
                if test_res.error_message: stat.add(f"Test {i} Error", test_res.error_message)
        return stat

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        if not (REALSENSE_AVAILABLE or ZED_AVAILABLE or CV2_AVAILABLE): 
            print("No usable camera SDKs (RealSense, ZED, OpenCV) found. CameraNode cannot start.", file=sys.stderr)
            if rclpy.ok(): rclpy.shutdown(); return
        node = CameraNode()
        if not node.initialization_ok:
             if node: node.get_logger().error("CameraNode initialization failed. Shutting down.")
             else: print("Critical error: CameraNode object could not be created.", file=sys.stderr)
             if rclpy.ok(): rclpy.shutdown()
             return
        if rclpy.ok(): rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info('Keyboard interrupt, shutting down.')
    except Exception as e:
        if node and hasattr(node, 'get_logger'): node.get_logger().error(f"Unhandled exception: {e}", exc_info=True)
        else: print(f"Unhandled exception: {e}", file=sys.stderr); import traceback; traceback.print_exc()
    finally:
        if node and rclpy.ok() and hasattr(node, 'destroy_node'): node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main() 