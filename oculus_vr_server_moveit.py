#!/usr/bin/env python3
"""
Oculus VR Server - MoveIt Edition
Migrated from Deoxys to MoveIt while preserving DROID-exact VRPolicy control

VR-to-Robot Control Pipeline:
1. VR Data Capture: Raw poses from Oculus Reader (50Hz internal thread)
2. Coordinate Transform: Apply calibrated transformation [X,Y,Z] → [-Y,X,Z]
3. Velocity Calculation: Position/rotation offsets with gains (pos=5, rot=2)
4. Velocity Limiting: Clip to [-1, 1] range
5. Delta Conversion: Scale by max_delta (0.075m linear, 0.15rad angular)
6. Position Target: Add deltas to current position/orientation
7. MoveIt Command: Send position + quaternion targets via IK solver (15Hz)

Migration Changes from Deoxys:
- MoveIt IK service replaces Deoxys internal IK
- ROS 2 trajectory actions replace Deoxys socket commands
- Forward kinematics for robot state instead of socket queries
- Enhanced collision avoidance and safety features

Features Preserved:
- DROID-exact control parameters and transformations
- Async architecture with threaded workers
- MCAP data recording with camera integration
- Intuitive forward direction calibration
- Origin calibration on grip press/release
- 50Hz VR polling with internal state thread
- Safety limiting and workspace bounds
- Performance optimizations and hot reload
"""

import time
import threading
import numpy as np
import signal
import sys
import argparse
from scipy.spatial.transform import Rotation as R
from typing import Dict, Optional, Tuple
import os
import queue
from dataclasses import dataclass
from collections import deque
import copy

# ROS 2 and MoveIt imports (replacing Deoxys)
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPlanningScene, GetPositionFK
from moveit_msgs.msg import PositionIKRequest, RobotState as MoveitRobotState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from std_msgs.msg import Header

# Import the Oculus Reader
from oculus_reader.reader import OculusReader

# Import simulation components
from simulation.fr3_sim_server import FR3SimServer

# Import MCAP data recorder
from frankateach.mcap_data_recorder import MCAPDataRecorder
from frankateach.mcap_verifier import MCAPVerifier

# Define constants locally (replacing frankateach.constants)
GRIPPER_OPEN = 0.0
GRIPPER_CLOSE = 1.0
ROBOT_WORKSPACE_MIN = np.array([-0.6, -0.6, 0.0])
ROBOT_WORKSPACE_MAX = np.array([0.6, 0.6, 1.0])
CONTROL_FREQ = 60  # Hz - Ultra-low latency VR processing with pose smoothing


@dataclass
class VRState:
    """Thread-safe VR controller state"""
    timestamp: float
    poses: Dict
    buttons: Dict
    movement_enabled: bool
    controller_on: bool
    
    def copy(self):
        """Deep copy for thread safety"""
        return VRState(
            timestamp=self.timestamp,
            poses=copy.deepcopy(self.poses),
            buttons=copy.deepcopy(self.buttons),
            movement_enabled=self.movement_enabled,
            controller_on=self.controller_on
        )


@dataclass
class RobotState:
    """Thread-safe robot state"""
    timestamp: float
    pos: np.ndarray
    quat: np.ndarray
    euler: np.ndarray
    gripper: float
    joint_positions: Optional[np.ndarray]
    
    def copy(self):
        """Deep copy for thread safety"""
        return RobotState(
            timestamp=self.timestamp,
            pos=self.pos.copy() if self.pos is not None else None,
            quat=self.quat.copy() if self.quat is not None else None,
            euler=self.euler.copy() if self.euler is not None else None,
            gripper=self.gripper,
            joint_positions=self.joint_positions.copy() if self.joint_positions is not None else None
        )


@dataclass
class TimestepData:
    """Data structure for MCAP recording"""
    timestamp: float
    vr_state: VRState
    robot_state: RobotState
    action: np.ndarray
    info: Dict


def vec_to_reorder_mat(vec):
    """Convert reordering vector to transformation matrix"""
    X = np.zeros((len(vec), len(vec)))
    for i in range(X.shape[0]):
        ind = int(abs(vec[i])) - 1
        X[i, ind] = np.sign(vec[i])
    return X


def rmat_to_quat(rot_mat):
    """Convert rotation matrix to quaternion (x,y,z,w)"""
    rotation = R.from_matrix(rot_mat)
    return rotation.as_quat()


def quat_to_rmat(quat):
    """Convert quaternion (x,y,z,w) to rotation matrix"""
    return R.from_quat(quat).as_matrix()


def quat_diff(target, source):
    """Calculate quaternion difference"""
    result = R.from_quat(target) * R.from_quat(source).inv()
    return result.as_quat()


def quat_to_euler(quat, degrees=False):
    """Convert quaternion to euler angles"""
    euler = R.from_quat(quat).as_euler("xyz", degrees=degrees)
    return euler


def euler_to_quat(euler, degrees=False):
    """Convert euler angles to quaternion"""
    return R.from_euler("xyz", euler, degrees=degrees).as_quat()


def add_angles(delta, source, degrees=False):
    """Add two sets of euler angles"""
    delta_rot = R.from_euler("xyz", delta, degrees=degrees)
    source_rot = R.from_euler("xyz", source, degrees=degrees)
    new_rot = delta_rot * source_rot
    return new_rot.as_euler("xyz", degrees=degrees)


class OculusVRServer(Node):  # INHERIT FROM ROS 2 NODE
    def __init__(self, 
                 debug=False, 
                 right_controller=True, 
                 ip_address=None,
                 simulation=False,
                 coord_transform=None,
                 rotation_mode="labelbox",
                 performance_mode=False,
                 enable_recording=True,
                 camera_configs=None,
                 verify_data=False,
                 camera_config_path=None,
                 enable_cameras=False):
        """
        Initialize the Oculus VR Server with MoveIt-based control
        
        Args:
            debug: If True, only print data without controlling robot
            right_controller: If True, use right controller for robot control
            ip_address: IP address of Quest device (None for USB connection)
            simulation: If True, use simulated FR3 robot instead of real hardware
            coord_transform: Custom coordinate transformation vector (default: adjusted for compatibility)
            rotation_mode: Rotation mapping mode - currently only "labelbox" is supported
            performance_mode: If True, enable performance optimizations
            enable_recording: If True, enable MCAP data recording functionality
            camera_configs: Camera configuration dictionary for recording
            verify_data: If True, verify MCAP data after successful recording
            camera_config_path: Path to camera configuration JSON file
            enable_cameras: If True, enable camera recording
        """
        # Initialize ROS 2 node FIRST
        super().__init__('oculus_vr_server_moveit')
        
        # Robot configuration (from simple_arm_control.py)
        self.robot_ip = "192.168.1.59"
        self.planning_group = "fr3_arm"  # Changed from panda_arm to fr3_arm for FR3 robot
        self.end_effector_link = "fr3_hand_tcp"
        self.base_frame = "fr3_link0"
        self.planning_frame = "fr3_link0"
        
        # Joint names for FR3
        self.joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]
        
        # Home position (ready pose)
        self.home_positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        
        # Store parameters
        self.debug = debug
        self.right_controller = right_controller
        self.simulation = simulation
        self.running = True
        self.verify_data = verify_data
        
        # Enhanced debugging features
        self.debug_moveit = True  # Enable MoveIt debugging for diagnosis
        self.debug_ik_failures = True  # Log IK failures for debugging
        self.debug_comm_stats = True  # Log communication statistics
        
        # Create service clients for MoveIt integration
        self.get_logger().info('🔄 Initializing MoveIt service clients...')
        
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.planning_scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        
        # Create action client for trajectory execution
        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory, '/fr3_arm_controller/follow_joint_trajectory'
        )
        
        # Joint state subscriber
        self.joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        # Wait for services (critical for reliability)
        self.get_logger().info('🔄 Waiting for MoveIt services...')
        
        services_ready = True
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("❌ IK service not available")
            services_ready = False
        else:
            self.get_logger().info("✅ IK service ready")
            
        if not self.planning_scene_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("❌ Planning scene service not available")
            services_ready = False
        else:
            self.get_logger().info("✅ Planning scene service ready")
            
        if not self.fk_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("❌ FK service not available")
            services_ready = False
        else:
            self.get_logger().info("✅ FK service ready")
            
        if not self.trajectory_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("❌ Trajectory action server not available")
            services_ready = False
        else:
            self.get_logger().info("✅ Trajectory action server ready")
            
        if not services_ready:
            if not self.debug:
                raise RuntimeError("Required MoveIt services not available. Ensure MoveIt is running.")
            else:
                self.get_logger().warn("⚠️  MoveIt services not available, but continuing in debug mode")
        
        self.get_logger().info('✅ All required MoveIt services ready!')
        
        # DROID VRPolicy exact parameters - preserved unchanged
        self.max_lin_vel = 1.0
        self.max_rot_vel = 1.0
        self.max_gripper_vel = 1.0
        self.spatial_coeff = 1.0
        self.pos_action_gain = 5.0
        self.rot_action_gain = 2.0
        self.gripper_action_gain = 3.0
        self.control_hz = CONTROL_FREQ
        self.control_interval = 1.0 / self.control_hz
        
        # DROID IK solver parameters for velocity-to-delta conversion
        self.max_lin_delta = 0.075
        self.max_rot_delta = 0.15
        self.max_gripper_delta = 0.25
        
        # Continue with ALL other initialization exactly as before...
        # Coordinate transformation setup
        if coord_transform is None:
            rmat_reorder = [-3, -1, 2, 4]  # Default transformation
            if self.debug:
                print("\n⚠️  Using adjusted coordinate transformation for better compatibility")
                print("   If rotation is still incorrect, try --coord-transform with different values")
        else:
            rmat_reorder = coord_transform
            
        self.global_to_env_mat = vec_to_reorder_mat(rmat_reorder)
        self.rotation_mode = rotation_mode
        
        if self.debug or coord_transform is not None:
            print("\n🔍 Coordinate Transformation:")
            print(f"   Position reorder vector: {rmat_reorder}")
            print(f"   Rotation mode: {rotation_mode}")
            print("   Position Transformation Matrix:")
            for i in range(4):
                row = self.global_to_env_mat[i]
                print(f"   [{row[0]:6.1f}, {row[1]:6.1f}, {row[2]:6.1f}, {row[3]:6.1f}]")
        
        # Initialize transformation matrices
        self.vr_to_global_mat = np.eye(4)
        
        # Controller ID
        self.controller_id = "r" if right_controller else "l"
        
        # Initialize state
        self.reset_state()
        
        # Initialize Oculus Reader
        print("🎮 Initializing Oculus Reader...")
        try:
            self.oculus_reader = OculusReader(
                ip_address=ip_address,
                print_FPS=False
            )
            print("✅ Oculus Reader initialized successfully")
        except Exception as e:
            print(f"❌ Failed to initialize Oculus Reader: {e}")
            if not self.debug:
                sys.exit(1)
            else:
                print("⚠️  Continuing in debug mode without Oculus Reader")
        
        # Simulation server setup
        self.sim_server = None
        if self.simulation:
            print("🤖 Starting FR3 simulation server...")
            self.sim_server = FR3SimServer(visualize=True)
            self.sim_server.start()
            time.sleep(1.0)
            print("✅ Simulation server started")
        
        # Camera and recording setup
        self.enable_recording = enable_recording
        self.data_recorder = None
        self.recording_active = False
        self.prev_a_button = False
        
        self.camera_manager = None
        self.enable_cameras = enable_cameras
        self.camera_config_path = camera_config_path
        
        if self.enable_cameras and self.camera_config_path:
            try:
                print("\n🔍 Testing camera functionality...")
                from frankateach.camera_test import test_cameras
                
                import yaml
                with open(self.camera_config_path, 'r') as f:
                    test_camera_configs = yaml.safe_load(f)
                
                all_passed, test_results = test_cameras(test_camera_configs)
                
                if not all_passed:
                    print("\n❌ Camera tests failed!")
                    print("   Some cameras are not functioning properly.")
                    if not self.debug:
                        response = input("\n   Continue anyway? (y/N): ")
                        if response.lower() != 'y':
                            print("   Exiting due to camera test failures.")
                            sys.exit(1)
                    print("   Continuing with available cameras...")
                
                from frankateach.camera_manager import CameraManager
                self.camera_manager = CameraManager(self.camera_config_path)
                print("📷 Camera manager initialized")
            except Exception as e:
                print(f"⚠️  Failed to initialize camera manager: {e}")
                self.camera_manager = None
        
        if self.enable_recording:
            self.data_recorder = MCAPDataRecorder(
                camera_configs=camera_configs,
                save_images=True,
                save_depth=True,
                camera_manager=self.camera_manager
            )
            print("📹 MCAP data recording enabled")
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # Start VR state listening thread
        self._state_thread = threading.Thread(target=self._update_internal_state)
        self._state_thread.daemon = True
        self._state_thread.start()
        
        # Performance mode setup
        self.enable_performance_mode = performance_mode
        if self.enable_performance_mode:
            self.control_hz = CONTROL_FREQ * 2
            self.control_interval = 1.0 / self.control_hz
            self.pos_action_gain = 10.0
            self.rot_action_gain = 3.0
            self.max_lin_delta = 0.05
            self.max_rot_delta = 0.1
            
            print("\n⚡ PERFORMANCE MODE ENABLED:")
            print(f"   Control frequency: {self.control_hz}Hz (2x faster)")
            print(f"   Position gain: {self.pos_action_gain} (100% higher)")
            print(f"   Rotation gain: {self.rot_action_gain} (50% higher)")
        
        # Threading and async setup
        self.translation_deadzone = 0.0005
        self.use_position_filter = True
        self.position_filter_alpha = 0.8
        self._last_vr_pos = None
        
        # Ultra-smooth pose filtering for 60Hz operation
        self.pose_smoothing_enabled = True
        self.pose_smoothing_alpha = 0.35  # More responsive (was 0.25) since basic control works
        self.velocity_smoothing_alpha = 0.25  # More responsive for better tracking
        self._smoothed_target_pos = None
        self._smoothed_target_quat = None
        self._smoothed_target_gripper = None
        self._last_command_time = 0.0
        self._pose_history = deque(maxlen=3)  # Smaller history for 60Hz (3 vs 5)
        
        # Adaptive command rate for smooth motion
        self.min_command_interval = 0.067  # 15Hz robot commands (optimized up from 10Hz)
        self.adaptive_smoothing = True  # Adjust smoothing based on motion speed
        
        # Async components
        self._vr_state_lock = threading.Lock()
        self._robot_state_lock = threading.Lock()
        self._robot_comm_lock = threading.Lock()
        self._latest_vr_state = None
        self._latest_robot_state = None
        
        # Thread-safe queues
        self.mcap_queue = queue.Queue(maxsize=1000)
        self.control_queue = queue.Queue(maxsize=10)
        
        # Thread management
        self._threads = []
        self._mcap_writer_thread = None
        self._robot_control_thread = None
        self._control_paused = False
        
        # Recording frequency
        self.recording_hz = self.control_hz
        self.recording_interval = 1.0 / self.recording_hz
        
        # Robot communication queues (for async MoveIt communication)
        self._robot_command_queue = queue.Queue(maxsize=2)
        self._robot_response_queue = queue.Queue(maxsize=2)
        self._robot_comm_thread = None
        
        # MoveIt communication statistics
        self._ik_success_count = 0
        self._ik_failure_count = 0
        self._trajectory_success_count = 0
        self._trajectory_failure_count = 0
        
        # Print status
        print("\n🎮 Oculus VR Server - MoveIt Edition (Optimized 15Hz)")
        print(f"   Using {'RIGHT' if right_controller else 'LEFT'} controller")
        print(f"   Mode: {'DEBUG' if debug else 'LIVE ROBOT CONTROL'}")
        print(f"   Robot: {'SIMULATED FR3' if simulation else 'REAL HARDWARE'}")
        print(f"   VR Processing: {self.control_hz}Hz (Ultra-low latency)")
        print(f"   Robot Commands: 15Hz (Optimized responsiveness)")
        print(f"   Position gain: {self.pos_action_gain}")
        print(f"   Rotation gain: {self.rot_action_gain}")
        print(f"   MoveIt integration: IK solver + collision avoidance + velocity-limited trajectories")
        
        print("\n📋 Controls:")
        print("   - HOLD grip button: Enable teleoperation")
        print("   - RELEASE grip button: Pause teleoperation")
        print("   - PRESS trigger: Close gripper")
        print("   - RELEASE trigger: Open gripper")
        
        if self.enable_recording:
            print("\n📹 Recording Controls:")
            print("   - A button: Start recording or stop current recording")
            print("   - B button: Mark recording as successful and save")
            print("   - Recordings saved to: ~/recordings/success")
        else:
            print("   - A/X button: Mark success and exit")
            print("   - B/Y button: Mark failure and exit")
        
        print("\n🧭 Forward Direction Calibration:")
        print("   - HOLD joystick button and MOVE controller forward")
        print("   - Move at least 3mm in your desired forward direction")
        print("   - Release joystick button to complete calibration")
        
        print("\n💡 Hot Reload:")
        print("   - Run with --hot-reload flag to enable automatic restart")
        print("   - The server will restart automatically when you save changes")
        
        print("\nPress Ctrl+C to exit gracefully\n")

    def reset_state(self):
        """Reset internal state - exactly as before"""
        self._state = {
            "poses": {},
            "buttons": {"A": False, "B": False, "X": False, "Y": False},
            "movement_enabled": False,
            "controller_on": True,
        }
        self.update_sensor = True
        self.reset_origin = True
        self.reset_orientation = True
        self.robot_origin = None
        self.vr_origin = None
        self.vr_state = None
        
        self.robot_pos = None
        self.robot_quat = None
        self.robot_euler = None
        self.robot_gripper = 0.0
        self.robot_joint_positions = None
        
        self.prev_joystick_state = False
        self.prev_grip_state = False
        
        self.calibrating_forward = False
        self.calibration_start_pose = None
        self.calibration_start_time = None
        self.vr_neutral_pose = None
        
        self.is_first_frame = True
        self._reset_robot_after_calibration = False
        self._last_controller_rot = None
        self._last_vr_pos = None
        self._last_action = np.zeros(7)
        
        # Joint trajectory smoothing
        self._last_joint_positions = None

    def signal_handler(self, signum, frame):
        """Handle Ctrl+C and other termination signals"""
        print(f"\n🛑 Received signal {signum}, shutting down gracefully...")
        self.stop_server()

    # =====================================
    # MOVEIT-SPECIFIC HELPER METHODS
    # =====================================
    
    def joint_state_callback(self, msg):
        """Store the latest joint state"""
        self.joint_state = msg
        if self.debug_comm_stats and hasattr(self, '_last_joint_state_time'):
            dt = time.time() - self._last_joint_state_time
            if dt > 0.1:  # Log if joint states are slow
                self.get_logger().warn(f"Slow joint state update: {dt*1000:.1f}ms")
        self._last_joint_state_time = time.time()

    def get_current_joint_positions(self):
        """Get current joint positions from joint_states topic with robust error handling"""
        # Wait for joint state if not available
        max_wait_time = 2.0
        start_time = time.time()
        
        while self.joint_state is None and (time.time() - start_time) < max_wait_time:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.01)
        
        if self.joint_state is None:
            if self.debug_moveit:
                self.get_logger().warn("No joint state available after waiting")
            return None
            
        positions = []
        missing_joints = []
        for joint_name in self.joint_names:
            if joint_name in self.joint_state.name:
                idx = self.joint_state.name.index(joint_name)
                positions.append(self.joint_state.position[idx])
            else:
                missing_joints.append(joint_name)
        
        if missing_joints:
            if self.debug_moveit:
                self.get_logger().warn(f"Missing joints in joint state: {missing_joints}")
                self.get_logger().warn(f"Available joints: {list(self.joint_state.name)}")
            return None
            
        return positions

    def get_current_end_effector_pose(self):
        """Get current end-effector pose using forward kinematics with robust error handling"""
        current_joints = self.get_current_joint_positions()
        if current_joints is None:
            self.get_logger().warn("Cannot get joint positions for FK")
            return None, None
        
        # Create FK request
        fk_request = GetPositionFK.Request()
        fk_request.fk_link_names = [self.end_effector_link]
        fk_request.header.frame_id = self.base_frame
        fk_request.header.stamp = self.get_clock().now().to_msg()
        
        # Set robot state
        fk_request.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        fk_request.robot_state.joint_state.name = self.joint_names
        fk_request.robot_state.joint_state.position = current_joints
        
        # Call FK service with retries
        max_retries = 3
        for attempt in range(max_retries):
            try:
                fk_start = time.time()
                fk_future = self.fk_client.call_async(fk_request)
                
                # Wait for response with timeout
                rclpy.spin_until_future_complete(self, fk_future, timeout_sec=0.5)
                fk_time = time.time() - fk_start
                
                if not fk_future.done():
                    self.get_logger().warn(f"FK service timeout on attempt {attempt + 1}")
                    continue
                
                fk_response = fk_future.result()
                
                if fk_response and fk_response.error_code.val == 1 and fk_response.pose_stamped:
                    pose = fk_response.pose_stamped[0].pose
                    pos = np.array([pose.position.x, pose.position.y, pose.position.z])
                    quat = np.array([pose.orientation.x, pose.orientation.y, 
                                    pose.orientation.z, pose.orientation.w])
                    
                    if self.debug_moveit:
                        self.get_logger().info(f"FK successful: pos=[{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
                    
                    return pos, quat
                else:
                    error_code = fk_response.error_code.val if fk_response else "No response"
                    self.get_logger().warn(f"FK failed with error code: {error_code} on attempt {attempt + 1}")
                    
            except Exception as e:
                self.get_logger().warn(f"FK attempt {attempt + 1} exception: {e}")
                
            if attempt < max_retries - 1:
                time.sleep(0.2)  # Wait before retry
        
        self.get_logger().error("FK failed after all retries")
        return None, None

    def get_planning_scene(self):
        """Get current planning scene for collision checking"""
        scene_request = GetPlanningScene.Request()
        scene_request.components.components = (
            scene_request.components.SCENE_SETTINGS |
            scene_request.components.ROBOT_STATE |
            scene_request.components.ROBOT_STATE_ATTACHED_OBJECTS |
            scene_request.components.WORLD_OBJECT_NAMES |
            scene_request.components.WORLD_OBJECT_GEOMETRY |
            scene_request.components.OCTOMAP |
            scene_request.components.TRANSFORMS |
            scene_request.components.ALLOWED_COLLISION_MATRIX |
            scene_request.components.LINK_PADDING_AND_SCALING |
            scene_request.components.OBJECT_COLORS
        )
        
        scene_start = time.time()
        scene_future = self.planning_scene_client.call_async(scene_request)
        rclpy.spin_until_future_complete(self, scene_future, timeout_sec=0.5)
        scene_time = time.time() - scene_start
        
        if self.debug_comm_stats and scene_time > 0.1:
            self.get_logger().warn(f"Slow planning scene fetch: {scene_time*1000:.1f}ms")
        
        return scene_future.result()

    def execute_trajectory(self, positions, duration=2.0):
        """Execute a trajectory to move joints to target positions and WAIT for completion"""
        if not self.trajectory_client.server_is_ready():
            if self.debug_moveit:
                self.get_logger().warn("Trajectory action server not ready")
            return False
            
        print(f"🎯 Executing trajectory to target positions (duration: {duration}s)...")
        
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Add single point
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        # Add zero velocities and accelerations for smooth stop at target
        point.velocities = [0.0] * len(self.joint_names)
        point.accelerations = [0.0] * len(self.joint_names)
        
        trajectory.points.append(point)
            
        # Create goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        # More forgiving tolerances for reset operations to prevent failures
        goal.path_tolerance = [
            # More forgiving tolerances to handle reset operations
            JointTolerance(name=name, position=0.02, velocity=0.2, acceleration=0.2) 
            for name in self.joint_names
        ]
        
        # More forgiving goal tolerance for successful completion
        goal.goal_tolerance = [
            JointTolerance(name=name, position=0.015, velocity=0.1, acceleration=0.1)
            for name in self.joint_names
        ]
        
        # Send goal and WAIT for completion (essential for reset operations)
        print("📤 Sending trajectory goal...")
        send_goal_future = self.trajectory_client.send_goal_async(goal)
        
        # Wait for goal to be accepted
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=2.0)
        
        if not send_goal_future.done():
            print("❌ Failed to send goal (timeout)")
            return False
            
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            print("❌ Goal was rejected")
            return False
        
        print("✅ Goal accepted, waiting for completion...")
        
        # Wait for execution to complete
        result_future = goal_handle.get_result_async()
        
        # Monitor progress with status updates
        start_time = time.time()
        last_update = 0
        
        while not result_future.done():
            elapsed = time.time() - start_time
            if elapsed - last_update >= 2.0:  # Update every 2 seconds
                print(f"   ⏱️  Executing... {elapsed:.1f}s elapsed")
                last_update = elapsed
            
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if elapsed > duration + 10.0:  # Give plenty of extra time for completion
                print("❌ Trajectory execution timeout")
                return False
        
        # Get final result
        result = result_future.result()
        
        if result.result.error_code == 0:  # SUCCESS
            print("✅ Trajectory execution completed successfully!")
            return True
        else:
            print(f"❌ Trajectory execution failed with error code: {result.result.error_code}")
            return False

    def compute_ik_for_pose(self, pos, quat):
        """Compute IK for Cartesian pose with enhanced debugging"""
        # Get planning scene
        scene_response = self.get_planning_scene()
        if scene_response is None:
            if self.debug_ik_failures:
                self.get_logger().warn("Cannot get planning scene for IK")
            return None
        
        # Create IK request
        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = self.planning_group
        ik_request.ik_request.robot_state = scene_response.scene.robot_state
        ik_request.ik_request.avoid_collisions = True
        ik_request.ik_request.timeout.sec = 0
        ik_request.ik_request.timeout.nanosec = int(0.1 * 1e9)  # 100ms timeout
        
        # Set target pose
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.base_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = float(pos[0])
        pose_stamped.pose.position.y = float(pos[1])
        pose_stamped.pose.position.z = float(pos[2])
        pose_stamped.pose.orientation.x = float(quat[0])
        pose_stamped.pose.orientation.y = float(quat[1])
        pose_stamped.pose.orientation.z = float(quat[2])
        pose_stamped.pose.orientation.w = float(quat[3])
        
        ik_request.ik_request.pose_stamped = pose_stamped
        ik_request.ik_request.ik_link_name = self.end_effector_link
        
        # Call IK service
        ik_start = time.time()
        ik_future = self.ik_client.call_async(ik_request)
        rclpy.spin_until_future_complete(self, ik_future, timeout_sec=0.2)
        ik_response = ik_future.result()
        ik_time = time.time() - ik_start
        
        if ik_response and ik_response.error_code.val == 1:
            # Success
            self._ik_success_count += 1
            
            # Extract joint positions for our 7 joints
            joint_positions = []
            for joint_name in self.joint_names:
                if joint_name in ik_response.solution.joint_state.name:
                    idx = ik_response.solution.joint_state.name.index(joint_name)
                    joint_positions.append(ik_response.solution.joint_state.position[idx])
            
            if self.debug_comm_stats and ik_time > 0.05:
                self.get_logger().warn(f"Slow IK computation: {ik_time*1000:.1f}ms")
            
            return joint_positions if len(joint_positions) == 7 else None
        else:
            # Failure
            self._ik_failure_count += 1
            
            if self.debug_ik_failures:
                error_code = ik_response.error_code.val if ik_response else "No response"
                self.get_logger().warn(f"IK failed: error_code={error_code}, time={ik_time*1000:.1f}ms")
                self.get_logger().warn(f"Target pose: pos=[{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}], "
                                     f"quat=[{quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f}]")
            
            return None

    def execute_single_point_trajectory(self, joint_positions):
        """Execute single-point trajectory (VR-style individual command) with optimized velocity limiting"""
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        # Keep 300ms execution time but optimize velocity profiles
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(0.3 * 1e9)  # 300ms execution
        
        # Add velocity profiles with smart limiting based on actual tolerance (0.5 rad/s)
        if hasattr(self, '_last_joint_positions') and self._last_joint_positions is not None:
            position_deltas = np.array(joint_positions) - np.array(self._last_joint_positions)
            # Calculate velocities for 300ms execution
            smooth_velocities = position_deltas / 0.3  # Velocity to reach target in 300ms
            smooth_velocities *= 0.25  # Scale down to stay well under 0.5 rad/s limit
            
            # Apply per-joint velocity limiting to stay under tolerance (0.4 rad/s max)
            max_velocity = 0.4  # Stay well under 0.5 rad/s tolerance
            for i in range(len(smooth_velocities)):
                if abs(smooth_velocities[i]) > max_velocity:
                    smooth_velocities[i] = max_velocity * np.sign(smooth_velocities[i])
            
            point.velocities = smooth_velocities.tolist()
        else:
            point.velocities = [0.0] * len(joint_positions)  # Stop at target for first command
        
        # Conservative acceleration limits
        point.accelerations = [0.0] * len(joint_positions)  # Let MoveIt handle acceleration
        
        trajectory.points.append(point)
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        # Optimized tolerances - slightly more forgiving than current limits
        goal.path_tolerance = [
            # Fine-tuned tolerances just above current robot limits
            JointTolerance(name=name, position=0.05, velocity=0.6, acceleration=0.5) 
            for name in self.joint_names
        ]
        
        # Store joint positions for next velocity calculation
        self._last_joint_positions = joint_positions
        
        # Send goal (non-blocking for high frequency)
        send_goal_future = self.trajectory_client.send_goal_async(goal)
        # Note: We don't wait for completion to maintain high frequency
        
        return True  # Assume success for high-frequency operation

    def execute_moveit_command(self, command):
        """Execute individual MoveIt command (VR teleoperation style)"""
        try:
            # Convert Cartesian pose to joint positions using IK
            joint_positions = self.compute_ik_for_pose(command.pos, command.quat)
            
            if joint_positions is None:
                return False
            
            # Execute single-point trajectory (like VR teleoperation)
            return self.execute_single_point_trajectory(joint_positions)
            
        except Exception as e:
            if self.debug_moveit:
                self.get_logger().warn(f"MoveIt command execution failed: {e}")
            return False

    def reset_robot(self, sync=True):
        """Reset robot to initial position using MoveIt trajectory with retry logic"""
        if self.debug:
            print("🔄 [DEBUG] Would reset robot to initial position")
            return np.array([0.4, 0.0, 0.3]), np.array([1.0, 0.0, 0.0, 0.0]), None
        
        print("🔄 Resetting robot to initial position...")
        
        # First, check if services are ready
        print("🔍 Checking MoveIt services...")
        if not self.ik_client.service_is_ready():
            print("⚠️  IK service not ready, waiting...")
            if not self.ik_client.wait_for_service(timeout_sec=5.0):
                print("❌ IK service still not ready after 5s")
        
        if not self.fk_client.service_is_ready():
            print("⚠️  FK service not ready, waiting...")
            if not self.fk_client.wait_for_service(timeout_sec=5.0):
                print("❌ FK service still not ready after 5s")
        
        if not self.trajectory_client.server_is_ready():
            print("⚠️  Trajectory server not ready, waiting...")
            if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
                print("❌ Trajectory server still not ready after 5s")
        
        # Wait for joint states to be available
        print("🔍 Waiting for joint states...")
        joint_wait_start = time.time()
        while self.joint_state is None and (time.time() - joint_wait_start) < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        if self.joint_state is None:
            print("❌ No joint states received after 5s")
        else:
            print(f"✅ Joint states available: {len(self.joint_state.name)} joints")
        
        # Execute trajectory to home position (now properly waits for completion)
        print(f"\n🏠 Moving robot to home position...")
        success = self.execute_trajectory(self.home_positions, duration=5.0)
        
        if success:
            print(f"✅ Robot successfully moved to home position!")
            # Give time for robot to settle
            print(f"⏱️  Waiting for robot to settle...")
            time.sleep(1.0)
            
            # Get new position via FK
            print(f"📍 Reading final robot state...")
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.1)
                
            pos, quat = self.get_current_end_effector_pose()
            joint_positions = self.get_current_joint_positions()
            
            if pos is not None and quat is not None:
                print(f"✅ Robot reset complete!")
                print(f"   Position: [{pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f}]")
                print(f"   Quaternion: [{quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f}]")
                
                return pos, quat, joint_positions
            else:
                print(f"⚠️  Warning: Could not read final robot state, but trajectory completed successfully")
                # Return default home pose as fallback
                default_pos = np.array([0.307, 0.000, 0.487])  # Approximate FR3 home position
                default_quat = np.array([1.0, 0.0, 0.0, 0.0])  # Neutral orientation
                return default_pos, default_quat, self.home_positions
        else:
            print(f"❌ Robot trajectory to home position failed")
            
            # Try to get current state as fallback
            print("🔍 Attempting to get current robot state as fallback...")
            try:
                for _ in range(10):
                    rclpy.spin_once(self, timeout_sec=0.1)
                    time.sleep(0.1)
                
                pos, quat = self.get_current_end_effector_pose()
                joint_positions = self.get_current_joint_positions()
                
                if pos is not None and quat is not None:
                    print("✅ Using current robot position as starting point")
                    print(f"   Position: [{pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f}]")
                    return pos, quat, joint_positions
                else:
                    print("❌ Still cannot read robot state")
            except Exception as e:
                print(f"❌ Exception getting current state: {e}")
            
            raise RuntimeError("Failed to reset robot and cannot read current state")

    def print_moveit_stats(self):
        """Print MoveIt communication statistics"""
        total_ik = self._ik_success_count + self._ik_failure_count
        total_traj = self._trajectory_success_count + self._trajectory_failure_count
        
        if total_ik > 0:
            ik_success_rate = (self._ik_success_count / total_ik) * 100
            print(f"📊 MoveIt IK Stats: {ik_success_rate:.1f}% success ({self._ik_success_count}/{total_ik})")
        
        if total_traj > 0:
            traj_success_rate = (self._trajectory_success_count / total_traj) * 100
            print(f"📊 Trajectory Stats: {traj_success_rate:.1f}% success ({self._trajectory_success_count}/{total_traj})")

    # =====================================
    # PRESERVED VR PROCESSING METHODS
    # =====================================
    
    def _update_internal_state(self, num_wait_sec=5, hz=50):
        """Continuously poll VR controller state at 50Hz - preserved exactly"""
        last_read_time = time.time()
        
        while self.running:
            # Regulate Read Frequency
            time.sleep(1 / hz)
            
            # Read Controller
            time_since_read = time.time() - last_read_time
            
            if hasattr(self, 'oculus_reader'):
                poses, buttons = self.oculus_reader.get_transformations_and_buttons()
                self._state["controller_on"] = time_since_read < num_wait_sec
            else:
                # Debug mode without Oculus Reader
                poses, buttons = {}, {}
                self._state["controller_on"] = True
            
            if poses == {}:
                continue
            
            # Get current button states
            current_grip = buttons.get(self.controller_id.upper() + "G", False)
            current_joystick = buttons.get(self.controller_id.upper() + "J", False)
            
            # Detect edge transitions
            grip_toggled = self.prev_grip_state != current_grip
            joystick_pressed = current_joystick and not self.prev_joystick_state
            joystick_released = not current_joystick and self.prev_joystick_state
            
            # Update control flags
            self.update_sensor = self.update_sensor or current_grip
            self.reset_origin = self.reset_origin or grip_toggled
            
            # Save Info
            self._state["poses"] = poses
            self._state["buttons"] = buttons
            self._state["movement_enabled"] = current_grip
            self._state["controller_on"] = True
            last_read_time = time.time()
            
            # Publish VR state to async system
            current_time = time.time()
            vr_state = VRState(
                timestamp=current_time,
                poses=copy.deepcopy(poses),
                buttons=copy.deepcopy(buttons),
                movement_enabled=current_grip,
                controller_on=True
            )
            
            with self._vr_state_lock:
                self._latest_vr_state = vr_state
            
            # Handle Forward Direction Calibration (preserved exactly)
            if self.controller_id in self._state["poses"]:
                pose_matrix = self._state["poses"][self.controller_id]
                
                # Start calibration when joystick is pressed
                if joystick_pressed:
                    self.calibrating_forward = True
                    self.calibration_start_pose = pose_matrix.copy()
                    self.calibration_start_time = time.time()
                    print(f"\n🎯 Forward calibration started - Move controller in desired forward direction")
                    print(f"   Hold the joystick and move at least 3mm forward")
                
                # Complete calibration when joystick is released
                elif joystick_released and self.calibrating_forward:
                    self.calibrating_forward = False
                    
                    if self.calibration_start_pose is not None:
                        # Get movement vector
                        start_pos = self.calibration_start_pose[:3, 3]
                        end_pos = pose_matrix[:3, 3]
                        movement_vec = end_pos - start_pos
                        movement_distance = np.linalg.norm(movement_vec)
                        
                        if movement_distance > 0.003:  # 3mm threshold
                            # Normalize movement vector
                            forward_vec = movement_vec / movement_distance
                            
                            print(f"\n✅ Forward direction calibrated!")
                            print(f"   Movement distance: {movement_distance*1000:.1f}mm")
                            print(f"   Forward vector: [{forward_vec[0]:.3f}, {forward_vec[1]:.3f}, {forward_vec[2]:.3f}]")
                            
                            # Create rotation to align this vector with robot's forward
                            temp_mat = np.eye(4)
                            temp_mat[:3, 3] = forward_vec
                            transformed_temp = self.global_to_env_mat @ temp_mat
                            transformed_forward = transformed_temp[:3, 3]
                            
                            # Calculate rotation to align with robot's +X axis
                            robot_forward = np.array([1.0, 0.0, 0.0])
                            rotation_axis = np.cross(transformed_forward, robot_forward)
                            rotation_angle = np.arccos(np.clip(np.dot(transformed_forward, robot_forward), -1.0, 1.0))
                            
                            if np.linalg.norm(rotation_axis) > 0.001:
                                rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
                                # Create rotation matrix using Rodrigues' formula
                                K = np.array([[0, -rotation_axis[2], rotation_axis[1]],
                                             [rotation_axis[2], 0, -rotation_axis[0]],
                                             [-rotation_axis[1], rotation_axis[0], 0]])
                                R_calibration = np.eye(3) + np.sin(rotation_angle) * K + (1 - np.cos(rotation_angle)) * K @ K
                            else:
                                # Movement is already aligned with robot forward or backward
                                if transformed_forward[0] < 0:  # Moving backward
                                    R_calibration = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
                                else:
                                    R_calibration = np.eye(3)
                            
                            # Update the VR to global transformation
                            self.vr_to_global_mat = np.eye(4)
                            self.vr_to_global_mat[:3, :3] = R_calibration
                            
                            try:
                                self.vr_to_global_mat = np.linalg.inv(self.calibration_start_pose) @ self.vr_to_global_mat
                            except:
                                print("Warning: Could not invert calibration pose")
                            
                            self.reset_orientation = False
                            self.vr_neutral_pose = np.asarray(self._state["poses"][self.controller_id]).copy()
                            print("📐 Stored neutral controller orientation")
                            
                            # Reset robot to home position after calibration
                            if not self.debug and not self.reset_orientation:
                                print("🏠 Moving robot to reset position after calibration...")
                                self._reset_robot_after_calibration = True
                            elif self.debug:
                                print("\n✅ Calibration complete! Ready for teleoperation.")
                                print("   Hold grip button to start controlling the robot")
                        else:
                            print(f"\n⚠️  Not enough movement detected ({movement_distance*1000:.1f}mm)")
                            print(f"   Please move controller at least 3mm in your desired forward direction")
                            self.reset_orientation = True
                
                # Show calibration progress
                elif self.calibrating_forward and current_joystick:
                    if time.time() - self.calibration_start_time > 0.5:
                        current_pos = pose_matrix[:3, 3]
                        start_pos = self.calibration_start_pose[:3, 3]
                        distance = np.linalg.norm(current_pos - start_pos) * 1000
                        print(f"   Current movement: {distance:.1f}mm", end='\r')
            
            # DROID-style calibration fallback
            if self.reset_orientation and not self.calibrating_forward:
                stop_updating = self._state["buttons"][self.controller_id.upper() + "J"] or self._state["movement_enabled"]
                if stop_updating:
                    rot_mat = np.asarray(self._state["poses"][self.controller_id])
                    self.reset_orientation = False
                    try:
                        rot_mat = np.linalg.inv(rot_mat)
                    except:
                        print(f"exception for rot mat: {rot_mat}")
                        rot_mat = np.eye(4)
                        self.reset_orientation = True
                    self.vr_to_global_mat = rot_mat
                    print("📐 Orientation reset (DROID-style)")
                    
                    self.vr_neutral_pose = np.asarray(self._state["poses"][self.controller_id]).copy()
                    print("📐 Stored neutral controller orientation")
                    
                    if not self.debug and not self.reset_orientation:
                        print("🏠 Moving robot to reset position after calibration...")
                        self._reset_robot_after_calibration = True
            
            # Update previous button states
            self.prev_grip_state = current_grip
            self.prev_joystick_state = current_joystick

    def _process_reading(self):
        """Apply coordinate transformations to VR controller pose - preserved exactly"""
        rot_mat = np.asarray(self._state["poses"][self.controller_id])
        
        # Apply position transformation
        transformed_mat = self.global_to_env_mat @ self.vr_to_global_mat @ rot_mat
        vr_pos = self.spatial_coeff * transformed_mat[:3, 3]
        
        # Apply position filtering to reduce noise/drift
        if self.use_position_filter and self._last_vr_pos is not None:
            pos_delta = vr_pos - self._last_vr_pos
            
            # Apply deadzone to filter out small movements
            for i in range(3):
                if abs(pos_delta[i]) < self.translation_deadzone:
                    pos_delta[i] = 0.0
            
            vr_pos = self._last_vr_pos + pos_delta
        
        self._last_vr_pos = vr_pos.copy()
        
        # Handle rotation - preserved exactly
        if hasattr(self, 'vr_neutral_pose') and self.vr_neutral_pose is not None:
            neutral_rot = R.from_matrix(self.vr_neutral_pose[:3, :3])
            current_rot = R.from_matrix(rot_mat[:3, :3])
            
            relative_rot = neutral_rot.inv() * current_rot
            rotvec = relative_rot.as_rotvec()
            angle = np.linalg.norm(rotvec)
            
            if angle > 0:
                axis = rotvec / angle
                transformed_axis = np.array([-axis[1], axis[0], axis[2]])
                transformed_rotvec = transformed_axis * angle
                transformed_rot = R.from_rotvec(transformed_rotvec)
                vr_quat = transformed_rot.as_quat()
            else:
                vr_quat = np.array([0, 0, 0, 1])
        else:
            transformed_rot_mat = self.global_to_env_mat[:3, :3] @ self.vr_to_global_mat[:3, :3] @ rot_mat[:3, :3]
            vr_quat = rmat_to_quat(transformed_rot_mat)
        
        vr_gripper = self._state["buttons"]["rightTrig" if self.controller_id == "r" else "leftTrig"][0]
        
        self.vr_state = {"pos": vr_pos, "quat": vr_quat, "gripper": vr_gripper}
    
    def _limit_velocity(self, lin_vel, rot_vel, gripper_vel):
        """Scales down the linear and angular magnitudes of the action - preserved exactly"""
        lin_vel_norm = np.linalg.norm(lin_vel)
        rot_vel_norm = np.linalg.norm(rot_vel)
        gripper_vel_norm = np.linalg.norm(gripper_vel)
        
        if lin_vel_norm > self.max_lin_vel:
            lin_vel = lin_vel * self.max_lin_vel / lin_vel_norm
        if rot_vel_norm > self.max_rot_vel:
            rot_vel = rot_vel * self.max_rot_vel / rot_vel_norm
        if gripper_vel_norm > self.max_gripper_vel:
            gripper_vel = gripper_vel * self.max_gripper_vel / gripper_vel_norm
        
        return lin_vel, rot_vel, gripper_vel
    
    def _calculate_action(self):
        """Calculate robot action from VR controller state - preserved exactly"""
        if self.update_sensor:
            self._process_reading()
            self.update_sensor = False
        
        if self.vr_state is None or self.robot_pos is None:
            return np.zeros(7), {}
        
        # Reset Origin On Release
        if self.reset_origin:
            self.robot_origin = {"pos": self.robot_pos, "quat": self.robot_quat}
            self.vr_origin = {"pos": self.vr_state["pos"], "quat": self.vr_state["quat"]}
            self.reset_origin = False
            print("📍 Origin calibrated")
        
        # Calculate Positional Action - DROID exact
        robot_pos_offset = self.robot_pos - self.robot_origin["pos"]
        target_pos_offset = self.vr_state["pos"] - self.vr_origin["pos"]
        pos_action = target_pos_offset - robot_pos_offset
        
        # Calculate Rotation Action for MoveIt
        vr_relative_rot = R.from_quat(self.vr_origin["quat"]).inv() * R.from_quat(self.vr_state["quat"])
        target_rot = R.from_quat(self.robot_origin["quat"]) * vr_relative_rot
        target_quat = target_rot.as_quat()
        
        robot_quat_offset = quat_diff(self.robot_quat, self.robot_origin["quat"])
        target_quat_offset = quat_diff(self.vr_state["quat"], self.vr_origin["quat"])
        quat_action = quat_diff(target_quat_offset, robot_quat_offset)
        euler_action = quat_to_euler(quat_action)
        
        # Calculate Gripper Action
        gripper_action = (self.vr_state["gripper"] * 1.5) - self.robot_gripper
        
        # Calculate Desired Pose
        target_pos = pos_action + self.robot_pos
        target_euler = add_angles(euler_action, self.robot_euler)
        target_cartesian = np.concatenate([target_pos, target_euler])
        target_gripper = self.vr_state["gripper"]
        
        # Scale Appropriately
        pos_action *= self.pos_action_gain
        euler_action *= self.rot_action_gain
        gripper_action *= self.gripper_action_gain
        
        # Apply velocity limits
        lin_vel, rot_vel, gripper_vel = self._limit_velocity(pos_action, euler_action, gripper_action)
        
        # Prepare Return Values
        info_dict = {
            "target_cartesian_position": target_cartesian, 
            "target_gripper_position": target_gripper,
            "target_quaternion": target_quat  # For MoveIt
        }
        action = np.concatenate([lin_vel, rot_vel, [gripper_vel]])
        action = action.clip(-1, 1)
        
        return action, info_dict
    
    def get_info(self):
        """Get controller state information - preserved exactly"""
        info = {
            "success": self._state["buttons"]["A"] if self.controller_id == 'r' else self._state["buttons"]["X"],
            "failure": self._state["buttons"]["B"] if self.controller_id == 'r' else self._state["buttons"]["Y"],
            "movement_enabled": self._state["movement_enabled"],
            "controller_on": self._state["controller_on"],
        }
        
        if self._state["poses"] and self._state["buttons"]:
            info["poses"] = self._state["poses"]
            info["buttons"] = self._state["buttons"]
        
        return info
        
    def velocity_to_position_target(self, velocity_action, current_pos, current_quat, action_info=None):
        """Convert velocity action to position target - preserved for MoveIt"""
        lin_vel = velocity_action[:3]
        rot_vel = velocity_action[3:6]
        gripper_vel = velocity_action[6]
        
        lin_vel_norm = np.linalg.norm(lin_vel)
        rot_vel_norm = np.linalg.norm(rot_vel)
        
        if lin_vel_norm > 1:
            lin_vel = lin_vel / lin_vel_norm
        if rot_vel_norm > 1:
            rot_vel = rot_vel / rot_vel_norm
            
        pos_delta = lin_vel * self.max_lin_delta
        rot_delta = rot_vel * self.max_rot_delta
        
        target_pos = current_pos + pos_delta
        
        # Use pre-calculated target quaternion for MoveIt
        if action_info and "target_quaternion" in action_info:
            target_quat = action_info["target_quaternion"]
        else:
            rot_delta_quat = euler_to_quat(rot_delta)
            current_rot = R.from_quat(current_quat)
            delta_rot = R.from_quat(rot_delta_quat)
            target_rot = delta_rot * current_rot
            target_quat = target_rot.as_quat()
        
        target_gripper = np.clip(self.robot_gripper + gripper_vel * self.control_interval, 0.0, 1.0)
        
        return target_pos, target_quat, target_gripper

    # =====================================
    # MIGRATED ROBOT COMMUNICATION WORKER
    # =====================================
    
    def _robot_comm_worker(self):
        """Handles robot communication via MoveIt services/actions"""
        self.get_logger().info("🔌 Robot communication thread started (MoveIt - 15Hz Optimized)")
        
        comm_count = 0
        total_comm_time = 0
        stats_last_printed = time.time()
        
        while self.running:
            try:
                # Get command from queue with timeout
                command = self._robot_command_queue.get(timeout=0.01)
                
                if command is None:  # Poison pill
                    break
                
                # Use the optimized rate limiting for 15Hz
                if not self.should_send_robot_command():
                    continue
                
                # Process MoveIt command
                comm_start = time.time()
                success = self.execute_moveit_command(command)
                comm_time = time.time() - comm_start
                
                comm_count += 1
                total_comm_time += comm_time
                self._last_command_time = time.time()
                
                # Get current robot state after command
                if success:
                    pos, quat = self.get_current_end_effector_pose()
                    joint_positions = self.get_current_joint_positions()
                    
                    if pos is not None and quat is not None:
                        # Create response in same format as Deoxys
                        response = type('RobotState', (), {
                            'pos': pos,
                            'quat': quat,
                            'gripper': command.gripper,
                            'joint_positions': np.array(joint_positions) if joint_positions else None
                        })()
                        
                        try:
                            self._robot_response_queue.put_nowait(response)
                        except queue.Full:
                            try:
                                self._robot_response_queue.get_nowait()
                                self._robot_response_queue.put_nowait(response)
                            except:
                                pass
                
                # Log communication stats periodically
                if time.time() - stats_last_printed > 10.0 and comm_count > 0:
                    avg_comm_time = total_comm_time / comm_count
                    actual_rate = comm_count / 10.0
                    self.get_logger().info(f"📡 Avg MoveIt comm: {avg_comm_time*1000:.1f}ms ({comm_count} commands)")
                    self.get_logger().info(f"📊 Actual robot rate: {actual_rate:.1f} commands/sec (target: 15Hz)")
                    if self.debug_comm_stats:
                        self.print_moveit_stats()
                    stats_last_printed = time.time()
                    comm_count = 0  # Reset counter
                    total_comm_time = 0
                
            except queue.Empty:
                continue
            except Exception as e:
                if self.running:
                    self.get_logger().error(f"❌ Error in MoveIt communication: {e}")
                    import traceback
                    traceback.print_exc()
                    time.sleep(0.1)
        
        self.get_logger().info("🔌 Robot communication thread stopped (MoveIt)")

    def _robot_control_worker(self):
        """Asynchronous robot control thread - preserved with ROS 2 spinning"""
        self.get_logger().info("🤖 Robot control thread started")
        self.get_logger().info(f"   Target control frequency: {self.control_hz}Hz")
        
        last_control_time = time.time()
        control_count = 0
        freq_check_time = time.time()
        
        while self.running:
            try:
                current_time = time.time()
                
                # Skip if control is paused
                if self._control_paused:
                    time.sleep(0.01)
                    continue
                
                # Control at specified frequency
                if current_time - last_control_time >= self.control_interval:
                    # Get latest VR state
                    with self._vr_state_lock:
                        vr_state = self._latest_vr_state.copy() if self._latest_vr_state else None
                    
                    # Get latest robot state
                    with self._robot_state_lock:
                        robot_state = self._latest_robot_state.copy() if self._latest_robot_state else None
                    
                    if vr_state and robot_state:
                        self._process_control_cycle(vr_state, robot_state, current_time)
                        control_count += 1
                    
                    last_control_time = current_time
                    
                    # Print actual frequency every second
                    if current_time - freq_check_time >= 1.0 and control_count > 0:
                        actual_freq = control_count / (current_time - freq_check_time)
                        if self.recording_active and self.debug_comm_stats:
                            self.get_logger().info(f"⚡ Control frequency: {actual_freq:.1f}Hz (target: {self.control_hz}Hz)")
                        control_count = 0
                        freq_check_time = current_time
                
                # Small sleep to prevent CPU spinning
                time.sleep(0.001)
                
            except Exception as e:
                if self.running:
                    self.get_logger().error(f"❌ Error in robot control: {e}")
                    import traceback
                    traceback.print_exc()
                    time.sleep(0.1)
        
        self.get_logger().info("🤖 Robot control thread stopped")

    def _data_recording_worker(self):
        """Records data at target frequency independent of robot control - preserved exactly"""
        self.get_logger().info("📊 Data recording thread started")
        
        last_record_time = time.time()
        record_count = 0
        freq_check_time = time.time()
        
        while self.running:
            try:
                current_time = time.time()
                
                if current_time - last_record_time >= self.recording_interval:
                    if self.recording_active and self.data_recorder:
                        with self._vr_state_lock:
                            vr_state = self._latest_vr_state.copy() if self._latest_vr_state else None
                        
                        with self._robot_state_lock:
                            robot_state = self._latest_robot_state.copy() if self._latest_robot_state else None
                        
                        if vr_state and robot_state:
                            info = {
                                "success": vr_state.buttons.get("A", False) if self.controller_id == 'r' else vr_state.buttons.get("X", False),
                                "failure": vr_state.buttons.get("B", False) if self.controller_id == 'r' else vr_state.buttons.get("Y", False),
                                "movement_enabled": vr_state.movement_enabled,
                                "controller_on": vr_state.controller_on,
                                "poses": vr_state.poses,
                                "buttons": vr_state.buttons
                            }
                            
                            action = np.zeros(7)
                            if vr_state.movement_enabled and hasattr(self, 'vr_state') and self.vr_state:
                                if hasattr(self, '_last_action'):
                                    action = self._last_action
                            
                            timestep_data = TimestepData(
                                timestamp=current_time,
                                vr_state=vr_state,
                                robot_state=robot_state,
                                action=action.copy(),
                                info=copy.deepcopy(info)
                            )
                            
                            try:
                                self.mcap_queue.put_nowait(timestep_data)
                                record_count += 1
                            except queue.Full:
                                self.get_logger().warn("⚠️  MCAP queue full, dropping frame")
                    
                    last_record_time = current_time
                    
                    if current_time - freq_check_time >= 1.0 and record_count > 0:
                        if self.recording_active and self.debug_comm_stats:
                            actual_freq = record_count / (current_time - freq_check_time)
                            self.get_logger().info(f"📊 Recording frequency: {actual_freq:.1f}Hz")
                        record_count = 0
                        freq_check_time = current_time
                
                time.sleep(0.001)
                
            except Exception as e:
                if self.running:
                    self.get_logger().error(f"❌ Error in data recording: {e}")
                    time.sleep(0.1)
        
        self.get_logger().info("📊 Data recording thread stopped")

    def _mcap_writer_worker(self):
        """Asynchronous MCAP writer thread - preserved exactly"""
        self.get_logger().info("📹 MCAP writer thread started")
        
        while self.running or not self.mcap_queue.empty():
            try:
                timestep_data = self.mcap_queue.get(timeout=0.1)
                
                if timestep_data is None:
                    break
                
                timestep = {
                    "observation": {
                        "timestamp": {
                            "robot_state": {
                                "read_start": int(timestep_data.timestamp * 1e9),
                                "read_end": int(timestep_data.timestamp * 1e9)
                            }
                        },
                        "robot_state": {
                            "joint_positions": timestep_data.robot_state.joint_positions.tolist() if timestep_data.robot_state.joint_positions is not None else [],
                            "joint_velocities": [],
                            "joint_efforts": [],
                            "cartesian_position": np.concatenate([
                                timestep_data.robot_state.pos,
                                timestep_data.robot_state.euler
                            ]).tolist(),
                            "cartesian_velocity": [],
                            "gripper_position": timestep_data.robot_state.gripper,
                            "gripper_velocity": 0.0
                        },
                        "controller_info": timestep_data.info
                    },
                    "action": timestep_data.action.tolist() if hasattr(timestep_data.action, 'tolist') else timestep_data.action
                }
                
                self.data_recorder.write_timestep(timestep, timestep_data.timestamp)
                
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"❌ Error in MCAP writer: {e}")
                import traceback
                traceback.print_exc()
        
        self.get_logger().info("📹 MCAP writer thread stopped")

    def _process_control_cycle(self, vr_state: VRState, robot_state: RobotState, current_time: float):
        """Process a single control cycle - adapted for MoveIt"""
        # Restore state from thread-safe structures
        self._state["poses"] = vr_state.poses
        self._state["buttons"] = vr_state.buttons
        self._state["movement_enabled"] = vr_state.movement_enabled
        self._state["controller_on"] = vr_state.controller_on
        
        # Update robot state
        self.robot_pos = robot_state.pos
        self.robot_quat = robot_state.quat
        self.robot_euler = robot_state.euler
        self.robot_gripper = robot_state.gripper
        self.robot_joint_positions = robot_state.joint_positions
        
        # Get controller info
        info = self.get_info()
        
        # Handle recording controls
        if self.enable_recording and self.data_recorder:
            current_a_button = info["success"]
            if current_a_button and not self.prev_a_button:
                if self.recording_active:
                    print("\n🛑 A button pressed - Stopping current recording...")
                    self.data_recorder.reset_recording()
                    self.recording_active = False
                    print("📹 Recording stopped (not saved)")
                else:
                    print("\n▶️  A button pressed - Starting recording...")
                    self.data_recorder.start_recording()
                    self.recording_active = True
                    print("📹 Recording started")
            self.prev_a_button = current_a_button
            
            if info["failure"] and self.recording_active:
                print("\n✅ B button pressed - Marking recording as successful...")
                saved_filepath = self.data_recorder.stop_recording(success=True)
                self.recording_active = False
                print("📹 Recording saved successfully")
                
                if self.verify_data and saved_filepath:
                    print("\n🔍 Verifying recorded data...")
                    try:
                        verifier = MCAPVerifier(saved_filepath)
                        results = verifier.verify(verbose=True)
                        
                        if not results["summary"]["is_valid"]:
                            print("\n⚠️  WARNING: Data verification found issues!")
                    except Exception as e:
                        print(f"\n❌ Error during verification: {e}")
        else:
            if info["success"]:
                print("\n✅ Success button pressed!")
                if not self.debug:
                    self.stop_server()
                    return
            
            if info["failure"]:
                print("\n❌ Failure button pressed!")
                if not self.debug:
                    self.stop_server()
                    return
        
        # Default action
        action = np.zeros(7)
        action_info = {}
        
        # Calculate action if movement is enabled
        if info["movement_enabled"] and self._state["poses"]:
            # Debug when movement is first enabled
            if self.debug and not hasattr(self, '_movement_was_enabled'):
                print(f"\n🎮 VR Movement ENABLED!")
                print(f"   Controller ID: {self.controller_id}")
                print(f"   Available poses: {list(self._state['poses'].keys())}")
                if self.controller_id in self._state["poses"]:
                    raw_pose = self._state["poses"][self.controller_id]
                    raw_pos = raw_pose[:3, 3]
                    print(f"   Raw controller position: [{raw_pos[0]:.3f}, {raw_pos[1]:.3f}, {raw_pos[2]:.3f}]")
                else:
                    print(f"   ⚠️  Controller {self.controller_id} not found in poses!")
                self._movement_was_enabled = True
            
            action, action_info = self._calculate_action()
            self._last_action = action.copy()
            
            # Debug VR action calculation
            if self.debug and hasattr(self, '_debug_counter'):
                self._debug_counter += 1
                if self._debug_counter % 30 == 0:  # Print every 30 cycles (every 0.5s at 60Hz)
                    print(f"\n🎮 VR Action Debug:")
                    print(f"   VR Controller Position: {self.vr_state['pos'] if self.vr_state else 'None'}")
                    print(f"   Robot Current Position: [{self.robot_pos[0]:.3f}, {self.robot_pos[1]:.3f}, {self.robot_pos[2]:.3f}]")
                    print(f"   Action (lin/rot/gripper): [{action[0]:.3f}, {action[1]:.3f}, {action[2]:.3f}] / [{action[3]:.3f}, {action[4]:.3f}, {action[5]:.3f}] / {action[6]:.3f}")
                    if 'target_cartesian_position' in action_info:
                        target_cart = action_info['target_cartesian_position']
                        print(f"   Target Position: [{target_cart[0]:.3f}, {target_cart[1]:.3f}, {target_cart[2]:.3f}]")
                    
                    # Debug calibration status
                    print(f"   🔧 Calibration Status:")
                    print(f"      Forward calibrated: {not self.reset_orientation}")
                    print(f"      Origin calibrated: {self.robot_origin is not None}")
                    if self.robot_origin:
                        robot_orig = self.robot_origin['pos']
                        print(f"      Robot origin: [{robot_orig[0]:.3f}, {robot_orig[1]:.3f}, {robot_orig[2]:.3f}]")
                    if self.vr_origin:
                        vr_orig = self.vr_origin['pos'] 
                        print(f"      VR origin: [{vr_orig[0]:.3f}, {vr_orig[1]:.3f}, {vr_orig[2]:.3f}]")
                        
                    # Debug VR controller raw data
                    if self.controller_id in self._state.get("poses", {}):
                        raw_pose_matrix = self._state["poses"][self.controller_id]
                        raw_pos = raw_pose_matrix[:3, 3]
                        print(f"      Raw VR position: [{raw_pos[0]:.3f}, {raw_pos[1]:.3f}, {raw_pos[2]:.3f}]")
            elif not hasattr(self, '_debug_counter'):
                self._debug_counter = 0
            
            target_pos, target_quat, target_gripper = self.velocity_to_position_target(
                action, self.robot_pos, self.robot_quat, action_info
            )
            
            # Apply workspace bounds
            target_pos = np.clip(target_pos, ROBOT_WORKSPACE_MIN, ROBOT_WORKSPACE_MAX)
            
            # Apply ultra-smooth pose filtering for 60Hz operation
            if self.pose_smoothing_enabled:
                target_pos, target_quat, target_gripper = self.smooth_pose_transition(
                    target_pos, target_quat, target_gripper
                )
            
            # Handle gripper control
            trigger_value = self._state["buttons"].get("rightTrig" if self.right_controller else "leftTrig", [0.0])[0]
            gripper_state = GRIPPER_CLOSE if trigger_value > 0.1 else GRIPPER_OPEN
            
            # Debug movement commands with velocity info
            if self.debug and hasattr(self, '_debug_counter') and self._debug_counter % 30 == 0:
                movement_delta = np.linalg.norm(target_pos - self.robot_pos)
                print(f"   Movement Delta: {movement_delta*1000:.1f}mm")
                print(f"   Smoothed Target: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]")
                print(f"   Gripper: {gripper_state} (trigger: {trigger_value:.2f})")
                
                # Show velocity limiting info if we have previous joint positions
                if hasattr(self, '_last_joint_positions') and self._last_joint_positions is not None:
                    # Simulate the velocity calculation for debugging
                    joint_positions = self.get_current_joint_positions()
                    if joint_positions:
                        test_ik = self.compute_ik_for_pose(target_pos, target_quat)
                        if test_ik:
                            deltas = np.array(test_ik) - np.array(self._last_joint_positions)
                            test_velocities = deltas / 0.3 * 0.25
                            max_vel = max(abs(v) for v in test_velocities)
                            print(f"   Max joint velocity: {max_vel:.3f} rad/s (limit: 0.4 rad/s)")
                            if max_vel > 0.4:
                                print(f"   ⚠️  Velocity limiting active!")
            
            # Send action to robot (MoveIt style)
            if not self.debug:
                # Create MoveIt-compatible action
                robot_action = type('MoveitAction', (), {
                    'pos': target_pos.flatten().astype(np.float32),
                    'quat': target_quat.flatten().astype(np.float32),
                    'gripper': gripper_state,
                    'reset': False,
                    'timestamp': time.time(),
                })()
                
                # Queue command for async sending
                try:
                    self._robot_command_queue.put_nowait(robot_action)
                except queue.Full:
                    try:
                        self._robot_command_queue.get_nowait()
                        self._robot_command_queue.put_nowait(robot_action)
                    except:
                        pass
                
                # Try to get latest response
                try:
                    franka_state = self._robot_response_queue.get_nowait()
                    
                    new_robot_state = RobotState(
                        timestamp=current_time,
                        pos=franka_state.pos,
                        quat=franka_state.quat,
                        euler=quat_to_euler(franka_state.quat),
                        gripper=1.0 if franka_state.gripper == GRIPPER_CLOSE else 0.0,
                        joint_positions=getattr(franka_state, 'joint_positions', None)
                    )
                    
                    with self._robot_state_lock:
                        self._latest_robot_state = new_robot_state
                    
                    self.robot_pos = new_robot_state.pos
                    self.robot_quat = new_robot_state.quat
                    self.robot_euler = new_robot_state.euler
                    self.robot_gripper = new_robot_state.gripper
                    self.robot_joint_positions = new_robot_state.joint_positions
                    
                except queue.Empty:
                    # Use predicted state
                    new_robot_state = RobotState(
                        timestamp=current_time,
                        pos=target_pos,
                        quat=target_quat,
                        euler=quat_to_euler(target_quat),
                        gripper=1.0 if gripper_state == GRIPPER_CLOSE else 0.0,
                        joint_positions=self.robot_joint_positions
                    )
                    
                    with self._robot_state_lock:
                        self._latest_robot_state = new_robot_state
                    
                    self.robot_pos = target_pos
                    self.robot_quat = target_quat
                    self.robot_euler = quat_to_euler(target_quat)
                    self.robot_gripper = new_robot_state.gripper
            else:
                # Debug mode simulation
                new_robot_state = RobotState(
                    timestamp=current_time,
                    pos=target_pos,
                    quat=target_quat,
                    euler=quat_to_euler(target_quat),
                    gripper=1.0 if gripper_state == GRIPPER_CLOSE else 0.0,
                    joint_positions=self.robot_joint_positions
                )
                
                with self._robot_state_lock:
                    self._latest_robot_state = new_robot_state
                
                self.robot_pos = target_pos
                self.robot_quat = target_quat
                self.robot_euler = quat_to_euler(target_quat)
                self.robot_gripper = new_robot_state.gripper
        else:
            new_robot_state = robot_state
            self._last_action = np.zeros(7)
            # Reset debug flag when movement is disabled
            if hasattr(self, '_movement_was_enabled'):
                if self.debug:
                    print("\n🛑 VR Movement DISABLED")
                delattr(self, '_movement_was_enabled')

    def control_loop(self):
        """Main control loop with ROS 2 integration"""
        message_count = 0
        last_debug_time = time.time()
        
        # Initialize robot on first frame
        if self.is_first_frame:
            init_pos, init_quat, init_joint_positions = self.reset_robot()
            self.robot_pos = init_pos
            self.robot_quat = init_quat
            self.robot_euler = quat_to_euler(init_quat)
            self.robot_gripper = 0.0
            self.robot_joint_positions = init_joint_positions
            self.is_first_frame = False
            
            with self._robot_state_lock:
                self._latest_robot_state = RobotState(
                    timestamp=time.time(),
                    pos=init_pos,
                    quat=init_quat,
                    euler=self.robot_euler,
                    gripper=self.robot_gripper,
                    joint_positions=init_joint_positions
                )
        
        # Start camera manager
        if self.camera_manager:
            try:
                self.camera_manager.start()
                print("📷 Camera manager started")
            except Exception as e:
                print(f"⚠️  Failed to start camera manager: {e}")
                self.camera_manager = None
        
        # Start worker threads
        if self.enable_recording and self.data_recorder:
            self._mcap_writer_thread = threading.Thread(target=self._mcap_writer_worker)
            self._mcap_writer_thread.daemon = True
            self._mcap_writer_thread.start()
            self._threads.append(self._mcap_writer_thread)
            
            self._data_recording_thread = threading.Thread(target=self._data_recording_worker)
            self._data_recording_thread.daemon = True
            self._data_recording_thread.start()
            self._threads.append(self._data_recording_thread)
        
        # Start robot communication thread (only if not in debug mode)
        if not self.debug:
            self._robot_comm_thread = threading.Thread(target=self._robot_comm_worker)
            self._robot_comm_thread.daemon = True
            self._robot_comm_thread.start()
            self._threads.append(self._robot_comm_thread)
        
        self._robot_control_thread = threading.Thread(target=self._robot_control_worker)
        self._robot_control_thread.daemon = True
        self._robot_control_thread.start()
        self._threads.append(self._robot_control_thread)
        
        # Main loop with ROS 2 spinning
        while self.running:
            try:
                current_time = time.time()
                
                # Add ROS 2 spinning for service calls
                rclpy.spin_once(self, timeout_sec=0.001)
                
                # Handle robot reset after calibration
                if hasattr(self, '_reset_robot_after_calibration') and self._reset_robot_after_calibration:
                    self._reset_robot_after_calibration = False
                    print("🤖 Executing robot reset after calibration...")
                    
                    self._control_paused = True
                    time.sleep(0.1)
                    
                    reset_pos, reset_quat, reset_joint_positions = self.reset_robot()
                    
                    with self._robot_state_lock:
                        self._latest_robot_state = RobotState(
                            timestamp=current_time,
                            pos=reset_pos,
                            quat=reset_quat,
                            euler=quat_to_euler(reset_quat),
                            gripper=0.0,
                            joint_positions=reset_joint_positions
                        )
                    
                    self.robot_pos = reset_pos
                    self.robot_quat = reset_quat
                    self.robot_euler = quat_to_euler(reset_quat)
                    self.robot_gripper = 0.0
                    self.robot_joint_positions = reset_joint_positions
                    
                    self.reset_origin = True
                    self._control_paused = False
                    
                    print("✅ Robot is now at home position, ready for teleoperation")
                
                # Debug output
                if self.debug and current_time - last_debug_time > 5.0:
                    with self._vr_state_lock:
                        vr_state = self._latest_vr_state
                    with self._robot_state_lock:
                        robot_state = self._latest_robot_state
                    
                    if vr_state and robot_state:
                        print(f"\n📊 MoveIt Status [{message_count:04d}]:")
                        print(f"   VR State: {(current_time - vr_state.timestamp):.3f}s ago")
                        print(f"   Robot State: {(current_time - robot_state.timestamp):.3f}s ago")
                        print(f"   MCAP Queue: {self.mcap_queue.qsize()} items")
                        print(f"   Recording: {'ACTIVE' if self.recording_active else 'INACTIVE'}")
                        if self.debug_comm_stats:
                            self.print_moveit_stats()
                        
                    last_debug_time = current_time
                    message_count += 1
                
                time.sleep(0.01)
                
            except Exception as e:
                if self.running:
                    self.get_logger().error(f"❌ Error in main loop: {e}")
                    import traceback
                    traceback.print_exc()
                    time.sleep(1)
    
    def start(self):
        """Start the server"""
        try:
            self.control_loop()
        except KeyboardInterrupt:
            print("\n🛑 Keyboard interrupt received")
            self.stop_server()
    
    def stop_server(self):
        """Gracefully stop the server"""
        if not self.running:
            return
            
        print("🛑 Stopping Oculus VR Server - MoveIt Edition...")
        self.running = False
        
        # Stop any active recording
        if self.recording_active and self.data_recorder:
            print("📹 Stopping active recording...")
            self.data_recorder.stop_recording(success=False)
            self.recording_active = False
        
        # Send poison pill to workers
        if self._mcap_writer_thread and self._mcap_writer_thread.is_alive():
            self.mcap_queue.put(None)
        
        if self._robot_comm_thread and self._robot_comm_thread.is_alive():
            self._robot_command_queue.put(None)
        
        # Stop threads
        for thread in self._threads:
            if thread.is_alive():
                thread.join(timeout=1.0)
        
        # Stop Oculus Reader
        if hasattr(self, 'oculus_reader'):
            try:
                self.oculus_reader.stop()
                print("✅ Oculus Reader stopped")
            except Exception as e:
                print(f"⚠️  Error stopping Oculus Reader: {e}")
        
        # Stop other components
        if self.camera_manager:
            try:
                self.camera_manager.stop()
                print("✅ Camera manager stopped")
            except Exception as e:
                print(f"⚠️  Error stopping camera manager: {e}")
        
        if self.sim_server:
            self.sim_server.stop()
            print("✅ Simulation server stopped")
        
        # Print final stats
        if self.debug_comm_stats:
            print("\n📊 Final MoveIt Statistics:")
            self.print_moveit_stats()
        
        print("✅ Server stopped gracefully")
        sys.exit(0)

    def smooth_pose_transition(self, target_pos, target_quat, target_gripper):
        """Apply exponential smoothing to robot poses for ultra-smooth motion"""
        current_time = time.time()
        
        # Initialize smoothed values on first call
        if self._smoothed_target_pos is None:
            self._smoothed_target_pos = target_pos.copy()
            self._smoothed_target_quat = target_quat.copy() 
            self._smoothed_target_gripper = target_gripper
            return target_pos, target_quat, target_gripper
        
        # Calculate motion speed for adaptive smoothing
        pos_delta = np.linalg.norm(target_pos - self._smoothed_target_pos)
        
        # Adaptive smoothing - use more smoothing for fast motions
        if self.adaptive_smoothing:
            # Increase smoothing for faster motions to prevent jerks
            speed_factor = min(pos_delta * 100, 1.0)  # Scale position delta
            adaptive_alpha = self.pose_smoothing_alpha * (1.0 - speed_factor * 0.5)
            adaptive_alpha = max(adaptive_alpha, 0.05)  # Minimum smoothing
        else:
            adaptive_alpha = self.pose_smoothing_alpha
        
        # Exponential smoothing for position
        self._smoothed_target_pos = (adaptive_alpha * target_pos + 
                                   (1.0 - adaptive_alpha) * self._smoothed_target_pos)
        
        # Spherical linear interpolation (SLERP) for quaternions - much smoother
        from scipy.spatial.transform import Rotation as R
        current_rot = R.from_quat(self._smoothed_target_quat)
        target_rot = R.from_quat(target_quat)
        
        # SLERP between current and target orientation
        smoothed_rot = current_rot.inv() * target_rot
        smoothed_rotvec = smoothed_rot.as_rotvec()
        smoothed_rotvec *= adaptive_alpha  # Scale rotation step
        final_rot = current_rot * R.from_rotvec(smoothed_rotvec)
        self._smoothed_target_quat = final_rot.as_quat()
        
        # Smooth gripper with velocity limiting
        gripper_delta = target_gripper - self._smoothed_target_gripper
        max_gripper_delta = 0.02  # Limit gripper speed
        gripper_delta = np.clip(gripper_delta, -max_gripper_delta, max_gripper_delta)
        self._smoothed_target_gripper = self._smoothed_target_gripper + gripper_delta
        
        # Add to pose history for trend analysis
        self._pose_history.append({
            'time': current_time,
            'pos': self._smoothed_target_pos.copy(),
            'quat': self._smoothed_target_quat.copy(),
            'gripper': self._smoothed_target_gripper
        })
        
        return self._smoothed_target_pos, self._smoothed_target_quat, self._smoothed_target_gripper

    def should_send_robot_command(self):
        """Determine if we should send a new robot command based on rate limiting and motion"""
        current_time = time.time()
        
        # Always respect minimum command interval (15Hz = 67ms)
        if current_time - self._last_command_time < self.min_command_interval:
            return False
        
        # If we have pose history, check if motion is significant enough
        if len(self._pose_history) >= 2:
            recent_pose = self._pose_history[-1]
            older_pose = self._pose_history[-2]
            
            # Calculate motion since last command
            pos_delta = np.linalg.norm(recent_pose['pos'] - older_pose['pos'])
            
            # Optimized motion detection for 15Hz - good balance of responsiveness
            # Allow commands for meaningful movement
            if pos_delta < 0.0008 and current_time - self._last_command_time < 0.15:  # 150ms max delay
                return False
        
        return True


def main():
    """Main function with ROS 2 initialization"""
    # Initialize ROS 2
    rclpy.init()
    
    try:
        parser = argparse.ArgumentParser(
            description='Oculus VR Server - MoveIt Edition',
            epilog='''
This server implements DROID-exact VRPolicy control with MoveIt integration.

Migration from Deoxys:
  - MoveIt IK service replaces Deoxys internal IK
  - ROS 2 trajectory actions replace Deoxys socket commands
  - Enhanced collision avoidance and safety features
  - Preserved async architecture and VR processing

Features:
  - DROID-exact control parameters and transformations
  - Async architecture with threaded workers  
  - MCAP data recording with camera integration
  - Intuitive forward direction calibration
  - Origin recalibration on grip press/release
  - MoveIt collision avoidance and planning

Controls:
  - Hold grip button: Enable teleoperation
  - Press A button: Start/stop recording (if enabled)
  - Press B button: Mark recording successful (if enabled)
  - Hold joystick + move: Calibrate forward direction

Hot Reload:
  - Run with --hot-reload flag to enable automatic restart
            ''',
            formatter_class=argparse.RawDescriptionHelpFormatter
        )
        parser.add_argument('--debug', action='store_true', 
                            help='Enable debug mode (no robot control)')
        parser.add_argument('--left-controller', action='store_true',
                            help='Use left controller instead of right (default: right)')
        parser.add_argument('--ip', type=str, default=None,
                            help='IP address of Quest device (default: USB connection)')
        parser.add_argument('--simulation', action='store_true',
                            help='Use simulated FR3 robot instead of real hardware')
        parser.add_argument('--coord-transform', nargs='+', type=float,
                            help='Custom coordinate transformation vector (format: x y z w)')
        parser.add_argument('--rotation-mode', type=str, default='labelbox',
                            choices=['labelbox'],
                            help='Rotation mapping mode (default: labelbox)')
        parser.add_argument('--hot-reload', action='store_true',
                            help='Enable hot reload mode (auto-restart on file changes)')
        parser.add_argument('--performance', action='store_true',
                            help='Enable performance mode for tighter tracking (2x frequency, higher gains)')
        parser.add_argument('--no-recording', action='store_true',
                            help='Disable MCAP data recording functionality')
        parser.add_argument('--verify-data', action='store_true',
                            help='Verify MCAP data integrity after successful recording')
        parser.add_argument('--camera-config', type=str, default=None,
                            help='Path to camera configuration YAML file (e.g., configs/cameras.yaml)')
        parser.add_argument('--enable-cameras', action='store_true',
                            help='Enable camera recording with MCAP data')
        parser.add_argument('--auto-discover-cameras', action='store_true',
                            help='Automatically discover and use all connected cameras')
        
        args = parser.parse_args()
        
        # If hot reload is requested, launch the hot reload wrapper
        if args.hot_reload:
            import subprocess
            
            new_args = [arg for arg in sys.argv[1:] if arg != '--hot-reload']
            
            print("🔥 Launching in hot reload mode...")
            
            if not os.path.exists('oculus_vr_server_hotreload.py'):
                print("❌ Hot reload script not found!")
                print("   Create oculus_vr_server_hotreload.py or use regular mode")
                sys.exit(1)
            
            try:
                subprocess.run([sys.executable, 'oculus_vr_server_hotreload.py'] + new_args)
            except KeyboardInterrupt:
                print("\n✅ Hot reload stopped")
            finally:
                rclpy.shutdown()
            sys.exit(0)
        
        # Handle auto-discovery of cameras
        if args.auto_discover_cameras:
            print("🔍 Auto-discovering cameras...")
            try:
                from frankateach.camera_utils import discover_all_cameras, generate_camera_config
                
                cameras = discover_all_cameras()
                if cameras:
                    temp_config = "/tmp/cameras_autodiscovered.yaml"
                    generate_camera_config(cameras, temp_config)
                    args.camera_config = temp_config
                    args.enable_cameras = True
                    print(f"✅ Using auto-discovered cameras from: {temp_config}")
                else:
                    print("⚠️  No cameras found during auto-discovery")
            except Exception as e:
                print(f"❌ Camera auto-discovery failed: {e}")
        
        # Load camera configuration
        camera_configs = None
        if args.camera_config:
            try:
                import yaml
                with open(args.camera_config, 'r') as f:
                    camera_configs = yaml.safe_load(f)
                print(f"📷 Loaded camera configuration from {args.camera_config}")
            except Exception as e:
                print(f"⚠️  Failed to load camera config: {e}")
                print("   Continuing without camera configuration")
        
        # Create server (now ROS 2 node)
        coord_transform = args.coord_transform
        server = OculusVRServer(
            debug=args.debug,
            right_controller=not args.left_controller,
            ip_address=args.ip,
            simulation=args.simulation,
            coord_transform=coord_transform,
            rotation_mode=args.rotation_mode,
            performance_mode=args.performance,
            enable_recording=not args.no_recording,
            camera_configs=camera_configs,
            verify_data=args.verify_data,
            camera_config_path=args.camera_config,
            enable_cameras=args.enable_cameras
        )
        
        server.start()
        
    except KeyboardInterrupt:
        print("\n🛑 Keyboard interrupt received")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup ROS 2
        if 'server' in locals():
            server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main() 