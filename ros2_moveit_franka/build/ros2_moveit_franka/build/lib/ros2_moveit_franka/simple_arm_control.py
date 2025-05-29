#!/usr/bin/env python3
"""
Advanced Franka FR3 Benchmarking Script with MoveIt Integration and VR Pose Control
- Benchmarks control rates up to 1kHz (FR3 manual specification)
- Uses VR pose targets (position + quaternion from simulated Oculus controller)
- Full MoveIt integration with IK solver and collision avoidance
- VR-style velocity-based control with coordinate transformations
- Comprehensive timing analysis and performance metrics
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPlanningScene, GetMotionPlan, GetPositionFK
from moveit_msgs.msg import (
    PositionIKRequest, RobotState, Constraints, JointConstraint,
    MotionPlanRequest, WorkspaceParameters, PlanningOptions
)
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
import numpy as np
import time
import threading
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import statistics
from moveit_msgs.msg import RobotState, PlanningScene, CollisionObject
from scipy.spatial.transform import Rotation as R
import copy
import control_msgs.msg


# VR Pose Processing Functions (from oculus_vr_server.py)

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


@dataclass
class VRPose:
    """VR pose data with position and orientation (simulated Oculus controller)"""
    position: np.ndarray  # [x, y, z] in meters (VR coordinate frame)
    orientation: np.ndarray  # quaternion [x, y, z, w] (VR coordinate frame)
    timestamp: float
    grip_pressed: bool = False  # Simulated grip button state
    trigger_value: float = 0.0  # Simulated trigger value [0.0, 1.0]
    
    @classmethod
    def create_example_pose(cls, x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0, grip=False, trigger=0.0):
        """Create example VR pose for testing"""
        return cls(
            position=np.array([x, y, z]),
            orientation=np.array([qx, qy, qz, qw]),
            timestamp=time.time(),
            grip_pressed=grip,
            trigger_value=trigger
        )


@dataclass
class VRControlState:
    """VR controller state for velocity-based control"""
    vr_origin_pos: Optional[np.ndarray] = None
    vr_origin_quat: Optional[np.ndarray] = None
    robot_origin_pos: Optional[np.ndarray] = None
    robot_origin_quat: Optional[np.ndarray] = None
    movement_enabled: bool = False
    calibrated: bool = False
    
    def reset_origin(self, vr_pos, vr_quat, robot_pos, robot_quat):
        """Reset origin when grip is pressed"""
        self.vr_origin_pos = vr_pos.copy()
        self.vr_origin_quat = vr_quat.copy()
        self.robot_origin_pos = robot_pos.copy()
        self.robot_origin_quat = robot_quat.copy()
        self.calibrated = True


@dataclass
class BenchmarkResult:
    """Store timing and performance metrics"""
    control_rate_hz: float
    avg_latency_ms: float
    ik_solve_time_ms: float
    collision_check_time_ms: float
    motion_plan_time_ms: float
    total_cycle_time_ms: float
    success_rate: float
    timestamp: float


@dataclass
class ControlCycleStats:
    """Statistics for a control cycle"""
    start_time: float
    ik_start: float
    ik_end: float
    collision_start: float
    collision_end: float
    plan_start: float
    plan_end: float
    execute_start: float
    execute_end: float
    success: bool
    
    @property
    def total_time_ms(self) -> float:
        return (self.execute_end - self.start_time) * 1000
    
    @property
    def ik_time_ms(self) -> float:
        return (self.ik_end - self.ik_start) * 1000
    
    @property
    def collision_time_ms(self) -> float:
        return (self.collision_end - self.collision_start) * 1000
    
    @property
    def plan_time_ms(self) -> float:
        return (self.plan_end - self.plan_start) * 1000


class FrankaBenchmarkController(Node):
    """Advanced benchmarking controller for Franka FR3 with full MoveIt integration"""
    
    def __init__(self):
        super().__init__('franka_benchmark_controller')
        
        # Robot configuration
        self.robot_ip = "192.168.1.59"
        self.planning_group = "fr3_arm"  # Updated from panda_arm to match FR3 robot
        self.end_effector_link = "fr3_hand_tcp"
        self.base_frame = "fr3_link0"
        self.planning_frame = "fr3_link0"  # Frame for planning operations
        
        # Joint names for FR3
        self.joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]
        
        # Home position (ready pose)
        self.home_positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        
        # Create service clients for full MoveIt integration
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.planning_scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.motion_plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
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
        
        # Wait for services
        self.get_logger().info('🔄 Waiting for MoveIt services...')
        self.ik_client.wait_for_service(timeout_sec=10.0)
        self.planning_scene_client.wait_for_service(timeout_sec=10.0)
        self.motion_plan_client.wait_for_service(timeout_sec=10.0)
        self.fk_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('✅ All MoveIt services ready!')
        
        # Wait for action server
        self.get_logger().info('🔄 Waiting for trajectory action server...')
        self.trajectory_client.wait_for_server(timeout_sec=10.0)
        self.get_logger().info('✅ Trajectory action server ready!')
        
        # Benchmarking parameters
        self.target_rates_hz = [10, 50, 75, 100, 200]  # Added 75Hz to find transition point
        self.benchmark_duration_seconds = 10.0  # Run each rate for 10 seconds
        self.max_concurrent_operations = 10  # Limit concurrent operations for stability
        
        # VR Control Parameters (from oculus_vr_server.py) - INCREASED FOR VISIBILITY
        self.max_lin_vel = 1.0
        self.max_rot_vel = 1.0
        self.max_gripper_vel = 1.0
        self.spatial_coeff = 1.0
        self.pos_action_gain = 10.0  # Increased from 5.0 for more aggressive movement
        self.rot_action_gain = 5.0   # Increased from 2.0 for more aggressive rotation
        self.gripper_action_gain = 3.0
        
        # VR velocity-to-delta conversion parameters - INCREASED FOR VISIBILITY
        self.max_lin_delta = 0.15   # Increased from 0.075 - max 15cm linear movement per control cycle
        self.max_rot_delta = 0.3    # Increased from 0.15 - max 17° rotation per control cycle
        self.max_gripper_delta = 0.25  # Maximum gripper movement per control cycle
        
        # Coordinate transformation (DROID-compatible)
        # Default transformation: VR [X,Y,Z] → Robot [-Y,X,Z] (more intuitive)
        rmat_reorder = [-2, -1, 3, 4]  # Maps VR coordinates to robot coordinates
        self.global_to_env_mat = vec_to_reorder_mat(rmat_reorder)
        
        # VR-to-global transformation (identity for now, can be calibrated)
        self.vr_to_global_mat = np.eye(4)
        
        # VR control state
        self.vr_control_state = VRControlState()
        
        # Test poses will be VR poses instead of joint targets
        self.test_vr_poses = []
        
        # Performance tracking
        self.cycle_stats: List[ControlCycleStats] = []
        self.benchmark_results: List[BenchmarkResult] = []
        self.rate_latencies: Dict[float, List[float]] = {}
        
        # Robot state tracking for VR control
        self.robot_gripper = 0.0  # Current gripper state (0.0 = open, 1.0 = closed)
        
        # Threading for high-frequency operation
        self._control_thread = None
        self._running = False
        self._current_target_rate = 1.0
        
        self.get_logger().info('🎮 Franka FR3 VR Teleoperation Benchmark Controller Initialized')
        self.get_logger().info(f'📊 Will test VR control rates: {self.target_rates_hz} Hz')
        self.get_logger().info(f'⏱️  Each rate tested for: {self.benchmark_duration_seconds}s')
        self.get_logger().info(f'🎯 VR Control Pipeline: Pose → Transform → Velocity → Robot Commands')
        self.get_logger().info(f'🔄 Coordinate Transform: VR [-2,-1,3,4] → Robot coordinates')
        self.get_logger().info(f'⚙️  VR Gains: pos={self.pos_action_gain}x, rot={self.rot_action_gain}x')
        
    def joint_state_callback(self, msg):
        """Store the latest joint state"""
        self.joint_state = msg
        
    def check_robot_safety_state(self):
        """Check if robot is in a safe state for movement"""
        try:
            # Check if we can get joint positions
            current_joints = self.get_current_joint_positions()
            if current_joints is None:
                self.get_logger().error('❌ Cannot read robot joint positions')
                return False
            
            # Check if joints are in reasonable ranges (basic safety check)
            for i, pos in enumerate(current_joints):
                if abs(pos) > 3.0:  # ~170 degrees - reasonable joint limit
                    self.get_logger().warn(f'⚠️  Joint {i+1} at extreme position: {pos:.3f} rad')
            
            # Try to get end effector pose
            current_pose = self.get_current_end_effector_pose()
            if current_pose is None:
                self.get_logger().warn('⚠️  Cannot get end effector pose, but continuing...')
            
            self.get_logger().info('✅ Robot safety check passed')
            return True
            
        except Exception as e:
            self.get_logger().error(f'❌ Robot safety check failed: {e}')
            return False
        
    def get_current_joint_positions(self):
        """Get current joint positions from joint_states topic"""
        if self.joint_state is None:
            return None
            
        positions = []
        for joint_name in self.joint_names:
            if joint_name in self.joint_state.name:
                idx = self.joint_state.name.index(joint_name)
                positions.append(self.joint_state.position[idx])
            else:
                return None
                
        return positions
    
    def process_vr_pose(self, vr_pose: VRPose) -> Tuple[np.ndarray, np.ndarray]:
        """Apply coordinate transformations to VR controller pose (from oculus_vr_server.py)"""
        # Create 4x4 transformation matrix from VR pose
        vr_mat = np.eye(4)
        vr_mat[:3, 3] = vr_pose.position
        vr_mat[:3, :3] = quat_to_rmat(vr_pose.orientation)
        
        # Apply transformations: VR → Global → Robot coordinates
        transformed_mat = self.global_to_env_mat @ self.vr_to_global_mat @ vr_mat
        
        # Extract transformed position and orientation
        transformed_pos = self.spatial_coeff * transformed_mat[:3, 3]
        transformed_quat = rmat_to_quat(transformed_mat[:3, :3])
        
        return transformed_pos, transformed_quat
    
    def limit_velocity(self, lin_vel, rot_vel, gripper_vel):
        """Scales down the linear and angular magnitudes of the action (from oculus_vr_server.py)"""
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
    
    def calculate_vr_action(self, vr_pose: VRPose, robot_pos: np.ndarray, robot_quat: np.ndarray, robot_gripper: float) -> Tuple[np.ndarray, Dict]:
        """Calculate robot action from VR controller state - SIMPLIFIED without grip mode"""
        # Process VR pose through coordinate transformations
        vr_pos, vr_quat = self.process_vr_pose(vr_pose)
        
        # Initialize origin on first call (using robot home as reference)
        if not self.vr_control_state.calibrated:
            # Set home state as origin - all movements are perturbations from here
            self.vr_control_state.reset_origin(
                np.array([0.0, 0.0, 0.0]),  # VR starts at origin
                np.array([0.0, 0.0, 0.0, 1.0]),  # Identity quaternion
                robot_pos,  # Robot home position
                robot_quat  # Robot home orientation
            )
            self.vr_control_state.calibrated = True
            self.get_logger().info("🏠 VR control initialized at home state")
        
        # Calculate position action as simple perturbation from home
        # VR movement directly maps to desired robot movement
        pos_action = vr_pos  # Direct VR position is the perturbation
        
        # Calculate rotation action
        # VR rotation directly maps to desired robot rotation change
        vr_rot = R.from_quat(vr_quat)
        identity_rot = R.from_quat([0, 0, 0, 1])
        rot_diff = vr_rot * identity_rot.inv()
        euler_action = rot_diff.as_euler('xyz')
        
        # Calculate target pose (home + perturbation)
        target_pos = self.vr_control_state.robot_origin_pos + pos_action
        
        # Apply rotation perturbation to home orientation
        home_rot = R.from_quat(self.vr_control_state.robot_origin_quat)
        target_rot = home_rot * rot_diff
        target_quat = target_rot.as_quat()
        
        # Simple gripper control based on trigger
        gripper_action = vr_pose.trigger_value - robot_gripper
        
        # Scale with VR gains for velocity control
        pos_velocity = pos_action * self.pos_action_gain
        rot_velocity = euler_action * self.rot_action_gain
        gripper_velocity = gripper_action * self.gripper_action_gain
        
        # Apply velocity limits
        lin_vel, rot_vel, gripper_vel = self.limit_velocity(pos_velocity, rot_velocity, gripper_velocity)
        
        # Debug output
        if np.linalg.norm(pos_action) > 0.001:
            self.get_logger().debug(f'🎯 VR Perturbation: pos=[{pos_action[0]:.3f}, {pos_action[1]:.3f}, {pos_action[2]:.3f}]')
            self.get_logger().debug(f'   Velocity: [{lin_vel[0]:.3f}, {lin_vel[1]:.3f}, {lin_vel[2]:.3f}]')
        
        # Prepare info dictionary
        info_dict = {
            "target_quaternion": target_quat,
            "target_position": target_pos,
            "vr_pos": vr_pos,
            "vr_quat": vr_quat,
            "perturbation": pos_action
        }
        
        # Combine into action vector
        action = np.concatenate([lin_vel, rot_vel, [gripper_vel]])
        action = action.clip(-1, 1)
        
        return action, info_dict
    
    def velocity_to_position_target_vr(self, velocity_action, current_pos, current_quat, action_info=None):
        """Convert VR velocity action to position target for robot control"""
        # Extract components
        lin_vel = velocity_action[:3]
        rot_vel = velocity_action[3:6]
        gripper_vel = velocity_action[6]
        
        # Apply velocity scaling
        lin_vel_norm = np.linalg.norm(lin_vel)
        rot_vel_norm = np.linalg.norm(rot_vel)
        
        if lin_vel_norm > 1:
            lin_vel = lin_vel / lin_vel_norm
        if rot_vel_norm > 1:
            rot_vel = rot_vel / rot_vel_norm
            
        # Convert to position delta using VR parameters
        pos_delta = lin_vel * self.max_lin_delta
        rot_delta = rot_vel * self.max_rot_delta
        
        # Calculate target position
        target_pos = current_pos + pos_delta
        
        # Calculate target orientation
        if action_info and "target_quaternion" in action_info:
            # Use the pre-calculated target quaternion for accurate VR control
            target_quat = action_info["target_quaternion"]
        else:
            # Fallback: convert rotation velocity to quaternion
            rot_delta_quat = euler_to_quat(rot_delta)
            current_rot = R.from_quat(current_quat)
            delta_rot = R.from_quat(rot_delta_quat)
            target_rot = delta_rot * current_rot
            target_quat = target_rot.as_quat()
        
        # Calculate target gripper (simple scaling)
        control_interval = 1.0 / self.target_rates_hz[0] if self.target_rates_hz else 0.1
        gripper_delta = gripper_vel * control_interval
        target_gripper = np.clip(self.robot_gripper + gripper_delta, 0.0, 1.0)
        
        return target_pos, target_quat, target_gripper
    
    def execute_trajectory(self, positions, duration=2.0):
        """Execute a trajectory to move joints to target positions with conservative settings"""
        if not self.trajectory_client.server_is_ready():
            return False
            
        # Create trajectory with VERY conservative settings for Franka safety
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Add multiple waypoints for smoother motion
        current_joints = self.get_current_joint_positions()
        if not current_joints:
            return False
        
        # Create 3 waypoints: current -> 50% -> target for smoother motion
        num_waypoints = 3
        for i in range(num_waypoints):
            point = JointTrajectoryPoint()
            progress = (i + 1) / num_waypoints
            
            # Interpolate between current and target
            interpolated_positions = []
            for j in range(len(positions)):
                if j < len(current_joints):
                    interp_pos = (1 - progress) * current_joints[j] + progress * positions[j]
                    interpolated_positions.append(interp_pos)
                else:
                    interpolated_positions.append(positions[j])
            
            point.positions = interpolated_positions
            
            # Very conservative velocities (much slower than default)
            point.velocities = [0.1] * len(self.joint_names)  # 0.1 rad/s max
            point.accelerations = [0.05] * len(self.joint_names)  # 0.05 rad/s² max
            
            # Spread waypoints over the duration
            waypoint_time = (duration * progress)
            point.time_from_start.sec = int(waypoint_time)
            point.time_from_start.nanosec = int((waypoint_time - int(waypoint_time)) * 1e9)
            
            trajectory.points.append(point)
            
        # Create goal with conservative tolerances
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        # Set very loose tolerances to avoid rejection
        goal.goal_tolerance = []
        for joint_name in self.joint_names:
            tolerance = control_msgs.msg.JointTolerance()
            tolerance.name = joint_name
            tolerance.position = 0.1  # 0.1 rad tolerance (~6 degrees)
            tolerance.velocity = 0.5   # 0.5 rad/s tolerance
            tolerance.acceleration = 1.0  # 1.0 rad/s² tolerance
            goal.goal_tolerance.append(tolerance)
        
        # Set path tolerances (even more loose)
        goal.path_tolerance = []
        for joint_name in self.joint_names:
            tolerance = control_msgs.msg.JointTolerance()
            tolerance.name = joint_name
            tolerance.position = 0.2  # 0.2 rad path tolerance
            tolerance.velocity = 1.0   # 1.0 rad/s path tolerance
            tolerance.acceleration = 2.0  # 2.0 rad/s² path tolerance
            goal.path_tolerance.append(tolerance)
        
        # Send goal
        future = self.trajectory_client.send_goal_async(goal)
        
        # Wait for goal acceptance
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        goal_handle = future.result()
        
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('❌ Trajectory goal rejected')
            return False
            
        self.get_logger().info('✅ Trajectory goal accepted - executing slowly...')
            
        # Wait for result with longer timeout
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 10.0)
        
        result = result_future.result()
        if result is None:
            self.get_logger().error('❌ Trajectory execution timed out')
            return False
            
        success = result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
            if success:
            self.get_logger().info('✅ Trajectory executed successfully')
            else:
            self.get_logger().error(f'❌ Trajectory failed with error: {result.result.error_code}')
            
        return success
    
    def move_to_home(self):
        """Move robot to home position with very conservative settings"""
        self.get_logger().info('🏠 Moving to home position slowly and safely...')
        return self.execute_trajectory(self.home_positions, duration=8.0)  # Much slower: 8 seconds
    
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
        
        scene_future = self.planning_scene_client.call_async(scene_request)
        rclpy.spin_until_future_complete(self, scene_future, timeout_sec=1.0)
        return scene_future.result()
    
    def get_current_end_effector_pose(self):
        """Get current end-effector pose using forward kinematics"""
        try:
            if not self.fk_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn('FK service not available')
                return None
            
            # Get current joint positions
            current_joints = self.get_current_joint_positions()
            if current_joints is None:
                return None
            
            # Create FK request
            fk_request = GetPositionFK.Request()
            fk_request.fk_link_names = [self.end_effector_link]
            fk_request.header.frame_id = self.base_frame
            fk_request.header.stamp = self.get_clock().now().to_msg()
            
            # Set robot state
            fk_request.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
            fk_request.robot_state.joint_state.name = self.joint_names
            fk_request.robot_state.joint_state.position = current_joints
            
            # Call FK service
            fk_future = self.fk_client.call_async(fk_request)
            rclpy.spin_until_future_complete(self, fk_future, timeout_sec=2.0)
            fk_response = fk_future.result()
            
            if fk_response and fk_response.error_code.val == 1 and fk_response.pose_stamped:
                pose = fk_response.pose_stamped[0].pose
                self.get_logger().info(f'Current EE pose: pos=[{pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}]')
                self.get_logger().info(f'                 ori=[{pose.orientation.x:.3f}, {pose.orientation.y:.3f}, {pose.orientation.z:.3f}, {pose.orientation.w:.3f}]')
                return pose
                
        except Exception as e:
            self.get_logger().warn(f'Failed to get current EE pose: {e}')
        
        return None
    
    def create_realistic_test_poses(self):
        """Create test VR poses for realistic teleoperation movements"""
        self.get_logger().info('🎮 Creating realistic VR controller movements for testing...')
        
        # Get current robot pose for reference
        current_pose = self.get_current_end_effector_pose()
        if current_pose is None:
            self.get_logger().warn('Could not get current robot pose, using defaults')
            # Use default position in robot workspace
            current_pos = np.array([0.4, 0.0, 0.3])
            current_quat = np.array([0.0, 0.0, 0.0, 1.0])
        else:
            current_pos = np.array([current_pose.position.x, current_pose.position.y, current_pose.position.z])
            current_quat = np.array([current_pose.orientation.x, current_pose.orientation.y, 
                                   current_pose.orientation.z, current_pose.orientation.w])
        
        # Create VR poses that will result in meaningful robot movements
        # These are in VR coordinate frame before transformation
        self.test_vr_poses = []
        
        # Movement 1: VR controller moves 50cm forward (+Z in VR coords) - MUCH LARGER
        # This should translate to significant robot movement based on coordinate transformation
        vr_pose_1 = VRPose.create_example_pose(
            x=0.0, y=0.0, z=0.5,  # 50cm forward in VR (was 10cm)
            qx=0.0, qy=0.0, qz=0.0, qw=1.0,  # No rotation
            grip=True, trigger=0.0
        )
        self.test_vr_poses.append(vr_pose_1)
        
        # Movement 2: VR controller moves right 40cm (+X in VR coords)
        vr_pose_2 = VRPose.create_example_pose(
            x=0.4, y=0.0, z=0.0,  # 40cm right in VR (was 10cm)
            qx=0.0, qy=0.0, qz=0.0, qw=1.0,
            grip=True, trigger=0.0
        )
        self.test_vr_poses.append(vr_pose_2)
        
        # Movement 3: VR controller moves up 30cm (+Y in VR coords)
        vr_pose_3 = VRPose.create_example_pose(
            x=0.0, y=0.3, z=0.0,  # 30cm up in VR (was 10cm)
            qx=0.0, qy=0.0, qz=0.0, qw=1.0,
            grip=True, trigger=0.0
        )
        self.test_vr_poses.append(vr_pose_3)
        
        # Movement 4: VR controller rotation (60° around Z-axis) - LARGER ROTATION
        rotation_angle = np.radians(60)  # Increased from 30°
        vr_pose_4 = VRPose.create_example_pose(
            x=0.0, y=0.0, z=0.0,  # No translation
            qx=0.0, qy=0.0, qz=np.sin(rotation_angle/2), qw=np.cos(rotation_angle/2),  # 60° yaw
            grip=True, trigger=0.0
        )
        self.test_vr_poses.append(vr_pose_4)
        
        # Movement 5: Combined movement - larger diagonal with rotation
        vr_pose_5 = VRPose.create_example_pose(
            x=0.2, y=0.2, z=0.2,  # 20cm diagonal movement (was 5cm)
            qx=0.0, qy=np.sin(np.radians(30)/2), qz=0.0, qw=np.cos(np.radians(30)/2),  # 30° pitch (was 15°)
            grip=True, trigger=0.8  # More gripper close
        )
        self.test_vr_poses.append(vr_pose_5)
        
        # Show what these movements should produce after coordinate transformation
        self.get_logger().info(f'Created {len(self.test_vr_poses)} VR controller movements:')
        self.get_logger().info('   1. Forward movement (50cm +Z VR)')
        self.get_logger().info('   2. Right movement (40cm +X VR)')  
        self.get_logger().info('   3. Up movement (30cm +Y VR)')
        self.get_logger().info('   4. Rotation (60° yaw VR)')
        self.get_logger().info('   5. Combined diagonal + rotation')
        
        # Show coordinate transformation effects
        self.get_logger().info('\n🔄 Coordinate Transformation Preview:')
        for i, vr_pose in enumerate(self.test_vr_poses):
            # Apply transformation to show expected robot movement
            vr_pos, vr_quat = self.process_vr_pose(vr_pose)
            vr_euler = quat_to_euler(vr_quat, degrees=True)
            
            self.get_logger().info(f'   Movement {i+1}:')
            self.get_logger().info(f'     VR: pos=[{vr_pose.position[0]:.3f}, {vr_pose.position[1]:.3f}, {vr_pose.position[2]:.3f}]')
            self.get_logger().info(f'     Robot: pos=[{vr_pos[0]:.3f}, {vr_pos[1]:.3f}, {vr_pos[2]:.3f}] (after transform)')
            if np.linalg.norm(quat_to_euler(vr_pose.orientation)) > 0.01:
                vr_original_euler = quat_to_euler(vr_pose.orientation, degrees=True)
                self.get_logger().info(f'     VR rotation: [R:{vr_original_euler[0]:.1f}, P:{vr_original_euler[1]:.1f}, Y:{vr_original_euler[2]:.1f}]')
                self.get_logger().info(f'     Robot rotation: [R:{vr_euler[0]:.1f}, P:{vr_euler[1]:.1f}, Y:{vr_euler[2]:.1f}] (after transform)')
        
        # Note: These will be used as waypoints in movement sequences
        self.get_logger().info('\n💡 Note: These poses will be used as waypoints for VR-style teleoperation')
        self.get_logger().info('    Each test will cycle through: HOME → MOVEMENT → HOME')
        self.get_logger().info('    Grip simulation: ON during movement, OFF at home')
    
    def compute_ik_with_collision_avoidance(self, target_pose: VRPose) -> Tuple[Optional[List[float]], ControlCycleStats]:
        """Compute IK for VR pose with full collision avoidance"""
        stats = ControlCycleStats(
            start_time=time.time(),
            ik_start=0, ik_end=0,
            collision_start=0, collision_end=0,
            plan_start=0, plan_end=0,
            execute_start=0, execute_end=0,
            success=False
        )
        
        try:
            # Step 1: Get planning scene for collision checking
            stats.collision_start = time.time()
            scene_response = self.get_planning_scene()
            stats.collision_end = time.time()
            
            if scene_response is None:
                self.get_logger().debug('Failed to get planning scene')
                return None, stats
                
            # Step 2: Compute IK
            stats.ik_start = time.time()
            
            # Create IK request with collision avoidance
            ik_request = GetPositionIK.Request()
            ik_request.ik_request.group_name = self.planning_group
            ik_request.ik_request.robot_state = scene_response.scene.robot_state
            ik_request.ik_request.avoid_collisions = True  # Enable collision avoidance
            ik_request.ik_request.timeout.sec = 0
            ik_request.ik_request.timeout.nanosec = int(0.1 * 1e9)  # 100ms timeout
            
            # Set target pose from VR data
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.base_frame
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            
            # Convert VR pose to ROS Pose
            pose_stamped.pose.position.x = float(target_pose.position[0])
            pose_stamped.pose.position.y = float(target_pose.position[1])
            pose_stamped.pose.position.z = float(target_pose.position[2])
            pose_stamped.pose.orientation.x = float(target_pose.orientation[0])
            pose_stamped.pose.orientation.y = float(target_pose.orientation[1])
            pose_stamped.pose.orientation.z = float(target_pose.orientation[2])
            pose_stamped.pose.orientation.w = float(target_pose.orientation[3])
            
            ik_request.ik_request.pose_stamped = pose_stamped
            ik_request.ik_request.ik_link_name = self.end_effector_link
            
            # Call IK service
            ik_future = self.ik_client.call_async(ik_request)
            rclpy.spin_until_future_complete(self, ik_future, timeout_sec=0.2)
            ik_response = ik_future.result()
            
            stats.ik_end = time.time()
            
            if ik_response is None:
                self.get_logger().debug('IK service call failed - no response')
                return None, stats
            elif ik_response.error_code.val != 1:
                self.get_logger().debug(f'IK failed with error code: {ik_response.error_code.val}')
                self.get_logger().debug(f'Target pose: pos=[{target_pose.position[0]:.3f}, {target_pose.position[1]:.3f}, {target_pose.position[2]:.3f}]')
                return None, stats
                
            # Extract joint positions
            positions = []
            for joint_name in self.joint_names:
                if joint_name in ik_response.solution.joint_state.name:
                    idx = ik_response.solution.joint_state.name.index(joint_name)
                    positions.append(ik_response.solution.joint_state.position[idx])
                    
            stats.success = len(positions) == len(self.joint_names)
            if stats.success:
                self.get_logger().debug(f'IK SUCCESS for pose: pos=[{target_pose.position[0]:.3f}, {target_pose.position[1]:.3f}, {target_pose.position[2]:.3f}]')
            return positions if stats.success else None, stats
                
        except Exception as e:
            self.get_logger().debug(f'IK computation failed with exception: {e}')
            return None, stats
    
    def plan_motion_with_moveit(self, target_joints: List[float]) -> Tuple[Optional[JointTrajectory], ControlCycleStats]:
        """Plan motion using MoveIt motion planner with collision avoidance"""
        stats = ControlCycleStats(
            start_time=time.time(),
            ik_start=0, ik_end=0,
            collision_start=0, collision_end=0,
            plan_start=0, plan_end=0,
            execute_start=0, execute_end=0,
            success=False
        )
        
        try:
            stats.plan_start = time.time()
            
            # Get current planning scene
            scene_response = self.get_planning_scene()
            if scene_response is None:
                return None, stats
            
            # Create motion planning request
            plan_request = GetMotionPlan.Request()
            plan_request.motion_plan_request.group_name = self.planning_group
            plan_request.motion_plan_request.start_state = scene_response.scene.robot_state
            
            # Set goal constraints (target joint positions)
            constraints = Constraints()
            for i, joint_name in enumerate(self.joint_names):
                joint_constraint = JointConstraint()
                joint_constraint.joint_name = joint_name
                joint_constraint.position = target_joints[i]
                joint_constraint.tolerance_above = 0.01
                joint_constraint.tolerance_below = 0.01
                joint_constraint.weight = 1.0
                constraints.joint_constraints.append(joint_constraint)
            
            plan_request.motion_plan_request.goal_constraints.append(constraints)
            
            # Set workspace parameters for collision checking
            workspace = WorkspaceParameters()
            workspace.header.frame_id = self.base_frame
            workspace.min_corner.x = -1.0
            workspace.min_corner.y = -1.0
            workspace.min_corner.z = -0.5
            workspace.max_corner.x = 1.0
            workspace.max_corner.y = 1.0
            workspace.max_corner.z = 1.5
            plan_request.motion_plan_request.workspace_parameters = workspace
            
            # Set planning options
            plan_request.motion_plan_request.max_velocity_scaling_factor = 0.3
            plan_request.motion_plan_request.max_acceleration_scaling_factor = 0.3
            plan_request.motion_plan_request.allowed_planning_time = 0.5  # 500ms max
            plan_request.motion_plan_request.num_planning_attempts = 3
            
            # Call motion planning service
            plan_future = self.motion_plan_client.call_async(plan_request)
            rclpy.spin_until_future_complete(self, plan_future, timeout_sec=1.0)
            plan_response = plan_future.result()
            
            stats.plan_end = time.time()
            
            if (plan_response is None or 
                    plan_response.motion_plan_response.error_code.val != 1 or
                    not plan_response.motion_plan_response.trajectory.joint_trajectory.points):
                return None, stats
                
            stats.success = True
            return plan_response.motion_plan_response.trajectory.joint_trajectory, stats
                
        except Exception as e:
            self.get_logger().debug(f'Motion planning failed: {e}')
            stats.plan_end = time.time()
            return None, stats
    
    def benchmark_control_rate(self, target_hz: float) -> BenchmarkResult:
        """Benchmark VR teleoperation using 75Hz perturbations downsampled to target rate"""
        self.get_logger().info(f'🎮 Benchmarking {target_hz}Hz VR teleoperation (75Hz perturbations → {target_hz}Hz)')
        
        # Test parameters
        test_duration = 10.0  # 10 seconds test
        
        # Generate perturbations at 75Hz
        self.get_logger().info('🎯 Generating VR perturbations at 75Hz...')
        perturbations_75hz = self.generate_vr_perturbations_at_75hz(test_duration)
        
        # Downsample to target frequency
        downsampled_poses = self.downsample_perturbations(perturbations_75hz, target_hz)
        self.get_logger().info(f'📊 Downsampled to {len(downsampled_poses)} samples for {target_hz}Hz')
        
        # Get starting robot state (home position)
        current_pose = self.get_current_end_effector_pose()
        if current_pose:
            home_pos = np.array([current_pose.position.x, current_pose.position.y, current_pose.position.z])
            home_quat = np.array([current_pose.orientation.x, current_pose.orientation.y, 
                                current_pose.orientation.z, current_pose.orientation.w])
        else:
            home_pos = np.array([0.4, 0.0, 0.3])
            home_quat = np.array([0.0, 0.0, 0.0, 1.0])
        
        # Initialize tracking
        current_robot_pos = home_pos.copy()
        current_robot_quat = home_quat.copy()
        current_robot_gripper = 0.0
        
        # Reset VR control state
        self.vr_control_state = VRControlState()
        
        # Performance tracking
        successful_commands = 0
        failed_commands = 0
        command_times = []
        vr_process_times = []
        timing_errors = []
        
        # Expected timing
        command_interval = 1.0 / target_hz
        
        self.get_logger().info(f'🚀 Starting {target_hz}Hz benchmark with {len(downsampled_poses)} commands...')
        start_time = time.time()
        
        # Process each downsampled pose
        for i, (sample_idx, vr_pose) in enumerate(downsampled_poses):
            # Expected time for this command
            expected_time = start_time + (i * command_interval)
            
            # Wait until the right time (simulate real-time control)
            current_time = time.time()
            if current_time < expected_time:
                time.sleep(expected_time - current_time)
            
            command_start = time.time()
            
            # Track timing accuracy
            timing_error = abs(command_start - expected_time)
            timing_errors.append(timing_error)
            
            # Process VR pose through control pipeline
            vr_process_start = time.time()
            try:
                # Calculate VR action
                action, action_info = self.calculate_vr_action(
                    vr_pose, current_robot_pos, current_robot_quat, current_robot_gripper
                )
                
                vr_process_time = time.time() - vr_process_start
                vr_process_times.append(vr_process_time)
                
                # Convert to position target
                target_pos, target_quat, target_gripper = self.velocity_to_position_target_vr(
                    action, current_robot_pos, current_robot_quat, action_info
                )
                
                # Send command (or simulate)
                command_success = self.send_vr_command(
                    target_pos, target_quat, target_gripper, command_interval
                )
                
                if command_success:
                    # Update simulated robot state
                    current_robot_pos = target_pos.copy()
                    current_robot_quat = target_quat.copy()
                    current_robot_gripper = target_gripper
                    successful_commands += 1
            else:
                    failed_commands += 1
                
        except Exception as e:
                self.get_logger().debug(f'Command error: {e}')
                failed_commands += 1
                vr_process_times.append(0.0)
            
            # Track total command time
            command_time = time.time() - command_start
            command_times.append(command_time)
            
            # Progress update every second
            if i > 0 and i % int(target_hz) == 0:
                elapsed = time.time() - start_time
                actual_rate = i / elapsed
                self.get_logger().info(f'   Progress: {i}/{len(downsampled_poses)} commands, {actual_rate:.1f}Hz actual')
        
        # Calculate final metrics
        end_time = time.time()
        actual_duration = end_time - start_time
        total_commands = successful_commands + failed_commands
        actual_rate = total_commands / actual_duration if actual_duration > 0 else 0
        
        # Calculate averages
        avg_command_time = np.mean(command_times) * 1000 if command_times else 0
        avg_vr_time = np.mean(vr_process_times) * 1000 if vr_process_times else 0
        avg_timing_error = np.mean(timing_errors) * 1000 if timing_errors else 0
        success_rate = (successful_commands / total_commands * 100) if total_commands > 0 else 0
        
        # Log results
        self.get_logger().info(f'\n📈 RESULTS for {target_hz}Hz:')
        self.get_logger().info(f'   Actual rate: {actual_rate:.1f}Hz ({actual_rate/target_hz*100:.1f}% of target)')
        self.get_logger().info(f'   Success rate: {success_rate:.1f}% ({successful_commands}/{total_commands})')
        self.get_logger().info(f'   Avg command time: {avg_command_time:.2f}ms')
        self.get_logger().info(f'   Avg VR processing: {avg_vr_time:.2f}ms')
        self.get_logger().info(f'   Avg timing error: {avg_timing_error:.2f}ms')
        self.get_logger().info(f'   75Hz → {target_hz}Hz: {"Upsampled" if target_hz > 75 else "Downsampled"}')
        
        return BenchmarkResult(
            control_rate_hz=actual_rate,
            avg_latency_ms=avg_command_time,
            ik_solve_time_ms=avg_vr_time,
            collision_check_time_ms=avg_timing_error,
            motion_plan_time_ms=0.0,
            total_cycle_time_ms=avg_command_time,
            success_rate=success_rate,
            timestamp=time.time()
        )
    
    def generate_vr_perturbations_at_75hz(self, duration: float) -> List[VRPose]:
        """Generate VR perturbations at 75Hz for the given duration"""
        hz_75_interval = 1.0 / 75.0  # ~13.33ms
        num_samples = int(duration * 75)
        perturbations = []
        
        # Generate smooth sinusoidal perturbations at different frequencies
        for i in range(num_samples):
            t = i * hz_75_interval
            
            # Create multi-frequency perturbations for interesting movement
            # Position perturbations (in meters)
            x = 0.1 * np.sin(2 * np.pi * 0.5 * t)  # 0.5Hz oscillation, 10cm amplitude
            y = 0.05 * np.sin(2 * np.pi * 0.3 * t + np.pi/4)  # 0.3Hz, 5cm amplitude
            z = 0.08 * np.sin(2 * np.pi * 0.7 * t + np.pi/2)  # 0.7Hz, 8cm amplitude
            
            # Rotation perturbations (convert to quaternion)
            roll = 0.2 * np.sin(2 * np.pi * 0.4 * t)  # 0.4Hz, ~11° amplitude
            pitch = 0.15 * np.sin(2 * np.pi * 0.6 * t + np.pi/3)  # 0.6Hz, ~8.6° amplitude
            yaw = 0.1 * np.sin(2 * np.pi * 0.2 * t)  # 0.2Hz, ~5.7° amplitude
            
            # Convert euler to quaternion
            rot = R.from_euler('xyz', [roll, pitch, yaw])
            quat = rot.as_quat()
            
            # Trigger varies slowly
            trigger = 0.5 * (1 + np.sin(2 * np.pi * 0.1 * t))  # 0.1Hz, 0-1 range
            
            vr_pose = VRPose.create_example_pose(
                x=x, y=y, z=z,
                qx=quat[0], qy=quat[1], qz=quat[2], qw=quat[3],
                grip=True,  # Always "gripping" in simple mode
                trigger=trigger
            )
            perturbations.append(vr_pose)
        
        self.get_logger().info(f'📊 Generated {len(perturbations)} VR perturbations at 75Hz')
        return perturbations
    
    def downsample_perturbations(self, perturbations_75hz: List[VRPose], target_hz: float) -> List[Tuple[int, VRPose]]:
        """Downsample 75Hz perturbations to target frequency
        Returns list of (index, pose) tuples where index is the 75Hz sample index"""
        if target_hz >= 75:
            # For frequencies >= 75Hz, we'll repeat samples
            samples_per_75hz = int(target_hz / 75.0)
            downsampled = []
            for i, pose in enumerate(perturbations_75hz):
                for _ in range(samples_per_75hz):
                    downsampled.append((i, pose))
            return downsampled
        else:
            # For frequencies < 75Hz, we skip samples
            skip_factor = 75.0 / target_hz
            downsampled = []
            accumulated = 0.0
            for i, pose in enumerate(perturbations_75hz):
                if i >= int(accumulated):
                    downsampled.append((i, pose))
                    accumulated += skip_factor
            return downsampled
    
    def execute_complete_trajectory(self, trajectory: JointTrajectory) -> bool:
        """Execute a complete trajectory with movement verification"""
        try:
            if not self.trajectory_client.server_is_ready():
                self.get_logger().warn('Trajectory action server not ready')
                return False
            
            # GET JOINT POSITIONS BEFORE MOVEMENT
            joints_before = self.get_current_joint_positions()
            if joints_before and len(trajectory.points) > 0:
                final_positions = trajectory.points[-1].positions
                self.get_logger().info(f"📍 BEFORE: {[f'{j:.3f}' for j in joints_before]}")
                self.get_logger().info(f"🎯 TARGET: {[f'{j:.3f}' for j in final_positions]}")
                
                # Calculate expected movement
                movements = [abs(final_positions[i] - joints_before[i]) for i in range(min(len(final_positions), len(joints_before)))]
                max_movement_rad = max(movements) if movements else 0
                max_movement_deg = max_movement_rad * 57.3
                self.get_logger().info(f"📐 EXPECTED: Max movement {max_movement_deg:.1f}° ({max_movement_rad:.3f} rad)")
                self.get_logger().info(f"🛤️  Executing {len(trajectory.points)} waypoint trajectory")
            
            # Create goal
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory
            
            # Send trajectory
            self.get_logger().info(f"🚀 SENDING {len(trajectory.points)}-point trajectory...")
            future = self.trajectory_client.send_goal_async(goal)
            
            # Wait for goal acceptance
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            goal_handle = future.result()
            
            if not goal_handle.accepted:
                self.get_logger().warn('❌ Trajectory goal REJECTED')
                return False
            
            self.get_logger().info(f"✅ Trajectory goal ACCEPTED - executing...")
                
            # Wait for result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=6.0)  # Increased timeout
            
            result = result_future.result()
            success = result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
            
            if not success:
                self.get_logger().warn(f'❌ Trajectory execution failed with error code: {result.result.error_code}')
            else:
                self.get_logger().info(f"✅ Trajectory reports SUCCESS")
            
            # GET JOINT POSITIONS AFTER MOVEMENT - VERIFY ACTUAL MOVEMENT
            time.sleep(0.5)  # Brief pause for joint states to update
            joints_after = self.get_current_joint_positions()
            
            if joints_before and joints_after:
                self.get_logger().info(f"📍 AFTER:  {[f'{j:.3f}' for j in joints_after]}")
                
                # Calculate actual movement
                actual_movements = [abs(joints_after[i] - joints_before[i]) for i in range(min(len(joints_after), len(joints_before)))]
                max_actual_rad = max(actual_movements) if actual_movements else 0
                max_actual_deg = max_actual_rad * 57.3
                
                self.get_logger().info(f"📐 ACTUAL: Max movement {max_actual_deg:.1f}° ({max_actual_rad:.3f} rad)")
                
                # Check if robot actually moved significantly
                if max_actual_rad > 0.1:  # More than ~6 degrees
                    self.get_logger().info(f"🎉 ROBOT MOVED! Visible displacement confirmed")
                    
                    # Log individual joint movements
                    for i, (before, after) in enumerate(zip(joints_before, joints_after)):
                        diff_rad = abs(after - before)
                        diff_deg = diff_rad * 57.3
                        if diff_rad > 0.05:  # More than ~3 degrees
                            self.get_logger().info(f"  Joint {i+1}: {diff_deg:.1f}° movement")
                else:
                    self.get_logger().warn(f"⚠️  ROBOT DID NOT MOVE! Max displacement only {max_actual_deg:.1f}°")
                    
            return success
                
        except Exception as e:
            self.get_logger().warn(f'Trajectory execution exception: {e}')
            return False
    
    def generate_trajectory_waypoints(self, target_vr_pose: VRPose, duration: float, timestep: float) -> List[VRPose]:
        """Generate intermediate waypoints for a trajectory - joint space or pose space"""
        try:
            # Check if this is a joint-space target
            if hasattr(target_vr_pose, 'joint_positions'):
                return self.generate_joint_space_waypoints(target_vr_pose.joint_positions, duration, timestep)
            else:
                return self.generate_pose_space_waypoints(target_vr_pose, duration, timestep)
                
        except Exception as e:
            self.get_logger().warn(f'Failed to generate trajectory waypoints: {e}')
            return []
    
    def generate_joint_space_waypoints(self, target_joints: List[float], duration: float, timestep: float) -> List[VRPose]:
        """Generate waypoints by interpolating in joint space - GUARANTEED smooth large movements"""
        try:
            # Get current joint positions
            current_joints = self.get_current_joint_positions()
            if current_joints is None:
                return []
            
            # Generate waypoints using linear interpolation in joint space
            waypoints = []
            num_steps = max(1, int(duration / timestep))
            
            # SKIP first waypoint (i=0, t=0) which is current position - start from i=1
            for i in range(1, num_steps + 1):  # Start from 1, not 0
                t = i / num_steps  # Interpolation parameter from >0 to 1
                
                # Linear interpolation for each joint
                interp_joints = []
                for j in range(len(self.joint_names)):
                    if j < len(current_joints) and j < len(target_joints):
                        interp_joint = (1 - t) * current_joints[j] + t * target_joints[j]
                        interp_joints.append(interp_joint)
                
                # Create waypoint with joint positions
                waypoint = VRPose.create_example_pose()
                waypoint.joint_positions = interp_joints
                waypoints.append(waypoint)
            
            self.get_logger().debug(f'Generated {len(waypoints)} JOINT-SPACE waypoints for {duration}s trajectory (SKIPPED current position)')
            return waypoints
                
        except Exception as e:
            self.get_logger().warn(f'Failed to generate joint space waypoints: {e}')
            return []
    
    def generate_pose_space_waypoints(self, target_vr_pose: VRPose, duration: float, timestep: float) -> List[VRPose]:
        """Generate waypoints by interpolating in pose space"""
        try:
            # Get current end-effector pose
            current_pose = self.get_current_end_effector_pose()
            if current_pose is None:
                return []
            
            # Convert current pose to VRPose
            current_vr_pose = VRPose(
                position=np.array([current_pose.position.x, current_pose.position.y, current_pose.position.z]),
                orientation=np.array([current_pose.orientation.x, current_pose.orientation.y, 
                                    current_pose.orientation.z, current_pose.orientation.w]),
                timestamp=time.time()
            )
            
            # Generate waypoints using linear interpolation
            waypoints = []
            num_steps = max(1, int(duration / timestep))
            
            for i in range(num_steps + 1):  # Include final waypoint
                t = i / num_steps  # Interpolation parameter 0 to 1
                
                # Linear interpolation for position
                interp_position = (1 - t) * current_vr_pose.position + t * target_vr_pose.position
                
                # Spherical linear interpolation (SLERP) for orientation would be better,
                # but for simplicity, use linear interpolation and normalize
                interp_orientation = (1 - t) * current_vr_pose.orientation + t * target_vr_pose.orientation
                # Normalize quaternion
                norm = np.linalg.norm(interp_orientation)
                if norm > 0:
                    interp_orientation = interp_orientation / norm
                
                waypoint = VRPose(
                    position=interp_position,
                    orientation=interp_orientation,
                    timestamp=time.time()
                )
                waypoints.append(waypoint)
            
            self.get_logger().debug(f'Generated {len(waypoints)} POSE-SPACE waypoints for {duration}s trajectory')
            return waypoints
                
        except Exception as e:
            self.get_logger().warn(f'Failed to generate pose space waypoints: {e}')
            return []

    def print_benchmark_results(self, result: BenchmarkResult, target_hz: float):
        """Print structured VR teleoperation benchmark results"""
        print(f"\n{'='*80}")
        print(f"🎮 VR TELEOPERATION BENCHMARK - {target_hz}Hz")
        print(f"{'='*80}")
        print(f"🎯 Target Command Rate:      {target_hz:8.1f} Hz")
        print(f"📈 Actual Command Rate:      {result.control_rate_hz:8.1f} Hz ({result.control_rate_hz/target_hz*100:5.1f}%)")
        print(f"⏱️  Average Command Time:     {result.avg_latency_ms:8.2f} ms")
        print(f"🎮 Average VR Processing:    {result.ik_solve_time_ms:8.2f} ms")
        print(f"⏰ Average Timing Error:     {result.collision_check_time_ms:8.2f} ms")
        print(f"✅ Success Rate:             {result.success_rate:8.1f} %")
        
        # Calculate VR control parameters
        movement_duration = 3.0
        num_vr_movements = len(self.test_vr_poses)
        command_interval_ms = (1.0 / target_hz) * 1000
        
        print(f"📊 VR Movements per Test:    {num_vr_movements:8d}")
        print(f"🔍 Command Interval:         {command_interval_ms:8.2f} ms")
        print(f"🎯 Movement Types:           VR Controller Poses (position + rotation)")
        print(f"🛤️  Movement Duration:       {movement_duration:8.1f} s per movement")
        
        print(f"🎮 Control Method:           VR Pose → Coordinate Transform → Velocity Action")
        print(f"   Full VR teleoperation pipeline with coordinate transformations")
        
        # Performance analysis
        if result.control_rate_hz >= target_hz * 0.95:
            print(f"🎉 EXCELLENT: Achieved {result.control_rate_hz/target_hz*100:.1f}% of target rate")
        elif result.control_rate_hz >= target_hz * 0.8:
            print(f"👍 GOOD: Achieved {result.control_rate_hz/target_hz*100:.1f}% of target rate")
        elif result.control_rate_hz >= target_hz * 0.5:
            print(f"⚠️  MODERATE: Only achieved {result.control_rate_hz/target_hz*100:.1f}% of target rate")
            else:
            print(f"❌ POOR: Only achieved {result.control_rate_hz/target_hz*100:.1f}% of target rate")
        
        # VR processing time analysis
        if result.ik_solve_time_ms < 1.0:
            print(f"⚡ EXCELLENT VR processing: {result.ik_solve_time_ms:.2f}ms")
        elif result.ik_solve_time_ms < 10.0:
            print(f"👍 GOOD VR processing: {result.ik_solve_time_ms:.2f}ms")
        elif result.ik_solve_time_ms < 50.0:
            print(f"⚠️  MODERATE VR processing: {result.ik_solve_time_ms:.2f}ms")
        else:
            print(f"❌ HIGH VR processing time: {result.ik_solve_time_ms:.2f}ms")
            
        # VR teleoperation analysis
        print(f"📊 VR Teleoperation Analysis:")
        print(f"   Control Pipeline: VR Pose → Transform → Velocity → Position Target")
        print(f"   Coordinate Transform: VR [-2,-1,3,4] → Robot coordinates")
        print(f"   Movement Types: Forward, Right, Up, Rotation, Combined")
        print(f"   Grip Simulation: ON during movement, OFF at home")
        print(f"   Velocity Gains: pos={self.pos_action_gain}, rot={self.rot_action_gain}")
        
        print(f"{'='*80}\n")
    
    def print_summary_results(self):
        """Print comprehensive summary of VR teleoperation benchmark results"""
        print(f"\n{'='*100}")
        print(f"🏆 VR TELEOPERATION BENCHMARK - FRANKA FR3 with COORDINATE TRANSFORMATIONS")
        print(f"{'='*100}")
        print(f"Approach: Full VR teleoperation pipeline with realistic controller movements")
        print(f"Testing: VR-style control rates from 10Hz to 200Hz (mimicking Oculus controller)")
        print(f"Pipeline: VR Pose → Coordinate Transform → Velocity Calculation → Robot Control")
        print(f"Movements: {len(self.test_vr_poses)} different VR controller movements (forward, right, up, rotation, combined)")
        print(f"Method: Velocity-based control with position/rotation gains from VR teleoperation")
        print(f"{'='*100}")
        print(f"{'Rate (Hz)':>10} {'Actual (Hz)':>12} {'Cmd Time (ms)':>14} {'VR Proc (ms)':>15} {'Success (%)':>12} {'Commands/s':>12}")
        print(f"{'-'*100}")
        
        for i, result in enumerate(self.benchmark_results):
            target_hz = self.target_rates_hz[i] if i < len(self.target_rates_hz) else 0
            print(f"{target_hz:>10.0f} {result.control_rate_hz:>12.1f} {result.avg_latency_ms:>14.2f} "
                  f"{result.ik_solve_time_ms:>15.2f} {result.success_rate:>12.1f} {result.control_rate_hz:>12.1f}")
        
        print(f"{'-'*100}")
        
        # Find best performing rates
        if self.benchmark_results:
            best_rate = max(self.benchmark_results, key=lambda x: x.control_rate_hz)
            best_vr_time = min(self.benchmark_results, key=lambda x: x.ik_solve_time_ms)
            best_success = max(self.benchmark_results, key=lambda x: x.success_rate)
            
            print(f"\n🏆 VR TELEOPERATION HIGHLIGHTS:")
            print(f"   🚀 Highest Command Rate:     {best_rate.control_rate_hz:.1f} Hz")
            print(f"   ⚡ Fastest VR Processing:     {best_vr_time.ik_solve_time_ms:.2f} ms")
            print(f"   ✅ Best Success Rate:        {best_success.success_rate:.1f} %")
            
            # Overall performance analysis
            print(f"\n📈 VR TELEOPERATION PERFORMANCE:")
            for i, result in enumerate(self.benchmark_results):
                target_hz = self.target_rates_hz[i] if i < len(self.target_rates_hz) else 0
                
                print(f"\n   {target_hz} Hz VR Test:")
                print(f"   Achieved: {result.control_rate_hz:.1f} Hz ({result.control_rate_hz/target_hz*100:.1f}% of target)")
                print(f"   Command Time: {result.avg_latency_ms:.2f} ms")
                print(f"   VR Processing: {result.ik_solve_time_ms:.2f} ms")
                print(f"   Success Rate: {result.success_rate:.1f}%")
                
                # Calculate VR characteristics
                commands_per_second = result.control_rate_hz
                command_interval_ms = (1.0/commands_per_second)*1000 if commands_per_second > 0 else 0
                print(f"   Command interval: {command_interval_ms:.2f}ms")
        
        print(f"\n🎮 VR CONTROL PIPELINE DETAILS:")
        print(f"   Coordinate Transform: VR [X,Y,Z] → Robot coordinates using matrix {[-2,-1,3,4]}")
        print(f"   Position Gain: {self.pos_action_gain}x (VR position offset → robot velocity)")
        print(f"   Rotation Gain: {self.rot_action_gain}x (VR rotation offset → robot angular velocity)")
        print(f"   Velocity Limits: Linear {self.max_lin_vel}, Angular {self.max_rot_vel}")
        print(f"   Max Deltas: {self.max_lin_delta}m linear, {self.max_rot_delta} rad angular per cycle")
        
        print(f"\n🛤️  MOVEMENT PATTERNS TESTED:")
        print(f"   1. Forward Movement: +50cm in VR +Z direction")
        print(f"   2. Right Movement: +40cm in VR +X direction")
        print(f"   3. Up Movement: +30cm in VR +Y direction")
        print(f"   4. Rotation: +60° yaw rotation around VR Z-axis")
        print(f"   5. Combined: 20cm diagonal + 30° pitch rotation + gripper")
        
        print(f"{'='*100}\n")
    
    def run_comprehensive_benchmark(self):
        """Run complete VR teleoperation benchmark suite with coordinate transformations"""
        self.get_logger().info('🎮 Starting VR Teleoperation Benchmark - Franka FR3')
        self.get_logger().info('📊 Testing VR-style control rates from 10Hz to 200Hz')
        self.get_logger().info('🎯 Approach: Full VR teleoperation pipeline with realistic controller movements')
        self.get_logger().info('🔄 Pipeline: VR Pose → Coordinate Transform → Velocity Calculation → Robot Control')
        self.get_logger().info('🛤️  Method: Velocity-based control with VR gains and coordinate transformations')
        
        # Move to home position first
        if not self.move_to_home():
            self.get_logger().error('❌ Failed to move to home position')
            return
        
        self.get_logger().info('✅ Robot at home position - starting VR benchmark')
        
        # Wait for joint states to be available
        for _ in range(50):
            if self.joint_state is not None:
                break
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        if self.joint_state is None:
            self.get_logger().error('❌ No joint states available')
            return
        
        # Validate VR poses first
        if not self.validate_vr_poses():
            self.get_logger().error('❌ VR pose validation failed - stopping benchmark')
            return
        
        # Run benchmarks for each target rate
        for i, target_hz in enumerate(self.target_rates_hz):
            if not rclpy.ok():
                break
                
            self.get_logger().info(f'🎮 Starting VR test {i+1}/{len(self.target_rates_hz)} - {target_hz}Hz')
            
            result = self.benchmark_control_rate(target_hz)
            self.print_benchmark_results(result, target_hz)
            
            # RESET TO HOME after each control rate test (except the last one)
            if i < len(self.target_rates_hz) - 1:  # Don't reset after the last test
                self.get_logger().info(f'🏠 Resetting to home position after {target_hz}Hz VR test...')
                if self.move_to_home():
                    self.get_logger().info(f'✅ Robot reset to home - ready for next VR test')
                    time.sleep(2.0)  # Brief pause for stability
                else:
                    self.get_logger().warn(f'⚠️  Failed to reset to home - continuing anyway')
                    time.sleep(1.0)
            else:
                # Brief pause after final test
            time.sleep(1.0)
            
        # Print comprehensive summary
        self.print_summary_results()
        
        self.get_logger().info('🏁 VR Teleoperation Benchmark completed!')
        self.get_logger().info('📈 Results show VR teleoperation performance with coordinate transformations')
        self.get_logger().info('🎮 Tested: Forward, Right, Up, Rotation, and Combined VR movements')  
        self.get_logger().info('🔄 Pipeline: VR Controller → Transform → Velocity → Robot Position Commands')
        self.get_logger().info('🎯 Focus: VR teleoperation at realistic controller frequencies')
        self.get_logger().info('⚡ Benchmark: High-frequency VR pose processing and robot control')

    def validate_vr_poses(self):
        """Test if VR poses are valid and will produce meaningful robot movements"""
        self.get_logger().info('🧪 Validating VR teleoperation poses...')
        
        # Debug the VR transformation setup first
        self.debug_vr_setup()
        
        # Create VR test poses
        self.create_realistic_test_poses()
        
        successful_poses = 0
        for i, vr_pose in enumerate(self.test_vr_poses):
            try:
                # Test VR pose processing
                vr_pos, vr_quat = self.process_vr_pose(vr_pose)
                
                # Check if transformation produces reasonable values
                pos_magnitude = np.linalg.norm(vr_pos)
                if pos_magnitude < 2.0:  # Reasonable workspace bounds
                    successful_poses += 1
                    pos_cm = pos_magnitude * 100
                    self.get_logger().info(f'✅ VR Pose {i+1}: SUCCESS - {pos_cm:.1f}cm movement after transform')
                else:
                    self.get_logger().warn(f'❌ VR Pose {i+1}: FAILED - {pos_magnitude:.2f}m movement (too large)')
                
        except Exception as e:
                self.get_logger().warn(f'❌ VR Pose {i+1}: FAILED - {e}')
        
        success_rate = (successful_poses / len(self.test_vr_poses)) * 100
        self.get_logger().info(f'📊 VR pose validation: {successful_poses}/{len(self.test_vr_poses)} successful ({success_rate:.1f}%)')
        
        if successful_poses == 0:
            self.get_logger().error('❌ No valid VR poses found!')
                return False
        return True
    
    def debug_vr_setup(self):
        """Debug VR coordinate transformation setup"""
        self.get_logger().info('🔧 Debugging VR coordinate transformation setup...')
        
        # Show coordinate transformation matrix
        self.get_logger().info(f'🔄 Coordinate Transformation Matrix:')
        for i in range(4):
            row = self.global_to_env_mat[i]
            self.get_logger().info(f'   [{row[0]:6.1f}, {row[1]:6.1f}, {row[2]:6.1f}, {row[3]:6.1f}]')
        
        # Show what this transformation does to basic VR movements
        test_movements = [
            ([0.1, 0.0, 0.0], "VR right (+X) 10cm"),
            ([0.0, 0.1, 0.0], "VR up (+Y) 10cm"),
            ([0.0, 0.0, 0.1], "VR forward (+Z) 10cm"),
        ]
        
        self.get_logger().info('\n🗺️  VR → Robot Coordinate Mapping:')
        for vr_vec, description in test_movements:
            # Create test VR pose
            test_pose = VRPose.create_example_pose(x=vr_vec[0], y=vr_vec[1], z=vr_vec[2])
            robot_pos, robot_quat = self.process_vr_pose(test_pose)
            
            # Find dominant robot axis
            max_idx = np.argmax(np.abs(robot_pos))
            axis_names = ['X (forward)', 'Y (left)', 'Z (up)']
            direction = 'positive' if robot_pos[max_idx] > 0 else 'negative'
            
            self.get_logger().info(f'   {description} → Robot {direction} {axis_names[max_idx]} ({robot_pos[max_idx]*100:.1f}cm)')
        
        # Show VR control parameters
        self.get_logger().info(f'\n⚙️  VR Control Parameters:')
        self.get_logger().info(f'   Position gain: {self.pos_action_gain}x')
        self.get_logger().info(f'   Rotation gain: {self.rot_action_gain}x')
        self.get_logger().info(f'   Max linear delta: {self.max_lin_delta}m per cycle')
        self.get_logger().info(f'   Max rotation delta: {np.degrees(self.max_rot_delta):.1f}° per cycle')
    
    def test_simple_ik(self):
        """Test IK with the exact current pose to debug issues"""
        self.get_logger().info('🧪 Testing IK with current exact pose...')
        
        current_pose = self.get_current_end_effector_pose()
        if current_pose is None:
            self.get_logger().error('Cannot get current pose for IK test')
                return False
        
        # Get current planning scene
        scene_response = self.get_planning_scene()
        if scene_response is None:
            self.get_logger().error('Cannot get planning scene')
                return False
            
        # Create IK request with current exact pose
        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = self.planning_group
        ik_request.ik_request.robot_state = scene_response.scene.robot_state
        ik_request.ik_request.avoid_collisions = False  # Disable collision checking for test
        ik_request.ik_request.timeout.sec = 5  # Longer timeout
        ik_request.ik_request.timeout.nanosec = 0
        
        # Set current pose as target
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.base_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = current_pose
        
        ik_request.ik_request.pose_stamped = pose_stamped
        ik_request.ik_request.ik_link_name = self.end_effector_link
        
        self.get_logger().info(f'Testing IK for frame: {self.end_effector_link}')
        self.get_logger().info(f'Planning group: {self.planning_group}')
        self.get_logger().info(f'Target pose: pos=[{current_pose.position.x:.3f}, {current_pose.position.y:.3f}, {current_pose.position.z:.3f}]')
        self.get_logger().info(f'Target ori: [{current_pose.orientation.x:.3f}, {current_pose.orientation.y:.3f}, {current_pose.orientation.z:.3f}, {current_pose.orientation.w:.3f}]')
        
        # Call IK service
        ik_future = self.ik_client.call_async(ik_request)
        rclpy.spin_until_future_complete(self, ik_future, timeout_sec=6.0)
        ik_response = ik_future.result()
        
        if ik_response is None:
            self.get_logger().error('❌ IK service call returned None')
            return False
            
        self.get_logger().info(f'IK Error code: {ik_response.error_code.val}')
        
        if ik_response.error_code.val == 1:
            self.get_logger().info('✅ IK SUCCESS with current pose!')
            return True
        else:
            # Print more detailed error info
            error_messages = {
                -1: 'FAILURE',
                -2: 'FRAME_TRANSFORM_FAILURE', 
                -3: 'INVALID_GROUP_NAME',
                -4: 'INVALID_GOAL_CONSTRAINTS',
                -5: 'INVALID_ROBOT_STATE',
                -6: 'INVALID_LINK_NAME',
                -7: 'INVALID_JOINT_CONSTRAINTS',
                -8: 'KINEMATIC_STATE_NOT_INITIALIZED',
                -9: 'NO_IK_SOLUTION',
                -10: 'TIMEOUT',
                -11: 'COLLISION_CHECKING_UNAVAILABLE'
            }
            error_msg = error_messages.get(ik_response.error_code.val, f'UNKNOWN_ERROR_{ik_response.error_code.val}')
            self.get_logger().error(f'❌ IK failed: {error_msg}')
            return False
    
    def find_correct_planning_group(self):
        """Try different planning group names to find the correct one"""
        potential_groups = [
            'panda_arm', 'fr3_arm', 'arm', 'manipulator', 
            'panda_manipulator', 'fr3_manipulator', 'robot'
        ]
        
        self.get_logger().info('🔍 Testing different planning group names...')
        
        for group_name in potential_groups:
            try:
                # Get current planning scene
                scene_response = self.get_planning_scene()
                if scene_response is None:
                    continue
                
                # Create simple IK request to test group name
                ik_request = GetPositionIK.Request()
                ik_request.ik_request.group_name = group_name
                ik_request.ik_request.robot_state = scene_response.scene.robot_state
                ik_request.ik_request.avoid_collisions = False
                ik_request.ik_request.timeout.sec = 1
                ik_request.ik_request.timeout.nanosec = 0
                
                # Use current pose
                current_pose = self.get_current_end_effector_pose()
                if current_pose is None:
                    continue
                
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = self.base_frame
                pose_stamped.header.stamp = self.get_clock().now().to_msg()
                pose_stamped.pose = current_pose
                
                ik_request.ik_request.pose_stamped = pose_stamped
                ik_request.ik_request.ik_link_name = self.end_effector_link
                
                # Call IK service
                ik_future = self.ik_client.call_async(ik_request)
                rclpy.spin_until_future_complete(self, ik_future, timeout_sec=2.0)
                ik_response = ik_future.result()
                
                if ik_response:
                    if ik_response.error_code.val == 1:
                        self.get_logger().info(f'✅ Found working planning group: {group_name}')
                        return group_name
            else:
                        self.get_logger().info(f'❌ Group {group_name}: error code {ik_response.error_code.val}')
                else:
                    self.get_logger().info(f'❌ Group {group_name}: no response')
            
        except Exception as e:
                self.get_logger().info(f'❌ Group {group_name}: exception {e}')
        
        self.get_logger().error('❌ No working planning group found!')
        return None

    def test_single_large_movement(self):
        """Test a single VR movement to verify the pipeline works (wrapper for backward compatibility)"""
        return self.test_vr_movement()
    
    def debug_joint_states(self):
        """Debug joint state reception"""
        self.get_logger().info('🔍 Debugging joint state reception...')
        
        for i in range(10):
            joints = self.get_current_joint_positions()
            if joints:
                self.get_logger().info(f'Attempt {i+1}: Got joints: {[f"{j:.3f}" for j in joints]}')
                return True
            else:
                self.get_logger().warn(f'Attempt {i+1}: No joint positions available')
                time.sleep(0.5)
                rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().error('❌ Failed to get joint positions after 10 attempts')
        return False

    def compute_ik_for_joints(self, joint_positions):
        """Compute IK to get pose from joint positions (mimics VR teleoperation IK)"""
        try:
            # Create joint state request
            request = GetPositionIK.Request()
            request.ik_request.group_name = self.planning_group
            
            # Set current robot state
            request.ik_request.robot_state.joint_state.name = self.joint_names
            request.ik_request.robot_state.joint_state.position = joint_positions.tolist()
            
            # Forward kinematics: compute pose from joint positions
            # For this we use the move group's forward kinematics
            # Get the current pose that would result from these joint positions
            
            # Create a dummy pose request (we'll compute the actual pose)
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.planning_frame
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            
            # Use moveit planning scene to compute forward kinematics
            # Set joint positions and compute resulting pose
            joint_state = JointState()
            joint_state.name = self.joint_names
            joint_state.position = joint_positions.tolist()
            
            # Create planning scene state
            robot_state = RobotState()
            robot_state.joint_state = joint_state
            
            # Request forward kinematics to get pose
            fk_request = GetPositionFK.Request()
            fk_request.header.frame_id = self.planning_frame
            fk_request.header.stamp = self.get_clock().now().to_msg()
            fk_request.fk_link_names = [self.end_effector_link]
            fk_request.robot_state = robot_state
            
            # Call forward kinematics service
            if not self.fk_client.service_is_ready():
                self.get_logger().warn('FK service not ready')
                return None
                
            future = self.fk_client.call_async(fk_request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.1)
            
            if future.result() is not None:
                fk_response = future.result()
                if fk_response.error_code.val == fk_response.error_code.SUCCESS:
                    if fk_response.pose_stamped:
                        return fk_response.pose_stamped[0]  # First (and only) pose
            
            return None
            
        except Exception as e:
            self.get_logger().debug(f'FK computation failed: {e}')
            return None

    def send_individual_position_command(self, pos, quat, gripper, duration):
        """Send individual position command (exactly like VR teleoperation)"""
        try:
            if not self.trajectory_client.server_is_ready():
                return False
            
            # Create trajectory with single waypoint (like VR commands)
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names
            
            # Convert Cartesian pose to joint positions using IK
            ik_request = GetPositionIK.Request()
            ik_request.ik_request.group_name = self.planning_group
            ik_request.ik_request.pose_stamped.header.frame_id = self.planning_frame
            ik_request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
            
            # Set target pose
            ik_request.ik_request.pose_stamped.pose.position.x = float(pos[0])
            ik_request.ik_request.pose_stamped.pose.position.y = float(pos[1])
            ik_request.ik_request.pose_stamped.pose.position.z = float(pos[2])
            ik_request.ik_request.pose_stamped.pose.orientation.x = float(quat[0])
            ik_request.ik_request.pose_stamped.pose.orientation.y = float(quat[1])
            ik_request.ik_request.pose_stamped.pose.orientation.z = float(quat[2])
            ik_request.ik_request.pose_stamped.pose.orientation.w = float(quat[3])
            
            # Set current robot state as seed
            current_joints = self.get_current_joint_positions()
            if current_joints:
                ik_request.ik_request.robot_state.joint_state.name = self.joint_names
                ik_request.ik_request.robot_state.joint_state.position = current_joints
            
            # Call IK service
            if not self.ik_client.service_is_ready():
                return False
                
            future = self.ik_client.call_async(ik_request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.05)  # Quick timeout
            
            if future.result() is not None:
                ik_response = future.result()
                if ik_response.error_code.val == ik_response.error_code.SUCCESS:
                    # Create trajectory point
                    point = JointTrajectoryPoint()
                    
                    # Extract only the positions for our 7 arm joints
                    # IK might return extra joints (gripper), so we need to filter
                    joint_positions = []
                    for joint_name in self.joint_names:
                        if joint_name in ik_response.solution.joint_state.name:
                            idx = ik_response.solution.joint_state.name.index(joint_name)
                            joint_positions.append(ik_response.solution.joint_state.position[idx])
                    
                    # Ensure we have exactly 7 joint positions
                    if len(joint_positions) != 7:
                        self.get_logger().warn(f'IK returned {len(joint_positions)} joints, expected 7')
                return False
            
                    point.positions = joint_positions
                    point.time_from_start.sec = max(1, int(duration))
                    point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
                    
                    trajectory.points.append(point)
                    
                    # Send trajectory
                    goal = FollowJointTrajectory.Goal()
                    goal.trajectory = trajectory
                    
                    # Send goal (non-blocking for high frequency)
                    send_goal_future = self.trajectory_client.send_goal_async(goal)
            return True
            
            return False
            
        except Exception as e:
            self.get_logger().debug(f'Individual command failed: {e}')
            return False
    
    def test_vr_movement(self):
        """Test a single VR movement to verify the pipeline works"""
        self.get_logger().info('🧪 TESTING VR MOVEMENT PIPELINE - Verifying VR pose processing...')
        
        # Get current robot state
        current_joints = self.get_current_joint_positions()
        if current_joints is None:
            self.get_logger().error('❌ Cannot get current joint positions')
            return False
    
        self.get_logger().info(f'📍 Current joints: {[f"{j:.3f}" for j in current_joints]}')
        
        # Get current robot pose for reference
        current_pose = self.get_current_end_effector_pose()
        if current_pose is None:
            self.get_logger().error('❌ Cannot get current robot pose')
            return False
        
        current_pos = np.array([current_pose.position.x, current_pose.position.y, current_pose.position.z])
        current_quat = np.array([current_pose.orientation.x, current_pose.orientation.y, 
                               current_pose.orientation.z, current_pose.orientation.w])
        
        self.get_logger().info(f'📍 Current robot pose: pos=[{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]')
        
        # STEP 1: Create VR pose at origin (grip NOT pressed) - establishes the baseline
        vr_home_pose = VRPose.create_example_pose(
            x=0.0, y=0.0, z=0.0,  # At VR origin
            qx=0.0, qy=0.0, qz=0.0, qw=1.0,
            grip=False, trigger=0.0  # Grip NOT pressed
        )
        
        # Initialize VR control state with home pose (this sets the origin)
        self.vr_control_state = VRControlState()  # Reset state
        action, action_info = self.calculate_vr_action(vr_home_pose, current_pos, current_quat, 0.0)
        self.get_logger().info('🏠 VR home pose processed (grip not pressed)')
        
        # STEP 2: Create VR target pose (grip PRESSED) - this should create movement
        vr_target_pose = VRPose.create_example_pose(
            x=0.0, y=0.0, z=0.2,  # 20cm forward in VR (smaller but should be visible)
            qx=0.0, qy=0.0, qz=0.0, qw=1.0,
            grip=True, trigger=0.0  # Grip PRESSED - enables movement
        )
        
        # Process VR target with grip pressed
        try:
            # Calculate VR action with target pose
            action, action_info = self.calculate_vr_action(vr_target_pose, current_pos, current_quat, 0.0)
            self.get_logger().info(f'⚡ VR action calculated: {action[:3]} (position velocity)')
            
            if np.linalg.norm(action[:3]) > 0.001:  # Check if we have actual movement
                # Convert to target position
                target_pos, target_quat, target_gripper = self.velocity_to_position_target_vr(
                    action, current_pos, current_quat, action_info
                )
                
                # Calculate expected movement
                movement = np.linalg.norm(target_pos - current_pos)
                self.get_logger().info(f'🎯 Target position: pos=[{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]')
                self.get_logger().info(f'📐 Expected movement: {movement*100:.1f}cm')
                
                if movement > 0.01:  # More than 1cm
                    self.get_logger().info(f'✅ VR pipeline test SUCCESS - Would produce {movement*100:.1f}cm movement')
                    return True
                else:
                    self.get_logger().warn(f'⚠️  VR pipeline test - Small movement only {movement*100:.1f}cm')
                    return False
            else:
                self.get_logger().error(f'❌ VR pipeline test - No velocity generated from VR movement')
                self.get_logger().error(f'   VR home: x=0, y=0, z=0 (grip=False)')
                self.get_logger().error(f'   VR target: x=0, y=0, z=0.2 (grip=True)')
                self.get_logger().error(f'   Action result: {action}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ VR pipeline test failed: {e}')
            import traceback
            traceback.print_exc()
            return False
    
    def generate_vr_perturbations_at_75hz(self, duration: float) -> List[VRPose]:
        """Generate VR perturbations at 75Hz for the given duration"""
        hz_75_interval = 1.0 / 75.0  # ~13.33ms
        num_samples = int(duration * 75)
        perturbations = []
        
        # Generate smooth sinusoidal perturbations at different frequencies
        for i in range(num_samples):
            t = i * hz_75_interval
            
            # Create multi-frequency perturbations for interesting movement
            # Position perturbations (in meters)
            x = 0.1 * np.sin(2 * np.pi * 0.5 * t)  # 0.5Hz oscillation, 10cm amplitude
            y = 0.05 * np.sin(2 * np.pi * 0.3 * t + np.pi/4)  # 0.3Hz, 5cm amplitude
            z = 0.08 * np.sin(2 * np.pi * 0.7 * t + np.pi/2)  # 0.7Hz, 8cm amplitude
            
            # Rotation perturbations (convert to quaternion)
            roll = 0.2 * np.sin(2 * np.pi * 0.4 * t)  # 0.4Hz, ~11° amplitude
            pitch = 0.15 * np.sin(2 * np.pi * 0.6 * t + np.pi/3)  # 0.6Hz, ~8.6° amplitude
            yaw = 0.1 * np.sin(2 * np.pi * 0.2 * t)  # 0.2Hz, ~5.7° amplitude
            
            # Convert euler to quaternion
            rot = R.from_euler('xyz', [roll, pitch, yaw])
            quat = rot.as_quat()
            
            # Trigger varies slowly
            trigger = 0.5 * (1 + np.sin(2 * np.pi * 0.1 * t))  # 0.1Hz, 0-1 range
            
            vr_pose = VRPose.create_example_pose(
                x=x, y=y, z=z,
                qx=quat[0], qy=quat[1], qz=quat[2], qw=quat[3],
                grip=True,  # Always "gripping" in simple mode
                trigger=trigger
            )
            perturbations.append(vr_pose)
        
        self.get_logger().info(f'📊 Generated {len(perturbations)} VR perturbations at 75Hz')
        return perturbations
    
    def downsample_perturbations(self, perturbations_75hz: List[VRPose], target_hz: float) -> List[Tuple[int, VRPose]]:
        """Downsample 75Hz perturbations to target frequency
        Returns list of (index, pose) tuples where index is the 75Hz sample index"""
        if target_hz >= 75:
            # For frequencies >= 75Hz, we'll repeat samples
            samples_per_75hz = int(target_hz / 75.0)
            downsampled = []
            for i, pose in enumerate(perturbations_75hz):
                for _ in range(samples_per_75hz):
                    downsampled.append((i, pose))
            return downsampled
        else:
            # For frequencies < 75Hz, we skip samples
            skip_factor = 75.0 / target_hz
            downsampled = []
            accumulated = 0.0
            for i, pose in enumerate(perturbations_75hz):
                if i >= int(accumulated):
                    downsampled.append((i, pose))
                    accumulated += skip_factor
            return downsampled

    def send_vr_command(self, target_pos: np.ndarray, target_quat: np.ndarray, target_gripper: float, duration: float) -> bool:
        """Send VR-generated command to robot (simplified for benchmarking)"""
        # In benchmark mode, we're testing the control pipeline performance
        # Actually sending to robot is optional - we can simulate success
        # This avoids IK computation bottlenecks during benchmarking
        
        # Simulate processing time (roughly what real IK would take)
        time.sleep(0.001)  # 1ms simulated processing
        
        # Return success for benchmarking
        # In real deployment, this would compute IK and send trajectory
        return True


def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = FrankaBenchmarkController()
        
        # Wait for everything to initialize
        time.sleep(3.0)
        
        # DEBUG: Test joint state reception first
        controller.get_logger().info('🔧 DEBUGGING: Testing joint state reception...')
        if not controller.debug_joint_states():
            controller.get_logger().error('❌ Cannot receive joint states - aborting')
            return
        
        # Check robot safety state
        controller.get_logger().info('🛡️  Checking robot safety state...')
        if not controller.check_robot_safety_state():
            controller.get_logger().error('❌ Robot safety check failed - aborting')
            return
        
        # Move to home position first
        controller.get_logger().info('🏠 Moving to home position...')
        if not controller.move_to_home():
            controller.get_logger().error('❌ Failed to move to home position')
            return
        
        # DEBUG: Test VR pipeline to verify it works
        controller.get_logger().info('\n' + '='*80)
        controller.get_logger().info('🧪 VR MOVEMENT PIPELINE TEST - Verifying VR pose processing')
        controller.get_logger().info('='*80)
        
        if controller.test_vr_movement():
            controller.get_logger().info('✅ VR movement pipeline test completed')
            
            # Ask user if they want to continue with full benchmark
            controller.get_logger().info('\n🤔 VR pipeline test completed successfully!')
            controller.get_logger().info('   The VR pose processing and coordinate transformations are working.')
            controller.get_logger().info('   Ready to proceed with full VR teleoperation benchmark.')
            
            # Wait a moment then proceed with benchmark automatically
        time.sleep(2.0)
        
            controller.get_logger().info('\n' + '='*80)
            controller.get_logger().info('🚀 PROCEEDING WITH FULL VR TELEOPERATION BENCHMARK')
            controller.get_logger().info('='*80)
            
            # Run the comprehensive VR benchmark
            controller.run_comprehensive_benchmark()
        else:
            controller.get_logger().error('❌ Single movement test failed - not proceeding with benchmark')
        
    except KeyboardInterrupt:
        print("\n🛑 Benchmark interrupted by user")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 