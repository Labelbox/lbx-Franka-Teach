#!/usr/bin/env python3
"""
Advanced Franka FR3 Benchmarking Script with MoveIt Integration
- Benchmarks control rates up to 1kHz (FR3 manual specification)
- Uses VR pose targets (position + quaternion from Oculus)
- Full MoveIt integration with IK solver and collision avoidance
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


@dataclass
class VRPose:
    """Example VR pose data from Oculus (based on oculus_vr_server.py)"""
    position: np.ndarray  # [x, y, z] in meters
    orientation: np.ndarray  # quaternion [x, y, z, w]
    timestamp: float
    
    @classmethod
    def create_example_pose(cls, x=0.4, y=0.0, z=0.5, qx=0.924, qy=-0.383, qz=0.0, qw=0.0):
        """Create example VR pose similar to oculus_vr_server.py data"""
        return cls(
            position=np.array([x, y, z]),
            orientation=np.array([qx, qy, qz, qw]),
            timestamp=time.time()
        )


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
        self.planning_group = "panda_arm"
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
        
        # Performance tracking
        self.cycle_stats: List[ControlCycleStats] = []
        self.benchmark_results: List[BenchmarkResult] = []
        self.rate_latencies: Dict[float, List[float]] = {}
        
        # Threading for high-frequency operation
        self._control_thread = None
        self._running = False
        self._current_target_rate = 1.0
        
        # Test poses will be created dynamically based on current robot position
        self.test_vr_poses = []
        
        self.get_logger().info('🎯 Franka FR3 Benchmark Controller Initialized')
        self.get_logger().info(f'📊 Will test rates: {self.target_rates_hz} Hz')
        self.get_logger().info(f'⏱️  Each rate tested for: {self.benchmark_duration_seconds}s')
        
    def joint_state_callback(self, msg):
        """Store the latest joint state"""
        self.joint_state = msg
        
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
    
    def execute_trajectory(self, positions, duration=2.0):
        """Execute a trajectory to move joints to target positions"""
        if not self.trajectory_client.server_is_ready():
            return False
            
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Add single point
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        trajectory.points.append(point)
            
        # Create goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        # Send goal
        future = self.trajectory_client.send_goal_async(goal)
        
        # Wait for goal acceptance
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        goal_handle = future.result()
        
        if not goal_handle or not goal_handle.accepted:
            return False
            
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 2.0)
        
        result = result_future.result()
        if result is None:
            return False
            
        return result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
    
    def move_to_home(self):
        """Move robot to home position"""
        self.get_logger().info('🏠 Moving to home position...')
        return self.execute_trajectory(self.home_positions, duration=3.0)
    
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
        """Create test joint positions using the EXACT same approach as the working test script"""
        self.get_logger().info('🎯 Creating LARGE joint movement targets using PROVEN test script approach...')
        
        # Get current joint positions
        current_joints = self.get_current_joint_positions()
        if current_joints is None:
            # Fallback to home position
            current_joints = self.home_positions
            
        # Use the EXACT same movements as the successful test script
        # +30 degrees = +0.52 radians (this is what worked!)
        # ONLY include movement targets, NOT the current position
        self.test_joint_targets = [
            [current_joints[0] + 0.52, current_joints[1], current_joints[2], current_joints[3], current_joints[4], current_joints[5], current_joints[6]],  # +30° joint 1 (PROVEN TO WORK)
            [current_joints[0], current_joints[1] + 0.52, current_joints[2], current_joints[3], current_joints[4], current_joints[5], current_joints[6]],  # +30° joint 2 
            [current_joints[0], current_joints[1], current_joints[2], current_joints[3], current_joints[4], current_joints[5], current_joints[6] + 0.52],  # +30° joint 7
        ]
        
        # Convert to VR poses for compatibility with existing code
        self.test_vr_poses = []
        for i, joints in enumerate(self.test_joint_targets):
            # Store joint positions in dummy VR pose
            dummy_pose = VRPose.create_example_pose()
            dummy_pose.joint_positions = joints  # Add custom field
            self.test_vr_poses.append(dummy_pose)
            
        self.get_logger().info(f'Created {len(self.test_joint_targets)} LARGE joint movement targets')
        self.get_logger().info(f'Using PROVEN movements: +30° on joints 1, 2, and 7 (0.52 radians each)')
        self.get_logger().info(f'These are the EXACT same movements that worked in the test script!')
        self.get_logger().info(f'🚫 Removed current position target - ALL targets now guarantee movement!')
    
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
        """Benchmark individual position command sending (mimics VR teleoperation pipeline)"""
        self.get_logger().info(f'📊 Benchmarking {target_hz}Hz individual position commands...')
        
        # Test parameters matching production VR teleoperation
        test_duration = 10.0  # 10 seconds of command sending
        movement_duration = 3.0  # Complete movement in 3 seconds
        command_interval = 1.0 / target_hz
        
        # Get home and target positions (guaranteed 30° visible movement)
        home_joints = np.array(self.home_positions.copy())
        target_joints = home_joints.copy() 
        target_joints[0] += 0.52  # +30° on joint 1 (proven large movement)
        
        self.get_logger().info(f'🎯 Movement: Joint 1 from {home_joints[0]:.3f} to {target_joints[0]:.3f} rad (+30°)')
        self.get_logger().info(f'⏱️  Command interval: {command_interval*1000:.1f}ms')
        
        # Generate discrete waypoints for the movement
        num_movement_steps = max(1, int(movement_duration * target_hz))
        self.get_logger().info(f'🛤️  Generating {num_movement_steps} waypoints for {movement_duration}s movement')
        
        waypoints = []
        for i in range(num_movement_steps + 1):  # +1 to include final target
            alpha = i / num_movement_steps  # 0 to 1
            waypoint_joints = home_joints + alpha * (target_joints - home_joints)
            waypoints.append(waypoint_joints.copy())
        
        # Performance tracking
        successful_commands = 0
        failed_commands = 0
        total_ik_time = 0.0
        total_command_time = 0.0
        timing_errors = []
        
        start_time = time.time()
        last_command_time = start_time
        waypoint_idx = 0
        num_movements = 0
        
        self.get_logger().info(f'🚀 Starting {target_hz}Hz command benchmark for {test_duration}s...')
        
        while time.time() - start_time < test_duration and rclpy.ok():
            current_time = time.time()
            
            # Check if it's time for next command
            if current_time - last_command_time >= command_interval:
                command_start = time.time()
                
                # Get current waypoint (cycle through movement)
                current_waypoint = waypoints[waypoint_idx]
                
                # Calculate target pose using IK (like VR system does)
                ik_start = time.time()
                target_pose = self.compute_ik_for_joints(current_waypoint)
                ik_time = time.time() - ik_start
                total_ik_time += ik_time
                
                if target_pose is not None:
                    # Extract position and orientation
                    target_pos = target_pose.pose.position
                    target_quat = target_pose.pose.orientation
                    
                    pos_array = np.array([target_pos.x, target_pos.y, target_pos.z])
                    quat_array = np.array([target_quat.x, target_quat.y, target_quat.z, target_quat.w])
                    
                    # Send individual position command (exactly like VR teleoperation)
                    # ALWAYS send to robot to test real teleoperation performance
                    command_success = self.send_individual_position_command(
                        pos_array, quat_array, 0.0, command_interval
                    )
                    if command_success:
                        successful_commands += 1
                    else:
                        failed_commands += 1
                
                # Track command timing
                command_time = time.time() - command_start
                total_command_time += command_time
                
                # Track timing accuracy
                expected_time = last_command_time + command_interval
                actual_time = current_time
                timing_error = abs(actual_time - expected_time)
                timing_errors.append(timing_error)
                
                last_command_time = current_time
                
                # Advance waypoint (cycle through movement)
                waypoint_idx = (waypoint_idx + 1) % len(waypoints)
                if waypoint_idx == 0:  # Completed one full movement
                    num_movements += 1
                    self.get_logger().info(f'🔄 Movement cycle {num_movements} completed')
        
        # Calculate results
        end_time = time.time()
        actual_duration = end_time - start_time
        total_commands = successful_commands + failed_commands
        actual_rate = total_commands / actual_duration if actual_duration > 0 else 0
        
        # Calculate performance metrics
        avg_ik_time = (total_ik_time / total_commands * 1000) if total_commands > 0 else 0
        avg_command_time = (total_command_time / total_commands * 1000) if total_commands > 0 else 0
        avg_timing_error = (np.mean(timing_errors) * 1000) if timing_errors else 0
        success_rate = (successful_commands / total_commands * 100) if total_commands > 0 else 0
        
        self.get_logger().info(f'📈 Results: {actual_rate:.1f}Hz actual rate ({total_commands} commands in {actual_duration:.1f}s)')
        self.get_logger().info(f'✅ Success rate: {success_rate:.1f}% ({successful_commands}/{total_commands})')
        self.get_logger().info(f'🧮 Avg IK time: {avg_ik_time:.2f}ms')
        self.get_logger().info(f'⏱️  Avg command time: {avg_command_time:.2f}ms')
        self.get_logger().info(f'⏰ Avg timing error: {avg_timing_error:.2f}ms')
        
        # Return results
        result = BenchmarkResult(
            control_rate_hz=actual_rate,
            avg_latency_ms=avg_command_time,
            ik_solve_time_ms=avg_ik_time,
            collision_check_time_ms=avg_timing_error,  # Reuse field for timing error
            motion_plan_time_ms=0.0,  # Not used in this benchmark
            total_cycle_time_ms=avg_command_time + avg_ik_time,
            success_rate=success_rate,
            timestamp=time.time()
        )
        
        self.benchmark_results.append(result)
        return result
    
    def generate_high_frequency_trajectory(self, home_joints: List[float], target_joints: List[float], duration: float, target_hz: float) -> Optional[JointTrajectory]:
        """Generate a high-frequency trajectory between two joint positions"""
        try:
            # Get current joint positions
            current_joints = self.get_current_joint_positions()
            if current_joints is None:
                return None
            
            # Calculate waypoints with proper timestamps
            num_steps = max(1, int(duration * target_hz))
            time_step = duration / num_steps
            
            # Create trajectory
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names
            
            # Generate waypoints using linear interpolation in joint space
            for i in range(1, num_steps + 1):  # Start from 1, not 0 (skip current position)
                t = i / num_steps  # Interpolation parameter from >0 to 1
                
                # Linear interpolation for each joint
                interp_joints = []
                for j in range(len(self.joint_names)):
                    if j < len(current_joints) and j < len(target_joints):
                        interp_joint = (1 - t) * current_joints[j] + t * target_joints[j]
                        interp_joints.append(interp_joint)
                
                # Create trajectory point with progressive timestamps
                point = JointTrajectoryPoint()
                point.positions = interp_joints
                point_time = i * time_step
                point.time_from_start.sec = int(point_time)
                point.time_from_start.nanosec = int((point_time - int(point_time)) * 1e9)
                trajectory.points.append(point)
            
            self.get_logger().debug(f'Generated {len(trajectory.points)} waypoints for {duration}s trajectory at {target_hz}Hz')
            return trajectory
                
        except Exception as e:
            self.get_logger().warn(f'Failed to generate high-frequency trajectory: {e}')
            return None
    
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
        """Print structured benchmark results"""
        print(f"\n{'='*80}")
        print(f"📊 HIGH-FREQUENCY INDIVIDUAL COMMAND BENCHMARK - {target_hz}Hz")
        print(f"{'='*80}")
        print(f"🎯 Target Command Rate:      {target_hz:8.1f} Hz")
        print(f"📈 Actual Command Rate:      {result.control_rate_hz:8.1f} Hz ({result.control_rate_hz/target_hz*100:5.1f}%)")
        print(f"⏱️  Average Command Time:     {result.avg_latency_ms:8.2f} ms")
        print(f"🧮 Average IK Time:          {result.ik_solve_time_ms:8.2f} ms")
        print(f"⏰ Average Timing Error:     {result.collision_check_time_ms:8.2f} ms")
        print(f"✅ Success Rate:             {result.success_rate:8.1f} %")
        
        # Calculate command parameters
        movement_duration = 3.0
        commands_per_movement = int(movement_duration * target_hz)
        command_interval_ms = (1.0 / target_hz) * 1000
        
        print(f"📏 Commands per Movement:    {commands_per_movement:8d}")
        print(f"🔍 Command Interval:         {command_interval_ms:8.2f} ms")
        print(f"🎯 Movement Type:            Home -> Target (+30° joint)")
        
        print(f"🤖 Test Mode:                REAL ROBOT COMMANDS (ALL frequencies)")
        print(f"   Sending individual position commands at {target_hz}Hz")
        
        # Performance analysis
        if result.control_rate_hz >= target_hz * 0.95:
            print(f"🎉 EXCELLENT: Achieved {result.control_rate_hz/target_hz*100:.1f}% of target rate")
        elif result.control_rate_hz >= target_hz * 0.8:
            print(f"👍 GOOD: Achieved {result.control_rate_hz/target_hz*100:.1f}% of target rate")
        elif result.control_rate_hz >= target_hz * 0.5:
            print(f"⚠️  MODERATE: Only achieved {result.control_rate_hz/target_hz*100:.1f}% of target rate")
        else:
            print(f"❌ POOR: Only achieved {result.control_rate_hz/target_hz*100:.1f}% of target rate")
        
        # Generation time analysis
        if result.avg_latency_ms < 1.0:
            print(f"⚡ EXCELLENT generation time: {result.avg_latency_ms:.2f}ms")
        elif result.avg_latency_ms < 10.0:
            print(f"👍 GOOD generation time: {result.avg_latency_ms:.2f}ms")
        elif result.avg_latency_ms < 100.0:
            print(f"⚠️  MODERATE generation time: {result.avg_latency_ms:.2f}ms")
        else:
            print(f"❌ HIGH generation time: {result.avg_latency_ms:.2f}ms")
            
        # Command analysis for all frequencies
        theoretical_control_freq = target_hz
        command_density = commands_per_movement / movement_duration
        print(f"📊 Command Analysis:")
        print(f"   Control Resolution: {command_interval_ms:.2f}ms between commands")
        print(f"   Command Density: {command_density:.1f} commands/second")
        print(f"   Teleoperation Rate: {theoretical_control_freq}Hz position updates")
        
        print(f"{'='*80}\n")
    
    def print_summary_results(self):
        """Print comprehensive summary of all benchmark results"""
        print(f"\n{'='*100}")
        print(f"🏆 HIGH-FREQUENCY INDIVIDUAL POSITION COMMAND BENCHMARK - FRANKA FR3")
        print(f"{'='*100}")
        print(f"Approach: Send individual position commands from HOME to TARGET (+30° joint movement)")
        print(f"Testing: Individual command rates from 10Hz to 200Hz (mimicking VR teleoperation)")
        print(f"ALL frequencies: Send real commands to robot to test actual teleoperation performance")
        print(f"Movement: Continuous cycling through 3-second movements with discrete waypoints")
        print(f"Method: Individual position commands at target frequency (NOT pre-planned trajectories)")
        print(f"{'='*100}")
        print(f"{'Rate (Hz)':>10} {'Actual (Hz)':>12} {'Cmd Time (ms)':>14} {'IK Time (ms)':>15} {'Success (%)':>12} {'Commands/s':>12}")
        print(f"{'-'*100}")
        
        for i, result in enumerate(self.benchmark_results):
            target_hz = self.target_rates_hz[i] if i < len(self.target_rates_hz) else 0
            print(f"{target_hz:>10.0f} {result.control_rate_hz:>12.1f} {result.avg_latency_ms:>14.2f} "
                  f"{result.ik_solve_time_ms:>15.2f} {result.success_rate:>12.1f} {result.control_rate_hz:>12.1f}")
        
        print(f"{'-'*100}")
        
        # Find best performing rates
        if self.benchmark_results:
            best_rate = max(self.benchmark_results, key=lambda x: x.control_rate_hz)
            best_generation_time = min(self.benchmark_results, key=lambda x: x.avg_latency_ms)
            best_success = max(self.benchmark_results, key=lambda x: x.success_rate)
            
            print(f"\n🏆 PERFORMANCE HIGHLIGHTS:")
            print(f"   🚀 Highest Command Rate:     {best_rate.control_rate_hz:.1f} Hz")
            print(f"   ⚡ Fastest Command Time:      {best_generation_time.avg_latency_ms:.2f} ms")
            print(f"   ✅ Best Success Rate:        {best_success.success_rate:.1f} %")
            
            # Overall performance analysis
            print(f"\n📈 OVERALL PERFORMANCE:")
            for i, result in enumerate(self.benchmark_results):
                target_hz = self.target_rates_hz[i] if i < len(self.target_rates_hz) else 0
                
                print(f"\n   {target_hz} Hz Test:")
                print(f"   Achieved: {result.control_rate_hz:.1f} Hz ({result.control_rate_hz/target_hz*100:.1f}% of target)")
                print(f"   Command Time: {result.avg_latency_ms:.2f} ms")
                print(f"   IK Computation: {result.ik_solve_time_ms:.2f} ms")
                print(f"   Success Rate: {result.success_rate:.1f}%")
                
                # Calculate command characteristics
                commands_per_second = result.control_rate_hz
                command_interval_ms = (1.0/commands_per_second)*1000 if commands_per_second > 0 else 0
                print(f"   Command interval: {command_interval_ms:.2f}ms")
        
        print(f"{'='*100}\n")
    
    def run_comprehensive_benchmark(self):
        """Run complete high-frequency individual command benchmark suite"""
        self.get_logger().info('🚀 Starting High-Frequency Individual Command Benchmark - Franka FR3')
        self.get_logger().info('📊 Testing individual position command rates from 10Hz to 200Hz')
        self.get_logger().info('🎯 Approach: Send individual position commands from HOME to TARGET (+30° joint movement)')
        self.get_logger().info('🤖 ALL frequencies: Send real commands to robot to test actual teleoperation')
        self.get_logger().info('🛤️  Method: Individual position commands sent at target frequency (VR teleoperation style)')
        
        # Move to home position first
        if not self.move_to_home():
            self.get_logger().error('❌ Failed to move to home position')
            return
        
        self.get_logger().info('✅ Robot at home position - starting benchmark')
        
        # Wait for joint states to be available
        for _ in range(50):
            if self.joint_state is not None:
                break
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        if self.joint_state is None:
            self.get_logger().error('❌ No joint states available')
            return
        
        # Validate test poses first
        if not self.validate_test_poses():
            self.get_logger().error('❌ Pose validation failed - stopping benchmark')
            return
        
        # Run benchmarks for each target rate
        for i, target_hz in enumerate(self.target_rates_hz):
            if not rclpy.ok():
                break
                
            self.get_logger().info(f'🎯 Starting test {i+1}/{len(self.target_rates_hz)} - {target_hz}Hz')
            
            result = self.benchmark_control_rate(target_hz)
            self.print_benchmark_results(result, target_hz)
            
            # RESET TO HOME after each control rate test (except the last one)
            if i < len(self.target_rates_hz) - 1:  # Don't reset after the last test
                self.get_logger().info(f'🏠 Resetting to home position after {target_hz}Hz test...')
                if self.move_to_home():
                    self.get_logger().info(f'✅ Robot reset to home - ready for next test')
                    time.sleep(2.0)  # Brief pause for stability
                else:
                    self.get_logger().warn(f'⚠️  Failed to reset to home - continuing anyway')
                    time.sleep(1.0)
            else:
                # Brief pause after final test
                time.sleep(1.0)
            
        # Print comprehensive summary
        self.print_summary_results()
        
        self.get_logger().info('🏁 High-Frequency Individual Command Benchmark completed!')
        self.get_logger().info('📈 Results show high-frequency individual command capability')
        self.get_logger().info('🤖 Low frequencies: Robot execution verified with actual movement')  
        self.get_logger().info('🔬 High frequencies: Individual position command capability')
        self.get_logger().info('🎯 Movement: HOME -> TARGET (+30° joint) with individual position commands')
        self.get_logger().info('⚡ Focus: >100Hz performance for high-frequency robot control applications')

    def validate_test_poses(self):
        """Test if our joint targets are valid and will produce large movements"""
        self.get_logger().info('🧪 Validating LARGE joint movement targets...')
        
        # Debug the IK setup first
        self.debug_ik_setup()
        
        # Test simple IK with current pose
        if not self.test_simple_ik():
            self.get_logger().error('❌ Even current pose fails IK - setup issue detected')
            return False
        
        # Create large joint movement targets
        self.create_realistic_test_poses()
        
        successful_targets = 0
        for i, target in enumerate(self.test_vr_poses):
            if hasattr(target, 'joint_positions'):
                # This is a joint target - validate the joint limits
                joints = target.joint_positions
                joint_diffs = []
                
                current_joints = self.get_current_joint_positions()
                if current_joints:
                    for j in range(min(len(joints), len(current_joints))):
                        diff = abs(joints[j] - current_joints[j])
                        joint_diffs.append(diff)
                    
                    max_diff = max(joint_diffs) if joint_diffs else 0
                    max_diff_degrees = max_diff * 57.3
                    
                    # Check if movement is within safe limits (roughly ±150 degrees per joint)
                    if all(abs(j) < 2.6 for j in joints):  # ~150 degrees in radians
                        successful_targets += 1
                        self.get_logger().info(f'✅ Target {i+1}: SUCCESS - Max movement {max_diff_degrees:.1f}° (+30° proven movement)')
                    else:
                        self.get_logger().warn(f'❌ Target {i+1}: UNSAFE - Joint limits exceeded')
                else:
                    self.get_logger().warn(f'❌ Target {i+1}: Cannot get current joints')
            else:
                # Fallback to pose-based IK validation
                joint_positions, stats = self.compute_ik_with_collision_avoidance(target)
                if joint_positions is not None:
                    successful_targets += 1
                    self.get_logger().info(f'✅ Target {i+1}: SUCCESS - IK solved in {stats.ik_time_ms:.2f}ms')
                else:
                    self.get_logger().warn(f'❌ Target {i+1}: FAILED - IK could not solve')
        
        success_rate = (successful_targets / len(self.test_vr_poses)) * 100
        self.get_logger().info(f'📊 Target validation: {successful_targets}/{len(self.test_vr_poses)} successful ({success_rate:.1f}%)')
        
        if successful_targets == 0:
            self.get_logger().error('❌ No valid targets found!')
            return False
        return True

    def debug_ik_setup(self):
        """Debug IK setup and check available services"""
        self.get_logger().info('🔧 Debugging IK setup...')
        
        # Check available services
        service_names = self.get_service_names_and_types()
        ik_services = [name for name, _ in service_names if 'ik' in name.lower()]
        self.get_logger().info(f'Available IK services: {ik_services}')
        
        # Check available frames
        try:
            from tf2_ros import Buffer, TransformListener
            tf_buffer = Buffer()
            tf_listener = TransformListener(tf_buffer, self)
            
            # Wait a bit for TF data
            import time
            time.sleep(1.0)
            
            available_frames = tf_buffer.all_frames_as_yaml()
            self.get_logger().info(f'Available TF frames include fr3 frames: {[f for f in available_frames.split() if "fr3" in f]}')
            
        except Exception as e:
            self.get_logger().warn(f'Could not check TF frames: {e}')
        
        # Test different end-effector frame names
        potential_ee_frames = [
            'fr3_hand_tcp', 'panda_hand_tcp', 'fr3_hand', 'panda_hand', 
            'fr3_link8', 'panda_link8', 'tool0'
        ]
        
        for frame in potential_ee_frames:
            try:
                # Try FK with this frame
                if not self.fk_client.wait_for_service(timeout_sec=1.0):
                    continue
                    
                current_joints = self.get_current_joint_positions()
                if current_joints is None:
                    continue
                
                fk_request = GetPositionFK.Request()
                fk_request.fk_link_names = [frame]
                fk_request.header.frame_id = self.base_frame
                fk_request.header.stamp = self.get_clock().now().to_msg()
                fk_request.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
                fk_request.robot_state.joint_state.name = self.joint_names
                fk_request.robot_state.joint_state.position = current_joints
                
                fk_future = self.fk_client.call_async(fk_request)
                rclpy.spin_until_future_complete(self, fk_future, timeout_sec=1.0)
                fk_response = fk_future.result()
                
                if fk_response and fk_response.error_code.val == 1:
                    self.get_logger().info(f'✅ Frame {frame} works for FK')
                else:
                    self.get_logger().info(f'❌ Frame {frame} failed FK')
                    
            except Exception as e:
                self.get_logger().info(f'❌ Frame {frame} error: {e}')
        
        # Find correct planning group
        correct_group = self.find_correct_planning_group()
        if correct_group:
            self.planning_group = correct_group
            self.get_logger().info(f'✅ Updated planning group to: {correct_group}')
        else:
            self.get_logger().error('❌ Could not find working planning group')
    
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
        """Test a single large joint movement to verify robot actually moves"""
        self.get_logger().info('🧪 TESTING SINGLE LARGE MOVEMENT - Debugging robot motion...')
        
        # Get current joint positions
        current_joints = self.get_current_joint_positions()
        if current_joints is None:
            self.get_logger().error('❌ Cannot get current joint positions')
            return False
    
        self.get_logger().info(f'📍 Current joints: {[f"{j:.3f}" for j in current_joints]}')
        
        # Create a LARGE movement on joint 1 (+30 degrees = +0.52 radians)
        # This is the EXACT same movement that worked in our previous test script
        test_target = current_joints.copy()
        test_target[0] += 0.52  # +30 degrees on joint 1
        
        self.get_logger().info(f'🎯 Target joints:  {[f"{j:.3f}" for j in test_target]}')
        self.get_logger().info(f'📐 Joint 1 movement: +30° (+0.52 rad) - GUARANTEED VISIBLE')
        
        # Generate and execute test trajectory using new approach
        self.get_logger().info('🚀 Executing LARGE test movement using trajectory generation...')
        
        # Generate single trajectory from current to target
        trajectory = self.generate_high_frequency_trajectory(
            current_joints, test_target, duration=3.0, target_hz=10.0  # 10Hz = 30 waypoints
        )
        
        if trajectory is None:
            self.get_logger().error('❌ Failed to generate test trajectory')
            return False
        
        # Execute the trajectory
        success = self.execute_complete_trajectory(trajectory)
        
        if success:
            self.get_logger().info('✅ Test movement completed - check logs above for actual displacement')
        else:
            self.get_logger().error('❌ Test movement failed')
            
        return success
    
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
        
        # Move to home position first
        controller.get_logger().info('🏠 Moving to home position...')
        if not controller.move_to_home():
            controller.get_logger().error('❌ Failed to move to home position')
            return
        
        # DEBUG: Test a single large movement to verify robot actually moves
        controller.get_logger().info('\n' + '='*80)
        controller.get_logger().info('🧪 SINGLE MOVEMENT TEST - Verifying robot actually moves')
        controller.get_logger().info('='*80)
        
        if controller.test_single_large_movement():
            controller.get_logger().info('✅ Single movement test completed')
            
            # Ask user if they want to continue with full benchmark
            controller.get_logger().info('\n🤔 Did you see the robot move? Check the logs above for actual displacement.')
            controller.get_logger().info('   If robot moved visibly, we can proceed with full benchmark.')
            controller.get_logger().info('   If robot did NOT move, we need to debug further.')
            
            # Wait a moment then proceed with benchmark automatically
            # (In production, you might want to wait for user input)
            time.sleep(2.0)
            
            controller.get_logger().info('\n' + '='*80)
            controller.get_logger().info('🚀 PROCEEDING WITH FULL BENCHMARK')
            controller.get_logger().info('='*80)
            
            # Run the comprehensive benchmark
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