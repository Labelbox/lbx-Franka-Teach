#!/usr/bin/env python3
"""
Franka Controller Module

Handles all robot control operations via MoveIt, preserving the exact
control pipeline from oculus_vr_server_moveit.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
import time
from typing import Dict, Optional, List
from scipy.spatial.transform import Rotation as R

# ROS 2 messages
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPlanningScene, GetPositionFK
from moveit_msgs.msg import PositionIKRequest, RobotState as MoveitRobotState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from franka_msgs.action import Grasp
from std_msgs.msg import Header

# ROS 2 Diagnostics
from diagnostic_updater import Updater, DiagnosticTask, DiagnosticStatus
from diagnostic_msgs.msg import KeyValue


class FrankaControllerTask(DiagnosticTask):
    """Diagnostic task for Franka robot controller status"""
    
    def __init__(self, name, controller):
        super().__init__(name)
        self.controller = controller
    
    def run(self, stat):
        """Run diagnostic check and report controller status"""
        # Check if services are available
        services_ready = self._check_services_availability()
        
        # Check robot connection status
        robot_connected = self.controller._last_joint_positions is not None
        
        # Overall health assessment
        if services_ready and robot_connected:
            stat.summary(DiagnosticStatus.OK, "Robot controller ready and operational")
        elif services_ready and not robot_connected:
            stat.summary(DiagnosticStatus.WARN, "Controller ready - waiting for robot data")
        elif not services_ready and robot_connected:
            stat.summary(DiagnosticStatus.ERROR, "Robot connected but MoveIt services unavailable")
        else:
            stat.summary(DiagnosticStatus.ERROR, "Robot controller not ready - services and robot unavailable")
        
        # Add detailed status information
        stat.add("Services Ready", str(services_ready))
        stat.add("Robot Connected", str(robot_connected))
        stat.add("Hardware Type", "Real Robot" if not self.controller.config.get('use_fake_hardware', False) else "Fake Hardware")
        
        # Performance metrics
        total_ik_attempts = self.controller._ik_success_count + self.controller._ik_failure_count
        if total_ik_attempts > 0:
            ik_success_rate = (self.controller._ik_success_count / total_ik_attempts) * 100
            stat.add("IK Success Rate (%)", f"{ik_success_rate:.1f}")
            stat.add("IK Total Attempts", str(total_ik_attempts))
        else:
            stat.add("IK Success Rate (%)", "N/A (No attempts)")
            stat.add("IK Total Attempts", "0")
        
        stat.add("Trajectory Commands Sent", str(self.controller._trajectory_success_count))
        
        # Service status details
        stat.add("IK Service", "Available" if self.controller.ik_client.service_is_ready() else "Unavailable")
        stat.add("Planning Scene Service", "Available" if self.controller.planning_scene_client.service_is_ready() else "Unavailable")
        stat.add("FK Service", "Available" if self.controller.fk_client.service_is_ready() else "Unavailable")
        stat.add("Trajectory Action", "Available" if self.controller.trajectory_client.server_is_ready() else "Unavailable")
        stat.add("Gripper Action", "Available" if self.controller.gripper_client.server_is_ready() else "Unavailable")
        
        # Configuration info
        stat.add("Planning Group", self.controller.config['robot']['planning_group'])
        stat.add("Base Frame", self.controller.config['robot']['base_frame'])
        stat.add("End Effector Link", self.controller.config['robot']['end_effector_link'])
        
        # Last command timing
        if self.controller._last_command_time > 0:
            time_since_last = time.time() - self.controller._last_command_time
            stat.add("Time Since Last Command (s)", f"{time_since_last:.1f}")
        else:
            stat.add("Time Since Last Command (s)", "Never")
        
        # Workspace bounds
        workspace_min = self.controller.config['robot']['workspace_min']
        workspace_max = self.controller.config['robot']['workspace_max']
        stat.add("Workspace X Range", f"[{workspace_min[0]:.2f}, {workspace_max[0]:.2f}]")
        stat.add("Workspace Y Range", f"[{workspace_min[1]:.2f}, {workspace_max[1]:.2f}]")
        stat.add("Workspace Z Range", f"[{workspace_min[2]:.2f}, {workspace_max[2]:.2f}]")
        
        return stat
    
    def _check_services_availability(self):
        """Check if all required MoveIt services are available"""
        return (self.controller.ik_client.service_is_ready() and
                self.controller.planning_scene_client.service_is_ready() and
                self.controller.fk_client.service_is_ready() and
                self.controller.trajectory_client.server_is_ready() and
                self.controller.gripper_client.server_is_ready())


class FrankaController:
    """Handles Franka robot control via MoveIt"""
    
    def __init__(self, node: Node, config: Dict):
        self.node = node
        self.config = config
        
        # Create MoveIt service clients
        self.ik_client = node.create_client(GetPositionIK, '/compute_ik')
        self.planning_scene_client = node.create_client(GetPlanningScene, '/get_planning_scene')
        self.fk_client = node.create_client(GetPositionFK, '/compute_fk')
        
        # Create action clients
        self.trajectory_client = ActionClient(
            node, FollowJointTrajectory, '/fr3_arm_controller/follow_joint_trajectory'
        )
        self.gripper_client = ActionClient(
            node, Grasp, '/fr3_gripper/grasp'
        )
        
        # Wait for services
        self._wait_for_services()
        
        # State tracking
        self._last_joint_positions = None
        self._last_gripper_command = None
        self._last_command_time = 0.0
        self.joint_state = None
        
        # Smoothing
        self._smoothed_target_pos = None
        self._smoothed_target_quat = None
        self._smoothed_target_gripper = None
        
        # Statistics
        self._ik_success_count = 0
        self._ik_failure_count = 0
        self._trajectory_success_count = 0
        self._trajectory_failure_count = 0
        
        # Initialize diagnostics
        self.diagnostic_updater = Updater(node)
        self.diagnostic_updater.setHardwareID(f"franka_fr3_controller_{config.get('robot', {}).get('robot_ip', 'unknown')}")
        self.diagnostic_updater.add(FrankaControllerTask("Franka Controller Status", self))
        
        self.node.get_logger().info("✅ Franka controller initialized with diagnostics")
    
    def update_diagnostics(self):
        """Update diagnostic information - should be called periodically"""
        self.diagnostic_updater.update()
    
    def _wait_for_services(self):
        """Wait for MoveIt services to be available"""
        self.node.get_logger().info('🔄 Waiting for MoveIt services...')
        
        services_ready = True
        timeout = self.config['moveit']['service_wait_timeout_sec']
        
        if not self.ik_client.wait_for_service(timeout_sec=timeout):
            self.node.get_logger().error("❌ IK service not available")
            services_ready = False
        else:
            self.node.get_logger().info("✅ IK service ready")
        
        if not self.planning_scene_client.wait_for_service(timeout_sec=timeout):
            self.node.get_logger().error("❌ Planning scene service not available")
            services_ready = False
        else:
            self.node.get_logger().info("✅ Planning scene service ready")
        
        if not self.fk_client.wait_for_service(timeout_sec=timeout):
            self.node.get_logger().error("❌ FK service not available")
            services_ready = False
        else:
            self.node.get_logger().info("✅ FK service ready")
        
        if not self.trajectory_client.wait_for_server(timeout_sec=timeout):
            self.node.get_logger().error("❌ Trajectory action server not available")
            services_ready = False
        else:
            self.node.get_logger().info("✅ Trajectory action server ready")
        
        if not self.gripper_client.wait_for_server(timeout_sec=timeout):
            self.node.get_logger().error("❌ Gripper action server not available")
            services_ready = False
        else:
            self.node.get_logger().info("✅ Gripper action server ready")
        
        if services_ready:
            self.node.get_logger().info('✅ All MoveIt services ready!')
        else:
            raise RuntimeError("Required MoveIt services not available")
    
    def execute_command(self, action: np.ndarray, action_info: Dict, robot_state):
        """Execute robot command"""
        # Check command rate limiting
        current_time = time.time()
        if current_time - self._last_command_time < self.config['vr_control']['min_command_interval']:
            return
        
        # Convert velocity action to position target
        target_pos, target_quat, target_gripper = self.velocity_to_position_target(
            action, robot_state.pos, robot_state.quat, action_info
        )
        
        # Apply workspace bounds
        workspace_min = np.array(self.config['robot']['workspace_min'])
        workspace_max = np.array(self.config['robot']['workspace_max'])
        target_pos = np.clip(target_pos, workspace_min, workspace_max)
        
        # Apply smoothing if enabled
        if self.config['vr_control']['pose_smoothing_enabled']:
            target_pos, target_quat, target_gripper = self.smooth_pose_transition(
                target_pos, target_quat, target_gripper
            )
        
        # Compute IK
        joint_positions = self.compute_ik_for_pose(target_pos, target_quat)
        
        if joint_positions is not None:
            # Execute trajectory
            self.execute_single_point_trajectory(joint_positions)
            
            # Handle gripper - get direct trigger value from action_info
            # This matches the original implementation where gripper state 
            # is determined directly from trigger, not from velocity integration
            if 'trigger_value' in action_info:
                # Use the raw VR trigger value for gripper control
                trigger_value = action_info['trigger_value']
                gripper_state = self.config['constants']['GRIPPER_CLOSE'] if trigger_value > self.config['gripper']['trigger_threshold'] else self.config['constants']['GRIPPER_OPEN']
                if self._should_send_gripper_command(gripper_state):
                    self.execute_gripper_command(gripper_state)
            
            self._last_command_time = current_time
    
    def velocity_to_position_target(self, velocity_action, current_pos, current_quat, action_info):
        """Convert velocity action to position target
        
        Takes normalized velocity commands [-1, 1] and scales them to actual
        position deltas using max_delta parameters. This is the velocity scaling
        step before sending targets to MoveIt's IK solver.
        
        Args:
            velocity_action: 7D array of normalized velocities (lin_vel, rot_vel, gripper_vel)
            current_pos: Current end-effector position
            current_quat: Current end-effector orientation quaternion
            action_info: Dictionary with pre-computed target info
            
        Returns:
            target_pos: Target position for IK solver
            target_quat: Target orientation for IK solver
            target_gripper: Target gripper state
        """
        lin_vel = velocity_action[:3]
        rot_vel = velocity_action[3:6]
        gripper_vel = velocity_action[6]
        
        # Apply velocity limiting
        lin_vel_norm = np.linalg.norm(lin_vel)
        rot_vel_norm = np.linalg.norm(rot_vel)
        
        if lin_vel_norm > 1:
            lin_vel = lin_vel / lin_vel_norm
        if rot_vel_norm > 1:
            rot_vel = rot_vel / rot_vel_norm
        
        # Convert to position deltas
        pos_delta = lin_vel * self.config['vr_control']['max_lin_delta']
        rot_delta = rot_vel * self.config['vr_control']['max_rot_delta']
        
        target_pos = current_pos + pos_delta
        
        # Use pre-calculated target quaternion if available
        if action_info and "target_quaternion" in action_info:
            target_quat = action_info["target_quaternion"]
        else:
            rot_delta_quat = R.from_euler("xyz", rot_delta).as_quat()
            current_rot = R.from_quat(current_quat)
            delta_rot = R.from_quat(rot_delta_quat)
            target_rot = delta_rot * current_rot
            target_quat = target_rot.as_quat()
        
        # Calculate target gripper
        control_interval = 1.0 / self.config['vr_control']['control_hz']
        target_gripper = np.clip(
            self._smoothed_target_gripper + gripper_vel * control_interval if self._smoothed_target_gripper is not None else 0.0,
            0.0, 1.0
        )
        
        return target_pos, target_quat, target_gripper
    
    def smooth_pose_transition(self, target_pos, target_quat, target_gripper):
        """Apply exponential smoothing to robot poses"""
        # Initialize on first call
        if self._smoothed_target_pos is None:
            self._smoothed_target_pos = target_pos.copy()
            self._smoothed_target_quat = target_quat.copy()
            self._smoothed_target_gripper = target_gripper
            return target_pos, target_quat, target_gripper
        
        # Calculate motion speed for adaptive smoothing
        pos_delta = np.linalg.norm(target_pos - self._smoothed_target_pos)
        
        # Adaptive smoothing
        alpha = self.config['vr_control']['pose_smoothing_alpha']
        if self.config['vr_control']['adaptive_smoothing']:
            speed_factor = min(pos_delta * 100, 1.0)
            alpha = alpha * (1.0 - speed_factor * 0.5)
            alpha = max(alpha, 0.05)
        
        # Smooth position
        self._smoothed_target_pos = (alpha * target_pos + 
                                   (1.0 - alpha) * self._smoothed_target_pos)
        
        # SLERP quaternions
        current_rot = R.from_quat(self._smoothed_target_quat)
        target_rot = R.from_quat(target_quat)
        smoothed_rot = current_rot.inv() * target_rot
        smoothed_rotvec = smoothed_rot.as_rotvec()
        smoothed_rotvec *= alpha
        final_rot = current_rot * R.from_rotvec(smoothed_rotvec)
        self._smoothed_target_quat = final_rot.as_quat()
        
        # Smooth gripper
        gripper_delta = target_gripper - self._smoothed_target_gripper
        max_gripper_delta = self.config['vr_control']['max_gripper_smoothing_delta']
        gripper_delta = np.clip(gripper_delta, -max_gripper_delta, max_gripper_delta)
        self._smoothed_target_gripper = self._smoothed_target_gripper + gripper_delta
        
        return self._smoothed_target_pos, self._smoothed_target_quat, self._smoothed_target_gripper
    
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
        rclpy.spin_until_future_complete(self.node, scene_future, timeout_sec=self.config['moveit']['planning_scene_timeout_sec'])
        
        return scene_future.result()
    
    def compute_ik_for_pose(self, pos, quat):
        """Compute IK for Cartesian pose with enhanced debugging"""
        # Get planning scene
        scene_response = self.get_planning_scene()
        if scene_response is None:
            if self.config['debug']['debug_ik_failures']:
                self.node.get_logger().warn("Cannot get planning scene for IK")
            return None
        
        # Create IK request
        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = self.config['robot']['planning_group']
        ik_request.ik_request.robot_state = scene_response.scene.robot_state
        ik_request.ik_request.avoid_collisions = True
        ik_request.ik_request.timeout.sec = 0
        ik_request.ik_request.timeout.nanosec = int(self.config['moveit']['ik_timeout_sec'] * 1e9)
        ik_request.ik_request.attempts = self.config['moveit']['ik_attempts']
        
        # Set target pose
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.config['robot']['base_frame']
        pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        pose_stamped.pose.position.x = float(pos[0])
        pose_stamped.pose.position.y = float(pos[1])
        pose_stamped.pose.position.z = float(pos[2])
        pose_stamped.pose.orientation.x = float(quat[0])
        pose_stamped.pose.orientation.y = float(quat[1])
        pose_stamped.pose.orientation.z = float(quat[2])
        pose_stamped.pose.orientation.w = float(quat[3])
        
        ik_request.ik_request.pose_stamped = pose_stamped
        ik_request.ik_request.ik_link_name = self.config['robot']['end_effector_link']
        
        # Call IK service
        ik_start = time.time()
        ik_future = self.ik_client.call_async(ik_request)
        rclpy.spin_until_future_complete(self.node, ik_future, timeout_sec=0.2)
        ik_response = ik_future.result()
        ik_time = time.time() - ik_start
        
        if ik_response and ik_response.error_code.val == 1:
            # Success
            self._ik_success_count += 1
            
            # Extract joint positions for our 7 joints
            joint_positions = []
            for joint_name in self.config['robot']['joint_names']:
                if joint_name in ik_response.solution.joint_state.name:
                    idx = ik_response.solution.joint_state.name.index(joint_name)
                    joint_positions.append(ik_response.solution.joint_state.position[idx])
            
            if self.config['debug']['debug_comm_stats'] and ik_time > 0.05:
                self.node.get_logger().warn(f"Slow IK computation: {ik_time*1000:.1f}ms")
            
            return joint_positions if len(joint_positions) == 7 else None
        else:
            # Failure
            self._ik_failure_count += 1
            
            if self.config['debug']['debug_ik_failures']:
                error_code = ik_response.error_code.val if ik_response else "No response"
                self.node.get_logger().warn(f"IK failed: error_code={error_code}, time={ik_time*1000:.1f}ms")
                self.node.get_logger().warn(f"Target pose: pos=[{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}], "
                                          f"quat=[{quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f}]")
            
            return None
    
    def execute_single_point_trajectory(self, joint_positions):
        """Execute single-point trajectory (VR-style individual command) with optimized velocity limiting"""
        trajectory = JointTrajectory()
        trajectory.joint_names = self.config['robot']['joint_names']
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        # Keep execution time from config
        duration = self.config['moveit']['trajectory_duration_single_point']
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(duration * 1e9)
        
        # Add velocity profiles with smart limiting
        if self._last_joint_positions is not None:
            position_deltas = np.array(joint_positions) - np.array(self._last_joint_positions)
            # Calculate velocities for trajectory execution
            # At 45Hz, we need faster execution to keep up with commands
            smooth_velocities = position_deltas / duration
            smooth_velocities *= self.config['moveit']['velocity_scale_factor']
            
            # Apply per-joint velocity limiting
            max_velocity = self.config['moveit']['max_joint_velocity']
            for i in range(len(smooth_velocities)):
                if abs(smooth_velocities[i]) > max_velocity:
                    smooth_velocities[i] = max_velocity * np.sign(smooth_velocities[i])
            
            point.velocities = smooth_velocities.tolist()
        else:
            point.velocities = [0.0] * len(joint_positions)
        
        # Conservative acceleration limits - let MoveIt handle smoothing
        point.accelerations = [0.0] * len(joint_positions)
        
        trajectory.points.append(point)
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        # Path tolerances from config
        goal.path_tolerance = [
            JointTolerance(
                name=name,
                position=self.config['moveit']['path_tolerance_position'],
                velocity=self.config['moveit']['path_tolerance_velocity'],
                acceleration=self.config['moveit']['path_tolerance_acceleration']
            )
            for name in self.config['robot']['joint_names']
        ]
        
        # Store joint positions for next velocity calculation
        self._last_joint_positions = joint_positions
        
        # Send goal (non-blocking for high frequency)
        send_goal_future = self.trajectory_client.send_goal_async(goal)
        
        # Update statistics
        self._trajectory_success_count += 1
        
        return True
    
    def execute_trajectory(self, positions, duration=None):
        """Execute a trajectory to move joints to target positions and WAIT for completion"""
        if duration is None:
            duration = self.config['moveit']['trajectory_duration_reset']
        
        self.node.get_logger().info(f"🎯 Executing trajectory to target positions (duration: {duration}s)...")
        
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.config['robot']['joint_names']
        
        # Add single point
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        # Add zero velocities and accelerations for smooth stop at target
        point.velocities = [0.0] * len(self.config['robot']['joint_names'])
        point.accelerations = [0.0] * len(self.config['robot']['joint_names'])
        
        trajectory.points.append(point)
        
        # Create goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        # Goal tolerances from config
        goal.goal_tolerance = [
            JointTolerance(
                name=name,
                position=self.config['moveit']['goal_tolerance_position'],
                velocity=self.config['moveit']['goal_tolerance_velocity'],
                acceleration=self.config['moveit']['goal_tolerance_acceleration']
            )
            for name in self.config['robot']['joint_names']
        ]
        
        # Send goal and WAIT for completion
        self.node.get_logger().info("📤 Sending trajectory goal...")
        send_goal_future = self.trajectory_client.send_goal_async(goal)
        
        # Wait for goal to be accepted
        rclpy.spin_until_future_complete(self.node, send_goal_future, timeout_sec=2.0)
        
        if not send_goal_future.done():
            self.node.get_logger().error("❌ Failed to send goal (timeout)")
            return False
        
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.node.get_logger().error("❌ Goal was rejected")
            return False
        
        self.node.get_logger().info("✅ Goal accepted, waiting for completion...")
        
        # Wait for execution to complete
        result_future = goal_handle.get_result_async()
        
        # Monitor progress
        start_time = time.time()
        last_update = 0
        
        while not result_future.done():
            elapsed = time.time() - start_time
            if elapsed - last_update >= 2.0:  # Update every 2 seconds
                self.node.get_logger().info(f"   ⏱️  Executing... {elapsed:.1f}s elapsed")
                last_update = elapsed
            
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
            if elapsed > duration + 10.0:  # Give extra time for completion
                self.node.get_logger().error("❌ Trajectory execution timeout")
                return False
        
        # Get final result
        result = result_future.result()
        
        if result.result.error_code == 0:  # SUCCESS
            self.node.get_logger().info("✅ Trajectory execution completed successfully!")
            return True
        else:
            self.node.get_logger().error(f"❌ Trajectory execution failed with error code: {result.result.error_code}")
            return False
    
    def _should_send_gripper_command(self, desired_state):
        """Check if gripper command should be sent
        
        Matches original behavior where gripper tracking is reset when 
        movement is disabled, allowing fresh state detection on re-enable.
        """
        if self._last_gripper_command is None:
            return True
        return self._last_gripper_command != desired_state
    
    def execute_gripper_command(self, gripper_state, timeout=2.0, wait_for_completion=False):
        """Execute gripper command (open/close) using Franka gripper action"""
        if not self.gripper_client.server_is_ready():
            if self.config['debug']['debug_moveit']:
                self.node.get_logger().warn("Gripper action server not ready")
            return False
        
        # Create gripper action goal
        goal = Grasp.Goal()
        
        if gripper_state == self.config['constants']['GRIPPER_CLOSE']:
            # Close gripper
            goal.width = self.config['gripper']['close_width']
            goal.speed = self.config['gripper']['speed']
            goal.force = self.config['gripper']['grasp_force']
            goal.epsilon.inner = self.config['gripper']['epsilon_inner']
            goal.epsilon.outer = self.config['gripper']['epsilon_outer']
        else:
            # Open gripper
            goal.width = self.config['gripper']['open_width']
            goal.speed = self.config['gripper']['speed']
            goal.force = 0.0
            goal.epsilon.inner = self.config['gripper']['epsilon_inner']
            goal.epsilon.outer = self.config['gripper']['epsilon_outer']
        
        # Send goal
        send_goal_future = self.gripper_client.send_goal_async(goal)
        
        # Update tracking
        self._last_gripper_command = gripper_state
        
        # Only log during testing/reset
        if wait_for_completion and self.config['debug']['debug_moveit']:
            action_type = "CLOSE" if gripper_state == self.config['constants']['GRIPPER_CLOSE'] else "OPEN"
            self.node.get_logger().info(f"🔧 Gripper command sent: {action_type} (width: {goal.width}, force: {goal.force})")
        
        # Optionally wait for completion
        if wait_for_completion:
            # Wait for goal to be accepted
            rclpy.spin_until_future_complete(self.node, send_goal_future, timeout_sec=2.0)
            
            if not send_goal_future.done():
                self.node.get_logger().error(f"❌ Gripper goal send timeout")
                return False
            
            goal_handle = send_goal_future.result()
            
            if not goal_handle.accepted:
                self.node.get_logger().error(f"❌ Gripper goal was rejected")
                return False
            
            # Wait for execution to complete
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=timeout)
            
            if not result_future.done():
                self.node.get_logger().error(f"❌ Gripper execution timeout after {timeout}s")
                return False
            
            result = result_future.result()
            return result.result.success
        
        # For VR control, we don't wait for completion to maintain responsiveness
        return True
    
    def get_current_end_effector_pose(self, joint_positions):
        """Get current end-effector pose using forward kinematics"""
        if joint_positions is None:
            self.node.get_logger().warn("Cannot get end-effector pose without joint positions")
            return None, None
        
        # Create FK request
        fk_request = GetPositionFK.Request()
        fk_request.fk_link_names = [self.config['robot']['end_effector_link']]
        fk_request.header.frame_id = self.config['robot']['base_frame']
        fk_request.header.stamp = self.node.get_clock().now().to_msg()
        
        # Set robot state
        fk_request.robot_state.joint_state.header.stamp = self.node.get_clock().now().to_msg()
        fk_request.robot_state.joint_state.name = self.config['robot']['joint_names']
        fk_request.robot_state.joint_state.position = joint_positions
        
        # Call FK service with retries
        max_retries = 3
        for attempt in range(max_retries):
            try:
                fk_start = time.time()
                fk_future = self.fk_client.call_async(fk_request)
                
                # Wait for response
                rclpy.spin_until_future_complete(self.node, fk_future, timeout_sec=self.config['moveit']['fk_timeout_sec'])
                fk_time = time.time() - fk_start
                
                if not fk_future.done():
                    self.node.get_logger().warn(f"FK service timeout on attempt {attempt + 1}")
                    continue
                
                fk_response = fk_future.result()
                
                if fk_response and fk_response.error_code.val == 1 and fk_response.pose_stamped:
                    pose = fk_response.pose_stamped[0].pose
                    pos = np.array([pose.position.x, pose.position.y, pose.position.z])
                    quat = np.array([pose.orientation.x, pose.orientation.y, 
                                    pose.orientation.z, pose.orientation.w])
                    
                    if self.config['debug']['debug_moveit']:
                        self.node.get_logger().info(f"FK successful: pos=[{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
                    
                    return pos, quat
                else:
                    error_code = fk_response.error_code.val if fk_response else "No response"
                    self.node.get_logger().warn(f"FK failed with error code: {error_code} on attempt {attempt + 1}")
                    
            except Exception as e:
                self.node.get_logger().warn(f"FK attempt {attempt + 1} exception: {e}")
            
            if attempt < max_retries - 1:
                time.sleep(0.2)  # Wait before retry
        
        self.node.get_logger().error("FK failed after all retries")
        return None, None
    
    def get_current_gripper_state(self, joint_state_msg):
        """Get current gripper state from joint states"""
        # Look for gripper joint in joint states
        gripper_joints = ['fr3_finger_joint1', 'fr3_finger_joint2']
        gripper_position = 0.0
        
        for joint_name in gripper_joints:
            if joint_name in joint_state_msg.name:
                idx = joint_state_msg.name.index(joint_name)
                gripper_position = max(gripper_position, joint_state_msg.position[idx])
        
        # Convert joint position to gripper state
        # FR3 gripper: 0.0 = closed, ~0.04 = open
        return self.config['constants']['GRIPPER_OPEN'] if gripper_position > 0.02 else self.config['constants']['GRIPPER_CLOSE']
    
    def reset_robot(self):
        """Reset robot to home position using MoveIt trajectory"""
        self.node.get_logger().info("🔄 Resetting robot to home position...")
        
        # First, check if services are ready
        self.node.get_logger().info("🔍 Checking MoveIt services...")
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().warn("⚠️  IK service not ready")
        
        if not self.fk_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().warn("⚠️  FK service not ready")
        
        if not self.trajectory_client.server_is_ready():
            self.node.get_logger().warn("⚠️  Trajectory server not ready")
        
        if not self.gripper_client.server_is_ready():
            self.node.get_logger().warn("⚠️  Gripper service not ready")
        
        # Execute trajectory to home position
        self.node.get_logger().info(f"🏠 Moving robot to home position...")
        home_positions = self.config['robot']['home_positions']
        success = self.execute_trajectory(home_positions)
        
        if success:
            self.node.get_logger().info(f"✅ Robot successfully moved to home position!")
            # Give time for robot to settle
            time.sleep(1.0)
            
            # Test gripper functionality
            self.node.get_logger().info(f"🔧 Testing gripper functionality...")
            
            # Close gripper
            close_success = self.execute_gripper_command(
                self.config['constants']['GRIPPER_CLOSE'],
                timeout=3.0,
                wait_for_completion=True
            )
            if close_success:
                self.node.get_logger().info(f"   ✅ Gripper CLOSE completed successfully")
            else:
                self.node.get_logger().warn(f"   ❌ Gripper CLOSE command failed")
            
            # Open gripper
            open_success = self.execute_gripper_command(
                self.config['constants']['GRIPPER_OPEN'],
                timeout=3.0,
                wait_for_completion=True
            )
            if open_success:
                self.node.get_logger().info(f"   ✅ Gripper OPEN completed successfully")
            else:
                self.node.get_logger().warn(f"   ❌ Gripper OPEN command failed")
            
            # Reset smoothing state
            self._smoothed_target_pos = None
            self._smoothed_target_quat = None
            self._smoothed_target_gripper = None
            self._last_gripper_command = None
            
            return success
        else:
            self.node.get_logger().error(f"❌ Robot trajectory to home position failed")
            return False
    
    def emergency_stop(self):
        """Emergency stop - halt all motion"""
        self.node.get_logger().error("🛑 EMERGENCY STOP - Halting all robot motion!")
        
        # Cancel any active trajectory goals
        if self.trajectory_client and self.trajectory_client._goal_handle:
            cancel_future = self.trajectory_client._goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self.node, cancel_future, timeout_sec=1.0)
        
        # Stop gripper motion
        if self.gripper_client and self.gripper_client._goal_handle:
            cancel_future = self.gripper_client._goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self.node, cancel_future, timeout_sec=1.0)
        
        # Reset state
        self._last_joint_positions = None
        self._smoothed_target_pos = None
        self._smoothed_target_quat = None
        self._smoothed_target_gripper = None
        
        self.node.get_logger().info("✅ Emergency stop executed")
    
    def print_stats(self):
        """Print MoveIt communication statistics"""
        total_ik = self._ik_success_count + self._ik_failure_count
        total_traj = self._trajectory_success_count + self._trajectory_failure_count
        
        if total_ik > 0:
            ik_success_rate = (self._ik_success_count / total_ik) * 100
            self.node.get_logger().info(f"📊 MoveIt IK Stats: {ik_success_rate:.1f}% success ({self._ik_success_count}/{total_ik})")
        
        if total_traj > 0:
            traj_success_rate = (self._trajectory_success_count / total_traj) * 100
            self.node.get_logger().info(f"📊 Trajectory Stats: {traj_success_rate:.1f}% success ({self._trajectory_success_count}/{total_traj})") 