#!/usr/bin/env python3
"""
Simplified VR Benchmark using MoveIt move_group action
- Bypasses trajectory controller issues
- Uses MoveIt's built-in planning and execution
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions, Constraints
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
import numpy as np
import time
from dataclasses import dataclass
from typing import List, Tuple
from scipy.spatial.transform import Rotation as R


@dataclass
class VRPose:
    """VR pose data"""
    position: np.ndarray  # [x, y, z]
    orientation: np.ndarray  # quaternion [x, y, z, w]
    timestamp: float
    trigger_value: float = 0.0


@dataclass
class BenchmarkResult:
    """Benchmark results"""
    target_hz: float
    actual_hz: float
    avg_command_time_ms: float
    success_rate: float


def vec_to_reorder_mat(vec):
    """Convert reordering vector to transformation matrix"""
    X = np.zeros((len(vec), len(vec)))
    for i in range(X.shape[0]):
        ind = int(abs(vec[i])) - 1
        X[i, ind] = np.sign(vec[i])
    return X


class VRBenchmarkMoveItNode(Node):
    def __init__(self):
        super().__init__('vr_benchmark_moveit')
        
        # Robot configuration
        self.planning_group = "fr3_arm"
        self.end_effector_link = "fr3_hand_tcp"
        self.base_frame = "fr3_link0"
        self.joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]
        self.home_positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        
        # VR control parameters
        self.pos_action_gain = 5.0  # Reduced for safety
        self.rot_action_gain = 3.0  # Reduced for safety
        
        # Coordinate transformation
        rmat_reorder = [-2, -1, 3, 4]
        self.global_to_env_mat = vec_to_reorder_mat(rmat_reorder)
        
        # Test frequencies - reduced for safety
        self.target_rates_hz = [5, 10, 20]  # Conservative rates
        
        # Create MoveGroup action client
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Joint state subscriber
        self.joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        # Wait for move_group action server
        self.get_logger().info('Waiting for move_group action server...')
        if self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().info('Move group action server ready!')
        else:
            self.get_logger().error('Move group action server not available!')
            return
        
        # Store results
        self.benchmark_results = []
        
    def joint_state_callback(self, msg):
        self.joint_state = msg
        
    def get_current_joint_positions(self):
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
    
    def generate_vr_perturbations(self, duration: float, hz: float) -> List[VRPose]:
        """Generate simple, safe VR perturbations"""
        interval = 1.0 / hz
        num_samples = int(duration * hz)
        perturbations = []
        
        for i in range(num_samples):
            t = i * interval
            
            # Very small, safe perturbations
            x = 0.02 * np.sin(2 * np.pi * 0.2 * t)      # 2cm @ 0.2Hz - very safe
            y = 0.015 * np.sin(2 * np.pi * 0.15 * t)    # 1.5cm @ 0.15Hz - gentle  
            z = 0.01 * np.sin(2 * np.pi * 0.3 * t)      # 1cm @ 0.3Hz - minimal
            
            # Very small rotation perturbations
            roll = 0.05 * np.sin(2 * np.pi * 0.1 * t)   # ~3° @ 0.1Hz - very gentle
            pitch = 0.03 * np.sin(2 * np.pi * 0.12 * t) # ~1.7° @ 0.12Hz - minimal
            yaw = 0.02 * np.sin(2 * np.pi * 0.08 * t)   # ~1.1° @ 0.08Hz - tiny
            
            # Convert to quaternion
            rot = R.from_euler('xyz', [roll, pitch, yaw])
            quat = rot.as_quat()
            
            vr_pose = VRPose(
                position=np.array([x, y, z]),
                orientation=quat,
                timestamp=t,
                trigger_value=0.5
            )
            perturbations.append(vr_pose)
        
        self.get_logger().info(f'Generated {len(perturbations)} safe perturbations at {hz}Hz')
        return perturbations
    
    def create_move_group_goal(self, target_pose: Pose) -> MoveGroup.Goal:
        """Create a MoveGroup action goal"""
        goal = MoveGroup.Goal()
        
        # Set up the motion plan request
        goal.request.group_name = self.planning_group
        goal.request.num_planning_attempts = 1  # Fast planning
        goal.request.allowed_planning_time = 1.0  # Quick timeout
        goal.request.max_velocity_scaling_factor = 0.1  # Very slow and safe
        goal.request.max_acceleration_scaling_factor = 0.1  # Very slow and safe
        
        # Set the target pose
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = self.base_frame
        pose_goal.header.stamp = self.get_clock().now().to_msg()
        pose_goal.pose = target_pose
        
        # Add pose constraint
        from moveit_msgs.msg import PositionConstraint, OrientationConstraint
        constraints = Constraints()
        
        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header = pose_goal.header
        pos_constraint.link_name = self.end_effector_link
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0
        constraints.position_constraints = [pos_constraint]
        
        goal.request.goal_constraints = [constraints]
        
        # Planning options
        goal.planning_options.plan_only = False  # Plan and execute
        goal.planning_options.look_around = False
        goal.planning_options.look_around_attempts = 0
        goal.planning_options.max_safe_execution_cost = 1.0
        goal.planning_options.replan = False
        goal.planning_options.replan_attempts = 0
        
        return goal
    
    def send_pose_command(self, target_pos: np.ndarray, target_quat: np.ndarray) -> bool:
        """Send pose command using MoveGroup action"""
        try:
            # Create target pose
            target_pose = Pose()
            target_pose.position.x = float(target_pos[0])
            target_pose.position.y = float(target_pos[1]) 
            target_pose.position.z = float(target_pos[2])
            target_pose.orientation.x = float(target_quat[0])
            target_pose.orientation.y = float(target_quat[1])
            target_pose.orientation.z = float(target_quat[2])
            target_pose.orientation.w = float(target_quat[3])
            
            # Create and send goal
            goal = self.create_move_group_goal(target_pose)
            
            # Send goal (non-blocking)
            future = self.move_group_client.send_goal_async(goal)
            
            # Quick check if goal was accepted
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.1)
            
            if future.done():
                goal_handle = future.result()
                if goal_handle and goal_handle.accepted:
                    return True
            
            return False
            
        except Exception as e:
            self.get_logger().debug(f'Pose command failed: {e}')
            return False
    
    def benchmark_control_rate(self, target_hz: float) -> BenchmarkResult:
        """Benchmark VR teleoperation at target frequency"""
        self.get_logger().info(f'\n🎮 Testing {target_hz}Hz with MoveGroup action')
        
        # Generate perturbations
        test_duration = 5.0  # Shorter test
        perturbations = self.generate_vr_perturbations(test_duration, target_hz)
        
        # Base robot pose (conservative workspace position)
        base_pos = np.array([0.3, 0.0, 0.4])  # Safe position
        base_quat = np.array([0.0, 0.0, 0.0, 1.0])  # Identity rotation
        
        # Tracking
        successful_commands = 0
        failed_commands = 0
        command_times = []
        
        command_interval = 1.0 / target_hz
        start_time = time.time()
        
        # Process each pose
        for i, vr_pose in enumerate(perturbations):
            # Timing
            expected_time = start_time + (i * command_interval)
            current_time = time.time()
            if current_time < expected_time:
                time.sleep(expected_time - current_time)
            
            command_start = time.time()
            
            try:
                # Apply VR transformation
                vr_mat = np.eye(4)
                vr_mat[:3, 3] = vr_pose.position
                vr_mat[:3, :3] = R.from_quat(vr_pose.orientation).as_matrix()
                
                # Transform to robot coordinates
                transformed_mat = self.global_to_env_mat @ vr_mat
                transformed_pos = transformed_mat[:3, 3]
                transformed_quat = R.from_matrix(transformed_mat[:3, :3]).as_quat()
                
                # Calculate target pose with small perturbation
                target_pos = base_pos + (transformed_pos * self.pos_action_gain)
                target_rot = R.from_quat(base_quat) * R.from_quat(transformed_quat)
                target_quat = target_rot.as_quat()
                
                # Send command
                success = self.send_pose_command(target_pos, target_quat)
                
                if success:
                    successful_commands += 1
                else:
                    failed_commands += 1
                
            except Exception as e:
                self.get_logger().debug(f'Error: {e}')
                failed_commands += 1
            
            command_times.append(time.time() - command_start)
            
            # Progress update
            if i > 0 and i % max(1, int(target_hz/2)) == 0:
                elapsed = time.time() - start_time
                actual_rate = i / elapsed
                self.get_logger().info(f'   Progress: {i}/{len(perturbations)}, {actual_rate:.1f}Hz')
        
        # Calculate results
        actual_duration = time.time() - start_time
        total_commands = successful_commands + failed_commands
        actual_rate = total_commands / actual_duration
        
        avg_command_time = np.mean(command_times) * 1000
        success_rate = (successful_commands / total_commands * 100) if total_commands > 0 else 0
        
        self.get_logger().info(f'   ✅ Results:')
        self.get_logger().info(f'      Actual rate: {actual_rate:.1f}Hz ({actual_rate/target_hz*100:.1f}%)')
        self.get_logger().info(f'      Command time: {avg_command_time:.2f}ms')
        self.get_logger().info(f'      Success rate: {success_rate:.1f}% ({successful_commands}/{total_commands})')
        
        return BenchmarkResult(
            target_hz=target_hz,
            actual_hz=actual_rate,
            avg_command_time_ms=avg_command_time,
            success_rate=success_rate
        )
    
    def run_benchmark(self):
        """Run complete benchmark suite"""
        self.get_logger().info('🚀 Starting Safe VR Teleoperation Benchmark with MoveGroup')
        self.get_logger().info('   Using MoveGroup action for reliable execution')
        self.get_logger().info('   Conservative speeds to avoid violations')
        
        # Wait for joint states
        for _ in range(10):
            if self.joint_state is not None:
                break
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        if self.joint_state is None:
            self.get_logger().error('No joint states received! Is the robot running?')
            return
        
        self.get_logger().info(f'📍 Current robot joints: {[f"{j:.3f}" for j in self.get_current_joint_positions()]}')
        
        # Run tests
        for target_hz in self.target_rates_hz:
            result = self.benchmark_control_rate(target_hz)
            self.benchmark_results.append(result)
            time.sleep(2.0)  # Pause between tests
        
        # Print summary
        self.print_summary()
        
    def print_summary(self):
        """Print benchmark summary"""
        print(f"\n{'='*80}")
        print(f"🏆 SAFE VR TELEOPERATION BENCHMARK SUMMARY")
        print(f"{'='*80}")
        print(f"{'Target Hz':>10} {'Actual Hz':>12} {'Achievement':>12} {'Success':>10} {'Cmd Time':>12}")
        print(f"{'-'*80}")
        
        for result in self.benchmark_results:
            achievement = result.actual_hz / result.target_hz * 100
            print(f"{result.target_hz:>10.0f} {result.actual_hz:>12.1f} {achievement:>11.1f}% "
                  f"{result.success_rate:>9.1f}% {result.avg_command_time_ms:>11.2f}ms")
        
        print(f"{'='*80}\n")
        
        if self.benchmark_results:
            best_rate = max(self.benchmark_results, key=lambda x: x.actual_hz)
            print(f"🏆 HIGHLIGHTS:")
            print(f"   Highest rate: {best_rate.actual_hz:.1f}Hz (target: {best_rate.target_hz}Hz)")
            print(f"   Using MoveGroup action for safety and reliability")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = VRBenchmarkMoveItNode()
        node.run_benchmark()
        
    except KeyboardInterrupt:
        print("\n🛑 Benchmark interrupted")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 