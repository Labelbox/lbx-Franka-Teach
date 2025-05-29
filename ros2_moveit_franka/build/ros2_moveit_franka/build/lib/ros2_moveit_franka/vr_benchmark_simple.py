#!/usr/bin/env python3
"""
Simplified VR Teleoperation Benchmark Script for Franka FR3
- Creates perturbations at 75Hz and downsamples to test rates
- No grip mode, just simple perturbations from home position
- Simulates robot responses for pure benchmark testing
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
import numpy as np
import time
from dataclasses import dataclass
from typing import List, Tuple, Dict
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
    avg_vr_processing_ms: float
    success_rate: float


def vec_to_reorder_mat(vec):
    """Convert reordering vector to transformation matrix"""
    X = np.zeros((len(vec), len(vec)))
    for i in range(X.shape[0]):
        ind = int(abs(vec[i])) - 1
        X[i, ind] = np.sign(vec[i])
    return X


class VRBenchmarkNode(Node):
    def __init__(self):
        super().__init__('vr_benchmark_simple')
        
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
        self.pos_action_gain = 10.0
        self.rot_action_gain = 5.0
        self.max_lin_vel = 1.0
        self.max_rot_vel = 1.0
        self.max_lin_delta = 0.15  # 15cm per cycle
        self.max_rot_delta = 0.3   # ~17 degrees per cycle
        
        # Coordinate transformation
        rmat_reorder = [-2, -1, 3, 4]
        self.global_to_env_mat = vec_to_reorder_mat(rmat_reorder)
        
        # Test frequencies
        self.target_rates_hz = [10, 50, 75, 100, 200]
        
        # Create service clients
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        
        # Create action client
        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory, '/fr3_arm_controller/follow_joint_trajectory'
        )
        
        # Joint state subscriber
        self.joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        # Wait for services
        self.get_logger().info('Waiting for services...')
        self.ik_client.wait_for_service(timeout_sec=5.0)
        self.fk_client.wait_for_service(timeout_sec=5.0)
        self.get_logger().info('Services ready!')
        
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
    
    def generate_vr_perturbations_at_75hz(self, duration: float) -> List[VRPose]:
        """Generate smooth sinusoidal perturbations at 75Hz"""
        hz_75_interval = 1.0 / 75.0
        num_samples = int(duration * 75)
        perturbations = []
        
        for i in range(num_samples):
            t = i * hz_75_interval
            
            # Position perturbations
            x = 0.1 * np.sin(2 * np.pi * 0.5 * t)      # 10cm @ 0.5Hz
            y = 0.05 * np.sin(2 * np.pi * 0.3 * t)     # 5cm @ 0.3Hz
            z = 0.08 * np.sin(2 * np.pi * 0.7 * t)     # 8cm @ 0.7Hz
            
            # Rotation perturbations
            roll = 0.2 * np.sin(2 * np.pi * 0.4 * t)   # ~11° @ 0.4Hz
            pitch = 0.15 * np.sin(2 * np.pi * 0.6 * t) # ~8.6° @ 0.6Hz
            yaw = 0.1 * np.sin(2 * np.pi * 0.2 * t)    # ~5.7° @ 0.2Hz
            
            # Convert to quaternion
            rot = R.from_euler('xyz', [roll, pitch, yaw])
            quat = rot.as_quat()
            
            # Trigger varies slowly
            trigger = 0.5 * (1 + np.sin(2 * np.pi * 0.1 * t))
            
            vr_pose = VRPose(
                position=np.array([x, y, z]),
                orientation=quat,
                timestamp=t,
                trigger_value=trigger
            )
            perturbations.append(vr_pose)
        
        self.get_logger().info(f'Generated {len(perturbations)} perturbations at 75Hz')
        return perturbations
    
    def downsample_perturbations(self, perturbations_75hz: List[VRPose], target_hz: float) -> List[Tuple[int, VRPose]]:
        """Downsample 75Hz perturbations to target frequency"""
        if target_hz >= 75:
            # Upsample by repeating
            samples_per_75hz = int(target_hz / 75.0)
            downsampled = []
            for i, pose in enumerate(perturbations_75hz):
                for _ in range(samples_per_75hz):
                    downsampled.append((i, pose))
            return downsampled
        else:
            # Downsample by skipping
            skip_factor = 75.0 / target_hz
            downsampled = []
            accumulated = 0.0
            for i, pose in enumerate(perturbations_75hz):
                if i >= int(accumulated):
                    downsampled.append((i, pose))
                    accumulated += skip_factor
            return downsampled
    
    def process_vr_pose(self, vr_pose: VRPose) -> Tuple[np.ndarray, np.ndarray]:
        """Apply coordinate transformations to VR pose"""
        # Create transformation matrix
        vr_mat = np.eye(4)
        vr_mat[:3, 3] = vr_pose.position
        vr_mat[:3, :3] = R.from_quat(vr_pose.orientation).as_matrix()
        
        # Apply transformation
        transformed_mat = self.global_to_env_mat @ vr_mat
        
        # Extract position and orientation
        transformed_pos = transformed_mat[:3, 3]
        transformed_quat = R.from_matrix(transformed_mat[:3, :3]).as_quat()
        
        return transformed_pos, transformed_quat
    
    def calculate_vr_action(self, vr_pose: VRPose, robot_pos: np.ndarray, robot_quat: np.ndarray) -> Tuple[np.ndarray, Dict]:
        """Calculate robot action from VR pose (simplified)"""
        # Process VR pose
        vr_pos, vr_quat = self.process_vr_pose(vr_pose)
        
        # Simple perturbation-based control
        # VR position is the desired perturbation from home
        pos_action = vr_pos
        
        # VR rotation is the desired rotation change
        rot_diff = R.from_quat(vr_quat) * R.from_quat([0, 0, 0, 1]).inv()
        euler_action = rot_diff.as_euler('xyz')
        
        # Calculate target pose
        target_pos = robot_pos + pos_action
        target_rot = R.from_quat(robot_quat) * rot_diff
        target_quat = target_rot.as_quat()
        
        # Scale velocities
        pos_velocity = pos_action * self.pos_action_gain
        rot_velocity = euler_action * self.rot_action_gain
        gripper_velocity = vr_pose.trigger_value
        
        # Limit velocities
        pos_vel_norm = np.linalg.norm(pos_velocity)
        rot_vel_norm = np.linalg.norm(rot_velocity)
        
        if pos_vel_norm > self.max_lin_vel:
            pos_velocity = pos_velocity * self.max_lin_vel / pos_vel_norm
        if rot_vel_norm > self.max_rot_vel:
            rot_velocity = rot_velocity * self.max_rot_vel / rot_vel_norm
            
        # Create action
        action = np.concatenate([pos_velocity, rot_velocity, [gripper_velocity]])
        
        info = {
            "target_position": target_pos,
            "target_quaternion": target_quat
        }
        
        return action, info
    
    def benchmark_control_rate(self, target_hz: float) -> BenchmarkResult:
        """Benchmark VR teleoperation at target frequency"""
        self.get_logger().info(f'\n🎮 Testing {target_hz}Hz (75Hz perturbations → {target_hz}Hz)')
        
        # Generate perturbations
        test_duration = 10.0
        perturbations_75hz = self.generate_vr_perturbations_at_75hz(test_duration)
        
        # Downsample
        downsampled_poses = self.downsample_perturbations(perturbations_75hz, target_hz)
        self.get_logger().info(f'   {len(downsampled_poses)} samples for {target_hz}Hz')
        
        # Simulated robot state
        robot_pos = np.array([0.4, 0.0, 0.3])  # Home position
        robot_quat = np.array([0.0, 0.0, 0.0, 1.0])
        
        # Tracking
        successful_commands = 0
        failed_commands = 0
        command_times = []
        vr_process_times = []
        
        command_interval = 1.0 / target_hz
        start_time = time.time()
        
        # Process each pose
        for i, (sample_idx, vr_pose) in enumerate(downsampled_poses):
            # Timing
            expected_time = start_time + (i * command_interval)
            current_time = time.time()
            if current_time < expected_time:
                time.sleep(expected_time - current_time)
            
            command_start = time.time()
            
            # Process VR pose
            vr_start = time.time()
            try:
                action, info = self.calculate_vr_action(vr_pose, robot_pos, robot_quat)
                
                # Get target pose from action info
                target_pos = info["target_position"]
                target_quat = info["target_quaternion"]
                target_gripper = vr_pose.trigger_value
                
                # Actually send command to ROS (not just simulate)
                command_success = self.send_vr_command(
                    target_pos, target_quat, target_gripper, command_interval
                )
                
                if command_success:
                    # Update simulated robot state for next iteration
                    robot_pos = target_pos.copy()
                    robot_quat = target_quat.copy()
                    successful_commands += 1
                else:
                    # Keep previous state if command failed
                    failed_commands += 1
                
                vr_process_times.append(time.time() - vr_start)
                
            except Exception as e:
                self.get_logger().debug(f'Error: {e}')
                failed_commands += 1
                vr_process_times.append(0.0)
            
            command_times.append(time.time() - command_start)
            
            # Progress update
            if i > 0 and i % int(target_hz) == 0:
                elapsed = time.time() - start_time
                actual_rate = i / elapsed
                self.get_logger().info(f'   Progress: {i}/{len(downsampled_poses)}, {actual_rate:.1f}Hz')
        
        # Calculate results
        actual_duration = time.time() - start_time
        total_commands = successful_commands + failed_commands
        actual_rate = total_commands / actual_duration
        
        avg_command_time = np.mean(command_times) * 1000
        avg_vr_time = np.mean(vr_process_times) * 1000
        success_rate = (successful_commands / total_commands * 100) if total_commands > 0 else 0
        
        self.get_logger().info(f'   ✅ Results:')
        self.get_logger().info(f'      Actual rate: {actual_rate:.1f}Hz ({actual_rate/target_hz*100:.1f}%)')
        self.get_logger().info(f'      Command time: {avg_command_time:.2f}ms')
        self.get_logger().info(f'      VR processing: {avg_vr_time:.2f}ms')
        self.get_logger().info(f'      Success rate: {success_rate:.1f}% ({successful_commands}/{total_commands})')
        self.get_logger().info(f'      Commands sent to ROS: {total_commands} (IK + Trajectory)')
        
        return BenchmarkResult(
            target_hz=target_hz,
            actual_hz=actual_rate,
            avg_command_time_ms=avg_command_time,
            avg_vr_processing_ms=avg_vr_time,
            success_rate=success_rate
        )
    
    def run_benchmark(self):
        """Run complete benchmark suite"""
        self.get_logger().info('🚀 Starting VR Teleoperation Benchmark')
        self.get_logger().info('   Approach: 75Hz perturbations downsampled to target rates')
        self.get_logger().info('   Testing: 10Hz, 50Hz, 75Hz, 100Hz, 200Hz')
        
        # Wait for joint states
        for _ in range(10):
            if self.joint_state is not None:
                break
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        # Run tests
        for target_hz in self.target_rates_hz:
            result = self.benchmark_control_rate(target_hz)
            self.benchmark_results.append(result)
            time.sleep(1.0)  # Brief pause between tests
        
        # Print summary
        self.print_summary()
        
    def print_summary(self):
        """Print benchmark summary"""
        print(f"\n{'='*80}")
        print(f"🏆 VR TELEOPERATION BENCHMARK SUMMARY")
        print(f"{'='*80}")
        print(f"{'Target Hz':>10} {'Actual Hz':>12} {'Achievement':>12} {'Cmd Time':>12} {'VR Time':>12}")
        print(f"{'-'*80}")
        
        for result in self.benchmark_results:
            achievement = result.actual_hz / result.target_hz * 100
            print(f"{result.target_hz:>10.0f} {result.actual_hz:>12.1f} {achievement:>11.1f}% "
                  f"{result.avg_command_time_ms:>11.2f}ms {result.avg_vr_processing_ms:>11.2f}ms")
        
        print(f"{'='*80}\n")
        
        # Find best results
        if self.benchmark_results:
            best_rate = max(self.benchmark_results, key=lambda x: x.actual_hz)
            best_processing = min(self.benchmark_results, key=lambda x: x.avg_vr_processing_ms)
            
            print(f"🏆 HIGHLIGHTS:")
            print(f"   Highest rate: {best_rate.actual_hz:.1f}Hz (target: {best_rate.target_hz}Hz)")
            print(f"   Fastest processing: {best_processing.avg_vr_processing_ms:.2f}ms")
            print(f"   Sweet spot: 75Hz achieves {[r for r in self.benchmark_results if r.target_hz == 75][0].actual_hz:.1f}Hz")

    def send_vr_command(self, target_pos: np.ndarray, target_quat: np.ndarray, target_gripper: float, duration: float) -> bool:
        """Send VR-generated command to robot via ROS trajectory action"""
        try:
            if not self.trajectory_client.server_is_ready():
                return False
            
            # Convert Cartesian pose to joint positions using IK
            if not self.ik_client.service_is_ready():
                return False
                
            # Create IK request
            ik_request = GetPositionIK.Request()
            ik_request.ik_request.group_name = self.planning_group
            ik_request.ik_request.pose_stamped.header.frame_id = self.base_frame
            ik_request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
            
            # Set target pose
            ik_request.ik_request.pose_stamped.pose.position.x = float(target_pos[0])
            ik_request.ik_request.pose_stamped.pose.position.y = float(target_pos[1])
            ik_request.ik_request.pose_stamped.pose.position.z = float(target_pos[2])
            ik_request.ik_request.pose_stamped.pose.orientation.x = float(target_quat[0])
            ik_request.ik_request.pose_stamped.pose.orientation.y = float(target_quat[1])
            ik_request.ik_request.pose_stamped.pose.orientation.z = float(target_quat[2])
            ik_request.ik_request.pose_stamped.pose.orientation.w = float(target_quat[3])
            
            # Set current robot state as seed
            current_joints = self.get_current_joint_positions()
            if current_joints:
                ik_request.ik_request.robot_state.joint_state.name = self.joint_names
                ik_request.ik_request.robot_state.joint_state.position = current_joints
            
            # Call IK service (quick timeout for real-time performance)
            ik_future = self.ik_client.call_async(ik_request)
            rclpy.spin_until_future_complete(self, ik_future, timeout_sec=0.05)
            
            ik_response = ik_future.result()
            if ik_response and ik_response.error_code.val == 1:
                # Extract joint positions for our 7 arm joints
                joint_positions = []
                for joint_name in self.joint_names:
                    if joint_name in ik_response.solution.joint_state.name:
                        idx = ik_response.solution.joint_state.name.index(joint_name)
                        joint_positions.append(ik_response.solution.joint_state.position[idx])
                
                if len(joint_positions) == 7:
                    # Create trajectory
                    trajectory = JointTrajectory()
                    trajectory.joint_names = self.joint_names
                    
                    point = JointTrajectoryPoint()
                    point.positions = joint_positions
                    point.time_from_start.sec = max(1, int(duration))
                    point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
                    trajectory.points.append(point)
                    
                    # Send trajectory (non-blocking for high frequency)
                    goal = FollowJointTrajectory.Goal()
                    goal.trajectory = trajectory
                    
                    # Send goal asynchronously
                    self.trajectory_client.send_goal_async(goal)
                    return True
            
            return False
            
        except Exception as e:
            self.get_logger().debug(f'VR command failed: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = VRBenchmarkNode()
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