#!/usr/bin/env python3
"""
VR Benchmark for Fake Hardware Testing
- Large, visible movements to verify command execution
- Uses MoveIt move_group action for safety
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
from typing import List
from scipy.spatial.transform import Rotation as R


@dataclass
class BenchmarkResult:
    """Benchmark results"""
    target_hz: float
    actual_hz: float
    avg_command_time_ms: float
    success_rate: float


class VRBenchmarkFakeNode(Node):
    def __init__(self):
        super().__init__('vr_benchmark_fake')
        
        # Robot configuration
        self.planning_group = "fr3_arm"
        self.end_effector_link = "fr3_hand_tcp"
        self.base_frame = "fr3_link0"
        
        # Create MoveGroup action client
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Joint state subscriber
        self.joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        # Wait for move_group action server
        self.get_logger().info('🤖 FAKE HARDWARE MODE - Waiting for move_group action server...')
        if self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().info('✅ Move group action server ready for fake hardware!')
        else:
            self.get_logger().error('❌ Move group action server not available!')
            return
        
    def joint_state_callback(self, msg):
        self.joint_state = msg
        
    def create_test_poses(self) -> List[Pose]:
        """Create a sequence of clearly different poses for testing"""
        poses = []
        
        # Base pose - center workspace
        base_pose = Pose()
        base_pose.position.x = 0.4
        base_pose.position.y = 0.0
        base_pose.position.z = 0.4
        base_pose.orientation.w = 1.0
        poses.append(base_pose)
        
        # Right pose - clearly different
        right_pose = Pose()
        right_pose.position.x = 0.4
        right_pose.position.y = -0.3  # 30cm to the right
        right_pose.position.z = 0.4
        right_pose.orientation.w = 1.0
        poses.append(right_pose)
        
        # Left pose - clearly different  
        left_pose = Pose()
        left_pose.position.x = 0.4
        left_pose.position.y = 0.3   # 30cm to the left
        left_pose.position.z = 0.4
        left_pose.orientation.w = 1.0
        poses.append(left_pose)
        
        return poses
        
    def create_move_group_goal(self, target_pose: Pose) -> MoveGroup.Goal:
        """Create a MoveGroup action goal with fake hardware settings"""
        goal = MoveGroup.Goal()
        
        # Set up the motion plan request
        goal.request.group_name = self.planning_group
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = 5.0  # More time for fake hardware
        goal.request.max_velocity_scaling_factor = 0.5  # Moderate speed for visibility
        goal.request.max_acceleration_scaling_factor = 0.5
        
        # Set the target pose
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = self.base_frame
        pose_goal.header.stamp = self.get_clock().now().to_msg()
        pose_goal.pose = target_pose
        
        # Create position constraint
        from moveit_msgs.msg import PositionConstraint
        constraints = Constraints()
        
        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header = pose_goal.header
        pos_constraint.link_name = self.end_effector_link
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0
        
        # Create constraint region (small sphere)
        from shape_msgs.msg import SolidPrimitive
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]  # 1cm tolerance
        pos_constraint.constraint_region.primitives = [sphere]
        pos_constraint.constraint_region.primitive_poses = [target_pose]
        pos_constraint.weight = 1.0
        
        constraints.position_constraints = [pos_constraint]
        goal.request.goal_constraints = [constraints]
        
        # Planning options
        goal.planning_options.plan_only = False  # Plan and execute
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        
        return goal
    
    def send_pose_command(self, target_pose: Pose) -> bool:
        """Send pose command using MoveGroup action"""
        try:
            # Create and send goal
            goal = self.create_move_group_goal(target_pose)
            
            self.get_logger().info(f'📤 Sending pose: [{target_pose.position.x:.2f}, {target_pose.position.y:.2f}, {target_pose.position.z:.2f}]')
            
            # Send goal and wait for acceptance
            future = self.move_group_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.done():
                goal_handle = future.result()
                if goal_handle and goal_handle.accepted:
                    self.get_logger().info('✅ Goal accepted by MoveGroup')
                    
                    # Wait for completion
                    result_future = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(self, result_future, timeout_sec=15.0)
                    
                    if result_future.done():
                        result = result_future.result()
                        if result and result.result.error_code.val == 1:  # SUCCESS
                            self.get_logger().info('🎉 Movement completed successfully!')
                            return True
                        else:
                            self.get_logger().warn(f'⚠️ Movement failed with error code: {result.result.error_code.val if result else "None"}')
                    else:
                        self.get_logger().warn('⏰ Movement timed out')
                else:
                    self.get_logger().warn('❌ Goal rejected by MoveGroup')
            else:
                self.get_logger().warn('⏰ Goal submission timed out')
            
            return False
            
        except Exception as e:
            self.get_logger().error(f'💥 Pose command failed: {e}')
            return False
    
    def run_simple_test(self):
        """Run a simple movement test with large, visible poses"""
        self.get_logger().info('🚀 Starting FAKE HARDWARE movement test')
        self.get_logger().info('   Large movements to verify robot response in RViz')
        
        # Wait for joint states
        for _ in range(20):
            if self.joint_state is not None:
                break
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        if self.joint_state is None:
            self.get_logger().error('❌ No joint states received! Is fake hardware running?')
            return
        
        self.get_logger().info('📡 Joint states received - robot is active!')
        
        # Create test poses
        test_poses = self.create_test_poses()
        
        self.get_logger().info(f'🎯 Testing {len(test_poses)} clearly different poses:')
        for i, pose in enumerate(test_poses):
            self.get_logger().info(f'   Pose {i+1}: [{pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f}]')
        
        successful_moves = 0
        total_moves = len(test_poses)
        
        # Execute each pose
        for i, pose in enumerate(test_poses):
            self.get_logger().info(f'\n🎯 Movement {i+1}/{total_moves}:')
            
            success = self.send_pose_command(pose)
            if success:
                successful_moves += 1
                self.get_logger().info('✅ Movement successful!')
            else:
                self.get_logger().error('❌ Movement failed!')
            
            # Pause between movements
            time.sleep(3.0)
        
        # Summary
        success_rate = (successful_moves / total_moves) * 100
        self.get_logger().info(f'\n🏆 FAKE HARDWARE TEST SUMMARY:')
        self.get_logger().info(f'   Total movements: {total_moves}')
        self.get_logger().info(f'   Successful: {successful_moves}')
        self.get_logger().info(f'   Success rate: {success_rate:.1f}%')
        
        if success_rate > 50:
            self.get_logger().info('🎉 FAKE HARDWARE TEST PASSED - Robot is responding!')
            self.get_logger().info('   Check RViz to see if the robot moved to different positions')
        else:
            self.get_logger().error('❌ FAKE HARDWARE TEST FAILED - Robot may not be responding')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = VRBenchmarkFakeNode()
        node.run_simple_test()
        
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 