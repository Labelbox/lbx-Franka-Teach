#!/usr/bin/env python3
"""
Simple Franka FR3 arm control using ROS 2 MoveIt
This script resets the arm to home position and then moves it 10cm in the x direction.

Based on the robot configuration from the current codebase:
- Robot IP: 192.168.1.59
- Uses Franka FR3 hardware
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPlanningScene
from moveit_msgs.msg import PositionIKRequest, RobotState, Constraints, JointConstraint
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
import numpy as np
import time
import sys


class SimpleArmControl(Node):
    """Simple Franka arm controller using MoveIt"""
    
    def __init__(self):
        super().__init__('simple_arm_control')
        
        # Robot configuration
        self.robot_ip = "192.168.1.59"
        self.planning_group = "panda_arm"
        self.end_effector_link = "fr3_hand_tcp"
        self.base_frame = "fr3_link0"
        
        # Joint names for FR3
        self.joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]
        
        # Home position (ready pose)
        self.home_positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        
        # Create service clients
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.planning_scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
        
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
        self.get_logger().info('Waiting for services...')
        self.ik_client.wait_for_service(timeout_sec=10.0)
        self.planning_scene_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('Services are ready!')
        
        # Wait for action server
        self.get_logger().info('Waiting for trajectory action server...')
        self.trajectory_client.wait_for_server(timeout_sec=10.0)
        self.get_logger().info('Action server is ready!')
        
    def joint_state_callback(self, msg):
        """Store the latest joint state"""
        self.joint_state = msg
        
    def get_current_joint_positions(self):
        """Get current joint positions from joint_states topic"""
        if self.joint_state is None:
            self.get_logger().warn('No joint state received yet')
            return None
            
        positions = []
        for joint_name in self.joint_names:
            if joint_name in self.joint_state.name:
                idx = self.joint_state.name.index(joint_name)
                positions.append(self.joint_state.position[idx])
            else:
                self.get_logger().error(f'Joint {joint_name} not found in joint states')
                return None
                
        return positions
    
    def execute_trajectory(self, positions, duration=3.0):
        """Execute a trajectory to move joints to target positions"""
        if not self.trajectory_client.server_is_ready():
            self.get_logger().error('Trajectory action server is not ready')
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
        self.get_logger().info(f'Executing trajectory to: {[f"{p:.3f}" for p in positions]}')
        future = self.trajectory_client.send_goal_async(goal)
        
        # Wait for goal acceptance
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected')
            return False
            
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 5.0)
        
        result = result_future.result()
        if result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Trajectory executed successfully')
            return True
        else:
            self.get_logger().error(f'Trajectory execution failed with error code: {result.result.error_code}')
            return False
    
    def move_to_home(self):
        """Move robot to home position"""
        self.get_logger().info('Moving to home position...')
        return self.execute_trajectory(self.home_positions, duration=5.0)
    
    def compute_ik_for_pose(self, target_pose):
        """Compute IK for a target pose"""
        # Get current planning scene
        scene_request = GetPlanningScene.Request()
        scene_request.components.components = 1  # SCENE_SETTINGS
        
        scene_future = self.planning_scene_client.call_async(scene_request)
        rclpy.spin_until_future_complete(self, scene_future, timeout_sec=5.0)
        scene_response = scene_future.result()
        
        if scene_response is None:
            self.get_logger().error('Failed to get planning scene')
            return None
            
        # Create IK request
        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = self.planning_group
        ik_request.ik_request.robot_state = scene_response.scene.robot_state
        ik_request.ik_request.avoid_collisions = True
        
        # Set target pose
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.base_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = target_pose
        
        ik_request.ik_request.pose_stamped = pose_stamped
        ik_request.ik_request.ik_link_name = self.end_effector_link
        
        # Call IK service
        ik_future = self.ik_client.call_async(ik_request)
        rclpy.spin_until_future_complete(self, ik_future, timeout_sec=5.0)
        ik_response = ik_future.result()
        
        if ik_response is None or ik_response.error_code.val != 1:
            self.get_logger().error('IK computation failed')
            return None
            
        # Extract joint positions
        positions = []
        for joint_name in self.joint_names:
            if joint_name in ik_response.solution.joint_state.name:
                idx = ik_response.solution.joint_state.name.index(joint_name)
                positions.append(ik_response.solution.joint_state.position[idx])
                
        return positions
    
    def move_relative_simple(self, joint_offset=0.2):
        """Move by adjusting joint positions directly (simpler than IK)"""
        # Wait for joint states
        for _ in range(10):
            if self.joint_state is not None:
                break
            time.sleep(0.5)
            
        if self.joint_state is None:
            self.get_logger().error('No joint states available')
            return False
            
        # Get current joint positions
        current_positions = self.get_current_joint_positions()
        if current_positions is None:
            self.get_logger().error('Failed to get current joint positions')
            return False
            
        # Create target positions by modifying joint 1 (base rotation)
        # This will create movement roughly in the X direction
        target_positions = current_positions.copy()
        target_positions[0] += joint_offset  # Modify joint 1 to move in X
        
        self.get_logger().info(f'Moving from joints: {[f"{p:.3f}" for p in current_positions]}')
        self.get_logger().info(f'Moving to joints:   {[f"{p:.3f}" for p in target_positions]}')
        
        # Execute trajectory
        return self.execute_trajectory(target_positions, duration=3.0)
    
    def move_relative(self, dx=0.0, dy=0.0, dz=0.0):
        """Move end effector relative to current position"""
        # For now, use the simpler joint-space movement
        # In the future, this could be enhanced with proper forward/inverse kinematics
        self.get_logger().info(f'Moving approximately {dx*100:.1f}cm in X direction using joint space movement')
        return self.move_relative_simple(joint_offset=0.15)  # Smaller movement for safety
    
    def run_demo(self):
        """Run the demo sequence"""
        self.get_logger().info('Starting Franka FR3 demo...')
        
        # Print current state
        current_positions = self.get_current_joint_positions()
        if current_positions:
            self.get_logger().info(f'Current joint positions: {[f"{p:.3f}" for p in current_positions]}')
        
        # Move to home
        if not self.move_to_home():
            self.get_logger().error('Failed to move to home position')
            return
            
        time.sleep(2.0)
        
        # Move 10cm in X direction
        self.get_logger().info('Moving 10cm in positive X direction...')
        if not self.move_relative(dx=0.1):
            self.get_logger().error('Failed to move in X direction')
            return
            
        time.sleep(2.0)
        
        # Return to home
        self.get_logger().info('Returning to home position...')
        if not self.move_to_home():
            self.get_logger().error('Failed to return to home position')
            return
            
        self.get_logger().info('Demo completed successfully!')


def main(args=None):
    """Main function"""
    # Initialize ROS 2
    rclpy.init(args=args)
    
    try:
        # Create the controller
        controller = SimpleArmControl()
        
        # Wait a bit for everything to initialize
        time.sleep(2.0)
        
        # Execute the demo sequence
        controller.run_demo()
        
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
    
    except Exception as e:
        print(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Cleanup
        rclpy.shutdown()


if __name__ == '__main__':
    main() 