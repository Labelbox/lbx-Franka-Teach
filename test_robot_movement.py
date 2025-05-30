#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class RobotMovementTest(Node):
    def __init__(self):
        super().__init__('robot_movement_test')
        
        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory, '/fr3_arm_controller/follow_joint_trajectory'
        )
        
        # Joint names for FR3
        self.joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]
        
        # Home position
        self.home_positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        
        # Test position (slight movement in joint 1)
        self.test_positions = [0.3, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

    def execute_trajectory(self, positions, duration=3.0):
        """Execute a trajectory to move joints to target positions"""
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Trajectory action server not available")
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
        
        self.get_logger().info(f"Sending trajectory to positions: {positions}")
        
        # Send goal
        future = self.trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        goal_handle = future.result()
        
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected")
            return False
            
        self.get_logger().info("Goal accepted, waiting for completion...")
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 2.0)
        
        result = result_future.result()
        if result is None:
            self.get_logger().error("Trajectory execution timeout")
            return False
        
        success = result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        
        if success:
            self.get_logger().info("✅ Trajectory executed successfully!")
        else:
            self.get_logger().error(f"❌ Trajectory failed with error code: {result.result.error_code}")
        
        return success

    def run_test(self):
        """Run movement test"""
        self.get_logger().info("🤖 Starting robot movement test...")
        
        # First go to home
        self.get_logger().info("Moving to home position...")
        if not self.execute_trajectory(self.home_positions, 3.0):
            return False
        
        time.sleep(1.0)
        
        # Then move to test position
        self.get_logger().info("Moving to test position (should see joint 1 move)...")
        if not self.execute_trajectory(self.test_positions, 3.0):
            return False
        
        time.sleep(1.0)
        
        # Return to home
        self.get_logger().info("Returning to home position...")
        if not self.execute_trajectory(self.home_positions, 3.0):
            return False
        
        self.get_logger().info("🎉 Test completed successfully!")
        return True

def main():
    rclpy.init()
    
    try:
        test_node = RobotMovementTest()
        success = test_node.run_test()
        
        if success:
            print("\n✅ Robot movement test PASSED")
            print("   The robot should have moved visibly during this test")
        else:
            print("\n❌ Robot movement test FAILED")
            print("   Check robot status and controller configuration")
            
    except Exception as e:
        print(f"❌ Test failed with error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 