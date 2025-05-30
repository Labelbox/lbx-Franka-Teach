#!/usr/bin/env python3
"""
Test script to diagnose robot reset to home position
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from sensor_msgs.msg import JointState
import time
import numpy as np

class RobotResetTest(Node):
    def __init__(self):
        super().__init__('robot_reset_test')
        
        # Robot configuration
        self.joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]
        
        # Home position (ready pose)
        self.home_positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        
        # Create action client for trajectory execution
        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory, '/fr3_arm_controller/follow_joint_trajectory'
        )
        
        # Joint state subscriber
        self.joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        print("🔧 Robot Reset Test - Waiting for services...")
        
        # Wait for trajectory action server
        if not self.trajectory_client.wait_for_server(timeout_sec=10.0):
            print("❌ Trajectory action server not available")
            return
        else:
            print("✅ Trajectory action server ready")
        
        # Wait for joint states
        print("🔍 Waiting for joint states...")
        start_time = time.time()
        while self.joint_state is None and (time.time() - start_time) < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        if self.joint_state is None:
            print("❌ No joint states received")
            return
        else:
            print(f"✅ Joint states received: {len(self.joint_state.name)} joints")
    
    def joint_state_callback(self, msg):
        self.joint_state = msg
    
    def get_current_joint_positions(self):
        """Get current joint positions"""
        if self.joint_state is None:
            print("❌ No joint state available")
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
            print(f"❌ Missing joints: {missing_joints}")
            return None
        
        return positions
    
    def show_current_position(self):
        """Show current joint positions"""
        positions = self.get_current_joint_positions()
        if positions:
            print(f"📍 Current Joint Positions:")
            for i, (name, pos) in enumerate(zip(self.joint_names, positions)):
                home_pos = self.home_positions[i]
                diff = abs(pos - home_pos)
                status = "✅" if diff < 0.1 else "❌"
                print(f"   {status} {name}: {pos:.6f} (home: {home_pos:.6f}, diff: {diff:.6f})")
            return positions
        return None
    
    def execute_home_trajectory(self, duration=5.0):
        """Execute trajectory to home position"""
        print(f"\n🏠 Executing trajectory to home position (duration: {duration}s)...")
        
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Add single point to home position
        point = JointTrajectoryPoint()
        point.positions = self.home_positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        # Add zero velocities and accelerations for smooth stop
        point.velocities = [0.0] * len(self.joint_names)
        point.accelerations = [0.0] * len(self.joint_names)
        
        trajectory.points.append(point)
        
        # Create goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        # Very forgiving tolerances
        goal.path_tolerance = [
            JointTolerance(name=name, position=0.05, velocity=0.5, acceleration=0.5) 
            for name in self.joint_names
        ]
        
        goal.goal_tolerance = [
            JointTolerance(name=name, position=0.02, velocity=0.2, acceleration=0.2)
            for name in self.joint_names
        ]
        
        # Send goal and wait for completion
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
        
        # Monitor progress
        start_time = time.time()
        last_update = 0
        
        while not result_future.done():
            elapsed = time.time() - start_time
            if elapsed - last_update >= 1.0:  # Update every second
                print(f"   ⏱️  Executing... {elapsed:.1f}s elapsed")
                last_update = elapsed
                
                # Show current position
                current_pos = self.get_current_joint_positions()
                if current_pos:
                    max_diff = max(abs(curr - home) for curr, home in zip(current_pos, self.home_positions))
                    print(f"   📍 Max joint difference from home: {max_diff:.6f} rad")
            
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if elapsed > duration + 5.0:  # Give extra time
                print("❌ Trajectory execution timeout")
                return False
        
        # Get result
        result = result_future.result()
        
        if result.result.error_code == 0:  # SUCCESS
            print("✅ Trajectory execution completed successfully!")
            return True
        else:
            print(f"❌ Trajectory execution failed with error code: {result.result.error_code}")
            return False
    
    def test_robot_reset(self):
        """Test robot reset to home position"""
        print("\n🚀 Testing robot reset to home position...")
        
        # Show initial position
        print("\n📍 Initial Position:")
        initial_pos = self.show_current_position()
        
        if not initial_pos:
            print("❌ Cannot read initial position")
            return False
        
        # Check if already at home
        max_diff = max(abs(curr - home) for curr, home in zip(initial_pos, self.home_positions))
        if max_diff < 0.05:
            print("✅ Robot is already at home position!")
            return True
        
        print(f"\n📏 Distance from home: {max_diff:.6f} rad (max joint difference)")
        
        # Execute trajectory to home
        success = self.execute_home_trajectory()
        
        if success:
            # Wait a bit for settling
            print("\n⏱️  Waiting for robot to settle...")
            time.sleep(2.0)
            
            # Get fresh joint state data
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.1)
            
            # Check final position
            print("\n📍 Final Position:")
            final_pos = self.show_current_position()
            
            if final_pos:
                max_diff = max(abs(curr - home) for curr, home in zip(final_pos, self.home_positions))
                if max_diff < 0.05:
                    print(f"\n🎉 SUCCESS! Robot reached home position (max diff: {max_diff:.6f} rad)")
                    return True
                else:
                    print(f"\n⚠️  Robot moved but didn't reach home (max diff: {max_diff:.6f} rad)")
                    return False
            else:
                print("\n❌ Cannot read final position")
                return False
        else:
            print("\n❌ Trajectory execution failed")
            return False

def main():
    rclpy.init()
    
    try:
        tester = RobotResetTest()
        
        # Run test
        success = tester.test_robot_reset()
        
        if success:
            print("\n🎉 Robot reset test PASSED!")
        else:
            print("\n💥 Robot reset test FAILED!")
            
    except Exception as e:
        print(f"❌ Test failed with exception: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main() 