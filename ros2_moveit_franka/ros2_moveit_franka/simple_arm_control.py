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
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
import sys
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
import time
from moveit_commander.conversions import pose_to_list


class FrankaArmController(Node):
    """Simple Franka arm controller using MoveIt"""
    
    def __init__(self):
        super().__init__('franka_arm_controller')
        
        # Initialize MoveIt commander
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Initialize the robot and scene
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Initialize the arm group (panda_arm is the standard group name for Franka)
        self.group_name = "panda_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        
        # Initialize gripper group
        self.gripper_group = moveit_commander.MoveGroupCommander("panda_hand")
        
        # Create display trajectory publisher
        self.display_trajectory_publisher = self.create_publisher(
            moveit_msgs.msg.DisplayTrajectory,
            '/move_group/display_planned_path',
            20
        )
        
        # Get basic information
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
        
        self.get_logger().info("="*50)
        self.get_logger().info("Franka FR3 Arm Controller Initialized")
        self.get_logger().info("="*50)
        self.get_logger().info(f"Planning frame: {self.planning_frame}")
        self.get_logger().info(f"End effector link: {self.eef_link}")
        self.get_logger().info(f"Available Planning Groups: {self.group_names}")
        
        # Configure planner settings for better performance
        self.move_group.set_planner_id("RRTConnectkConfigDefault")
        self.move_group.set_planning_time(10.0)
        self.move_group.set_num_planning_attempts(10)
        self.move_group.set_max_velocity_scaling_factor(0.3)
        self.move_group.set_max_acceleration_scaling_factor(0.3)
        
        self.get_logger().info("MoveIt planner configured for safe operation")
    
    def print_robot_state(self):
        """Print current robot state information"""
        current_pose = self.move_group.get_current_pose().pose
        current_joints = self.move_group.get_current_joint_values()
        
        self.get_logger().info("Current robot state:")
        self.get_logger().info(f"  Position: x={current_pose.position.x:.3f}, y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}")
        self.get_logger().info(f"  Orientation: x={current_pose.orientation.x:.3f}, y={current_pose.orientation.y:.3f}, z={current_pose.orientation.z:.3f}, w={current_pose.orientation.w:.3f}")
        self.get_logger().info(f"  Joint values: {[f'{j:.3f}' for j in current_joints]}")
    
    def go_to_home_position(self):
        """Move the robot to home/ready position"""
        self.get_logger().info("Moving to home position...")
        
        # Use the predefined "ready" pose if available, otherwise use custom home position
        try:
            # Try to use named target first
            self.move_group.set_named_target("ready")
            success = self.move_group.go(wait=True)
            
            if success:
                self.get_logger().info("✅ Successfully moved to 'ready' position")
            else:
                raise Exception("Failed to move to 'ready' position")
                
        except Exception as e:
            self.get_logger().warn(f"'ready' position not available: {e}")
            self.get_logger().info("Using custom home position...")
            
            # Define a safe home position for Franka (based on workspace limits from constants)
            home_joints = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]  # Safe home configuration
            
            self.move_group.go(home_joints, wait=True)
            self.get_logger().info("✅ Successfully moved to custom home position")
        
        # Stop any residual motion
        self.move_group.stop()
        self.print_robot_state()
        
        return True
    
    def move_in_x_direction(self, distance_meters=0.10):
        """Move the end effector by specified distance in X direction"""
        self.get_logger().info(f"Moving {distance_meters*100:.1f}cm in +X direction...")
        
        # Get current pose
        current_pose = self.move_group.get_current_pose().pose
        
        # Create target pose
        target_pose = Pose()
        target_pose.position.x = current_pose.position.x + distance_meters
        target_pose.position.y = current_pose.position.y
        target_pose.position.z = current_pose.position.z
        target_pose.orientation = current_pose.orientation
        
        self.get_logger().info(f"Current position: x={current_pose.position.x:.3f}, y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}")
        self.get_logger().info(f"Target position:  x={target_pose.position.x:.3f}, y={target_pose.position.y:.3f}, z={target_pose.position.z:.3f}")
        
        # Set the target pose
        self.move_group.set_pose_target(target_pose)
        
        # Plan and execute
        self.get_logger().info("Planning trajectory...")
        success = self.move_group.go(wait=True)
        
        # Stop any residual motion
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        if success:
            self.get_logger().info("✅ Successfully moved in X direction")
            self.print_robot_state()
            return True
        else:
            self.get_logger().error("❌ Failed to move in X direction")
            return False
    
    def open_gripper(self):
        """Open the gripper"""
        self.get_logger().info("Opening gripper...")
        try:
            # Set gripper to open position (typically max joint values)
            self.gripper_group.set_named_target("open")
            success = self.gripper_group.go(wait=True)
            
            if success:
                self.get_logger().info("✅ Gripper opened")
            else:
                # Fallback: set joint values directly
                self.gripper_group.set_joint_value_target([0.04, 0.04])  # Open position
                self.gripper_group.go(wait=True)
                self.get_logger().info("✅ Gripper opened (fallback method)")
                
        except Exception as e:
            self.get_logger().warn(f"Gripper control failed: {e}")
    
    def close_gripper(self):
        """Close the gripper"""
        self.get_logger().info("Closing gripper...")
        try:
            # Set gripper to closed position
            self.gripper_group.set_named_target("close")
            success = self.gripper_group.go(wait=True)
            
            if success:
                self.get_logger().info("✅ Gripper closed")
            else:
                # Fallback: set joint values directly
                self.gripper_group.set_joint_value_target([0.0, 0.0])  # Closed position
                self.gripper_group.go(wait=True)
                self.get_logger().info("✅ Gripper closed (fallback method)")
                
        except Exception as e:
            self.get_logger().warn(f"Gripper control failed: {e}")
    
    def execute_demo_sequence(self):
        """Execute the requested demo: reset to home and move 10cm in X"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("STARTING FRANKA FR3 DEMO SEQUENCE")
        self.get_logger().info("="*60)
        
        try:
            # Step 1: Print initial state
            self.get_logger().info("\n🔍 STEP 1: Current robot state")
            self.print_robot_state()
            
            # Step 2: Open gripper
            self.get_logger().info("\n🤏 STEP 2: Opening gripper")
            self.open_gripper()
            time.sleep(1.0)
            
            # Step 3: Move to home position
            self.get_logger().info("\n🏠 STEP 3: Moving to home position")
            if not self.go_to_home_position():
                self.get_logger().error("❌ Failed to reach home position")
                return False
            time.sleep(2.0)
            
            # Step 4: Move 10cm in X direction
            self.get_logger().info("\n➡️  STEP 4: Moving 10cm in +X direction")
            if not self.move_in_x_direction(0.10):
                self.get_logger().error("❌ Failed to move in X direction")
                return False
            time.sleep(2.0)
            
            # Step 5: Return to home
            self.get_logger().info("\n🏠 STEP 5: Returning to home position")
            if not self.go_to_home_position():
                self.get_logger().error("❌ Failed to return to home position")
                return False
            
            self.get_logger().info("\n" + "="*60)
            self.get_logger().info("✅ DEMO SEQUENCE COMPLETED SUCCESSFULLY!")
            self.get_logger().info("="*60)
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Demo sequence failed: {str(e)}")
            import traceback
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
            return False
    
    def shutdown(self):
        """Properly shutdown the controller"""
        self.get_logger().info("Shutting down Franka arm controller...")
        moveit_commander.roscpp_shutdown()


def main(args=None):
    """Main function"""
    # Initialize ROS 2
    rclpy.init(args=args)
    
    try:
        # Create the controller
        controller = FrankaArmController()
        
        # Wait a bit for everything to initialize
        time.sleep(2.0)
        
        # Execute the demo sequence
        success = controller.execute_demo_sequence()
        
        if success:
            controller.get_logger().info("Demo completed. Press Ctrl+C to exit.")
            # Keep the node alive for monitoring
            rclpy.spin(controller)
        else:
            controller.get_logger().error("Demo failed!")
        
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
    
    except Exception as e:
        print(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Cleanup
        if 'controller' in locals():
            controller.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 