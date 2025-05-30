#!/usr/bin/env python3
"""
Simple test script to debug robot state reading issues
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
import time
import numpy as np

class RobotStateTest(Node):
    def __init__(self):
        super().__init__('robot_state_test')
        
        # Robot configuration
        self.joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]
        self.end_effector_link = "fr3_hand_tcp"
        self.base_frame = "fr3_link0"
        
        # Create FK client
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        
        # Joint state subscriber
        self.joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        print("🔧 Robot State Test - Waiting for services...")
        
        # Wait for FK service
        if not self.fk_client.wait_for_service(timeout_sec=10.0):
            print("❌ FK service not available")
            return
        else:
            print("✅ FK service ready")
        
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
            print(f"   Available joints: {list(self.joint_state.name)}")
    
    def joint_state_callback(self, msg):
        self.joint_state = msg
    
    def get_joint_positions(self):
        """Get current joint positions"""
        if self.joint_state is None:
            print("❌ No joint state available")
            return None
        
        positions = []
        missing_joints = []
        
        print(f"🔍 Looking for joints: {self.joint_names}")
        print(f"   Available in joint_state: {list(self.joint_state.name)}")
        
        for joint_name in self.joint_names:
            if joint_name in self.joint_state.name:
                idx = self.joint_state.name.index(joint_name)
                positions.append(self.joint_state.position[idx])
                print(f"   ✅ {joint_name}: {self.joint_state.position[idx]:.6f}")
            else:
                missing_joints.append(joint_name)
                print(f"   ❌ {joint_name}: MISSING")
        
        if missing_joints:
            print(f"❌ Missing joints: {missing_joints}")
            return None
        
        return positions
    
    def get_end_effector_pose(self):
        """Get end effector pose via FK"""
        joint_positions = self.get_joint_positions()
        if joint_positions is None:
            return None, None
        
        print(f"🔧 Calling FK service...")
        
        # Create FK request
        fk_request = GetPositionFK.Request()
        fk_request.fk_link_names = [self.end_effector_link]
        fk_request.header.frame_id = self.base_frame
        fk_request.header.stamp = self.get_clock().now().to_msg()
        
        # Set robot state
        fk_request.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        fk_request.robot_state.joint_state.name = self.joint_names
        fk_request.robot_state.joint_state.position = joint_positions
        
        try:
            # Call FK service
            fk_future = self.fk_client.call_async(fk_request)
            rclpy.spin_until_future_complete(self, fk_future, timeout_sec=2.0)
            
            if not fk_future.done():
                print("❌ FK service timeout")
                return None, None
            
            fk_response = fk_future.result()
            
            print(f"🔧 FK response received")
            print(f"   Error code: {fk_response.error_code.val}")
            print(f"   Pose stamped count: {len(fk_response.pose_stamped) if fk_response.pose_stamped else 0}")
            
            if fk_response and fk_response.error_code.val == 1 and fk_response.pose_stamped:
                pose = fk_response.pose_stamped[0].pose
                pos = np.array([pose.position.x, pose.position.y, pose.position.z])
                quat = np.array([pose.orientation.x, pose.orientation.y, 
                                pose.orientation.z, pose.orientation.w])
                
                print(f"✅ FK successful!")
                print(f"   Position: [{pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f}]")
                print(f"   Quaternion: [{quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f}]")
                
                return pos, quat
            else:
                print(f"❌ FK failed with error code: {fk_response.error_code.val if fk_response else 'No response'}")
                return None, None
                
        except Exception as e:
            print(f"❌ FK exception: {e}")
            return None, None
    
    def test_robot_state(self):
        """Test robot state reading"""
        print("\n🚀 Testing robot state reading...")
        
        for attempt in range(3):
            print(f"\n📋 Test attempt {attempt + 1}/3")
            
            # Get fresh data
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.1)
            
            pos, quat = self.get_end_effector_pose()
            
            if pos is not None and quat is not None:
                print(f"✅ Robot state test PASSED on attempt {attempt + 1}")
                return True
            else:
                print(f"❌ Robot state test FAILED on attempt {attempt + 1}")
        
        print("\n❌ All robot state tests FAILED")
        return False

def main():
    rclpy.init()
    
    try:
        tester = RobotStateTest()
        
        # Run test
        success = tester.test_robot_state()
        
        if success:
            print("\n🎉 Robot state reading is working correctly!")
        else:
            print("\n💥 Robot state reading has issues!")
            
    except Exception as e:
        print(f"❌ Test failed with exception: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main() 