"""VR Input Handler for ROS2/MoveIt teleoperation"""

import asyncio
import numpy as np
from dataclasses import dataclass
from typing import Dict, Optional
import time

from geometry_msgs.msg import PoseStamped, Pose
from franka_vr_ros2.core.types import VRState
from franka_vr_ros2.input_devices import MetaQuestReader


class VRInputHandler:
    """Async VR input handler - preserves original functionality"""
    
    def __init__(self, node, right_controller=True, ip_address=None):
        self.node = node
        self.controller_id = "r" if right_controller else "l"
        
        # Initialize input device (Meta Quest for now)
        self.input_device = MetaQuestReader(
            ip_address=ip_address,
            print_fps=False,
            auto_start=False  # We'll start it manually
        )
        
        # Publishers
        self.pose_pub = node.create_publisher(
            PoseStamped, 'vr_controller_pose', 10
        )
        
        # State
        self._latest_state = None
        self._running = False
        
        # Calibration state (preserved from original)
        self.vr_neutral_pose = None
        self.vr_to_global_mat = np.eye(4)
        self.calibrating_forward = False
        self.calibration_start_pose = None
        self.calibration_start_time = None
        
        # Button state tracking
        self.prev_joystick_state = False
        self.prev_grip_state = False
        
    async def start(self):
        """Start async polling loop"""
        self._running = True
        self.input_device.start()
        asyncio.create_task(self._poll_loop())
        
    async def stop(self):
        """Stop polling and cleanup"""
        self._running = False
        self.input_device.stop()
        
    async def _poll_loop(self):
        """Poll VR controller at 90Hz"""
        while self._running:
            try:
                poses, buttons = self.input_device.get_transformations_and_buttons()
                
                if self.controller_id in poses:
                    # Create VR state
                    state = VRState(
                        timestamp=time.time(),
                        pose=self._matrix_to_pose(poses[self.controller_id]),
                        buttons=buttons,
                        movement_enabled=buttons.get(f"{self.controller_id.upper()}G", False),
                        controller_on=True
                    )
                    
                    # Handle calibration (preserved from original)
                    self._handle_calibration(state, poses[self.controller_id], buttons)
                    
                    # Update latest state
                    self._latest_state = state
                    
                    # Publish pose
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.node.get_clock().now().to_msg()
                    pose_msg.header.frame_id = "vr_base"
                    pose_msg.pose = state.pose
                    self.pose_pub.publish(pose_msg)
                    
            except Exception as e:
                self.node.get_logger().error(f"VR polling error: {e}")
                
            await asyncio.sleep(0.011)  # ~90Hz
            
    def _handle_calibration(self, state: VRState, pose_matrix: np.ndarray, buttons: Dict):
        """Handle forward direction calibration - preserved from original"""
        # Get current button states
        current_grip = buttons.get(self.controller_id.upper() + "G", False)
        current_joystick = buttons.get(self.controller_id.upper() + "J", False)
        
        # Detect edge transitions
        joystick_pressed = current_joystick and not self.prev_joystick_state
        joystick_released = not current_joystick and self.prev_joystick_state
        
        # Start calibration when joystick is pressed
        if joystick_pressed:
            self.calibrating_forward = True
            self.calibration_start_pose = pose_matrix.copy()
            self.calibration_start_time = time.time()
            self.node.get_logger().info("Forward calibration started - Move controller forward")
            
        # Complete calibration when joystick is released
        elif joystick_released and self.calibrating_forward:
            self.calibrating_forward = False
            
            if self.calibration_start_pose is not None:
                # Get movement vector
                start_pos = self.calibration_start_pose[:3, 3]
                end_pos = pose_matrix[:3, 3]
                movement_vec = end_pos - start_pos
                movement_distance = np.linalg.norm(movement_vec)
                
                if movement_distance > 0.003:  # 3mm threshold
                    # Normalize movement vector
                    forward_vec = movement_vec / movement_distance
                    
                    self.node.get_logger().info(
                        f"Forward calibrated! Distance: {movement_distance*1000:.1f}mm"
                    )
                    
                    # Store calibration
                    self.vr_neutral_pose = pose_matrix.copy()
                    
                    # Create rotation to align forward vector
                    # This is simplified - full implementation would match original
                    self.vr_to_global_mat = np.linalg.inv(self.calibration_start_pose)
                else:
                    self.node.get_logger().warn(
                        f"Not enough movement ({movement_distance*1000:.1f}mm)"
                    )
                    
        # Update previous button states
        self.prev_grip_state = current_grip
        self.prev_joystick_state = current_joystick
        
    def _matrix_to_pose(self, matrix: np.ndarray) -> Pose:
        """Convert 4x4 transformation matrix to ROS Pose message"""
        from scipy.spatial.transform import Rotation as R
        
        pose = Pose()
        
        # Extract position
        pose.position.x = matrix[0, 3]
        pose.position.y = matrix[1, 3]
        pose.position.z = matrix[2, 3]
        
        # Extract rotation as quaternion
        rotation = R.from_matrix(matrix[:3, :3])
        quat = rotation.as_quat()  # [x, y, z, w]
        
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        
        return pose
        
    async def get_state(self) -> Optional[VRState]:
        """Get latest VR state (non-blocking)"""
        return self._latest_state
        
    def get_latest_state(self) -> Optional[VRState]:
        """Sync version for recording thread"""
        return self._latest_state 