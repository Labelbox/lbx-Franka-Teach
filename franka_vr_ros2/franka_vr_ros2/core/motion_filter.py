"""
Motion Filter - Kalman/Complementary filtering with optional prediction
"""

import numpy as np
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R
from collections import deque
import time


class MotionFilter:
    """Motion filtering and prediction for smooth tracking"""
    
    def __init__(self, node, enable_prediction=True):
        self.node = node
        self.enable_prediction = enable_prediction
        
        # Filter parameters
        self.position_alpha = 0.95  # High = less filtering
        self.orientation_alpha = 0.98  # Even higher for responsive rotation
        self.prediction_horizon_ms = 10.0  # Predict 10ms ahead
        
        # State tracking
        self._last_pose = None
        self._last_time = None
        self._velocity_buffer = deque(maxlen=5)  # For velocity estimation
        
        # Deadzone for noise filtering
        self.position_deadzone = 0.0005  # 0.5mm
        
        self.node.get_logger().info(f"Motion Filter initialized:")
        self.node.get_logger().info(f"  Prediction: {'Enabled' if enable_prediction else 'Disabled'}")
        self.node.get_logger().info(f"  Position alpha: {self.position_alpha}")
        self.node.get_logger().info(f"  Orientation alpha: {self.orientation_alpha}")
        
    def filter_pose(self, pose: Pose) -> Pose:
        """Apply filtering and optional prediction to pose"""
        current_time = time.time()
        
        if self._last_pose is None:
            self._last_pose = pose
            self._last_time = current_time
            return pose
        
        # Time delta
        dt = current_time - self._last_time
        if dt <= 0:
            return self._last_pose
        
        # Filter position
        filtered_pos = self._filter_position(pose.position, dt)
        
        # Filter orientation
        filtered_quat = self._filter_orientation(pose.orientation)
        
        # Create filtered pose
        filtered_pose = Pose()
        filtered_pose.position = filtered_pos
        filtered_pose.orientation = filtered_quat
        
        # Apply prediction if enabled
        if self.enable_prediction and len(self._velocity_buffer) > 2:
            filtered_pose = self._predict_pose(filtered_pose, dt)
        
        # Update state
        self._last_pose = filtered_pose
        self._last_time = current_time
        
        return filtered_pose
    
    def _filter_position(self, new_pos, dt):
        """Apply position filtering with deadzone"""
        # Calculate position change
        pos_delta = np.array([
            new_pos.x - self._last_pose.position.x,
            new_pos.y - self._last_pose.position.y,
            new_pos.z - self._last_pose.position.z
        ])
        
        # Apply deadzone
        for i in range(3):
            if abs(pos_delta[i]) < self.position_deadzone:
                pos_delta[i] = 0.0
        
        # Calculate velocity for prediction
        if dt > 0:
            velocity = pos_delta / dt
            self._velocity_buffer.append((velocity, current_time))
        
        # Apply exponential moving average filter
        filtered_pos = Pose().position
        filtered_pos.x = self.position_alpha * new_pos.x + (1 - self.position_alpha) * self._last_pose.position.x
        filtered_pos.y = self.position_alpha * new_pos.y + (1 - self.position_alpha) * self._last_pose.position.y
        filtered_pos.z = self.position_alpha * new_pos.z + (1 - self.position_alpha) * self._last_pose.position.z
        
        return filtered_pos
    
    def _filter_orientation(self, new_orient):
        """Apply orientation filtering using SLERP"""
        # Convert to quaternions
        q1 = np.array([
            self._last_pose.orientation.x,
            self._last_pose.orientation.y,
            self._last_pose.orientation.z,
            self._last_pose.orientation.w
        ])
        
        q2 = np.array([
            new_orient.x,
            new_orient.y,
            new_orient.z,
            new_orient.w
        ])
        
        # Use scipy for SLERP
        r1 = R.from_quat(q1)
        r2 = R.from_quat(q2)
        
        # SLERP interpolation
        slerp = R.Slerp([0, 1], [r1, r2])
        filtered_rot = slerp(1 - self.orientation_alpha)
        filtered_quat = filtered_rot.as_quat()
        
        # Create orientation message
        filtered_orient = Pose().orientation
        filtered_orient.x = filtered_quat[0]
        filtered_orient.y = filtered_quat[1]
        filtered_orient.z = filtered_quat[2]
        filtered_orient.w = filtered_quat[3]
        
        return filtered_orient
    
    def _predict_pose(self, pose, dt):
        """Predict future pose based on velocity"""
        if len(self._velocity_buffer) < 2:
            return pose
        
        # Calculate average velocity
        velocities = [v[0] for v in self._velocity_buffer]
        avg_velocity = np.mean(velocities, axis=0)
        
        # Predict position
        prediction_time = self.prediction_horizon_ms / 1000.0
        predicted_pos = Pose().position
        predicted_pos.x = pose.position.x + avg_velocity[0] * prediction_time
        predicted_pos.y = pose.position.y + avg_velocity[1] * prediction_time
        predicted_pos.z = pose.position.z + avg_velocity[2] * prediction_time
        
        # Create predicted pose (keep orientation as-is for now)
        predicted_pose = Pose()
        predicted_pose.position = predicted_pos
        predicted_pose.orientation = pose.orientation
        
        return predicted_pose 