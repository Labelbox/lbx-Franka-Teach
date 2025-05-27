"""
FR3 Robot Model - Kinematics and Dynamics for Franka FR3
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
from typing import Tuple, Optional, List


class FR3RobotModel:
    """
    Franka FR3 Robot Model with forward kinematics and joint limits
    Based on DH parameters and specifications from Franka documentation
    """
    
    def __init__(self):
        # FR3 DH parameters (modified DH convention)
        # a, d, alpha values from Franka documentation
        self.dh_params = {
            'a': [0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0],  # link lengths
            'd': [0.333, 0, 0.316, 0, 0.384, 0, 0, 0.107],  # link offsets
            'alpha': [0, -np.pi/2, np.pi/2, np.pi/2, -np.pi/2, np.pi/2, np.pi/2, 0]  # link twists
        }
        
        # Joint limits (radians) - from FR3 configuration
        self.joint_limits_low = np.array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159])
        self.joint_limits_high = np.array([2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159])
        
        # Rest pose (home position)
        self.rest_pose = np.array([-0.13935425877571106, -0.020481698215007782, -0.05201413854956627, 
                                   -2.0691256523132324, 0.05058913677930832, 2.0028650760650635, 
                                   -0.9167874455451965])
        
        # Torque limits (Nm)
        self.torque_limits = np.array([87., 87., 87., 87., 12., 12., 12.])
        
        # Joint damping coefficients
        self.joint_damping = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        
        # Number of joints
        self.num_joints = 7
        
        # End-effector link index
        self.ee_link_idx = 7
        
    def dh_transform(self, a: float, d: float, alpha: float, theta: float) -> np.ndarray:
        """
        Calculate transformation matrix from DH parameters
        
        Args:
            a: Link length
            d: Link offset
            alpha: Link twist
            theta: Joint angle
            
        Returns:
            4x4 transformation matrix
        """
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
    
    def forward_kinematics(self, joint_angles: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Calculate forward kinematics for FR3 robot
        
        Args:
            joint_angles: 7-element array of joint angles (radians)
            
        Returns:
            position: 3D position of end-effector
            quaternion: Orientation as quaternion (x,y,z,w)
        """
        if len(joint_angles) != self.num_joints:
            raise ValueError(f"Expected {self.num_joints} joint angles, got {len(joint_angles)}")
        
        # Initialize transformation matrix
        T = np.eye(4)
        
        # Apply DH transformations for each joint
        for i in range(self.num_joints):
            T_i = self.dh_transform(
                self.dh_params['a'][i],
                self.dh_params['d'][i],
                self.dh_params['alpha'][i],
                joint_angles[i]
            )
            T = T @ T_i
        
        # Add final transformation to end-effector
        T_ee = self.dh_transform(
            self.dh_params['a'][7],
            self.dh_params['d'][7],
            self.dh_params['alpha'][7],
            0
        )
        T = T @ T_ee
        
        # Extract position and orientation
        position = T[:3, 3]
        rotation_matrix = T[:3, :3]
        quaternion = R.from_matrix(rotation_matrix).as_quat()  # Returns [x, y, z, w]
        
        return position, quaternion
    
    def get_link_transforms(self, joint_angles: np.ndarray) -> List[np.ndarray]:
        """
        Get transformation matrices for all links
        
        Args:
            joint_angles: 7-element array of joint angles
            
        Returns:
            List of 4x4 transformation matrices for each link
        """
        transforms = []
        T = np.eye(4)
        
        for i in range(self.num_joints + 1):
            if i < self.num_joints:
                T_i = self.dh_transform(
                    self.dh_params['a'][i],
                    self.dh_params['d'][i],
                    self.dh_params['alpha'][i],
                    joint_angles[i] if i < len(joint_angles) else 0
                )
                T = T @ T_i
            else:
                # Final end-effector transform
                T_ee = self.dh_transform(
                    self.dh_params['a'][7],
                    self.dh_params['d'][7],
                    self.dh_params['alpha'][7],
                    0
                )
                T = T @ T_ee
            
            transforms.append(T.copy())
        
        return transforms
    
    def check_joint_limits(self, joint_angles: np.ndarray) -> Tuple[bool, Optional[List[int]]]:
        """
        Check if joint angles are within limits
        
        Args:
            joint_angles: Array of joint angles
            
        Returns:
            valid: True if all joints within limits
            violations: List of joint indices that violate limits (None if valid)
        """
        violations = []
        
        for i in range(self.num_joints):
            if joint_angles[i] < self.joint_limits_low[i] or joint_angles[i] > self.joint_limits_high[i]:
                violations.append(i)
        
        return len(violations) == 0, violations if violations else None
    
    def clip_joint_angles(self, joint_angles: np.ndarray) -> np.ndarray:
        """
        Clip joint angles to valid range
        
        Args:
            joint_angles: Array of joint angles
            
        Returns:
            Clipped joint angles
        """
        return np.clip(joint_angles, self.joint_limits_low, self.joint_limits_high)
    
    def jacobian(self, joint_angles: np.ndarray) -> np.ndarray:
        """
        Calculate the geometric Jacobian matrix
        
        Args:
            joint_angles: Current joint angles
            
        Returns:
            6x7 Jacobian matrix [linear_velocity; angular_velocity]
        """
        # Get all link transforms
        transforms = self.get_link_transforms(joint_angles)
        
        # End-effector position
        p_ee = transforms[-1][:3, 3]
        
        # Initialize Jacobian
        J = np.zeros((6, self.num_joints))
        
        # Calculate Jacobian columns
        for i in range(self.num_joints):
            # Get z-axis of joint i (rotation axis)
            if i == 0:
                z_i = np.array([0, 0, 1])  # Base z-axis
                p_i = np.array([0, 0, 0])  # Base position
            else:
                z_i = transforms[i-1][:3, 2]  # z-axis of previous link
                p_i = transforms[i-1][:3, 3]  # position of previous link
            
            # Linear velocity component (z_i x (p_ee - p_i))
            J[:3, i] = np.cross(z_i, p_ee - p_i)
            
            # Angular velocity component (z_i)
            J[3:, i] = z_i
        
        return J 