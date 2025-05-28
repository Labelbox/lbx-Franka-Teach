"""
Labelbox Transform - Preserves exact coordinate transformations from original implementation
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose


class LabelboxTransform:
    """Preserves exact coordinate transformations from original implementation"""
    
    def __init__(self, node, position_reorder=None, rotation_mode='labelbox'):
        self.node = node
        self.position_reorder = position_reorder or [-3, -1, 2, 4]
        self.rotation_mode = rotation_mode
        
        # Create transformation matrices
        self.global_to_env_mat = self._vec_to_reorder_mat(self.position_reorder)
        self.vr_to_global_mat = np.eye(4)
        
        # Neutral pose for rotation calculations
        self.neutral_quat = None
        
        # Log transformation info
        self.node.get_logger().info(f"Labelbox Transform initialized:")
        self.node.get_logger().info(f"  Position reorder: {self.position_reorder}")
        self.node.get_logger().info(f"  Rotation mode: {self.rotation_mode}")
        
    def set_calibration(self, vr_to_global_mat, neutral_pose=None):
        """Update calibration from VR input handler"""
        self.vr_to_global_mat = vr_to_global_mat
        if neutral_pose is not None:
            # Extract neutral quaternion from pose matrix
            neutral_rot = R.from_matrix(neutral_pose[:3, :3])
            self.neutral_quat = neutral_rot.as_quat()
        
    def transform_pose(self, vr_pose: Pose) -> Pose:
        """Apply Labelbox coordinate transformation"""
        # Convert pose to matrix
        vr_mat = self._pose_to_matrix(vr_pose)
        
        # Apply position transformation
        transformed_mat = self.global_to_env_mat @ self.vr_to_global_mat @ vr_mat
        robot_pos = transformed_mat[:3, 3]
        
        # Apply rotation transformation (labelbox mode)
        vr_quat = np.array([
            vr_pose.orientation.x,
            vr_pose.orientation.y,
            vr_pose.orientation.z,
            vr_pose.orientation.w
        ])
        robot_quat = self._apply_labelbox_rotation(vr_quat)
        
        # Create output pose
        robot_pose = Pose()
        robot_pose.position.x = robot_pos[0]
        robot_pose.position.y = robot_pos[1]
        robot_pose.position.z = robot_pos[2]
        robot_pose.orientation.x = robot_quat[0]
        robot_pose.orientation.y = robot_quat[1]
        robot_pose.orientation.z = robot_quat[2]
        robot_pose.orientation.w = robot_quat[3]
        
        return robot_pose
    
    def _apply_labelbox_rotation(self, vr_quat):
        """Apply labelbox rotation mapping - preserved from original"""
        # If we have a neutral pose, calculate relative rotation
        if self.neutral_quat is not None:
            # Calculate relative rotation from neutral pose
            neutral_rot = R.from_quat(self.neutral_quat)
            current_rot = R.from_quat(vr_quat)
            
            # Get relative rotation
            relative_rot = neutral_rot.inv() * current_rot
            
            # Convert to axis-angle
            rotvec = relative_rot.as_rotvec()
            angle = np.linalg.norm(rotvec)
            
            if angle > 0:
                axis = rotvec / angle
                
                # Apply labelbox transformation
                # VR axes (confirmed by calibration): X=pitch, Y=roll, Z=yaw
                # Robot axes: X=roll, Y=pitch, Z=yaw
                # Desired mapping:
                # VR Roll (Y-axis) → Robot Pitch (Y-axis) - INVERTED for ergonomics
                # VR Pitch (X-axis) → Robot Roll (X-axis)
                # VR Yaw (Z-axis) → Robot Yaw (Z-axis)
                # We need to swap X and Y components and negate Y for inverted roll
                transformed_axis = np.array([-axis[1], axis[0], axis[2]])
                
                # Create new rotation from transformed axis and same angle
                transformed_rotvec = transformed_axis * angle
                transformed_rot = R.from_rotvec(transformed_rotvec)
                return transformed_rot.as_quat()
            else:
                # No rotation
                return np.array([0, 0, 0, 1])
        else:
            # No neutral pose yet, return original quaternion
            # (will be updated once calibration is complete)
            return vr_quat
    
    def _vec_to_reorder_mat(self, vec):
        """Convert reorder vector to transformation matrix"""
        X = np.zeros((len(vec), len(vec)))
        for i in range(len(vec)):
            ind = int(abs(vec[i])) - 1
            X[i, ind] = np.sign(vec[i])
        return X
    
    def _pose_to_matrix(self, pose):
        """Convert ROS Pose to 4x4 transformation matrix"""
        mat = np.eye(4)
        
        # Position
        mat[0, 3] = pose.position.x
        mat[1, 3] = pose.position.y
        mat[2, 3] = pose.position.z
        
        # Orientation
        quat = [pose.orientation.x, pose.orientation.y, 
                pose.orientation.z, pose.orientation.w]
        rot = R.from_quat(quat)
        mat[:3, :3] = rot.as_matrix()
        
        return mat 