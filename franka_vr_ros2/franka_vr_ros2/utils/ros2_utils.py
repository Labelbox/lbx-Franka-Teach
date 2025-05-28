"""
ROS2 utility functions
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
from geometry_msgs.msg import Pose, Transform, Quaternion, Vector3
from scipy.spatial.transform import Rotation as R


def create_reliable_qos(depth=10):
    """Create a reliable QoS profile for important topics"""
    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=DurabilityPolicy.VOLATILE
    )
    return qos


def create_best_effort_qos(depth=1):
    """Create a best-effort QoS profile for high-frequency topics"""
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=DurabilityPolicy.VOLATILE
    )
    return qos


def numpy_to_pose(position: np.ndarray, quaternion: np.ndarray) -> Pose:
    """Convert numpy arrays to ROS Pose message"""
    pose = Pose()
    pose.position.x = float(position[0])
    pose.position.y = float(position[1])
    pose.position.z = float(position[2])
    pose.orientation.x = float(quaternion[0])
    pose.orientation.y = float(quaternion[1])
    pose.orientation.z = float(quaternion[2])
    pose.orientation.w = float(quaternion[3])
    return pose


def pose_to_numpy(pose: Pose) -> tuple:
    """Convert ROS Pose to numpy arrays (position, quaternion)"""
    position = np.array([
        pose.position.x,
        pose.position.y,
        pose.position.z
    ])
    quaternion = np.array([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    ])
    return position, quaternion


def transform_to_matrix(transform: Transform) -> np.ndarray:
    """Convert ROS Transform to 4x4 transformation matrix"""
    mat = np.eye(4)
    
    # Translation
    mat[0, 3] = transform.translation.x
    mat[1, 3] = transform.translation.y
    mat[2, 3] = transform.translation.z
    
    # Rotation
    quat = [
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w
    ]
    rot = R.from_quat(quat)
    mat[:3, :3] = rot.as_matrix()
    
    return mat


def matrix_to_transform(matrix: np.ndarray) -> Transform:
    """Convert 4x4 transformation matrix to ROS Transform"""
    transform = Transform()
    
    # Translation
    transform.translation.x = matrix[0, 3]
    transform.translation.y = matrix[1, 3]
    transform.translation.z = matrix[2, 3]
    
    # Rotation
    rot = R.from_matrix(matrix[:3, :3])
    quat = rot.as_quat()
    transform.rotation.x = quat[0]
    transform.rotation.y = quat[1]
    transform.rotation.z = quat[2]
    transform.rotation.w = quat[3]
    
    return transform 