"""
FR3 Robot PyBullet Visualizer - Physics-based 3D visualization
Based on deoxys PyBullet implementation
"""

import numpy as np
import pybullet as p
import pybullet_data
import time
import pathlib
from typing import Optional, List, Tuple

from .fr3_robot_model import FR3RobotModel


class FR3PyBulletVisualizer:
    """
    PyBullet-based 3D Visualizer for Franka FR3 robot
    """
    
    def __init__(self, robot_model: FR3RobotModel, gui: bool = True):
        """
        Initialize PyBullet visualizer
        
        Args:
            robot_model: FR3RobotModel instance
            gui: Whether to show GUI (False for headless)
        """
        self.robot_model = robot_model
        self.gui = gui
        
        # Connect to PyBullet
        if self.gui:
            self.physics_client = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        else:
            self.physics_client = p.connect(p.DIRECT)
        
        # Set up physics parameters
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1.0 / 240.0)
        
        # Add PyBullet data path
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Load plane
        self.plane_id = p.loadURDF("plane.urdf", [0, 0, 0])
        
        # Try to load Panda URDF from deoxys path first
        deoxys_path = pathlib.Path(__file__).parent.parent / "lbx-deoxys_control/deoxys/deoxys/franka_interface/robot_models/panda"
        panda_urdf = deoxys_path / "panda.urdf"
        
        if panda_urdf.exists():
            urdf_path = str(panda_urdf)
        else:
            # Fallback to PyBullet's built-in Franka Panda model
            urdf_path = "franka_panda/panda.urdf"
        
        # Load robot
        self.robot_id = p.loadURDF(
            urdf_path,
            basePosition=[0, 0, 0],
            baseOrientation=[0, 0, 0, 1],
            useFixedBase=True,
            flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT
        )
        
        # Get joint info
        self.num_joints = p.getNumJoints(self.robot_id)
        self.joint_indices = []
        self.joint_names = []
        
        # Find the 7 arm joints (excluding gripper)
        for i in range(self.num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('utf-8')
            joint_type = joint_info[2]
            
            # Only consider revolute joints for the arm
            if joint_type == p.JOINT_REVOLUTE and 'finger' not in joint_name.lower():
                self.joint_indices.append(i)
                self.joint_names.append(joint_name)
                
                # Set joint limits
                if len(self.joint_indices) <= 7:
                    idx = len(self.joint_indices) - 1
                    p.changeDynamics(
                        self.robot_id, i,
                        jointLowerLimit=self.robot_model.joint_limits_low[idx],
                        jointUpperLimit=self.robot_model.joint_limits_high[idx]
                    )
        
        # Keep only first 7 joints (arm joints)
        self.joint_indices = self.joint_indices[:7]
        self.joint_names = self.joint_names[:7]
        
        # Find end-effector link
        self.ee_link_index = 11  # Usually link 11 for Panda
        
        # Gripper joint indices (if available)
        self.gripper_indices = []
        for i in range(self.num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('utf-8')
            if 'finger' in joint_name.lower():
                self.gripper_indices.append(i)
        
        # Set camera
        if self.gui:
            p.resetDebugVisualizerCamera(
                cameraDistance=1.5,
                cameraYaw=45,
                cameraPitch=-30,
                cameraTargetPosition=[0.5, 0, 0.3]
            )
        
        # Trajectory visualization
        self.trajectory_points = []
        self.trajectory_ids = []
        self.max_trajectory_points = 500
        
        # Workspace visualization
        self._draw_workspace_bounds()
        
        # Target marker
        self.target_marker_id = None
        
    def _draw_workspace_bounds(self):
        """Draw robot workspace boundary as lines"""
        # Define workspace corners
        x_min, y_min, z_min = 0.2, -0.4, 0.05
        x_max, y_max, z_max = 0.75, 0.4, 0.7
        
        # Define edges of the box
        edges = [
            # Bottom face
            ([x_min, y_min, z_min], [x_max, y_min, z_min]),
            ([x_max, y_min, z_min], [x_max, y_max, z_min]),
            ([x_max, y_max, z_min], [x_min, y_max, z_min]),
            ([x_min, y_max, z_min], [x_min, y_min, z_min]),
            # Top face
            ([x_min, y_min, z_max], [x_max, y_min, z_max]),
            ([x_max, y_min, z_max], [x_max, y_max, z_max]),
            ([x_max, y_max, z_max], [x_min, y_max, z_max]),
            ([x_min, y_max, z_max], [x_min, y_min, z_max]),
            # Vertical edges
            ([x_min, y_min, z_min], [x_min, y_min, z_max]),
            ([x_max, y_min, z_min], [x_max, y_min, z_max]),
            ([x_max, y_max, z_min], [x_max, y_max, z_max]),
            ([x_min, y_max, z_min], [x_min, y_max, z_max]),
        ]
        
        # Draw edges
        for start, end in edges:
            p.addUserDebugLine(
                start, end,
                lineColorRGB=[0.5, 0.5, 0.5],
                lineWidth=1,
                lifeTime=0  # Permanent
            )
    
    def update_robot_pose(self, joint_angles: np.ndarray, 
                         gripper_state: float = 0.0,
                         show_trajectory: bool = True):
        """
        Update robot visualization with new joint angles
        
        Args:
            joint_angles: 7-element array of joint angles
            gripper_state: Gripper state (0=open, 1=closed)
            show_trajectory: Whether to show end-effector trajectory
        """
        # Set joint positions
        for i, joint_idx in enumerate(self.joint_indices):
            p.resetJointState(
                self.robot_id,
                joint_idx,
                joint_angles[i]
            )
        
        # Set gripper position if available
        if self.gripper_indices:
            # Panda gripper: 0.04 = open, 0.0 = closed
            gripper_pos = 0.04 * (1 - gripper_state)
            for gripper_idx in self.gripper_indices:
                p.resetJointState(
                    self.robot_id,
                    gripper_idx,
                    gripper_pos
                )
        
        # Get end-effector position for trajectory
        if show_trajectory:
            ee_state = p.getLinkState(self.robot_id, self.ee_link_index)
            ee_pos = ee_state[0]
            
            # Add to trajectory
            self.trajectory_points.append(ee_pos)
            
            # Limit trajectory length
            if len(self.trajectory_points) > self.max_trajectory_points:
                self.trajectory_points.pop(0)
                # Remove old line
                if self.trajectory_ids:
                    p.removeUserDebugItem(self.trajectory_ids.pop(0))
            
            # Draw trajectory line
            if len(self.trajectory_points) > 1:
                line_id = p.addUserDebugLine(
                    self.trajectory_points[-2],
                    self.trajectory_points[-1],
                    lineColorRGB=[0, 1, 0],
                    lineWidth=2,
                    lifeTime=0
                )
                self.trajectory_ids.append(line_id)
        
        # Step simulation for rendering
        p.stepSimulation()
    
    def set_target_marker(self, position: np.ndarray, orientation: Optional[np.ndarray] = None):
        """
        Display a target marker at the specified position
        
        Args:
            position: 3D position for the marker
            orientation: Optional quaternion for orientation
        """
        # Remove old marker
        if self.target_marker_id is not None:
            p.removeBody(self.target_marker_id)
        
        # Create visual shape for marker
        visual_shape_id = p.createVisualShape(
            shapeType=p.GEOM_SPHERE,
            radius=0.02,
            rgbaColor=[1, 0, 0, 0.5]
        )
        
        # Create marker
        if orientation is None:
            orientation = [0, 0, 0, 1]
            
        self.target_marker_id = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=visual_shape_id,
            basePosition=position,
            baseOrientation=orientation
        )
    
    def clear_trajectory(self):
        """Clear the trajectory visualization"""
        for line_id in self.trajectory_ids:
            p.removeUserDebugItem(line_id)
        self.trajectory_ids.clear()
        self.trajectory_points.clear()
    
    def get_camera_image(self, width: int = 640, height: int = 480):
        """
        Get camera image from current viewpoint
        
        Args:
            width: Image width
            height: Image height
            
        Returns:
            RGB image as numpy array
        """
        # Get current camera info
        view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=[0.5, 0, 0.3],
            distance=1.5,
            yaw=45,
            pitch=-30,
            roll=0,
            upAxisIndex=2
        )
        
        proj_matrix = p.computeProjectionMatrixFOV(
            fov=60,
            aspect=float(width) / height,
            nearVal=0.1,
            farVal=100.0
        )
        
        # Get camera image
        _, _, rgb, _, _ = p.getCameraImage(
            width=width,
            height=height,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )
        
        return np.array(rgb)[:, :, :3]
    
    def close(self):
        """Close the PyBullet connection"""
        p.disconnect(self.physics_client) 