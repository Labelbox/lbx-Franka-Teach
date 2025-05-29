"""
Data types for VR teleoperation
"""

from dataclasses import dataclass
from typing import Dict, Optional
import numpy as np
from geometry_msgs.msg import Pose
import copy


@dataclass
class VRState:
    """Thread-safe VR controller state"""
    timestamp: float
    pose: Pose
    buttons: Dict[str, bool]
    movement_enabled: bool
    controller_on: bool
    
    def copy(self):
        """Deep copy for thread safety"""
        return VRState(
            timestamp=self.timestamp,
            pose=copy.deepcopy(self.pose),
            buttons=copy.deepcopy(self.buttons),
            movement_enabled=self.movement_enabled,
            controller_on=self.controller_on
        )


@dataclass
class RobotState:
    """Thread-safe robot state"""
    timestamp: float
    pos: np.ndarray
    quat: np.ndarray
    gripper: float
    joint_positions: Optional[np.ndarray]
    
    def copy(self):
        """Deep copy for thread safety"""
        return RobotState(
            timestamp=self.timestamp,
            pos=self.pos.copy() if self.pos is not None else None,
            quat=self.quat.copy() if self.quat is not None else None,
            gripper=self.gripper,
            joint_positions=self.joint_positions.copy() if self.joint_positions is not None else None
        ) 