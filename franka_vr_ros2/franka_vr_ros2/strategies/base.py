"""
Base class for control strategies
"""

from abc import ABC, abstractmethod
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from franka_vr_ros2.core.types import RobotState
from typing import Optional


class ControlStrategyBase(ABC):
    """Base class for control strategies"""
    
    def __init__(self, node, robot_ip):
        self.node = node
        self.robot_ip = robot_ip
        self._robot_state = None
        self._initialized = False
        
        # Subscribe to joint states
        self.joint_state_sub = node.create_subscription(
            JointState,
            'joint_states',
            self._joint_state_callback,
            10
        )
        
    def _joint_state_callback(self, msg: JointState):
        """Update robot state from joint states"""
        # Extract end-effector pose from joint states
        # This is a simplified version - in practice, you'd use forward kinematics
        self._robot_state = RobotState(
            timestamp=self.node.get_clock().now().seconds_nanoseconds()[0],
            pos=None,  # Would be computed from FK
            quat=None,  # Would be computed from FK
            gripper=0.0,  # Would come from gripper state
            joint_positions=msg.position if hasattr(msg, 'position') else None
        )
        
    @abstractmethod
    async def initialize(self):
        """Initialize the control strategy"""
        pass
    
    @abstractmethod
    async def send_command(self, target_pose: Pose):
        """Send control command to robot"""
        pass
    
    def get_robot_state(self) -> Optional[RobotState]:
        """Get current robot state"""
        return self._robot_state
    
    def is_initialized(self) -> bool:
        """Check if strategy is initialized"""
        return self._initialized 