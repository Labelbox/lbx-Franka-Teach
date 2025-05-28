"""Control strategies for VR teleoperation"""

from franka_vr_ros2.strategies.base import ControlStrategyBase
from franka_vr_ros2.strategies.moveit_servo import MoveItServoStrategy
from franka_vr_ros2.strategies.direct_ik import DirectIKStrategy
from franka_vr_ros2.strategies.cartesian_pose import CartesianPoseStrategy


def get_strategy(strategy_name: str, node, robot_ip: str) -> ControlStrategyBase:
    """Factory function to create control strategies"""
    strategies = {
        'moveit_servo': MoveItServoStrategy,
        'direct_ik': DirectIKStrategy,
        'cartesian_pose': CartesianPoseStrategy,
    }
    
    if strategy_name not in strategies:
        raise ValueError(f"Unknown strategy: {strategy_name}. Available: {list(strategies.keys())}")
    
    return strategies[strategy_name](node, robot_ip) 