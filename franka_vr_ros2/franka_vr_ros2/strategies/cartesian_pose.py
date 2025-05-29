"""
Cartesian Pose control strategy for direct pose commands
"""

from geometry_msgs.msg import Pose, PoseStamped
from franka_vr_ros2.strategies.base import ControlStrategyBase


class CartesianPoseStrategy(ControlStrategyBase):
    """Direct Cartesian pose control strategy"""
    
    def __init__(self, node, robot_ip):
        super().__init__(node, robot_ip)
        
        # Publisher for Cartesian pose commands
        self.pose_pub = node.create_publisher(
            PoseStamped,
            '/cartesian_pose_controller/command',
            10
        )
        
        self.node.get_logger().info("Cartesian Pose strategy created")
        
    async def initialize(self):
        """Initialize Cartesian controller"""
        # TODO: Check if controller is running
        self._initialized = True
        self.node.get_logger().info("Cartesian Pose strategy initialized")
        
    async def send_command(self, target_pose: Pose):
        """Send Cartesian pose command"""
        if not self._initialized:
            return
            
        # Create stamped pose
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "panda_link0"
        pose_stamped.pose = target_pose
        
        # Publish command
        self.pose_pub.publish(pose_stamped) 