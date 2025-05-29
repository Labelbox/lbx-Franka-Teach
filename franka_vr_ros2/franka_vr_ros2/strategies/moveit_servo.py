"""
MoveIt Servo control strategy for smooth teleoperation
"""

import asyncio
import numpy as np
from geometry_msgs.msg import TwistStamped, Pose, PoseStamped
from std_srvs.srv import Trigger
from control_msgs.msg import JointJog
from franka_vr_ros2.strategies.base import ControlStrategyBase
from scipy.spatial.transform import Rotation as R


class MoveItServoStrategy(ControlStrategyBase):
    """MoveIt Servo control strategy for smooth teleoperation"""
    
    def __init__(self, node, robot_ip):
        super().__init__(node, robot_ip)
        
        # Publishers
        self.twist_pub = node.create_publisher(
            TwistStamped, 
            '/servo_node/delta_twist_cmds', 
            10
        )
        
        self.pose_pub = node.create_publisher(
            PoseStamped,
            '/servo_node/pose_target_cmds',
            10
        )
        
        # Service clients
        self.servo_start_client = node.create_client(
            Trigger, 
            '/servo_node/start_servo'
        )
        
        self.servo_stop_client = node.create_client(
            Trigger,
            '/servo_node/stop_servo'
        )
        
        self._last_pose = None
        self._control_mode = 'twist'  # 'twist' or 'pose'
        
        self.node.get_logger().info("MoveIt Servo strategy created")
        
    async def initialize(self):
        """Start MoveIt Servo"""
        # Wait for servo service
        self.node.get_logger().info("Waiting for servo_node services...")
        
        while not self.servo_start_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for servo_node to start...')
            
        # Start servo
        self.node.get_logger().info("Starting MoveIt Servo...")
        request = Trigger.Request()
        
        try:
            future = self.servo_start_client.call_async(request)
            response = await future
            
            if response.success:
                self._initialized = True
                self.node.get_logger().info("MoveIt Servo started successfully")
            else:
                self.node.get_logger().error(f"Failed to start servo: {response.message}")
                
        except Exception as e:
            self.node.get_logger().error(f"Error starting servo: {e}")
            
    async def send_command(self, target_pose: Pose):
        """Convert pose to twist and send to servo"""
        if not self._initialized:
            return
            
        if self._control_mode == 'twist':
            await self._send_twist_command(target_pose)
        else:
            await self._send_pose_command(target_pose)
            
    async def _send_twist_command(self, target_pose: Pose):
        """Send twist command for velocity-based control"""
        if self._last_pose is None:
            self._last_pose = target_pose
            return
            
        # Calculate twist from pose difference
        twist = TwistStamped()
        twist.header.stamp = self.node.get_clock().now().to_msg()
        twist.header.frame_id = "panda_link0"  # Base frame
        
        # Position difference (direct tracking)
        dt = 0.004  # 250Hz target
        twist.twist.linear.x = (target_pose.position.x - self._last_pose.position.x) / dt
        twist.twist.linear.y = (target_pose.position.y - self._last_pose.position.y) / dt
        twist.twist.linear.z = (target_pose.position.z - self._last_pose.position.z) / dt
        
        # Orientation difference using quaternion
        q1 = np.array([
            self._last_pose.orientation.x,
            self._last_pose.orientation.y,
            self._last_pose.orientation.z,
            self._last_pose.orientation.w
        ])
        
        q2 = np.array([
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w
        ])
        
        # Calculate angular velocity from quaternion difference
        r1 = R.from_quat(q1)
        r2 = R.from_quat(q2)
        
        # Relative rotation
        r_diff = r2 * r1.inv()
        
        # Convert to axis-angle
        rotvec = r_diff.as_rotvec()
        
        # Angular velocity
        twist.twist.angular.x = rotvec[0] / dt
        twist.twist.angular.y = rotvec[1] / dt
        twist.twist.angular.z = rotvec[2] / dt
        
        # Apply velocity limits
        max_linear_vel = 1.0  # m/s
        max_angular_vel = 1.0  # rad/s
        
        # Limit linear velocity
        linear_vel = np.array([
            twist.twist.linear.x,
            twist.twist.linear.y,
            twist.twist.linear.z
        ])
        linear_norm = np.linalg.norm(linear_vel)
        if linear_norm > max_linear_vel:
            linear_vel = linear_vel * max_linear_vel / linear_norm
            twist.twist.linear.x = linear_vel[0]
            twist.twist.linear.y = linear_vel[1]
            twist.twist.linear.z = linear_vel[2]
            
        # Limit angular velocity
        angular_vel = np.array([
            twist.twist.angular.x,
            twist.twist.angular.y,
            twist.twist.angular.z
        ])
        angular_norm = np.linalg.norm(angular_vel)
        if angular_norm > max_angular_vel:
            angular_vel = angular_vel * max_angular_vel / angular_norm
            twist.twist.angular.x = angular_vel[0]
            twist.twist.angular.y = angular_vel[1]
            twist.twist.angular.z = angular_vel[2]
        
        # Publish twist
        self.twist_pub.publish(twist)
        self._last_pose = target_pose
        
    async def _send_pose_command(self, target_pose: Pose):
        """Send pose command for position-based control"""
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "panda_link0"
        pose_stamped.pose = target_pose
        
        self.pose_pub.publish(pose_stamped)
        
    async def stop(self):
        """Stop MoveIt Servo"""
        if self._initialized:
            request = Trigger.Request()
            future = self.servo_stop_client.call_async(request)
            await future
            self._initialized = False 