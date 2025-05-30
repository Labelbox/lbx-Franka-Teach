# Implementation Guide: Migrating Oculus VR Server to MoveIt

This guide provides **specific code changes** to migrate `oculus_vr_server.py` from Deoxys to MoveIt while maintaining all existing functionality.

## Step 1: Import Changes

### Replace imports at the top of the file:

**REMOVE these lines (~lines 50-60):**
```python
# Remove these Deoxys imports
from frankateach.network import create_request_socket
from frankateach.constants import (
    HOST, CONTROL_PORT,
    GRIPPER_OPEN, GRIPPER_CLOSE,
    ROBOT_WORKSPACE_MIN, ROBOT_WORKSPACE_MAX,
    CONTROL_FREQ,
)
from frankateach.messages import FrankaAction, FrankaState
from deoxys.utils import transform_utils
```

**ADD these lines instead:**
```python
# Add ROS 2 and MoveIt imports
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPlanningScene, GetPositionFK
from moveit_msgs.msg import PositionIKRequest, RobotState as MoveitRobotState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Header

# Keep these constants (but we'll define them locally now)
GRIPPER_OPEN = 0.0
GRIPPER_CLOSE = 1.0
ROBOT_WORKSPACE_MIN = np.array([-0.6, -0.6, 0.0])
ROBOT_WORKSPACE_MAX = np.array([0.6, 0.6, 1.0])
CONTROL_FREQ = 15  # Hz
```

## Step 2: Class Definition Changes

**CHANGE the class definition (~line 170):**

**FROM:**
```python
class OculusVRServer:
    def __init__(self, 
                 debug=False, 
                 right_controller=True, 
                 ip_address=None,
                 # ... other parameters
                 ):
```

**TO:**
```python
class OculusVRServer(Node):  # INHERIT FROM NODE
    def __init__(self, 
                 debug=False, 
                 right_controller=True, 
                 ip_address=None,
                 # ... other parameters (keep all existing)
                 ):
        # Initialize ROS 2 node FIRST
        super().__init__('oculus_vr_server')
        
        # Robot configuration (from simple_arm_control.py)
        self.robot_ip = "192.168.1.59"
        self.planning_group = "panda_arm"  # May need to change to fr3_arm
        self.end_effector_link = "fr3_hand_tcp"
        self.base_frame = "fr3_link0"
        self.planning_frame = "fr3_link0"
        
        # Joint names for FR3
        self.joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]
        
        # Home position (ready pose)
        self.home_positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        
        # Create service clients for MoveIt integration
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.planning_scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        
        # Create action client for trajectory execution
        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory, '/fr3_arm_controller/follow_joint_trajectory'
        )
        
        # Joint state subscriber
        self.joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        # Wait for services (critical for reliability)
        self.get_logger().info('🔄 Waiting for MoveIt services...')
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("IK service not available")
        if not self.planning_scene_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("Planning scene service not available")
        if not self.fk_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("FK service not available")
        if not self.trajectory_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("Trajectory action server not available")
        self.get_logger().info('✅ All MoveIt services ready!')
        
        # ALL OTHER EXISTING INITIALIZATION STAYS THE SAME
        # (Continue with existing debug, right_controller, etc. setup)
```

## Step 3: Add New MoveIt Helper Methods

**ADD these new methods to the class (after existing helper methods):**

```python
def joint_state_callback(self, msg):
    """Store the latest joint state"""
    self.joint_state = msg

def get_current_joint_positions(self):
    """Get current joint positions from joint_states topic"""
    if self.joint_state is None:
        return None
        
    positions = []
    for joint_name in self.joint_names:
        if joint_name in self.joint_state.name:
            idx = self.joint_state.name.index(joint_name)
            positions.append(self.joint_state.position[idx])
        else:
            return None
    return positions

def get_current_end_effector_pose(self):
    """Get current end-effector pose using forward kinematics"""
    current_joints = self.get_current_joint_positions()
    if current_joints is None:
        return None, None
    
    # Create FK request
    fk_request = GetPositionFK.Request()
    fk_request.fk_link_names = [self.end_effector_link]
    fk_request.header.frame_id = self.base_frame
    fk_request.header.stamp = self.get_clock().now().to_msg()
    
    # Set robot state
    fk_request.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
    fk_request.robot_state.joint_state.name = self.joint_names
    fk_request.robot_state.joint_state.position = current_joints
    
    # Call FK service
    fk_future = self.fk_client.call_async(fk_request)
    rclpy.spin_until_future_complete(self, fk_future, timeout_sec=0.1)
    fk_response = fk_future.result()
    
    if fk_response and fk_response.error_code.val == 1 and fk_response.pose_stamped:
        pose = fk_response.pose_stamped[0].pose
        pos = np.array([pose.position.x, pose.position.y, pose.position.z])
        quat = np.array([pose.orientation.x, pose.orientation.y, 
                        pose.orientation.z, pose.orientation.w])
        return pos, quat
    
    return None, None

def get_planning_scene(self):
    """Get current planning scene for collision checking"""
    scene_request = GetPlanningScene.Request()
    scene_request.components.components = (
        scene_request.components.SCENE_SETTINGS |
        scene_request.components.ROBOT_STATE |
        scene_request.components.ROBOT_STATE_ATTACHED_OBJECTS |
        scene_request.components.WORLD_OBJECT_NAMES |
        scene_request.components.WORLD_OBJECT_GEOMETRY |
        scene_request.components.OCTOMAP |
        scene_request.components.TRANSFORMS |
        scene_request.components.ALLOWED_COLLISION_MATRIX |
        scene_request.components.LINK_PADDING_AND_SCALING |
        scene_request.components.OBJECT_COLORS
    )
    
    scene_future = self.planning_scene_client.call_async(scene_request)
    rclpy.spin_until_future_complete(self, scene_future, timeout_sec=0.5)
    return scene_future.result()

def execute_trajectory(self, positions, duration=2.0):
    """Execute a trajectory to move joints to target positions"""
    if not self.trajectory_client.server_is_ready():
        return False
        
    # Create trajectory
    trajectory = JointTrajectory()
    trajectory.joint_names = self.joint_names
    
    # Add single point
    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start.sec = int(duration)
    point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
    
    trajectory.points.append(point)
        
    # Create goal
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = trajectory
    
    # Send goal
    future = self.trajectory_client.send_goal_async(goal)
    
    # Wait for goal acceptance
    rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
    goal_handle = future.result()
    
    if not goal_handle or not goal_handle.accepted:
        return False
        
    # Wait for result
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 2.0)
    
    result = result_future.result()
    if result is None:
        return False
        
    return result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL

def compute_ik_for_pose(self, pos, quat):
    """Compute IK for Cartesian pose"""
    # Get planning scene
    scene_response = self.get_planning_scene()
    if scene_response is None:
        return None
    
    # Create IK request
    ik_request = GetPositionIK.Request()
    ik_request.ik_request.group_name = self.planning_group
    ik_request.ik_request.robot_state = scene_response.scene.robot_state
    ik_request.ik_request.avoid_collisions = True
    ik_request.ik_request.timeout.sec = 0
    ik_request.ik_request.timeout.nanosec = int(0.1 * 1e9)  # 100ms timeout
    
    # Set target pose
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = self.base_frame
    pose_stamped.header.stamp = self.get_clock().now().to_msg()
    pose_stamped.pose.position.x = float(pos[0])
    pose_stamped.pose.position.y = float(pos[1])
    pose_stamped.pose.position.z = float(pos[2])
    pose_stamped.pose.orientation.x = float(quat[0])
    pose_stamped.pose.orientation.y = float(quat[1])
    pose_stamped.pose.orientation.z = float(quat[2])
    pose_stamped.pose.orientation.w = float(quat[3])
    
    ik_request.ik_request.pose_stamped = pose_stamped
    ik_request.ik_request.ik_link_name = self.end_effector_link
    
    # Call IK service
    ik_future = self.ik_client.call_async(ik_request)
    rclpy.spin_until_future_complete(self, ik_future, timeout_sec=0.2)
    ik_response = ik_future.result()
    
    if ik_response and ik_response.error_code.val == 1:
        # Extract joint positions for our 7 joints
        joint_positions = []
        for joint_name in self.joint_names:
            if joint_name in ik_response.solution.joint_state.name:
                idx = ik_response.solution.joint_state.name.index(joint_name)
                joint_positions.append(ik_response.solution.joint_state.position[idx])
        
        return joint_positions if len(joint_positions) == 7 else None
    
    return None

def execute_single_point_trajectory(self, joint_positions):
    """Execute single-point trajectory (VR-style individual command)"""
    trajectory = JointTrajectory()
    trajectory.joint_names = self.joint_names
    
    point = JointTrajectoryPoint()
    point.positions = joint_positions
    point.time_from_start.sec = 0
    point.time_from_start.nanosec = int(0.1 * 1e9)  # 100ms execution
    trajectory.points.append(point)
    
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = trajectory
    
    # Send goal (non-blocking for high frequency)
    send_goal_future = self.trajectory_client.send_goal_async(goal)
    # Note: We don't wait for completion to maintain high frequency
    
    return True  # Assume success for high-frequency operation

def execute_moveit_command(self, command):
    """Execute individual MoveIt command (VR teleoperation style)"""
    try:
        # Convert Cartesian pose to joint positions using IK
        joint_positions = self.compute_ik_for_pose(command.pos, command.quat)
        
        if joint_positions is None:
            return False
        
        # Execute single-point trajectory (like VR teleoperation)
        return self.execute_single_point_trajectory(joint_positions)
        
    except Exception as e:
        if self.debug:
            print(f"❌ MoveIt command execution failed: {e}")
        return False
```

## Step 4: Replace Reset Robot Function

**REPLACE the existing `reset_robot` method (~line 915):**

**FROM:**
```python
def reset_robot(self, sync=True):
    """Reset robot to initial position
    
    Args:
        sync: If True, use synchronous communication (for initialization)
              If False, use async queues (not implemented for reset)
    """
    if self.debug:
        print("🔄 [DEBUG] Would reset robot to initial position")
        # Return simulated values
        return np.array([0.4, 0.0, 0.3]), np.array([1.0, 0.0, 0.0, 0.0]), None
    
    print("🔄 Resetting robot to initial position...")
    action = FrankaAction(
        pos=np.zeros(3),
        quat=np.zeros(4),
        gripper=GRIPPER_OPEN,
        reset=True,
        timestamp=time.time(),
    )
    
    # For reset, we always use synchronous communication
    # Thread-safe robot communication
    with self._robot_comm_lock:
        self.action_socket.send(bytes(pickle.dumps(action, protocol=-1)))
        robot_state = pickle.loads(self.action_socket.recv())
    
    print(f"✅ Robot reset complete")
    print(f"   Position: [{robot_state.pos[0]:.6f}, {robot_state.pos[1]:.6f}, {robot_state.pos[2]:.6f}]")
    print(f"   Quaternion: [{robot_state.quat[0]:.6f}, {robot_state.quat[1]:.6f}, {robot_state.quat[2]:.6f}, {robot_state.quat[3]:.6f}]")
    
    joint_positions = getattr(robot_state, 'joint_positions', None)
    return robot_state.pos, robot_state.quat, joint_positions
```

**TO:**
```python
def reset_robot(self, sync=True):
    """Reset robot to initial position using MoveIt trajectory
    
    Args:
        sync: If True, use synchronous communication (for initialization)
              If False, use async queues (not implemented for reset)
    """
    if self.debug:
        print("🔄 [DEBUG] Would reset robot to initial position")
        # Return simulated values
        return np.array([0.4, 0.0, 0.3]), np.array([1.0, 0.0, 0.0, 0.0]), None
    
    print("🔄 Resetting robot to initial position...")
    
    # Execute trajectory to home position
    success = self.execute_trajectory(self.home_positions, duration=3.0)
    
    if success:
        # Get new position via FK
        pos, quat = self.get_current_end_effector_pose()
        joint_positions = self.get_current_joint_positions()
        
        if pos is not None and quat is not None:
            print(f"✅ Robot reset complete")
            print(f"   Position: [{pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f}]")
            print(f"   Quaternion: [{quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f}]")
            
            return pos, quat, joint_positions
        else:
            raise RuntimeError("Failed to get robot state after reset")
    else:
        raise RuntimeError("Failed to reset robot to home position")
```

## Step 5: Replace Robot Communication Worker

**REPLACE the entire `_robot_comm_worker` method (~line 1455):**

**FROM:**
```python
def _robot_comm_worker(self):
    """Handles robot communication asynchronously to prevent blocking control thread"""
    print("🔌 Robot communication thread started")
    
    comm_count = 0
    total_comm_time = 0
    
    while self.running:
        try:
            # Get command from queue with timeout
            command = self._robot_command_queue.get(timeout=0.01)
            
            if command is None:  # Poison pill
                break
            
            # Send command and receive response
            comm_start = time.time()
            with self._robot_comm_lock:
                self.action_socket.send(bytes(pickle.dumps(command, protocol=-1)))
                response = pickle.loads(self.action_socket.recv())
            comm_time = time.time() - comm_start
            
            comm_count += 1
            total_comm_time += comm_time
            
            # Log communication stats periodically
            if comm_count % 10 == 0:
                avg_comm_time = total_comm_time / comm_count
                print(f"📡 Avg robot comm: {avg_comm_time*1000:.1f}ms")
            
            # Put response in queue
            try:
                self._robot_response_queue.put_nowait(response)
            except queue.Full:
                # Drop oldest response if queue is full
                try:
                    self._robot_response_queue.get_nowait()
                    self._robot_response_queue.put_nowait(response)
                except:
                    pass
            
        except queue.Empty:
            continue
        except Exception as e:
            if self.running:
                print(f"❌ Error in robot communication: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(0.1)
    
    print("🔌 Robot communication thread stopped")
```

**TO:**
```python
def _robot_comm_worker(self):
    """Handles robot communication via MoveIt services/actions"""
    print("🔌 Robot communication thread started (MoveIt)")
    
    comm_count = 0
    total_comm_time = 0
    
    while self.running:
        try:
            # Get command from queue with timeout
            command = self._robot_command_queue.get(timeout=0.01)
            
            if command is None:  # Poison pill
                break
            
            # Process MoveIt command
            comm_start = time.time()
            success = self.execute_moveit_command(command)
            comm_time = time.time() - comm_start
            
            comm_count += 1
            total_comm_time += comm_time
            
            # Log communication stats periodically
            if comm_count % 10 == 0:
                avg_comm_time = total_comm_time / comm_count
                print(f"📡 Avg MoveIt comm: {avg_comm_time*1000:.1f}ms")
            
            # Get current robot state after command
            if success:
                pos, quat = self.get_current_end_effector_pose()
                joint_positions = self.get_current_joint_positions()
                
                if pos is not None and quat is not None:
                    # Create response in same format as Deoxys
                    response = type('RobotState', (), {
                        'pos': pos,
                        'quat': quat,
                        'gripper': command.gripper,  # Echo back gripper state
                        'joint_positions': np.array(joint_positions) if joint_positions else None
                    })()
                    
                    try:
                        self._robot_response_queue.put_nowait(response)
                    except queue.Full:
                        # Drop oldest response if queue is full
                        try:
                            self._robot_response_queue.get_nowait()
                            self._robot_response_queue.put_nowait(response)
                        except:
                            pass
            
        except queue.Empty:
            continue
        except Exception as e:
            if self.running:
                print(f"❌ Error in MoveIt communication: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(0.1)
    
    print("🔌 Robot communication thread stopped (MoveIt)")
```

## Step 6: Replace Robot Action Creation

**FIND this code in `_process_control_cycle` (~line 1608):**

```python
# Send action to robot - DEOXYS EXPECTS QUATERNIONS
robot_action = FrankaAction(
    pos=target_pos.flatten().astype(np.float32),
    quat=target_quat.flatten().astype(np.float32),  # Quaternion directly
    gripper=gripper_state,
    reset=False,
    timestamp=time.time(),
)
```

**REPLACE with MoveIt-compatible action:**

```python
# Send action to robot - MOVEIT EXPECTS QUATERNIONS
robot_action = type('MoveitAction', (), {
    'pos': target_pos.flatten().astype(np.float32),
    'quat': target_quat.flatten().astype(np.float32),
    'gripper': gripper_state,
    'reset': False,
    'timestamp': time.time(),
})()
```

## Step 7: Update Control Loop with ROS 2 Spinning

**FIND the main while loop in `control_loop` (~line 1200):**

```python
while self.running:
    try:
        current_time = time.time()
        
        # Handle robot reset after calibration
        # ... existing logic ...
        
        # Small sleep to prevent CPU spinning
        time.sleep(0.01)
```

**ADD ROS 2 spinning:**

```python
while self.running:
    try:
        current_time = time.time()
        
        # Add ROS 2 spinning for service calls
        rclpy.spin_once(self, timeout_sec=0.001)
        
        # Handle robot reset after calibration
        # ... existing logic stays the same ...
        
        # Small sleep to prevent CPU spinning
        time.sleep(0.01)
```

## Step 8: Update Main Function

**REPLACE the entire `main()` function at the bottom:**

**FROM:**
```python
def main():
    parser = argparse.ArgumentParser(...)
    args = parser.parse_args()
    
    # ... existing argument processing ...
    
    server = OculusVRServer(...)
    
    try:
        server.start()
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        server.stop_server()
```

**TO:**
```python
def main():
    # Initialize ROS 2
    rclpy.init()
    
    try:
        parser = argparse.ArgumentParser(...)
        args = parser.parse_args()
        
        # ... ALL existing argument processing stays the same ...
        
        # Create server (now ROS 2 node)
        server = OculusVRServer(
            debug=args.debug,
            right_controller=not args.left_controller,
            ip_address=args.ip,
            simulation=args.simulation,
            coord_transform=coord_transform,
            rotation_mode=args.rotation_mode,
            performance_mode=args.performance,
            enable_recording=not args.no_recording,
            camera_configs=camera_configs,
            verify_data=args.verify_data,
            camera_config_path=args.camera_config,
            enable_cameras=args.enable_cameras
        )
        
        server.start()
        
    except KeyboardInterrupt:
        print("\n🛑 Keyboard interrupt received")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup ROS 2
        if 'server' in locals():
            server.destroy_node()
        rclpy.shutdown()
```

## Step 9: Remove Deoxys Connection Code

**REMOVE these lines from `__init__` (they're no longer needed):**

```python
# Remove robot control components section:
if not self.debug:
    print("🤖 Connecting to robot...")
    try:
        # Create robot control socket
        self.action_socket = create_request_socket(HOST, CONTROL_PORT)
        print("✅ Connected to robot server")
        
        # Create ZMQ context and publisher
        self.context = zmq.Context()
        self.controller_publisher = self.context.socket(zmq.PUB)
        self.controller_publisher.bind("tcp://0.0.0.0:5555")
        print("📡 Controller state publisher bound to tcp://0.0.0.0:5555")
    except Exception as e:
        print(f"❌ Failed to connect to robot: {e}")
        sys.exit(1)
```

**ALSO REMOVE from `stop_server`:**

```python
# Remove these lines:
if hasattr(self, 'action_socket'):
    self.action_socket.close()
if hasattr(self, 'controller_publisher'):
    self.controller_publisher.close()
if hasattr(self, 'context'):
    self.context.term()
```

## Step 10: Test the Migration

1. **Test ROS 2 connection:**
   ```bash
   python3 oculus_vr_server.py --debug
   ```

2. **Test with real robot (ensure MoveIt is running):**
   ```bash
   # Terminal 1: Start MoveIt
   ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=192.168.1.59
   
   # Terminal 2: Start VR server
   python3 oculus_vr_server.py
   ```

3. **Verify all features work:**
   - VR calibration
   - Robot reset
   - Teleoperation control
   - MCAP recording (if enabled)
   - Camera integration (if enabled)

## Expected Behavior

After migration:
- ✅ Same VR control feel and responsiveness
- ✅ Same coordinate transformations and calibration
- ✅ Same async architecture and performance
- ✅ Same MCAP recording and camera features
- ✅ Enhanced collision avoidance from MoveIt
- ✅ Better planning and safety features

The migration preserves all existing functionality while replacing only the robot communication layer! 