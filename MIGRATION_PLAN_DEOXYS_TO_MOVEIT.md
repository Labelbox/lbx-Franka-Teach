# Migration Plan: Oculus VR Server from Deoxys to MoveIt

## Overview

This migration plan preserves the **exact async architecture, VR transformations, MCAP recording, camera management, and control flow** while replacing only the Deoxys robot communication layer with MoveIt-based control.

## Key Principle: **Minimal Changes, Maximum Compatibility**

- ✅ **KEEP**: All VR processing, coordinate transformations, async threads, MCAP recording
- ✅ **KEEP**: DROID-exact control parameters, velocity calculations, position targeting  
- ✅ **KEEP**: Thread-safe queues, timing control, performance optimizations
- 🔄 **REPLACE**: Only the robot communication layer (Deoxys → MoveIt)

## Migration Changes

### 1. Class Structure Changes

#### Current (Deoxys-based):
```python
class OculusVRServer:
    def __init__(self, ...):
        # Deoxys socket connection
        self.action_socket = create_request_socket(HOST, CONTROL_PORT)
```

#### Target (MoveIt-based):
```python
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK, GetPlanningScene
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

class OculusVRServer(Node):  # INHERIT FROM ROS 2 NODE
    def __init__(self, ...):
        super().__init__('oculus_vr_server')
        
        # Robot configuration (from simple_arm_control.py)
        self.robot_ip = "192.168.1.59"
        self.planning_group = "panda_arm"  # or fr3_arm
        self.end_effector_link = "fr3_hand_tcp"
        self.base_frame = "fr3_link0"
        self.joint_names = ['fr3_joint1', 'fr3_joint2', ..., 'fr3_joint7']
        self.home_positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        
        # MoveIt service clients (from simple_arm_control.py)
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.planning_scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        
        # Trajectory action client
        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory, '/fr3_arm_controller/follow_joint_trajectory'
        )
        
        # Joint state subscriber
        self.joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        # Wait for services (exactly like simple_arm_control.py)
        self.ik_client.wait_for_service(timeout_sec=10.0)
        self.planning_scene_client.wait_for_service(timeout_sec=10.0)
        self.fk_client.wait_for_service(timeout_sec=10.0)
        self.trajectory_client.wait_for_server(timeout_sec=10.0)
        
        # ALL OTHER INITIALIZATION STAYS EXACTLY THE SAME
        # VR setup, async queues, camera manager, MCAP recorder, etc.
```

### 2. Robot State Management Changes

#### Current (Deoxys socket-based):
```python
def get_current_robot_state(self):
    self.action_socket.send(b"get_state")
    response = self.action_socket.recv()
    robot_state = pickle.loads(response)
    return robot_state.pos, robot_state.quat, robot_state.joint_positions
```

#### Target (ROS 2 subscription + FK):
```python
def joint_state_callback(self, msg):
    """Store the latest joint state (from simple_arm_control.py)"""
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
        return None
    
    # Use FK service (exactly like simple_arm_control.py)
    fk_request = GetPositionFK.Request()
    fk_request.fk_link_names = [self.end_effector_link]
    fk_request.header.frame_id = self.base_frame
    fk_request.robot_state.joint_state.name = self.joint_names
    fk_request.robot_state.joint_state.position = current_joints
    
    fk_future = self.fk_client.call_async(fk_request)
    rclpy.spin_until_future_complete(self, fk_future, timeout_sec=0.1)
    fk_response = fk_future.result()
    
    if fk_response and fk_response.error_code.val == 1:
        pose = fk_response.pose_stamped[0].pose
        pos = np.array([pose.position.x, pose.position.y, pose.position.z])
        quat = np.array([pose.orientation.x, pose.orientation.y, 
                        pose.orientation.z, pose.orientation.w])
        return pos, quat
    return None, None
```

### 3. Robot Reset Function Changes

#### Current (Deoxys reset command):
```python
def reset_robot(self, sync=True):
    action = FrankaAction(pos=np.zeros(3), quat=np.zeros(4), gripper=GRIPPER_OPEN, reset=True)
    self.action_socket.send(bytes(pickle.dumps(action, protocol=-1)))
    robot_state = pickle.loads(self.action_socket.recv())
    return robot_state.pos, robot_state.quat, robot_state.joint_positions
```

#### Target (MoveIt trajectory to home):
```python
def reset_robot(self, sync=True):
    """Reset robot to initial position using MoveIt trajectory"""
    if self.debug:
        return np.array([0.4, 0.0, 0.3]), np.array([1.0, 0.0, 0.0, 0.0]), None
    
    print("🔄 Resetting robot to initial position...")
    
    # Execute trajectory to home position (from simple_arm_control.py)
    success = self.execute_trajectory(self.home_positions, duration=3.0)
    
    if success:
        # Get new position via FK
        pos, quat = self.get_current_end_effector_pose()
        joint_positions = self.get_current_joint_positions()
        
        print("✅ Robot reset complete")
        return pos, quat, joint_positions
    else:
        raise RuntimeError("Failed to reset robot to home position")

def execute_trajectory(self, positions, duration=2.0):
    """Execute trajectory (from simple_arm_control.py)"""
    trajectory = JointTrajectory()
    trajectory.joint_names = self.joint_names
    
    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start.sec = int(duration)
    point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
    trajectory.points.append(point)
    
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = trajectory
    
    future = self.trajectory_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
    goal_handle = future.result()
    
    if not goal_handle.accepted:
        return False
    
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 2.0)
    result = result_future.result()
    
    return result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
```

### 4. Robot Communication Worker Changes

This is the **core migration** - replace the entire `_robot_comm_worker` function:

#### Current (Deoxys socket communication):
```python
def _robot_comm_worker(self):
    while self.running:
        command = self._robot_command_queue.get(timeout=0.01)
        if command is None:
            break
        
        # Deoxys socket communication
        with self._robot_comm_lock:
            self.action_socket.send(bytes(pickle.dumps(command, protocol=-1)))
            response = pickle.loads(self.action_socket.recv())
        
        self._robot_response_queue.put_nowait(response)
```

#### Target (MoveIt service/action communication):
```python
def _robot_comm_worker(self):
    """Handles robot communication via MoveIt services/actions"""
    print("🔌 Robot communication thread started (MoveIt)")
    
    while self.running:
        try:
            # Get command from queue
            command = self._robot_command_queue.get(timeout=0.01)
            if command is None:  # Poison pill
                break
            
            # Process MoveIt command
            success = self.execute_moveit_command(command)
            
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
                    
                    self._robot_response_queue.put_nowait(response)
            
        except queue.Empty:
            continue
        except Exception as e:
            if self.running:
                print(f"❌ Error in MoveIt communication: {e}")

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
        print(f"❌ MoveIt command execution failed: {e}")
        return False

def compute_ik_for_pose(self, pos, quat):
    """Compute IK for Cartesian pose (from simple_arm_control.py)"""
    # Get planning scene
    scene_response = self.get_planning_scene()
    if scene_response is None:
        return None
    
    # Create IK request
    ik_request = GetPositionIK.Request()
    ik_request.ik_request.group_name = self.planning_group
    ik_request.ik_request.robot_state = scene_response.scene.robot_state
    ik_request.ik_request.avoid_collisions = True
    ik_request.ik_request.timeout.nanosec = int(0.1 * 1e9)  # 100ms timeout
    
    # Set target pose
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = self.base_frame
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
```

### 5. Data Structure Changes

#### Replace Deoxys imports:
```python
# REMOVE these Deoxys imports:
# from frankateach.network import create_request_socket
# from frankateach.messages import FrankaAction, FrankaState
# from deoxys.utils import transform_utils

# ADD these MoveIt imports:
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
```

#### Create MoveIt-compatible action structure:
```python
@dataclass
class MoveitAction:
    """MoveIt-compatible action (replaces FrankaAction)"""
    pos: np.ndarray
    quat: np.ndarray  
    gripper: float
    reset: bool
    timestamp: float
```

### 6. Main Loop Changes

#### Add ROS 2 spinning to control loop:
```python
def control_loop(self):
    """Main control loop with ROS 2 integration"""
    
    # ALL EXISTING INITIALIZATION STAYS THE SAME
    # (robot reset, camera manager, worker threads, etc.)
    
    while self.running:
        try:
            # Add ROS 2 spinning for service calls
            rclpy.spin_once(self, timeout_sec=0.001)
            
            # ALL EXISTING CONTROL LOGIC STAYS THE SAME
            # (robot reset handling, debug output, etc.)
            
        except Exception as e:
            # Same error handling
```

#### Update main function:
```python
def main():
    # Initialize ROS 2
    rclpy.init()
    
    try:
        # ALL EXISTING ARGUMENT PARSING STAYS THE SAME
        
        # Create server (now ROS 2 node)
        server = OculusVRServer(...)
        
        server.start()
        
    except KeyboardInterrupt:
        print("\n🛑 Keyboard interrupt received")
    finally:
        # Cleanup ROS 2
        if 'server' in locals():
            server.destroy_node()
        rclpy.shutdown()
```

## Migration Steps

### Phase 1: Basic Structure (Day 1)
1. ✅ Add ROS 2 imports and remove Deoxys imports
2. ✅ Change class to inherit from `Node`
3. ✅ Add MoveIt service clients and joint state subscription
4. ✅ Update `__init__` method with ROS 2 setup
5. ✅ Test that ROS 2 node starts and connects to services

### Phase 2: Robot State Management (Day 2)  
1. ✅ Implement `joint_state_callback` and `get_current_joint_positions`
2. ✅ Implement `get_current_end_effector_pose` using FK
3. ✅ Update robot state initialization in `control_loop`
4. ✅ Test robot state reading and FK conversion

### Phase 3: Robot Communication (Day 3)
1. ✅ Replace `_robot_comm_worker` with MoveIt version
2. ✅ Implement `compute_ik_for_pose` and `execute_single_point_trajectory`
3. ✅ Implement `execute_moveit_command` function
4. ✅ Test basic robot movement commands

### Phase 4: Reset Function (Day 4)
1. ✅ Replace `reset_robot` with MoveIt trajectory version
2. ✅ Implement `execute_trajectory` function
3. ✅ Test robot reset to home position
4. ✅ Verify calibration still works after reset

### Phase 5: Integration Testing (Day 5)
1. ✅ Test full VR teleoperation pipeline
2. ✅ Verify MCAP recording still works
3. ✅ Test camera integration (if enabled)
4. ✅ Performance testing and optimization

## Testing Strategy

### Unit Tests
- ✅ ROS 2 service connections
- ✅ Joint state reading and FK conversion  
- ✅ IK computation for VR poses
- ✅ Trajectory execution

### Integration Tests
- ✅ Full VR → robot control pipeline
- ✅ Recording and camera integration
- ✅ Reset and calibration workflows
- ✅ High-frequency control performance

### Validation Criteria
- ✅ Same control behavior as Deoxys version
- ✅ Same async performance characteristics
- ✅ All MCAP/camera features working
- ✅ Maintain >30Hz control rate capability

## What Stays Exactly The Same

✅ **VR Processing**: All coordinate transformations, calibration, button handling  
✅ **Async Architecture**: All thread management, queues, timing control  
✅ **MCAP Recording**: Complete recording system with camera integration  
✅ **Control Logic**: DROID-exact velocity calculations and position targeting  
✅ **User Interface**: All command-line args, calibration procedures, controls  
✅ **Performance**: Same threading model and optimization strategies  

## Success Metrics

1. ✅ **Functional Parity**: Identical VR control behavior vs Deoxys version
2. ✅ **Performance Parity**: Maintain high-frequency control capabilities  
3. ✅ **Feature Completeness**: All recording, camera, calibration features work
4. ✅ **Code Maintainability**: Minimal changes, clear separation of concerns

This migration preserves the sophisticated async architecture and VR processing while gaining the benefits of MoveIt's advanced collision avoidance, planning, and robot control capabilities. 