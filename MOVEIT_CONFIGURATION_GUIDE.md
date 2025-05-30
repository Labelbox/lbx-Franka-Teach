# MoveIt Configuration Guide for Oculus VR Server

This guide covers the MoveIt configuration requirements for the migrated `oculus_vr_server_moveit.py`.

## Required MoveIt Configuration

### 1. Planning Group Configuration

The VR server expects these planning group settings in your MoveIt configuration:

```yaml
# In your moveit_config/config/franka_fr3.srdf or similar
planning_groups:
  - name: panda_arm  # or fr3_arm
    joints:
      - fr3_joint1
      - fr3_joint2
      - fr3_joint3
      - fr3_joint4
      - fr3_joint5
      - fr3_joint6
      - fr3_joint7
    end_effector_link: fr3_hand_tcp
    base_link: fr3_link0
```

### 2. IK Solver Configuration

For best performance, configure a fast IK solver:

```yaml
# In your kinematics.yaml
panda_arm:  # or fr3_arm
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.1  # Fast timeout for VR
  kinematics_solver_attempts: 3
```

**Recommended IK Solvers (in order of preference):**
1. **QuIK** (fastest: 5-6μs) - if available
2. **PoseIK** (fast: ~1ms) - good balance
3. **BioIK** (flexible: 2-5ms) - good for complex constraints
4. **KDL** (standard: 5-20ms) - fallback option

### 3. Joint Controller Configuration

Ensure your robot has a trajectory controller:

```yaml
# In your ros2_control configuration
fr3_arm_controller:
  type: joint_trajectory_controller/JointTrajectoryController
  joints:
    - fr3_joint1
    - fr3_joint2
    - fr3_joint3
    - fr3_joint4
    - fr3_joint5
    - fr3_joint6
    - fr3_joint7
  command_interfaces:
    - position
  state_interfaces:
    - position
    - velocity
  action_ns: follow_joint_trajectory
```

### 4. Launch File Modifications

If you need to modify the existing launch file, add these parameters for VR compatibility:

```python
# Add to your moveit.launch.py
return LaunchDescription([
    # ... existing nodes ...
    
    # Add these parameters for VR server compatibility
    DeclareLaunchArgument(
        'allow_trajectory_execution',
        default_value='true',
        description='Enable trajectory execution'
    ),
    DeclareLaunchArgument(
        'fake_execution',
        default_value='false', 
        description='Use fake execution for simulation'
    ),
    DeclareLaunchArgument(
        'pipeline',
        default_value='ompl',
        description='Planning pipeline to use'
    ),
    
    # Move group node with VR-friendly settings
    Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.trajectory_execution,
            moveit_config.joint_limits,
            {
                'allow_trajectory_execution': LaunchConfiguration('allow_trajectory_execution'),
                'fake_execution': LaunchConfiguration('fake_execution'),
                'capabilities': 'move_group/MoveGroupCartesianPathService '
                               'move_group/MoveGroupExecuteTrajectoryAction '
                               'move_group/MoveGroupKinematicsService '
                               'move_group/MoveGroupMoveAction '
                               'move_group/MoveGroupPickPlaceAction '
                               'move_group/MoveGroupPlanService '
                               'move_group/MoveGroupQueryPlannersService '
                               'move_group/MoveGroupStateValidationService '
                               'move_group/MoveGroupGetPlanningSceneService '
                               'move_group/ClearOctomapService',
                'planning_scene_monitor_options': {
                    'robot_description': 'robot_description',
                    'joint_state_topic': '/joint_states',
                    'attached_collision_object_topic': '/move_group/attached_collision_object',
                    'publish_planning_scene_topic': '/move_group/monitored_planning_scene',
                    'publish_geometry_updates': True,
                    'publish_state_updates': True,
                    'publish_transforms_updates': True
                }
            }
        ]
    ),
])
```

## Performance Optimization

### 1. IK Service Timeout

The VR server uses fast IK timeouts for responsive control:

```yaml
# In kinematics.yaml - optimize for VR
panda_arm:
  kinematics_solver_timeout: 0.1  # 100ms max
  kinematics_solver_attempts: 1   # Single attempt for speed
```

### 2. Planning Scene Updates

For high-frequency VR control, you may want to reduce planning scene update rates:

```yaml
# In your launch file parameters
planning_scene_monitor_options:
  publish_planning_scene: false     # Disable if not needed
  publish_geometry_updates: false   # Disable if not needed  
  publish_state_updates: true       # Keep for joint states
```

### 3. Collision Checking

The VR server enables collision checking by default. To disable for better performance:

```python
# In oculus_vr_server_moveit.py, modify compute_ik_for_pose():
ik_request.ik_request.avoid_collisions = False  # Disable collision checking
```

## Verification Commands

Test your MoveIt configuration before running the VR server:

```bash
# 1. Check if MoveIt services are available
ros2 service list | grep -E "(compute_ik|compute_fk|get_planning_scene)"

# 2. Test IK service
ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK \
  "{ik_request: {group_name: 'panda_arm', pose_stamped: {header: {frame_id: 'fr3_link0'}, pose: {position: {x: 0.5, y: 0.0, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}}"

# 3. Check joint state topic
ros2 topic echo /joint_states --once

# 4. Test trajectory action
ros2 action list | grep follow_joint_trajectory
```

## Troubleshooting

### Common Issues:

1. **Service timeout errors**
   - Increase timeout in `oculus_vr_server_moveit.py`
   - Check MoveIt node is running: `ros2 node list | grep move_group`

2. **IK failures**
   - Check target poses are reachable
   - Verify kinematics.yaml configuration
   - Enable debug logging: `--debug-ik-failures`

3. **Joint state not available**
   - Verify robot is publishing to `/joint_states`
   - Check joint names match between robot and MoveIt config

4. **Trajectory execution fails**
   - Verify controller is loaded: `ros2 control list_controllers`
   - Check trajectory action server: `ros2 action list`

## Configuration Files Location

Your MoveIt configuration should be in:
```
ros2_moveit_franka/
├── config/
│   ├── franka_fr3.srdf
│   ├── kinematics.yaml
│   ├── joint_limits.yaml
│   └── ros2_controllers.yaml
└── launch/
    └── moveit.launch.py
```

## Testing the Configuration

1. **Start MoveIt:**
   ```bash
   ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=192.168.1.59
   ```

2. **Test VR server in debug mode:**
   ```bash
   python3 oculus_vr_server_moveit.py --debug
   ```

3. **Run with real robot:**
   ```bash
   python3 oculus_vr_server_moveit.py
   ```

The migrated VR server should connect to all MoveIt services and report successful initialization! 