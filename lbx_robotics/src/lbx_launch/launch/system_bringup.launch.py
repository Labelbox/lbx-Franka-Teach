#!/usr/bin/env python3
"""
System Bringup Launch File

Launches the complete VR-based Franka control system with all components:
- Cameras first (if enabled) - wait for initialization
- MoveIt for robot control (delayed to avoid timing conflicts)
- VR input processing
- System manager for orchestration
- Data recording (optional)

Usage:
  ros2 launch lbx_launch system_bringup.launch.py
  ros2 launch lbx_launch system_bringup.launch.py enable_recording:=true
  ros2 launch lbx_launch system_bringup.launch.py enable_cameras:=true camera_config:=/path/to/cameras.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    declare_robot_ip = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.59',
        description='IP address of the Franka robot'
    )
    
    declare_use_fake_hardware = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Use simulated robot instead of real hardware'
    )
    
    declare_enable_rviz = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Enable RViz visualization'
    )
    
    declare_enable_recording = DeclareLaunchArgument(
        'enable_recording',
        default_value='true',
        description='Enable MCAP data recording'
    )
    
    declare_enable_cameras = DeclareLaunchArgument(
        'enable_cameras',
        default_value='false',
        description='Enable camera integration'
    )
    
    # Get workspace path for camera config default
    workspace_root = os.environ.get('COLCON_WS', os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))))
    default_camera_config = os.path.join(workspace_root, 'lbx_robotics', 'configs', 'sensors', 'realsense_cameras.yaml')
    
    declare_camera_config = DeclareLaunchArgument(
        'camera_config',
        default_value=default_camera_config,
        description='Path to camera configuration YAML file'
    )
    
    declare_vr_ip = DeclareLaunchArgument(
        'vr_ip',
        default_value='',
        description='IP address of Quest device (empty for USB connection)'
    )
    
    declare_use_right_controller = DeclareLaunchArgument(
        'use_right_controller',
        default_value='true',
        description='Use right controller (true) or left controller (false)'
    )
    
    declare_hot_reload = DeclareLaunchArgument(
        'hot_reload',
        default_value='false',
        description='Enable hot reload for development'
    )
    
    declare_vr_mode = DeclareLaunchArgument(
        'vr_mode',
        default_value='usb',
        description='VR connection mode (usb or network)'
    )
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='Logging level (DEBUG, INFO, WARN, ERROR)'
    )
    
    declare_camera_init_delay = DeclareLaunchArgument(
        'camera_init_delay',
        default_value='5.0',
        description='Delay in seconds to wait for camera initialization before starting robot nodes'
    )
    
    # Config file for system_manager_node
    # Assuming franka_vr_control_config.yaml is in lbx_franka_control/config/
    # If not, adjust the FindPackageShare path or move the file to lbx_launch/config/
    franka_control_config = PathJoinSubstitution([
        FindPackageShare('lbx_franka_control'), 
        'config',
        'franka_vr_control_config.yaml'
    ])
    
    # ==========================================
    # PHASE 1: CAMERA INITIALIZATION (if enabled)
    # ==========================================
    
    # Camera Manager Node (starts first if cameras enabled)
    camera_namespace_group = GroupAction(
        actions=[
            PushRosNamespace('cameras'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('lbx_vision_camera'),
                        'launch',
                        'camera.launch.py'
                    ])
                ]),
                launch_arguments={
                    'camera_config_file': LaunchConfiguration('camera_config'),
                }.items(),
                condition=IfCondition(LaunchConfiguration('enable_cameras'))
            )
        ]
    )
    
    # ==========================================
    # PHASE 2: ROBOT AND SYSTEM NODES (delayed)
    # ==========================================
    
    # MoveIt launch (delayed to allow camera initialization)
    moveit_launch_delayed = TimerAction(
        period=PythonExpression([
            "float('", LaunchConfiguration('camera_init_delay'), 
            "') if '", LaunchConfiguration('enable_cameras'), "' == 'true' else 0.0"
        ]),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('lbx_franka_moveit'),
                        'launch',
                        'moveit_server.launch.py'
                    ])
                ),
                launch_arguments={
                    'robot_ip': LaunchConfiguration('robot_ip'),
                    'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
                    'enable_rviz': LaunchConfiguration('enable_rviz'),  # Let MoveIt handle RViz
                    'load_gripper': 'true',  # Always load gripper for VR control
                }.items()
            )
        ]
    )
    
    # VR Input Node (delayed)
    vr_namespace_group_delayed = TimerAction(
        period=PythonExpression([
            "float('", LaunchConfiguration('camera_init_delay'), 
            "') + 1.0 if '", LaunchConfiguration('enable_cameras'), "' == 'true' else 1.0"
        ]),
        actions=[
            GroupAction(
                actions=[
                    PushRosNamespace('vr'),
                    Node(
                        package='lbx_input_oculus',
                        executable='oculus_node',
                        name='oculus_reader',
                        parameters=[{
                            'use_network': PythonExpression(["'", LaunchConfiguration('vr_mode'), "' == 'network'"]),
                            'ip_address': LaunchConfiguration('vr_ip'),
                            'poll_rate_hz': 60.0,
                            'publish_rate_hz': 60.0,
                            'queue_size': 10,
                        }],
                        output='screen'
                    )
                ]
            )
        ]
    )
    
    # System nodes (delayed slightly more to ensure MoveIt is ready)
    system_nodes_delayed = TimerAction(
        period=PythonExpression([
            "float('", LaunchConfiguration('camera_init_delay'), 
            "') + 3.0 if '", LaunchConfiguration('enable_cameras'), "' == 'true' else 2.0"
        ]),
        actions=[
            # System Monitor Node (start early to track all components)
            Node(
                package='lbx_franka_control',
                executable='system_monitor',
                name='system_monitor',
                output='screen',
                parameters=[{
                    'publish_rate': 1.0,  # Publish diagnostics at 1Hz
                    'camera_config': LaunchConfiguration('camera_config'),
                    'enable_cameras': LaunchConfiguration('enable_cameras'),
                }]
            ),
            
            # System Orchestrator Node (lightweight coordinator)
            Node(
                package='lbx_franka_control',
                executable='system_orchestrator',
                name='system_orchestrator',
                output='screen',
                parameters=[{
                    'log_level': LaunchConfiguration('log_level'),
                }]
            ),
            
            # System Manager Node (robot state management and services)
            Node(
                package='lbx_franka_control',
                executable='system_manager',
                name='system_manager',
                output='screen',
                parameters=[{
                    'config_file': franka_control_config,
                    'robot_name': 'fr3',
                    'log_level': LaunchConfiguration('log_level'),
                }]
            ),
            
            # Robot Control Node (handles MoveIt and robot commands)
            Node(
                package='lbx_franka_control',
                executable='robot_control_node',
                name='robot_control_node',
                output='screen',
                parameters=[{
                    'config_file': franka_control_config,
                    'robot_name': 'fr3',
                    'control_rate': 45.0,
                    'log_level': LaunchConfiguration('log_level'),
                }]
            ),
            
            # VR Teleoperation Node (processes VR input)
            Node(
                package='lbx_franka_control',
                executable='vr_teleop_node',
                name='vr_teleop_node',
                output='screen',
                parameters=[{
                    'config_file': franka_control_config,
                    'control_rate': 45.0,
                    'calibration_file': os.path.join(
                        os.path.expanduser('~'),
                        'vr_calibration_data.json'
                    ),
                    'log_level': LaunchConfiguration('log_level'),
                }],
                remappings=[
                    # Map VR controller input from oculus node
                    ('/vr/controller_pose', PythonExpression([
                        "'/vr/right_controller/pose' if '",
                        LaunchConfiguration('use_right_controller'),
                        "' == 'true' else '/vr/left_controller/pose'"
                    ])),
                    ('/vr/controller_joy', PythonExpression([
                        "'/vr/right_controller_joy' if '",
                        LaunchConfiguration('use_right_controller'),
                        "' == 'true' else '/vr/left_controller_joy'"
                    ])),
                ]
            ),
            
            # Main System Node (primary terminal interface and system monitor)
            Node(
                package='lbx_franka_control',
                executable='main_system',
                name='main_system',
                output='screen',
                parameters=[{
                    'config_file': franka_control_config,
                    'log_level': LaunchConfiguration('log_level'),
                }],
                # Pass launch parameters as environment variables
                additional_env={
                    'VR_IP': LaunchConfiguration('vr_ip'),
                    'ENABLE_CAMERAS': LaunchConfiguration('enable_cameras'),
                    'CAMERA_CONFIG': LaunchConfiguration('camera_config'),
                    'HOT_RELOAD': LaunchConfiguration('hot_reload'),
                    'VERIFY_DATA': 'false',  # Not used anymore
                    'USE_FAKE_HARDWARE': LaunchConfiguration('use_fake_hardware'),
                    'ROBOT_IP': LaunchConfiguration('robot_ip'),
                    'ENABLE_RVIZ': LaunchConfiguration('enable_rviz'),
                    'ENABLE_RECORDING': LaunchConfiguration('enable_recording'),
                }
            ),
        ]
    )
    
    # Data Recorder Node (delayed to start after other nodes are stable)
    data_recorder_delayed = TimerAction(
        period=PythonExpression([
            "float('", LaunchConfiguration('camera_init_delay'), 
            "') + 4.0 if '", LaunchConfiguration('enable_cameras'), "' == 'true' else 3.0"
        ]),
        actions=[
            Node(
                package='lbx_data_recorder',
                executable='mcap_recorder_node',
                name='mcap_recorder_node',
                parameters=[{
                    'output_dir': os.path.expanduser('~/recordings'),
                    'save_images': True,
                    'save_depth': True,
                    'enable_cameras': LaunchConfiguration('enable_cameras'),
                    'camera_config': LaunchConfiguration('camera_config'),
                }],
                condition=IfCondition("false"), # Temporarily disable MCAP recorder
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        # Declare arguments
        declare_robot_ip,
        declare_use_fake_hardware,
        declare_enable_rviz,
        declare_enable_recording,
        declare_enable_cameras,
        declare_camera_config,
        declare_vr_ip,
        declare_use_right_controller,
        declare_hot_reload,
        declare_vr_mode,
        declare_log_level,
        declare_camera_init_delay,
        
        # PHASE 1: Start cameras first (immediate)
        camera_namespace_group,
        
        # PHASE 2: Start robot and system nodes (delayed)
        moveit_launch_delayed,
        vr_namespace_group_delayed,
        system_nodes_delayed,
        data_recorder_delayed,
    ]) 