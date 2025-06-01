#!/usr/bin/env python3
"""
System Bringup Launch File

Launches the complete VR-based Franka control system with all components:
- MoveIt for robot control
- VR input processing
- System manager for orchestration
- Data recording (optional)
- Camera integration (optional)

Usage:
  ros2 launch lbx_launch system_bringup.launch.py
  ros2 launch lbx_launch system_bringup.launch.py enable_recording:=true
  ros2 launch lbx_launch system_bringup.launch.py enable_cameras:=true camera_config:=/path/to/cameras.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
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
    
    declare_camera_config = DeclareLaunchArgument(
        'camera_config',
        default_value='',
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
        description='Use right controller (false for left)'
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
    
    # Config file for system_manager_node
    # Assuming franka_vr_control_config.yaml is in lbx_franka_control/config/
    # If not, adjust the FindPackageShare path or move the file to lbx_launch/config/
    franka_control_config = PathJoinSubstitution([
        FindPackageShare('lbx_franka_control'), 
        'config',
        'franka_vr_control_config.yaml'
    ])
    
    # RViz configuration file
    # TODO: Create the RViz config file or use MoveIt's default
    # Commenting out for now since the file doesn't exist
    # rviz_config_path = PathJoinSubstitution([
    #     FindPackageShare('lbx_launch'), 
    #     'config',
    #     'franka_vr_control.rviz'
    # ])
    
    # MoveIt launch
    moveit_launch = IncludeLaunchDescription(
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
    
    # VR Input Node
    vr_namespace_group = GroupAction(
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
    
    # System Manager Node (main orchestrator)
    system_manager_node = Node(
        package='lbx_franka_control',
        executable='system_manager',
        name='system_manager',
        parameters=[{
            'config_file': franka_control_config,
            'use_right_controller': LaunchConfiguration('use_right_controller'),
            'hot_reload': LaunchConfiguration('hot_reload'),
            'enable_recording': LaunchConfiguration('enable_recording'),
            'log_level': LaunchConfiguration('log_level'),
        }],
        output='screen',
        remappings=[
            # Remap VR inputs to match oculus_node outputs
            ('/vr/controller_pose', PythonExpression([
                "'/vr/right_controller_pose' if '", LaunchConfiguration('use_right_controller'), 
                "' == 'true' else '/vr/left_controller_pose'"
            ])),
            ('/vr/controller_buttons', '/vr/buttons'),
        ]
    )
    
    # Data Recorder Node (conditional)
    data_recorder_node = Node(
        package='lbx_data_recorder',
        executable='mcap_recorder_node',
        name='data_recorder',
        parameters=[{
            'output_dir': os.path.expanduser('~/recordings'),
            'save_images': True,
            'save_depth': True,
            'enable_cameras': LaunchConfiguration('enable_cameras'),
            'camera_config': LaunchConfiguration('camera_config'),
        }],
        condition=IfCondition(LaunchConfiguration('enable_recording')),
        output='screen'
    )
    
    # Camera Manager Node (conditional)
    camera_namespace_group = GroupAction(
        actions=[
            PushRosNamespace('cameras'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('lbx_vision_camera'),
                        'launch',
                        'camera_system.launch.py'
                    ])
                ]),
                launch_arguments={
                    'camera_config': LaunchConfiguration('camera_config'),
                }.items(),
                condition=IfCondition(LaunchConfiguration('enable_cameras'))
            )
        ]
    )
    
    # System monitoring and visualization tools
    # TODO: Create RViz config and uncomment this section
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz',
    #     arguments=['-d', rviz_config_path],
    #     condition=IfCondition(LaunchConfiguration('enable_rviz'))
    # )
    
    # Status echo nodes for debugging
    status_echo_node = Node(
        package='ros2',
        executable='topic',
        name='status_echo',
        arguments=['echo', '/system_status'],
        output='screen'
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
        
        # Launch components
        moveit_launch,
        vr_namespace_group,
        system_manager_node,
        data_recorder_node,
        camera_namespace_group,
        
        # Optional: Add system status monitoring
        # status_echo_node,
    ]) 