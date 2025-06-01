#!/usr/bin/env python3
"""
Integrated System Launch File for Labelbox Robotics VR Teleoperation

This launch file brings up the complete system including:
- VR input (Oculus reader)
- Franka control system
- MoveIt components
- Data recording
- Camera system (optional)
- Visualization tools
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.59',
        description='IP address of the Franka robot'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    enable_recording_arg = DeclareLaunchArgument(
        'enable_recording',
        default_value='true',
        description='Enable data recording functionality'
    )
    
    enable_cameras_arg = DeclareLaunchArgument(
        'enable_cameras',
        default_value='false',
        description='Enable camera system'
    )
    
    camera_config_arg = DeclareLaunchArgument(
        'camera_config',
        default_value='auto',
        description='Camera configuration file or "auto" for discovery'
    )
    
    use_right_controller_arg = DeclareLaunchArgument(
        'use_right_controller',
        default_value='true',
        description='Use right VR controller (false for left)'
    )
    
    vr_mode_arg = DeclareLaunchArgument(
        'vr_mode',
        default_value='usb',
        description='VR connection mode: "usb" or "network"'
    )
    
    vr_ip_arg = DeclareLaunchArgument(
        'vr_ip',
        default_value='',
        description='IP address for network VR mode'
    )
    
    hot_reload_arg = DeclareLaunchArgument(
        'hot_reload',
        default_value='false',
        description='Enable hot reload for development'
    )
    
    verify_data_arg = DeclareLaunchArgument(
        'verify_data',
        default_value='false',
        description='Enable data verification mode'
    )
    
    # Get package paths
    lbx_franka_control_share = get_package_share_directory('lbx_franka_control')
    lbx_franka_moveit_share = get_package_share_directory('lbx_franka_moveit')
    lbx_input_oculus_share = get_package_share_directory('lbx_input_oculus')
    lbx_data_recorder_share = get_package_share_directory('lbx_data_recorder')
    
    # Get configuration file
    config_path = os.path.join(
        os.path.dirname(__file__),
        '../configs/control/franka_vr_control_config.yaml'
    )
    
    # 1. MoveIt Components
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lbx_franka_moveit_share, 'launch', 'moveit.launch.py')
        ),
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items()
    )
    
    # 2. VR Input System
    vr_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lbx_input_oculus_share, 'launch', 'oculus.launch.py')
        ),
        launch_arguments={
            'mode': LaunchConfiguration('vr_mode'),
            'ip': LaunchConfiguration('vr_ip'),
            'use_right_controller': LaunchConfiguration('use_right_controller'),
        }.items()
    )
    
    # 3. Franka Control System (System Manager + Controller)
    system_manager_node = Node(
        package='lbx_franka_control',
        executable='system_manager',
        name='system_manager',
        output='screen',
        parameters=[{
            'config_path': config_path,
            'use_right_controller': LaunchConfiguration('use_right_controller'),
        }]
    )
    
    # 4. Data Recording System (conditional)
    data_recorder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lbx_data_recorder_share, 'launch', 'recorder.launch.py')
        ),
        launch_arguments={
            'output_dir': os.path.expanduser('~/lbx_recordings'),
            'recording_hz': '60',
            'verify_data': LaunchConfiguration('verify_data'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_recording'))
    )
    
    # 5. Camera System (conditional)
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('lbx_vision_camera'), 
                        'launch', 'cameras.launch.py')
        ),
        launch_arguments={
            'config': LaunchConfiguration('camera_config'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_cameras'))
    )
    
    # 6. Main System Integration Node
    main_system_node = Node(
        package='lbx_franka_control',
        executable='main_system',
        name='labelbox_robotics_system',
        output='screen',
        parameters=[{
            'config_path': config_path,
        }],
        env={
            'VR_IP': LaunchConfiguration('vr_ip'),
            'ENABLE_CAMERAS': LaunchConfiguration('enable_cameras'),
            'CAMERA_CONFIG': LaunchConfiguration('camera_config'),
            'HOT_RELOAD': LaunchConfiguration('hot_reload'),
            'VERIFY_DATA': LaunchConfiguration('verify_data'),
        }
    )
    
    # 7. Diagnostics Aggregator
    diagnostics_node = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='diagnostic_aggregator',
        parameters=[{
            'analyzers': {
                'vr': {
                    'type': 'diagnostic_aggregator/GenericAnalyzer',
                    'path': 'VR System',
                    'contains': ['vr', 'oculus']
                },
                'robot': {
                    'type': 'diagnostic_aggregator/GenericAnalyzer',
                    'path': 'Robot',
                    'contains': ['franka', 'moveit', 'joint']
                },
                'system': {
                    'type': 'diagnostic_aggregator/GenericAnalyzer',
                    'path': 'System',
                    'contains': ['system', 'manager', 'recording']
                }
            }
        }]
    )
    
    # 8. System Monitor (publishes diagnostics)
    system_monitor_node = Node(
        package='lbx_franka_control',
        executable='system_monitor',
        name='system_monitor',
        output='screen',
        parameters=[{
            'publish_rate': 1.0,  # 1Hz diagnostics
        }]
    )
    
    return LaunchDescription([
        # Launch arguments
        robot_ip_arg,
        use_rviz_arg,
        enable_recording_arg,
        enable_cameras_arg,
        camera_config_arg,
        use_right_controller_arg,
        vr_mode_arg,
        vr_ip_arg,
        hot_reload_arg,
        verify_data_arg,
        
        # Launch components in order
        moveit_launch,           # 1. MoveIt (includes robot state publisher)
        vr_launch,              # 2. VR input system
        system_manager_node,    # 3. Core control system
        data_recorder_launch,   # 4. Data recording (conditional)
        camera_launch,          # 5. Cameras (conditional)
        diagnostics_node,       # 6. Diagnostics aggregator
        system_monitor_node,    # 7. System health monitor
        main_system_node,       # 8. Main system integration (UI and monitoring)
    ]) 