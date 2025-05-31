#!/usr/bin/env python3
"""
Main system bringup launch file for the LBX Robotics workspace.
This file orchestrates the launch of all core components:
- MoveIt Server (including robot drivers and RViz)
- Oculus Input Node
- RealSense Vision Node
- Franka Control/Teleoperation Node
- Data Recorder Node

Parameters can be passed to this launch file to configure components,
or they can be read from workspace_config.yaml and franka_config.yaml.
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

def load_yaml_config(context, config_file_name):
    config_path = PathJoinSubstitution([
        FindPackageShare('lbx_robotics'),
        'configs',
        config_file_name
    ]).perform(context)
    try:
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        LogInfo(msg=f"Error loading {config_file_name}: {e}").execute(context)
        return {}

def launch_setup(context, *args, **kwargs):
    # --- Load Configurations ---
    ws_config = load_yaml_config(context, 'workspace_config.yaml').get('workspace', {})
    franka_config = load_yaml_config(context, 'franka_config.yaml').get('franka_fr3', {})
    packages_config = load_yaml_config(context, 'workspace_config.yaml').get('packages', {})

    # --- Declare Launch Arguments (with defaults from config) ---
    robot_ip = LaunchConfiguration('robot_ip', default=franka_config.get('robot_ip', '192.168.1.59'))
    use_fake_hardware = LaunchConfiguration('use_fake_hardware', default=str(franka_config.get('use_fake_hardware', False)))
    enable_rviz = LaunchConfiguration('enable_rviz', default=str(franka_config.get('enable_rviz', True)))
    log_level = LaunchConfiguration('log_level', default=ws_config.get('logging', {}).get('default_level', 'INFO'))

    # --- Create list of actions to launch ---
    launch_actions = []

    # --- 1. MoveIt Server (includes robot drivers, MoveGroup, etc.) ---
    if packages_config.get('lbx_franka_moveit', True):
        moveit_server_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('lbx_franka_moveit'),
                    'launch',
                    'moveit_server.launch.py'
                ])
            ]),
            launch_arguments={
                'robot_ip': robot_ip,
                'use_fake_hardware': use_fake_hardware,
                'enable_rviz': enable_rviz,
                'load_gripper': str(franka_config.get('load_gripper', True)),
            }.items()
        )
        launch_actions.append(moveit_server_launch)

    # --- 2. Oculus Input Node ---
    if packages_config.get('lbx_input_oculus', True):
        oculus_input_node = Node(
            package='lbx_input_oculus',
            executable='oculus_node.py', # Assuming Python executable
            name='oculus_input_node',
            namespace=LaunchConfiguration('vr_namespace', default=ws_config.get('core',{}).get('vr_namespace', '/vr')),
            output='screen',
            parameters=[{
                'log_level': log_level,
                # Add other Oculus specific parameters from a config file if needed
            }],
            condition=PythonExpression(["'true' == '", use_fake_hardware, "' or ", "'true' == '", LaunchConfiguration('enable_oculus', default='true') , "'"]) # Example condition
        )
        launch_actions.append(oculus_input_node)

    # --- 3. RealSense Vision Node ---
    if packages_config.get('lbx_vision_realsense', True):
        realsense_node_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('lbx_vision_realsense'),
                    'launch',
                    'realsense.launch.py' # Assuming this launch file exists
                ])
            ]),
            launch_arguments={
                'log_level': log_level,
                'config_file': PathJoinSubstitution([
                    FindPackageShare('lbx_robotics'), 'configs', 'realsense_cameras.yaml'
                ])
            }.items(),
            condition=PythonExpression(["'true' == '", use_fake_hardware, "' or ", "'true' == '", LaunchConfiguration('enable_realsense', default='true') , "'"]) # Example condition
        )
        launch_actions.append(realsense_node_launch)
        
    # --- 4. Franka Control/Teleoperation Node ---
    if packages_config.get('lbx_franka_control', True):
        franka_control_node = Node(
            package='lbx_franka_control',
            executable='teleop_node.py', # Assuming Python executable
            name='franka_teleop_node',
            namespace=LaunchConfiguration('robot_namespace', default=ws_config.get('core',{}).get('robot_namespace', '/fr3')),
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('lbx_robotics'), 'configs', 'franka_config.yaml']),
                PathJoinSubstitution([FindPackageShare('lbx_franka_control'), 'config', 'teleop_config.yaml']), # Assuming a teleop_config.yaml
                {
                    'log_level': log_level,
                    'control_mode': franka_config.get('control_mode', 'moveit')
                }
            ]
        )
        launch_actions.append(franka_control_node)

    # --- 5. Data Recorder Node ---
    if packages_config.get('lbx_data_recorder', True):
        data_recorder_node = Node(
            package='lbx_data_recorder',
            executable='mcap_recorder_node.py', # Assuming Python executable
            name='mcap_recorder_node',
            output='screen',
            parameters=[{
                'log_level': log_level,
                'default_format': ws_config.get('recording', {}).get('default_format', 'mcap'),
                'topics_to_record': ws_config.get('recording', {}).get('topics_to_record', [])
            }],
            condition=LaunchConfiguration('enable_recorder', default='false')
        )
        launch_actions.append(data_recorder_node)

    # --- Logging Information ---
    launch_actions.append(LogInfo(msg=[
        "LBX Robotics System Bringup Launching...\n",
        "Robot IP: ", robot_ip, "\n",
        "Use Fake Hardware: ", use_fake_hardware, "\n",
        "Log Level: ", log_level, "\n",
        "Enabled Packages: ", str({k:v for k,v in packages_config.items() if v}),
    ]))

    return [GroupAction(actions=launch_actions)]

def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument('robot_ip', description='IP address of the Franka robot.'))
    declared_arguments.append(DeclareLaunchArgument('use_fake_hardware', description='Use fake hardware for testing.'))
    declared_arguments.append(DeclareLaunchArgument('enable_rviz', description='Enable RViz visualization.'))
    declared_arguments.append(DeclareLaunchArgument('log_level', description='Logging level for nodes.'))
    declared_arguments.append(DeclareLaunchArgument('vr_namespace', default_value='/vr', description='Namespace for VR components.'))
    declared_arguments.append(DeclareLaunchArgument('robot_namespace', default_value='/fr3', description='Namespace for robot components.'))
    declared_arguments.append(DeclareLaunchArgument('enable_oculus', default_value='true', description='Enable Oculus input node.'))
    declared_arguments.append(DeclareLaunchArgument('enable_realsense', default_value='true', description='Enable RealSense vision node.'))
    declared_arguments.append(DeclareLaunchArgument('enable_recorder', default_value='false', description='Enable data recorder node.'))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)]) 