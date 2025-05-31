#!/usr/bin/env python3
"""
Main system bringup launch file for the LBX Robotics workspace.
This file orchestrates the launch of all core components:
- MoveIt Server (including robot drivers and RViz)
- Oculus Input Node
- Generic Vision/Camera Node (e.g., for RealSense or ZED)
- Franka Control/Teleoperation Node
- Data Recorder Node

Parameters can be passed to this launch file to configure components,
or they can be read from workspace_config.yaml and franka_config.yaml.
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, GroupAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
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
        print(f"[ERROR] Error loading {config_file_name}: {e}")
        return {}

def launch_setup(context, *args, **kwargs):
    # --- Load Configurations ---
    ws_config_path_obj = PathJoinSubstitution([
        FindPackageShare('lbx_robotics'), 'configs', 'workspace_config.yaml'
    ])
    franka_config_path_obj = PathJoinSubstitution([
        FindPackageShare('lbx_robotics'), 'configs', 'moveit', 'franka_config.yaml'
    ])
    # Camera config determination logic is now primarily within CameraNode.
    # We pass the 'active_camera_type' to potentially influence which *default* config CameraNode tries to load
    # if no specific camera_config_file is given to it.
    # However, for clarity in system_bringup, we can still construct a potential path to pass.
    active_camera_type_val = context.launch_configurations.get('active_camera_type', 'realsense')
    camera_config_file_override = context.launch_configurations.get('camera_config_file_override', '')
    
    specific_camera_config_path = ''
    if camera_config_file_override and camera_config_file_override != '':
        specific_camera_config_path = camera_config_file_override
    else:
        if active_camera_type_val == 'realsense':
            specific_camera_config_path = PathJoinSubstitution([
                FindPackageShare('lbx_robotics'), 'configs', 'sensors', 'realsense_cameras.yaml']
            ).perform(context)
        elif active_camera_type_val == 'zed':
            specific_camera_config_path = PathJoinSubstitution([
                FindPackageShare('lbx_robotics'), 'configs', 'sensors', 'zed_camera.yaml'] # Ensure this file exists
            ).perform(context)
        # If 'none' or 'opencv', specific_camera_config_path remains empty, letting CameraNode try defaults or fail.

    ws_config_path = ws_config_path_obj.perform(context)
    franka_config_path = franka_config_path_obj.perform(context)

    ws_config = {}
    franka_config_main = {}
    packages_config = {}

    try:
        with open(ws_config_path, 'r') as f:
            full_ws_config = yaml.safe_load(f)
            ws_config = full_ws_config.get('workspace', {})
            packages_config = full_ws_config.get('packages', {})
    except Exception as e:
        print(f"[ERROR] system_bringup: Error loading workspace_config.yaml: {e}")

    try:
        with open(franka_config_path, 'r') as f:
            franka_config_main = yaml.safe_load(f).get('franka_fr3', {})
    except Exception as e:
        print(f"[ERROR] system_bringup: Error loading franka_config.yaml: {e}")

    # --- Access Launch Arguments (which might override YAML values) ---
    robot_ip_val = context.launch_configurations.get('robot_ip')
    use_fake_hardware_val = context.launch_configurations.get('use_fake_hardware')
    enable_rviz_val = context.launch_configurations.get('enable_rviz')
    log_level_val = context.launch_configurations.get('log_level')
    enable_oculus_val = context.launch_configurations.get('enable_oculus')
    vr_namespace_val = context.launch_configurations.get('vr_namespace')
    enable_vision_val = context.launch_configurations.get('enable_vision')
    enable_recorder_val = context.launch_configurations.get('enable_recorder')
    robot_namespace_val = context.launch_configurations.get('robot_namespace')

    launch_actions = []

    # --- 1. MoveIt Server ---
    if packages_config.get('lbx_franka_moveit', True):
        moveit_server_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('lbx_franka_moveit'), 'launch', 'moveit_server.launch.py'])
            ]),
            launch_arguments={
                'robot_ip': robot_ip_val,
                'use_fake_hardware': use_fake_hardware_val,
                'start_rviz': enable_rviz_val,
                'franka_config_file': franka_config_path,
            }.items()
        )
        launch_actions.append(moveit_server_launch)

    # --- 2. Oculus Input Node ---
    if packages_config.get('lbx_input_oculus', True):
        oculus_launch_file_path = PathJoinSubstitution([
            FindPackageShare('lbx_input_oculus'), 'launch', 'oculus_input.launch.py'
        ])
        oculus_input_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(oculus_launch_file_path),
            condition=PythonExpression(["'true' == '", enable_oculus_val, "'"])
        )
        namespaced_oculus_launch = GroupAction(
            actions=[
                PushRosNamespace(namespace=vr_namespace_val),
                oculus_input_launch
            ],
        )
        launch_actions.append(namespaced_oculus_launch)

    # --- 3. Generic Vision/Camera Node ---
    if packages_config.get('lbx_vision_camera', True) and enable_vision_val == 'true' and active_camera_type_val != 'none':
        vision_launch_args = {
            'camera_config_file': specific_camera_config_path, 
            'enable_publishing': 'true',
            'run_startup_tests': 'true',
            'log_level': log_level_val,
        }
        # Only add auto_config_generation_dir if specific_camera_config_path is empty, signaling auto-detection path
        if not specific_camera_config_path or specific_camera_config_path == '':
            vision_launch_args['auto_config_generation_dir'] = PathJoinSubstitution([
                FindPackageShare('lbx_robotics'), 'configs', 'sensors']
            ).perform(context)
            
        vision_camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('lbx_vision_camera'), 'launch', 'camera.launch.py'])]),
            launch_arguments=vision_launch_args.items(),
            condition=PythonExpression(["'true' == '", enable_vision_val, "'"]) # Redundant if check above, but good practice
        )
        launch_actions.append(vision_camera_launch)
        
    # --- 4. Franka Control/Teleoperation Node ---
    if packages_config.get('lbx_franka_control', True):
        teleop_config_path = PathJoinSubstitution([FindPackageShare('lbx_franka_control'), 'config', 'teleop_config.yaml'])
        franka_control_node = Node(
            package='lbx_franka_control',
            executable='teleop_node.py',
            name='franka_teleop_node',
            namespace=robot_namespace_val,
            output='screen',
            parameters=[
                franka_config_path,
                teleop_config_path,
                {
                }
            ],
            arguments=['--ros-args', '--log-level', log_level_val]
        )
        launch_actions.append(franka_control_node)

    # --- 5. Data Recorder Node ---
    if packages_config.get('lbx_data_recorder', True):
        recorder_config_path = PathJoinSubstitution([FindPackageShare('lbx_data_recorder'), 'config', 'recorder_config.yaml'])
        data_recorder_node = Node(
            package='lbx_data_recorder',
            executable='mcap_recorder_node.py',
            name='mcap_recorder_node',
            output='screen',
            parameters=[recorder_config_path],
            arguments=['--ros-args', '--log-level', log_level_val],
            condition=PythonExpression(["'true' == '", enable_recorder_val, "'"])
        )
        launch_actions.append(data_recorder_node)

    log_actions = [
        LogInfo(msg=f"--- LBX Robotics System Bringup ---"),
        LogInfo(msg=f"Robot IP: {robot_ip_val}"),
        LogInfo(msg=f"Use Fake Hardware: {use_fake_hardware_val}"),
        LogInfo(msg=f"Global Log Level: {log_level_val}"),
        LogInfo(msg=f"Oculus Enabled: {enable_oculus_val}, Namespace: {vr_namespace_val}"),
        LogInfo(msg=f"Vision: {enable_vision_val} (ActiveType: {active_camera_type_val}, ConfigUsed: {specific_camera_config_path or 'NodeAutoDetected'})"),
        LogInfo(msg=f"Recorder Enabled: {enable_recorder_val}"),
        LogInfo(msg=f"Packages Configured to Launch (from workspace_config.yaml): { {k:v for k,v in packages_config.items() if v} }")
    ]

    return [GroupAction(actions=log_actions + launch_actions)]

def generate_launch_description():
    default_ws_config = PathJoinSubstitution([FindPackageShare('lbx_robotics'), 'configs', 'workspace_config.yaml'])
    default_franka_config = PathJoinSubstitution([FindPackageShare('lbx_robotics'), 'configs', 'moveit', 'franka_config.yaml'])
    
    declared_arguments = [
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.59', description='IP address of the Franka robot.'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false', description='Use fake hardware for testing.'),
        DeclareLaunchArgument('enable_rviz', default_value='true', description='Enable RViz visualization.'),
        DeclareLaunchArgument('log_level', default_value='info', description='Global logging level for most nodes.'),
        DeclareLaunchArgument('vr_namespace', default_value='/vr', description='Namespace for VR components.'),
        DeclareLaunchArgument('robot_namespace', default_value='/fr3', description='Namespace for robot components.'),
        DeclareLaunchArgument('enable_oculus', default_value='true', description='Enable Oculus input node.'),
        DeclareLaunchArgument('enable_vision', default_value='true', description='Enable generic vision camera node.'),
        DeclareLaunchArgument('active_camera_type', default_value='realsense', choices=['realsense', 'zed', 'none'], description='Primary camera type to configure/use (realsense, zed, or none).'),
        DeclareLaunchArgument('camera_config_file_override', default_value='', description='Full path to a specific camera config YAML to override auto-selection.'),
        DeclareLaunchArgument('enable_recorder', default_value='false', description='Enable data recorder node.'),
        
        DeclareLaunchArgument('workspace_config_file', default_value=default_ws_config, description='Path to workspace_config.yaml'),
        DeclareLaunchArgument('franka_config_file', default_value=default_franka_config, description='Path to franka_config.yaml for MoveIt')
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)]) 