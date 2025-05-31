#!/usr/bin/env python3
"""
Launch file for the LBX Franka FR3 MoveIt Server.
This launch file starts the core MoveIt2 components (move_group, rviz, etc.)
based on configurations from lbx_franka_description and lbx_franka_moveit packages.
It references franka_config.yaml for robot-specific parameters.
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def launch_setup(context, *args, **kwargs):
    # --- Load Core Configuration ---
    # Load parameters from the central franka_config.yaml
    config_path = PathJoinSubstitution([
        FindPackageShare('lbx_robotics'), # Assuming configs are in the root of lbx_robotics
        'configs',
        'franka_config.yaml'
    ]).perform(context)

    try:
        with open(config_path, 'r') as f:
            config_params = yaml.safe_load(f)
            franka_params = config_params.get('franka_fr3', {})
    except Exception as e:
        print(f"Error loading franka_config.yaml: {e}")
        franka_params = {}

    # --- Launch Arguments (can override config) ---
    robot_ip = LaunchConfiguration('robot_ip', default=franka_params.get('robot_ip', '192.168.1.59'))
    use_fake_hardware = LaunchConfiguration('use_fake_hardware', default=str(franka_params.get('use_fake_hardware', False)))
    enable_rviz = LaunchConfiguration('enable_rviz', default=str(franka_params.get('enable_rviz', True)))
    load_gripper = LaunchConfiguration('load_gripper', default=str(franka_params.get('load_gripper', True)))
    
    # --- Determine MoveIt Config Package ---
    # This relies on franka_ros2 being installed and providing franka_fr3_moveit_config
    # If you have a custom MoveIt config, point to that package instead.
    # For now, we assume the standard Franka-provided MoveIt configuration.
    moveit_config_pkg = franka_params.get('moveit_config_package', 'franka_fr3_moveit_config')
    
    # --- Robot Description ---
    # Using xacro to process the URDF
    robot_description_content = Command([
        PathJoinSubstitution([FindPackageShare(franka_params.get('description_package', 'franka_description')), "robots", "fr3", "fr3.urdf.xacro"]), " ", 
        "robot_ip:=", robot_ip, " ",
        "use_fake_hardware:=", use_fake_hardware, " ",
        "hand_tcp_frame:=", franka_params.get('end_effector_link', 'fr3_hand_tcp') # Ensure this xacro arg exists
    ])

    robot_description = {'robot_description': robot_description_content}

    # --- Robot State Publisher ---
    # This is usually part of the franka_fr3_moveit_config launch, but included here for clarity
    # if not launched via the main MoveIt launch.
    # We expect franka_fr3_moveit_config/moveit.launch.py to handle this.

    # --- MoveIt Core Launch ---
    # This includes move_group, trajectory execution, and other essential MoveIt nodes.
    # It's crucial that this points to a valid MoveIt launch file.
    # Typically, this would be franka_fr3_moveit_config/launch/moveit.launch.py
    moveit_core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(moveit_config_pkg),
                'launch',
                'moveit.launch.py' # Standard MoveIt launch file
            ])
        ]),
        launch_arguments={
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake_hardware,
            'load_gripper': load_gripper, # Franka's MoveIt launch often has this
            'use_rviz': 'false', # We will launch RViz separately if enabled
            # Pass other relevant parameters if your MoveIt launch supports them
            # 'robot_description': robot_description_content, # Sometimes passed directly
        }.items()
    )

    # --- RViz Visualization ---
    rviz_config_file_default = PathJoinSubstitution([
        FindPackageShare(moveit_config_pkg),
        'rviz',
        'moveit.rviz' # Default RViz config from the MoveIt package
    ])
    rviz_config_file = LaunchConfiguration('rviz_config', default=rviz_config_file_default)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[robot_description, {'use_sim_time': use_fake_hardware}],
        condition=IfCondition(enable_rviz)
    )
    
    # --- Logging Information ---
    log_info = LogInfo(msg=[
        "LBX Franka MoveIt Server Launching...\n",
        "Robot IP: ", robot_ip, "\n",
        "Use Fake Hardware: ", use_fake_hardware, "\n",
        "Enable RViz: ", enable_rviz, "\n",
        "MoveIt Config Package: ", moveit_config_pkg, "\n",
    ])

    return [log_info, moveit_core_launch, rviz_node]

def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_ip',
            description='IP address of the Franka robot.',
            # Default will be picked from franka_config.yaml by launch_setup
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            description='Use fake hardware for testing (true/false).',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'enable_rviz',
            description='Enable RViz visualization (true/false).',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'load_gripper',
            description='Load gripper model and controllers (true/false).',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_config',
            description='Path to the RViz configuration file.',
            # Default will be set in launch_setup
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)]) 