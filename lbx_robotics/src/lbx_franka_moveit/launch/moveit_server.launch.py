#!/usr/bin/env python3
"""
Launch file for the LBX Franka FR3 MoveIt Server.
This launch file starts the core MoveIt2 components (move_group, rviz, etc.)
based on configurations from lbx_franka_description and lbx_franka_moveit packages.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node # Node might still be needed for RViz if not handled by included launch
from launch_ros.substitutions import FindPackageShare

# OpaqueFunction is removed from generate_launch_description, launch_setup is simplified or removed

def generate_launch_description():
    # Declare arguments that this launch file will accept and pass through
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.59',
        description='IP address of the Franka robot.'
    )
    
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Use fake hardware for testing.'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Enable RViz visualization.'
    )
    
    load_gripper_arg = DeclareLaunchArgument(
        'load_gripper',
        default_value='true',
        description='Load gripper model and controllers.'
    )

    # arm_id is not explicitly passed to franka_fr3_moveit_config/moveit.launch.py by the reference
    # It will use its own default or expect it from xacro if franka_description is used internally.
    # We assume franka_fr3_moveit_config handles its own arm_id or uses a default like 'panda' or 'fr3'.

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('franka_fr3_moveit_config'), # Use the rviz config from franka_fr3_moveit_config
            'rviz',
            'moveit.rviz'
        ]),
        description='Path to the RViz configuration file.'
    )

    # Get launch configurations to pass to the included launch file
    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    enable_rviz = LaunchConfiguration('enable_rviz') # This will be passed as 'use_rviz' to the included launch
    load_gripper = LaunchConfiguration('load_gripper')
    rviz_config = LaunchConfiguration('rviz_config')

    # Include the Franka FR3 MoveIt launch file
    # This is the core of the MoveIt setup from the franka_fr3_moveit_config package.
    # It is expected to handle robot_description generation, RobotStatePublisher, ros2_control_node, move_group, etc.
    franka_moveit_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('franka_fr3_moveit_config'),
                'launch',
                'moveit.launch.py' # This is the standard MoveIt launch for franka
            ])
        ]),
        launch_arguments={
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake_hardware,
            'load_gripper': load_gripper,
            'use_rviz': enable_rviz, # franka_fr3_moveit_config/moveit.launch.py typically uses 'use_rviz'
            'rviz_config_file': rviz_config # and 'rviz_config_file' for the rviz configuration
            # Note: We are NOT passing 'robot_description' or 'arm_id' here.
            # The included moveit.launch.py should handle these based on its own xacro files
            # and the arguments like robot_ip and use_fake_hardware.
        }.items()
    )

    declared_arguments_list = [
        robot_ip_arg,
        use_fake_hardware_arg,
        enable_rviz_arg,
        load_gripper_arg,
        rviz_config_arg,
    ]

    return LaunchDescription(declared_arguments_list + [franka_moveit_launch_include]) 