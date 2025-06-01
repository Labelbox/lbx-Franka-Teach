#!/usr/bin/env python3
"""
Launch file for the LBX Franka FR3 MoveIt Server.
This launch file starts the core MoveIt2 components (move_group, rviz, etc.)
based on configurations from lbx_franka_description and lbx_franka_moveit packages.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.1.59',
            description='IP address of the Franka robot.'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Use fake hardware for testing.'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'enable_rviz',
            default_value='true',
            description='Enable RViz visualization.'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'load_gripper',
            default_value='true',
            description='Load gripper model and controllers.'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('franka_fr3_moveit_config'),
                'rviz',
                'moveit.rviz'
            ]),
            description='Path to the RViz configuration file.'
        )
    )
    
    # Get launch configurations
    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    enable_rviz = LaunchConfiguration('enable_rviz')
    load_gripper = LaunchConfiguration('load_gripper')
    rviz_config = LaunchConfiguration('rviz_config')
    
    # Include the standard Franka MoveIt launch file
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('franka_fr3_moveit_config'),
                'launch',
                'moveit.launch.py'
            ])
        ),
        launch_arguments={
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake_hardware,
            'load_gripper': load_gripper,
            'use_rviz': enable_rviz,
            'rviz_config': rviz_config,
        }.items()
    )
    
    return LaunchDescription(
        declared_arguments + [moveit_launch]
    ) 