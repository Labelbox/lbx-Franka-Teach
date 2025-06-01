#!/usr/bin/env python3
"""
Launch file for the LBX Franka FR3 MoveIt Server.
This launch file starts the core MoveIt2 components (move_group, rviz, etc.)
based on configurations from lbx_franka_description and lbx_franka_moveit packages.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    # Get launch configurations
    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    enable_rviz = LaunchConfiguration('enable_rviz')
    load_gripper = LaunchConfiguration('load_gripper')
    rviz_config = LaunchConfiguration('rviz_config')
    # arm_id = LaunchConfiguration('arm_id') # arm_id no longer needed here for robot_description
    
    # robot_description_content is now generated and published by RobotStatePublisher in system_bringup.launch.py
    # robot_description_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name='xacro')]),
    #         ' ',
    #         PathJoinSubstitution([
    #             FindPackageShare('lbx_franka_description'),
    #             'urdf',
    #             'fr3.urdf.xacro'
    #         ]),
    #         ' arm_id:=',
    #         arm_id,
    #         ' robot_ip:=',
    #         robot_ip,
    #         ' use_fake_hardware:=',
    #         use_fake_hardware,
    #     ]
    # )
    
    # Include the standard Franka MoveIt launch file
    # It should pick up /robot_description from the topic
    moveit_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('franka_fr3_moveit_config'),
                'launch',
                'moveit.launch.py'
            ])
        ),
        launch_arguments=({
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake_hardware,
            'load_gripper': load_gripper,
            'use_rviz': enable_rviz,
            'rviz_config': rviz_config,
            # 'robot_description': robot_description_content, # Removed to use topic
        }).items()
    )
    
    return [moveit_launch_include]

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
    
    # arm_id is no longer directly used here for robot_description, 
    # but might be needed if other parts of this launch file or included files use it.
    # For now, let's keep it declared.
    declared_arguments.append(
        DeclareLaunchArgument(
            'arm_id',
            default_value='fr3',
            description='Name of the arm.'
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
    
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    ) 