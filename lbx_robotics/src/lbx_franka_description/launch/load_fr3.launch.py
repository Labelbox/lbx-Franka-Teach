#!/usr/bin/env python3
"""
Launch file to load FR3 URDF with ros2_control configuration
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.1.59',
            description='IP address of the Franka robot'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Use fake hardware for testing'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'arm_id',
            default_value='fr3',
            description='Name of the arm'
        )
    )
    
    # Get parameters
    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    arm_id = LaunchConfiguration('arm_id')
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('lbx_franka_description'),
                'urdf',
                'fr3.urdf.xacro'
            ]),
            ' arm_id:=',
            arm_id,
            ' robot_ip:=',
            robot_ip,
            ' use_fake_hardware:=',
            use_fake_hardware,
        ]
    )
    
    robot_description = {'robot_description': robot_description_content}
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )
    
    # Joint State Publisher (for fake hardware)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'source_list': ['franka/joint_states'], 'rate': 30}
        ],
    )
    
    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_publisher_node,
    ]
    
    return LaunchDescription(declared_arguments + nodes_to_start) 