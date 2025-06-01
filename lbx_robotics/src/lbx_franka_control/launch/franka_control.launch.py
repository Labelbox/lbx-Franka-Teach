#!/usr/bin/env python3
"""
Franka Control Launch File

Launches the system manager node with appropriate configuration.
This is typically called by the main system_bringup.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('lbx_robotics'),
            'configs',
            'control',
            'franka_vr_control_config.yaml'
        ]),
        description='Path to the control configuration file'
    )
    
    declare_use_left_controller = DeclareLaunchArgument(
        'use_left_controller',
        default_value='false',
        description='Use left controller instead of right'
    )
    
    declare_performance_mode = DeclareLaunchArgument(
        'performance_mode',
        default_value='false',
        description='Enable performance mode'
    )
    
    declare_enable_recording = DeclareLaunchArgument(
        'enable_recording',
        default_value='true',
        description='Enable data recording'
    )
    
    # System Manager Node
    system_manager_node = Node(
        package='lbx_franka_control',
        executable='system_manager',
        name='system_manager',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'use_left_controller': LaunchConfiguration('use_left_controller'),
            'performance_mode': LaunchConfiguration('performance_mode'),
            'enable_recording': LaunchConfiguration('enable_recording'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        declare_config_file,
        declare_use_left_controller,
        declare_performance_mode,
        declare_enable_recording,
        system_manager_node,
    ]) 