#!/usr/bin/env python3
"""
Launch file for data recorder node
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value=os.path.expanduser('~/lbx_recordings'),
        description='Directory to save recordings'
    )
    
    recording_hz_arg = DeclareLaunchArgument(
        'recording_hz',
        default_value='60.0',
        description='Recording frequency in Hz'
    )
    
    verify_data_arg = DeclareLaunchArgument(
        'verify_data',
        default_value='false',
        description='Verify MCAP data after recording'
    )
    
    # Data recorder node
    recorder_node = Node(
        package='lbx_data_recorder',
        executable='recorder_node',
        name='data_recorder',
        output='screen',
        parameters=[{
            'output_dir': LaunchConfiguration('output_dir'),
            'recording_hz': LaunchConfiguration('recording_hz'),
            'queue_size': 1000,
            'verify_after_recording': LaunchConfiguration('verify_data'),
        }]
    )
    
    return LaunchDescription([
        output_dir_arg,
        recording_hz_arg,
        verify_data_arg,
        recorder_node,
    ]) 