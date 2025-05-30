#!/usr/bin/env python3
"""
Launch file for Franka FR3 MoveIt demo
This launch file starts the Franka MoveIt configuration and runs the simple arm control demo.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Declare launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.59',
        description='IP address of the Franka robot'
    )
    
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Use fake hardware for testing (true/false)'
    )
    
    start_demo_arg = DeclareLaunchArgument(
        'start_demo',
        default_value='true',
        description='Automatically start the demo sequence'
    )
    
    # Get launch configurations
    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    start_demo = LaunchConfiguration('start_demo')
    
    # Include the Franka FR3 MoveIt launch file
    franka_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('franka_fr3_moveit_config'),
                'launch',
                'moveit.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake_hardware,
            'load_gripper': 'true',
        }.items()
    )
    
    # Launch our demo node
    demo_node = Node(
        package='ros2_moveit_franka',
        executable='simple_arm_control',
        name='franka_demo_controller',
        output='screen',
        parameters=[
            {'use_sim_time': False}
        ],
        condition=IfCondition(start_demo)
    )
    
    # Launch RViz for visualization
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('franka_fr3_moveit_config'),
        'rviz',
        'moveit.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            {'use_sim_time': False}
        ]
    )
    
    return LaunchDescription([
        robot_ip_arg,
        use_fake_hardware_arg,
        start_demo_arg,
        franka_moveit_launch,
        rviz_node,
        demo_node,
    ]) 