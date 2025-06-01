import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Node for Oculus input
    oculus_input_node = Node(
        package='lbx_input_oculus',
        executable='oculus_node',
        name='oculus_input_node',
        output='screen',
        parameters=[{
            # Add any default parameters here if needed
            'use_sim_time': False,
        }],
        # remappings=[
        #     # Example remappings if needed
        #     # ('~/left_controller_pose', '/oculus/left_pose'), 
        #     # ('~/right_controller_pose', '/oculus/right_pose'),
        # ],
    )

    return LaunchDescription([
        oculus_input_node,
    ]) 