import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Path to the default config file
    default_config_path = os.path.join(
        get_package_share_directory('lbx_input_oculus'),
        'config',
        'oculus_config.yaml'
    )

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Full path to the Oculus node configuration file.'
    )
    
    # Node for Oculus input
    oculus_input_node = Node(
        package='lbx_input_oculus',
        executable='oculus_node',
        name='oculus_input_node', # Node name used in the config file
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        # remappings=[
        #     # Example remappings if needed
        #     # ('~/left_controller_pose', '/oculus/left_pose'), 
        #     # ('~/right_controller_pose', '/oculus/right_pose'),
        # ],
    )

    return LaunchDescription([
        config_file_arg,
        oculus_input_node,
    ]) 