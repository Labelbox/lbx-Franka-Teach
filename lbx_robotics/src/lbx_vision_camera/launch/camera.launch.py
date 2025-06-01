import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Default for camera_config_file is now empty, relying on node's auto-detection.
    # User can still provide a specific path to override.
    default_camera_config_file_value = '' 

    # Fix: Use environment variable or fallback to a sensible default path
    # instead of the non-existent 'lbx_robotics' package
    def get_configs_sensors_dir():
        # Try to get from environment variable first
        workspace_root = os.environ.get('COLCON_WS', None)
        if workspace_root:
            return os.path.join(workspace_root, 'lbx_robotics', 'configs', 'sensors')
        
        # Fallback: try to detect based on current package location
        try:
            lbx_vision_camera_share = get_package_share_directory('lbx_vision_camera')
            # Navigate from install/lbx_vision_camera/share/lbx_vision_camera to workspace root
            workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(lbx_vision_camera_share))))
            return os.path.join(workspace_root, 'lbx_robotics', 'configs', 'sensors')
        except:
            # Last resort: hardcoded fallback
            return '/tmp/camera_configs'

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'camera_config_file',
            default_value=default_camera_config_file_value,
            description='Full path to the cameras configuration YAML file. If empty, node attempts auto-detection.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'auto_config_generation_dir',
            default_value=get_configs_sensors_dir(),
            description='Directory where auto-generated camera configs will be placed if needed.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'enable_publishing', default_value='true',
            description='Enable/disable all data publishing from the node.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'run_startup_tests', default_value='true',
            description='Run camera hardware tests on startup.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'publish_rate_hz', default_value='30.0',
            description='Target rate for publishing camera data.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'enable_pointcloud', default_value='true',
            description='Enable/disable point cloud publishing (if depth data is available).'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'diagnostics_publish_rate_hz', default_value='0.2',
            description='Rate for publishing diagnostic information.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='Logging level for the camera_node (debug, info, warn, error, fatal).'
        )
    )

    camera_node = Node(
        package='lbx_vision_camera',
        executable='camera_node',
        name='vision_camera_node',
        output='screen',
        parameters=[{
            'camera_config_file': LaunchConfiguration('camera_config_file'),
            'auto_config_generation_dir': LaunchConfiguration('auto_config_generation_dir'),
            'enable_publishing': LaunchConfiguration('enable_publishing'),
            'run_startup_tests': LaunchConfiguration('run_startup_tests'),
            'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
            'enable_pointcloud': LaunchConfiguration('enable_pointcloud'),
            'diagnostics_publish_rate_hz': LaunchConfiguration('diagnostics_publish_rate_hz'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )

    return LaunchDescription(declared_arguments + [camera_node]) 