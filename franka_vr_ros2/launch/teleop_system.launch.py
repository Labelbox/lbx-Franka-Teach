"""
ROS2 Launch file for VR teleoperation system
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def load_yaml(package_name, file_path):
    """Load a yaml file from a package"""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    """Generate launch description for VR teleoperation"""
    
    # Declare arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip', 
        default_value='192.168.1.59',  # Same as Deoxys/franka_right.yml
        description='IP address of the Franka robot'
    )
    
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='fr3',
        description='Type of robot (fr3, panda, etc.)'
    )
    
    control_strategy_arg = DeclareLaunchArgument(
        'control_strategy', 
        default_value='moveit_servo',
        description='Control strategy to use'
    )
    
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware', 
        default_value='false',
        description='Use fake hardware for simulation'
    )
    
    # Get package directory
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    # Load configurations based on robot type
    robot_type = LaunchConfiguration('robot_type')
    robot_config_path = f'config/robots/{robot_type}/robot_config.yaml'
    srdf_path = f'config/robots/{robot_type}/moveit/franka_{robot_type}.srdf'
    kinematics_path = f'config/robots/{robot_type}/moveit/kinematics.yaml'
    joint_limits_path = f'config/robots/{robot_type}/moveit/joint_limits.yaml'
    moveit_servo_path = f'config/robots/{robot_type}/moveit/moveit_servo.yaml'
    controllers_path = f'config/robots/{robot_type}/moveit/controllers.yaml'
    
    # Load configurations
    robot_description_config = None  # Would load from URDF
    robot_description_semantic_config = None
    kinematics_config = None
    joint_limits_config = None
    moveit_servo_config = None
    robot_config = None
    
    # Try to load configs from our package
    try:
        # For now, hardcode FR3 path since LaunchConfiguration can't be used in file paths at load time
        fr3_srdf_path = os.path.join(pkg_dir, 'config/robots/fr3/moveit/franka_fr3.srdf')
        with open(fr3_srdf_path, 'r') as f:
            robot_description_semantic_config = f.read()
            
        kinematics_config = load_yaml('franka_vr_ros2', 'config/robots/fr3/moveit/kinematics.yaml')
        joint_limits_config = load_yaml('franka_vr_ros2', 'config/robots/fr3/moveit/joint_limits.yaml')
        moveit_servo_config = load_yaml('franka_vr_ros2', 'config/robots/fr3/moveit/moveit_servo.yaml')
        robot_config = load_yaml('franka_vr_ros2', 'config/robots/fr3/robot_config.yaml')
    except Exception as e:
        print(f"Warning: Could not load some configurations: {e}")
    
    # Robot driver node (placeholder - would be actual Franka driver)
    robot_driver = Node(
        package='franka_ros2',
        executable='franka_control_node',
        parameters=[
            robot_config if robot_config else {},
            {
                'robot_ip': LaunchConfiguration('robot_ip'),
                'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            }
        ],
        condition=IfCondition("control_strategy != 'debug'")
    )
    
    # MoveIt Servo node (conditional on strategy)
    moveit_servo = Node(
        package='moveit_servo',
        executable='servo_node',
        parameters=[
            moveit_servo_config if moveit_servo_config else {
                'use_intra_process_comms': True,
                'command_in_type': 'unitless',
                'scale': {
                    'linear': 0.4,
                    'rotational': 0.8
                },
                'low_pass_filter_coeff': 2.0,
                'publish_period': 0.008,  # 125Hz
                'incoming_command_timeout': 0.1,
                'command_out_topic': '/fr3_arm_controller/joint_trajectory',
                'command_in_topic': '/servo_node/delta_twist_cmds',
                'planning_frame': 'fr3_link0',
                'ee_frame': 'fr3_hand'
            },
            {
                'robot_description': robot_description_config,
                'robot_description_semantic': robot_description_semantic_config,
                'robot_description_kinematics': kinematics_config,
            }
        ],
        condition=IfCondition("control_strategy == 'moveit_servo'")
    )
    
    # Move Group node for MoveIt
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[
            {
                'robot_description': robot_description_config,
                'robot_description_semantic': robot_description_semantic_config,
                'robot_description_kinematics': kinematics_config,
                'robot_description_planning': joint_limits_config,
                'planning_scene_monitor_options': {
                    'publish_planning_scene': True,
                    'publish_geometry_updates': True,
                    'publish_state_updates': True,
                    'publish_transforms_updates': True,
                    'publish_robot_description': True,
                    'publish_robot_description_semantic': True,
                },
                'use_sim_time': False,
                'planning_pipelines': ['ompl'],
                'default_planning_pipeline': 'ompl',
            }
        ],
        condition=IfCondition("control_strategy == 'moveit_servo'")
    )
    
    # Joint state publisher for simulation
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'rate': 100,
            'source_list': ['/franka/joint_states']
        }],
        condition=IfCondition(LaunchConfiguration('use_fake_hardware'))
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_config,
            'publish_frequency': 100.0
        }]
    )
    
    return LaunchDescription([
        robot_ip_arg,
        robot_type_arg,
        control_strategy_arg,
        use_fake_hardware_arg,
        robot_driver,
        moveit_servo,
        move_group,
        joint_state_publisher,
        robot_state_publisher
    ]) 