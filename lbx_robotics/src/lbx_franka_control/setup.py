from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'lbx_franka_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='labelbox',
    maintainer_email='robotics@labelbox.com',
    description='VR-based Franka control system using MoveIt',
    license='MIT',
    entry_points={
        'console_scripts': [
            'system_manager = lbx_franka_control.system_manager:main',
            'franka_controller = lbx_franka_control.franka_controller:main',
            'system_orchestrator = lbx_franka_control.system_orchestrator:main',
            'main_system = lbx_franka_control.main_system:main',
            'robot_control_node = lbx_franka_control.robot_control_node:main',
            'vr_teleop_node = lbx_franka_control.vr_teleop_node:main',
            'system_monitor = lbx_franka_control.system_monitor:main',
        ],
    },
) 