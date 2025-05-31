from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ros2_moveit_franka'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') if os.path.exists('config') else []),
    ],
    install_requires=[
        'setuptools',
        'psutil',  # For system monitoring
        'dataclasses',  # For health metrics
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 MoveIt package for controlling Franka FR3 arm with robust error handling',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robust_franka_control = ros2_moveit_franka.robust_franka_control:main',
            'system_health_monitor = ros2_moveit_franka.system_health_monitor:main',
        ],
    },
) 