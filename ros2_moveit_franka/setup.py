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
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 MoveIt package for controlling Franka FR3 arm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'franka_moveit_control = ros2_moveit_franka.franka_moveit_control:main',
            'simple_arm_control = ros2_moveit_franka.simple_arm_control:main',
            'vr_benchmark_simple = ros2_moveit_franka.vr_benchmark_simple:main',
        ],
    },
) 