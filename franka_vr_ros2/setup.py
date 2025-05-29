from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'franka_vr_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/**/*.yaml', recursive=True)),
        (os.path.join('share', package_name, 'config'),
            glob('config/**/*.yml', recursive=True)),
    ],
    install_requires=[
        'setuptools',
        'numpy>=1.19.0,<2.0',
        'scipy>=1.7.0',
        'rclpy>=3.3.0',
        'geometry_msgs',
        'sensor_msgs',
        'moveit_msgs',
        'tf2_ros',
        'tf2_geometry_msgs',
        'uvloop>=0.16.0',
        'aiofiles>=0.8.0',
        'pure-python-adb>=0.3.0.dev0',
    ],
    zip_safe=True,
    maintainer='Franka VR Team',
    maintainer_email='your_email@example.com',
    description='High-performance VR teleoperation for Franka robots using ROS2 and MoveIt',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'oculus_vr_server = franka_vr_ros2.oculus_vr_server:main',
        ],
    },
    python_requires='>=3.8',
) 