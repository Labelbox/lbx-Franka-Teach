from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lbx_vision_camera'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'sensor_msgs',
        'geometry_msgs',
        'cv_bridge',
        'tf2_ros_py',
        'opencv-python',
        'PyYAML',
        'numpy',
        'transformations',
        'pyrealsense2', # Keep realsense for now, can be made optional later
        # Add ZED SDK python dep here if it becomes a direct pip installable item
        'diagnostic_updater',
    ],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='ROS2 package to manage and publish generic camera data (e.g., RealSense, ZED).',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'camera_node = lbx_vision_camera.camera_node:main', # Renamed executable and node file
        ],
    },
) 