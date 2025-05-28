from setuptools import setup, find_packages

setup(
    name='franka_vr_ros2',
    version='0.1.0',
    packages=find_packages(),
    install_requires=[
        'numpy>=1.19.0,<2.0',
        'scipy',
        'rclpy>=3.3.0',
        'uvloop',
        'aiofiles',
    ],
    entry_points={
        'console_scripts': [
            'oculus_vr_server = oculus_vr_server:main',
        ],
    },
    author='Franka VR Team',
    description='ROS2/MoveIt VR teleoperation for Franka FR3',
    python_requires='>=3.8',
) 