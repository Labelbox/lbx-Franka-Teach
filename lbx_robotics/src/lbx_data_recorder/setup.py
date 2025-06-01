from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'lbx_data_recorder'

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
        # Config files if any
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Labelbox Robotics',
    maintainer_email='robotics@labelbox.com',
    description='High-performance data recording for VR teleoperation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'recorder_node = lbx_data_recorder.recorder_node:main',
            'mcap_recorder_node = lbx_data_recorder.mcap_recorder_node:main',
        ],
    },
) 