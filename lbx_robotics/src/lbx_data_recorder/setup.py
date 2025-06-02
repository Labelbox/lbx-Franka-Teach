from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lbx_data_recorder'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Data recording and MCAP integration for robotics teleoperation',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'mcap_recorder_node = lbx_data_recorder.mcap_recorder_node:main',
            'recorder_node = lbx_data_recorder.recorder_node:main',
        ],
    },
) 