from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lbx_input_oculus'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include APK directory
        (os.path.join('share', package_name, 'APK'), glob(os.path.join('lbx_input_oculus', 'oculus_reader', 'APK', '*'))),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'pure-python-adb',
        'transformations',
        'diagnostic_updater',
    ],
    zip_safe=True,
    maintainer='Labelbox Robotics',
    maintainer_email='robotics@labelbox.com',
    description='ROS2 package to read and publish Meta Oculus Quest controller and tracking data.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'oculus_node = lbx_input_oculus.oculus_node:main',
        ],
    },
) 