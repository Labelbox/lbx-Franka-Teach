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
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'oculus_reader', 'APK'), glob(os.path.join('lbx_input_oculus', 'oculus_reader', 'APK', '*.apk'))),
    ],
    install_requires=['setuptools', 'numpy', 'pure-python-adb', 'tf-transformations'], # Add tf-transformations
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='ROS2 package to read and publish Oculus Quest controller data.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'oculus_node = lbx_input_oculus.oculus_node:main',
        ],
    },
) 