from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'lbx_launch'

setup(
    name=package_name,
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.xml'))), # If you have XML launch files
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))), # If you have config files
    ],
    zip_safe=True
) 