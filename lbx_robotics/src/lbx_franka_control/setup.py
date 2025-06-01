from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'lbx_franka_control'

# Metadata is now in pyproject.toml
setup(
    name=package_name,
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    zip_safe=True,
) 