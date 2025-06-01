from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'lbx_data_recorder'

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
        # Config files if any
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml', recursive=True)),
    ],
    zip_safe=True # install_requires, entry_points etc. are in pyproject.toml
) 