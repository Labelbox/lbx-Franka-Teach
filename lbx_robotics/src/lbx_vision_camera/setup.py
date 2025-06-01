from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lbx_vision_camera'
# The camera_utilities submodule will remain as is internally for now

# All metadata is now in pyproject.toml
# This setup.py can be very minimal or even removed if only pyproject.toml is used
# However, for colcon/ROS 2 ament_python, a minimal setup.py is often still expected.
setup(
    name=package_name, # Still good to have name here for some tools
    packages=find_packages(exclude=['test']),
    # Data files still need to be specified here for ament_python
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'], # Explicitly list setuptools here too
    zip_safe=True # Standard option
    # No install_requires, entry_points, license, description, etc. - moved to pyproject.toml
    # tests_require can be moved to [project.optional-dependencies] in pyproject.toml
) 