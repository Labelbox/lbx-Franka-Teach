from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lbx_input_oculus'

# All metadata is now in pyproject.toml
setup(
    name=package_name,
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'oculus_reader', 'APK'), glob(os.path.join('lbx_input_oculus', 'oculus_reader', 'APK', '*.apk'))),
    ],
    zip_safe=True
) 