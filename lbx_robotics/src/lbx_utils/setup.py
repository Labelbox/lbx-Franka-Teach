from setuptools import find_packages, setup

package_name = 'lbx_utils'

# Metadata is now in pyproject.toml
setup(
    name=package_name,
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    zip_safe=True
) 