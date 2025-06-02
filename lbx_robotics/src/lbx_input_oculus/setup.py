from setuptools import find_packages, setup
from setuptools.command.install import install
import os
import shutil
from glob import glob

package_name = 'lbx_input_oculus'

class CustomInstallCommand(install):
    """Custom install command to create ROS2-expected directory structure."""
    def run(self):
        # Run the standard install
        install.run(self)
        
        # Create the lib/package_name directory structure that ROS2 expects
        lib_dir = os.path.join(self.install_lib, '..', 'lib', package_name)
        bin_dir = os.path.join(self.install_scripts)
        
        # Create the directory if it doesn't exist
        os.makedirs(lib_dir, exist_ok=True)
        
        # Copy executables from bin to lib/package_name
        if os.path.exists(bin_dir):
            for filename in os.listdir(bin_dir):
                src_file = os.path.join(bin_dir, filename)
                dst_file = os.path.join(lib_dir, filename)
                if os.path.isfile(src_file):
                    shutil.copy2(src_file, dst_file)
                    # Make sure it's executable
                    os.chmod(dst_file, 0o755)

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Note: Using system-wide oculus-reader package instead of local files
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'pure-python-adb',
        'transformations', 
        'diagnostic_updater',
        'oculus-reader',  # System-wide oculus reader package
    ],
    zip_safe=False,  # Changed to False to ensure package files are accessible
    maintainer='Labelbox Robotics',
    maintainer_email='robotics@labelbox.com',
    description='ROS2 package to read and publish Meta Oculus Quest controller and tracking data.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'oculus_node = lbx_input_oculus.oculus_node:main',
        ],
    },
    cmdclass={
        'install': CustomInstallCommand,
    },
    # Note: Using system-wide oculus-reader package
    include_package_data=True,
) 