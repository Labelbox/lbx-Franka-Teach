from setuptools import setup, find_packages
from setuptools.command.install import install
import os
import shutil
from glob import glob

package_name = 'lbx_franka_control'

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
    version='0.1.0',
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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='labelbox',
    maintainer_email='robotics@labelbox.com',
    description='VR-based Franka control system using MoveIt',
    license='MIT',
    entry_points={
        'console_scripts': [
            'system_manager = lbx_franka_control.system_manager:main',
            'main_system = lbx_franka_control.main_system:main',
            'system_monitor = lbx_franka_control.system_monitor:main',
        ],
    },
    cmdclass={
        'install': CustomInstallCommand,
    },
) 