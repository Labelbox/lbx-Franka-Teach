from setuptools import find_packages, setup
from setuptools.command.install import install
import os
import shutil
from glob import glob

package_name = 'lbx_vision_camera'
# The camera_utilities submodule will remain as is internally for now

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
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'sensor_msgs',
        'geometry_msgs',
        'cv_bridge',
        'tf2_ros_py',
        'opencv-python',
        'PyYAML',
        'numpy',
        'transformations',
        'pyrealsense2', # Keep realsense for now, can be made optional later
        # Add ZED SDK python dep here if it becomes a direct pip installable item
        'diagnostic_updater',
    ],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='ROS2 package to manage and publish generic camera data (e.g., RealSense, ZED).',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'camera_node = lbx_vision_camera.camera_node:main', # Renamed executable and node file
        ],
    },
    cmdclass={
        'install': CustomInstallCommand,
    },
) 