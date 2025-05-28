#!/usr/bin/env python3
"""
Camera Discovery Utilities for Labelbox Franka Teach System
Provides functions to enumerate and identify connected cameras
"""

import subprocess
import re
from typing import List, Dict, Optional
from pathlib import Path

# Camera-specific imports
try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


def get_realsense_cameras() -> List[Dict[str, str]]:
    """
    Enumerate all connected Intel RealSense cameras
    
    Returns:
        List of dictionaries containing camera info:
        - serial_number: Camera serial number
        - name: Camera model name
        - firmware: Firmware version
        - usb_port: USB port location (if available)
        - physical_port: Physical port path
    """
    if not REALSENSE_AVAILABLE:
        print("⚠️  Intel RealSense SDK not available")
        return []
    
    cameras = []
    
    try:
        ctx = rs.context()
        devices = ctx.query_devices()
        
        for i, device in enumerate(devices):
            camera_info = {
                'serial_number': device.get_info(rs.camera_info.serial_number),
                'name': device.get_info(rs.camera_info.name),
                'firmware': device.get_info(rs.camera_info.firmware_version),
            }
            
            # Try to get USB port information
            try:
                camera_info['usb_type'] = device.get_info(rs.camera_info.usb_type_descriptor)
            except:
                camera_info['usb_type'] = 'Unknown'
                
            # Try to get physical port
            try:
                camera_info['physical_port'] = device.get_info(rs.camera_info.physical_port)
            except:
                camera_info['physical_port'] = 'Unknown'
                
            # Get USB port from system if possible
            usb_port = get_usb_port_for_device(camera_info['serial_number'])
            if usb_port:
                camera_info['usb_port'] = usb_port
            else:
                # Try to extract from physical port
                if camera_info['physical_port'] != 'Unknown':
                    # Extract USB port from path like /sys/devices/pci0000:00/0000:00:0d.0/usb2/2-3/2-3.3/...
                    # We want to extract "2-3.3" from this path
                    port_match = re.search(r'/usb\d+/\d+-\d+/([\d-]+\.[\d.]+)', camera_info['physical_port'])
                    if port_match:
                        camera_info['usb_port'] = port_match.group(1)
                
            cameras.append(camera_info)
            
    except Exception as e:
        print(f"❌ Error enumerating RealSense cameras: {e}")
        
    return cameras


def get_usb_cameras() -> List[Dict[str, str]]:
    """
    Enumerate all USB cameras (including webcams)
    
    Returns:
        List of dictionaries containing camera info:
        - device_id: OpenCV device ID (0, 1, 2, etc.)
        - device_path: /dev/video* path
        - name: Camera name from v4l2
        - usb_port: USB port location
    """
    cameras = []
    
    # Method 1: Try OpenCV enumeration with suppressed warnings
    if CV2_AVAILABLE:
        # Suppress OpenCV warnings temporarily
        import os
        old_opencv_log = os.environ.get('OPENCV_LOG_LEVEL', '')
        os.environ['OPENCV_LOG_LEVEL'] = 'FATAL'
        
        try:
            for device_id in range(20):  # Check first 20 devices
                try:
                    cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
                    if cap.isOpened():
                        # Test if we can actually read a frame
                        ret, _ = cap.read()
                        if ret:
                            # Get device path
                            device_path = f"/dev/video{device_id}"
                            
                            camera_info = {
                                'device_id': device_id,
                                'device_path': device_path,
                                'name': 'Unknown USB Camera',
                                'driver': 'Unknown'
                            }
                            
                            # Try to get more info using v4l2
                            v4l2_info = get_v4l2_info(device_path)
                            if v4l2_info:
                                camera_info.update(v4l2_info)
                                
                            cameras.append(camera_info)
                        cap.release()
                except:
                    # Skip devices that can't be opened
                    pass
        finally:
            # Restore OpenCV log level
            os.environ['OPENCV_LOG_LEVEL'] = old_opencv_log

    # Method 2: Parse /dev/video* devices directly
    try:
        video_devices = list(Path('/dev').glob('video*'))
        
        for device_path in video_devices:
            device_id = int(device_path.name.replace('video', ''))
            
            # Skip if already found via OpenCV
            if any(cam['device_id'] == device_id for cam in cameras):
                continue
                
            v4l2_info = get_v4l2_info(str(device_path))
            if v4l2_info:
                camera_info = {
                    'device_id': device_id,
                    'device_path': str(device_path),
                    'name': v4l2_info.get('name', 'Unknown'),
                    'driver': v4l2_info.get('driver', 'Unknown')
                }
                camera_info.update(v4l2_info)
                cameras.append(camera_info)
                
    except Exception as e:
        print(f"⚠️  Error scanning /dev/video devices: {e}")
        
    return cameras


def get_v4l2_info(device_path: str) -> Optional[Dict[str, str]]:
    """
    Get detailed information about a V4L2 device using v4l2-ctl
    
    Args:
        device_path: Path to device (e.g., /dev/video0)
        
    Returns:
        Dictionary with device information or None if failed
    """
    try:
        # Run v4l2-ctl to get device info
        result = subprocess.run(
            ['v4l2-ctl', '--device', device_path, '--info'],
            capture_output=True,
            text=True,
            timeout=2
        )
        
        if result.returncode != 0:
            return None
            
        info = {}
        output = result.stdout
        
        # Parse driver info
        driver_match = re.search(r'Driver name\s*:\s*(.+)', output)
        if driver_match:
            info['driver'] = driver_match.group(1).strip()
            
        # Parse card/device name
        card_match = re.search(r'Card type\s*:\s*(.+)', output)
        if card_match:
            info['name'] = card_match.group(1).strip()
            
        # Parse bus info (contains USB port)
        bus_match = re.search(r'Bus info\s*:\s*(.+)', output)
        if bus_match:
            bus_info = bus_match.group(1).strip()
            info['bus_info'] = bus_info
            
            # Extract USB port if present
            # Try different patterns for USB port extraction
            # Pattern 1: usb-0000:00:14.0-4 -> extract "4"
            # Pattern 2: usb-0000:00:0d.0-3.3 -> extract "3.3"
            usb_match = re.search(r'usb-[0-9a-f:.]+-([0-9.]+)', bus_info)
            if usb_match:
                info['usb_port'] = usb_match.group(1)
            else:
                # Try alternate pattern
                usb_match = re.search(r'usb-.*-([0-9]+(?:\.[0-9]+)*)', bus_info)
                if usb_match:
                    info['usb_port'] = usb_match.group(1)
                
        return info
        
    except subprocess.TimeoutExpired:
        return None
    except FileNotFoundError:
        # v4l2-ctl not installed
        return None
    except Exception:
        return None


def get_usb_port_for_device(serial_number: str) -> Optional[str]:
    """
    Get USB port for a device by serial number using system information
    
    Args:
        serial_number: Device serial number
        
    Returns:
        USB port string (e.g., "1-2.3") or None
    """
    try:
        # Search in /sys/bus/usb/devices/
        usb_devices = Path('/sys/bus/usb/devices/').glob('*')
        
        for device in usb_devices:
            # Check if this is a USB device (not a hub or controller)
            if not re.match(r'^\d+-\d+', device.name):
                continue
                
            # Try to read serial number
            serial_file = device / 'serial'
            if serial_file.exists():
                try:
                    with open(serial_file, 'r') as f:
                        device_serial = f.read().strip()
                        
                    if device_serial == serial_number:
                        return device.name
                except:
                    continue
                    
    except Exception:
        pass
        
    return None


def discover_all_cameras(quiet=True) -> Dict[str, List[Dict[str, str]]]:
    """
    Discover all connected cameras of all supported types
    
    Args:
        quiet: If True, suppress OpenCV warnings
    
    Returns:
        Dictionary with camera type as key and list of cameras as value
    """
    all_cameras = {}
    
    # Temporarily suppress warnings if requested
    if quiet:
        import os
        old_opencv_log = os.environ.get('OPENCV_LOG_LEVEL', '')
        os.environ['OPENCV_LOG_LEVEL'] = 'FATAL'

    # Discover RealSense cameras
    print("🔍 Searching for Intel RealSense cameras...")
    realsense_cameras = get_realsense_cameras()
    if realsense_cameras:
        all_cameras['realsense'] = realsense_cameras
        print(f"   Found {len(realsense_cameras)} RealSense camera(s)")
    else:
        print("   No RealSense cameras found")
        
    # Discover USB cameras
    print("🔍 Searching for USB cameras...")
    usb_cameras = get_usb_cameras()
    if usb_cameras:
        all_cameras['usb'] = usb_cameras
        print(f"   Found {len(usb_cameras)} USB camera(s)")
    else:
        print("   No USB cameras found")
        
    # Restore OpenCV log level if we changed it
    if quiet:
        os.environ['OPENCV_LOG_LEVEL'] = old_opencv_log
        
    return all_cameras


def print_camera_info(cameras: Dict[str, List[Dict[str, str]]]):
    """Pretty print discovered camera information"""
    
    print("\n📷 DISCOVERED CAMERAS:")
    print("=" * 60)
    
    if not cameras:
        print("No cameras found!")
        return
        
    for camera_type, camera_list in cameras.items():
        print(f"\n{camera_type.upper()} Cameras:")
        print("-" * 40)
        
        for i, camera in enumerate(camera_list):
            print(f"\nCamera {i + 1}:")
            
            if camera_type == 'realsense':
                print(f"  Model: {camera['name']}")
                print(f"  Serial: {camera['serial_number']}")
                print(f"  Firmware: {camera['firmware']}")
                if 'usb_port' in camera:
                    print(f"  USB Port: {camera['usb_port']}")
                if camera['physical_port'] != 'Unknown':
                    print(f"  Physical Port: {camera['physical_port']}")
                    
            elif camera_type == 'usb':
                print(f"  Name: {camera['name']}")
                print(f"  Device: {camera['device_path']} (ID: {camera['device_id']})")
                print(f"  Driver: {camera['driver']}")
                if 'usb_port' in camera:
                    print(f"  USB Port: {camera['usb_port']}")
                if 'bus_info' in camera:
                    print(f"  Bus Info: {camera['bus_info']}")


def generate_camera_config(cameras: Dict[str, List[Dict[str, str]]], 
                          output_path: str = None,
                          split_by_manufacturer: bool = True):
    """
    Generate camera configuration files from discovered cameras
    
    Args:
        cameras: Dictionary of discovered cameras
        output_path: Path to save the configuration file (used only if split_by_manufacturer is False)
        split_by_manufacturer: If True, create separate config files for each manufacturer
    """
    import yaml
    from pathlib import Path
    
    if split_by_manufacturer:
        # Create separate configs for each manufacturer
        configs_created = []
        
        # Intel RealSense cameras
        if 'realsense' in cameras and cameras['realsense']:
            intel_config = {
                'manufacturer': 'Intel RealSense',
                'cameras': {},
                'camera_settings': {
                    'jpeg_quality': 90,
                    'depth_encoding': '16UC1',
                    'depth_scale': 0.001,
                    'enable_threading': True,
                    'buffer_size': 5,
                    'enable_sync': False,
                    'sync_tolerance_ms': 10,
                    # RealSense-specific settings
                    'realsense_defaults': {
                        'depth_processing_preset': 1,  # 1=High Accuracy, 2=High Density, 3=Medium
                        'align_depth_to_color': True,
                        'decimation_filter': 2,
                        'spatial_filter': True,
                        'temporal_filter': True,
                        'hole_filling_filter': 1,  # 0=disabled, 1=2-pixel, 2=4-pixel, 3=8-pixel, 4=16-pixel, 5=unlimited
                        'enable_auto_exposure': True,
                        'auto_exposure_priority': False  # False = maintain framerate
                    }
                }
            }

            # Add each RealSense camera
            for i, camera in enumerate(cameras['realsense']):
                camera_id = f"realsense_{i}"
                intel_config['cameras'][camera_id] = {
                    'enabled': True,
                    'type': 'realsense',
                    'serial_number': camera['serial_number'],
                    'port_offset': i,
                    'config': {
                        'width': 640,
                        'height': 480,
                        'fps': 30,
                        'enable_depth': True,
                        # These will use defaults from camera_settings if not overridden
                        'depth_processing_preset': None,  # Use default
                        'align_depth_to_color': None,     # Use default
                    },
                    'description': f"{camera['name']} (Serial: {camera['serial_number']})",
                }
                
                # Add USB port info if available
                if 'usb_port' in camera:
                    intel_config['cameras'][camera_id]['usb_port_info'] = camera['usb_port']
            
            # Save Intel config
            intel_path = Path("configs/cameras_intel.yaml")
            intel_path.parent.mkdir(parents=True, exist_ok=True)
            
            with open(intel_path, 'w') as f:
                yaml.dump(intel_config, f, default_flow_style=False, sort_keys=False)
            
            configs_created.append(str(intel_path))
            print(f"✅ Created Intel RealSense config: {intel_path}")
            print(f"   Configured {len(cameras['realsense'])} RealSense camera(s)")
        
        # Generic USB/Webcam cameras
        usb_cameras = []
        for cam in cameras.get('usb', []):
            # Skip RealSense cameras detected as USB
            if 'realsense' not in cam['name'].lower() and 'intel(r) realsense' not in cam['name'].lower():
                usb_cameras.append(cam)
        
        if usb_cameras:
            usb_config = {
                'manufacturer': 'Generic USB Cameras',
                'cameras': {},
                'camera_settings': {
                    'jpeg_quality': 90,
                    'enable_threading': True,
                    'buffer_size': 5,
                    'enable_sync': False,
                    'sync_tolerance_ms': 10,
                    # USB camera defaults
                    'usb_defaults': {
                        'auto_exposure': True,
                        'auto_white_balance': True,
                        'brightness': 128,  # 0-255
                        'contrast': 128,    # 0-255
                        'saturation': 128,  # 0-255
                        'gain': 0,          # 0-255
                    }
                }
            }

            # Add USB cameras
            cam_count = 0
            for cam in usb_cameras:
                # Categorize by type
                if 'integrated' in cam['name'].lower():
                    camera_id = f"integrated_{cam_count}"
                    enabled = False  # Laptop webcams disabled by default
                else:
                    camera_id = f"usb_{cam_count}"
                    enabled = False  # Still disabled by default for safety
                
                usb_config['cameras'][camera_id] = {
                    'enabled': enabled,
                    'type': 'usb',
                    'device_id': cam['device_id'],
                    'device_path': cam.get('device_path', f"/dev/video{cam['device_id']}"),
                    'port_offset': cam_count,
                    'config': {
                        'width': 640,
                        'height': 480,
                        'fps': 30,
                        'fourcc': 'MJPG',  # Motion JPEG for better performance
                    },
                    'description': cam['name']
                }
                
                # Add USB port info if available
                if 'usb_port' in cam:
                    usb_config['cameras'][camera_id]['usb_port_info'] = cam['usb_port']
                
                cam_count += 1
            
            # Save USB config
            usb_path = Path("configs/cameras_usb.yaml")
            usb_path.parent.mkdir(parents=True, exist_ok=True)
            
            with open(usb_path, 'w') as f:
                yaml.dump(usb_config, f, default_flow_style=False, sort_keys=False)
            
            configs_created.append(str(usb_path))
            print(f"✅ Created USB camera config: {usb_path}")
            print(f"   Configured {len(usb_cameras)} USB camera(s)")
        
        # Future: ZED cameras would go here
        # if 'zed' in cameras and cameras['zed']:
        #     zed_config = {...}
        
        return configs_created
    
    else:
        # Original single-file implementation (backward compatibility)
        if output_path is None:
            output_path = "configs/cameras_discovered.yaml"
            
        config = {
            'cameras': {},
            'camera_settings': {
                'jpeg_quality': 90,
                'depth_encoding': '16UC1',
                'depth_scale': 0.001,
                'enable_threading': True,
                'buffer_size': 5,
                'enable_sync': False,
                'sync_tolerance_ms': 10
            }
        }
        
        # Add all cameras to single config
        camera_count = 0
        
        # Add RealSense cameras
        for i, camera in enumerate(cameras.get('realsense', [])):
            camera_id = f"realsense_{i}"
            config['cameras'][camera_id] = {
                'enabled': True,
                'type': 'realsense',
                'serial_number': camera['serial_number'],
                'port_offset': camera_count,
                'config': {
                    'width': 640,
                    'height': 480,
                    'fps': 30,
                    'enable_depth': True,
                },
                'description': f"{camera['name']} (Serial: {camera['serial_number']})"
            }
            camera_count += 1
            
        # Save single config
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        with open(output_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False, sort_keys=False)
            
        print(f"\n✅ Generated camera configuration: {output_path}")
        print(f"   Total cameras configured: {camera_count}")
        
        return [str(output_path)]


if __name__ == "__main__":
    """Run camera discovery when executed directly"""
    
    print("🎥 Labelbox Franka Teach Camera Discovery Tool")
    print("=" * 60)
    
    # Discover all cameras
    cameras = discover_all_cameras()
    
    # Print information
    print_camera_info(cameras)
    
    # Ask if user wants to generate config
    if cameras:
        print("\n" + "=" * 60)
        response = input("\nGenerate camera configuration file? (y/n): ")
        
        if response.lower() == 'y':
            generate_camera_config(cameras)
            print("\n💡 Edit the generated file to:")
            print("   - Enable/disable specific cameras")
            print("   - Adjust resolution and FPS settings")
            print("   - Update serial numbers if needed") 