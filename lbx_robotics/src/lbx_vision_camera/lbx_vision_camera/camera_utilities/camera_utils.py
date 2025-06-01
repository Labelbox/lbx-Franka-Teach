#!/usr/bin/env python3
"""
Camera Discovery Utilities for Labelbox Robotics System
Provides functions to enumerate and identify connected RealSense or ZED cameras.
"""

import subprocess # Still needed for ZED SDK tools if used for discovery
import re
from typing import List, Dict, Optional
from pathlib import Path
import logging

# Camera-specific imports
try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False

# ZED SDK Python API (if it becomes discoverable or needed for utils)
# For now, ZED discovery might rely on ZED SDK command-line tools if available
# or assume config specifies it directly.
ZED_AVAILABLE = False # Placeholder, update if ZED discovery is added here
try:
    import pyzed.sl as sl
    # Basic check to see if ZED SDK can list devices, otherwise discovery is hard
    # This is a simplified check; robust ZED discovery might need more.
    if hasattr(sl, 'Camera') and hasattr(sl.Camera, 'get_device_list'):
        ZED_AVAILABLE = True
except ImportError:
    pass 

logger = logging.getLogger(__name__)

def set_camera_utils_logger(node_logger):
    global logger
    logger = node_logger

def get_realsense_cameras() -> List[Dict[str, str]]:
    if not REALSENSE_AVAILABLE:
        logger.info("Intel RealSense SDK (pyrealsense2) not available. Cannot discover RealSense cameras.")
        return []
    cameras = []
    try:
        ctx = rs.context()
        devices = ctx.query_devices()
        if not devices:
            logger.info("No RealSense devices found by pyrealsense2 context.")
            return []
        for device in devices:
            info = {
                'serial_number': device.get_info(rs.camera_info.serial_number),
                'name': device.get_info(rs.camera_info.name),
                'firmware': device.get_info(rs.camera_info.firmware_version),
                'type': 'realsense' # Add type for easier processing
            }
            try: info['usb_type'] = device.get_info(rs.camera_info.usb_type_descriptor)
            except: info['usb_type'] = 'Unknown'
            try: info['physical_port'] = device.get_info(rs.camera_info.physical_port)
            except: info['physical_port'] = 'Unknown'
            cameras.append(info)
    except Exception as e:
        logger.error(f"Error enumerating RealSense cameras: {e}")
    return cameras

def get_zed_cameras() -> List[Dict[str, str]]:
    if not ZED_AVAILABLE:
        logger.info("ZED SDK (pyzed) not available or not discoverable. Cannot discover ZED cameras via SDK.")
        return []
    cameras = []
    try:
        device_list = sl.Camera.get_device_list()
        if not device_list:
            logger.info("No ZED devices found by pyzed SDK.")
            return []
        for i, dev in enumerate(device_list):
            cameras.append({
                'serial_number': str(dev.serial_number),
                'name': f"ZED {dev.camera_model.name}", # Accessing enum name
                'id': dev.id,
                'type': 'zed' # Add type for easier processing
            })
    except Exception as e:
        logger.error(f"Error enumerating ZED cameras: {e}")
    return cameras


def discover_all_cameras(node_logger_instance=None) -> Dict[str, List[Dict[str, str]]]:
    if node_logger_instance: set_camera_utils_logger(node_logger_instance)
    all_cameras_discovered = {}
    
    logger.info("🔍 Searching for Intel RealSense cameras...")
    realsense_cams = get_realsense_cameras()
    if realsense_cams:
        all_cameras_discovered['realsense'] = realsense_cams
    logger.info(f"   Found {len(realsense_cams)} RealSense camera(s).")
    
    logger.info("🔍 Searching for ZED cameras...")
    zed_cams = get_zed_cameras()
    if zed_cams:
        all_cameras_discovered['zed'] = zed_cams
    logger.info(f"   Found {len(zed_cams)} ZED camera(s).")
    
    return all_cameras_discovered

def print_camera_info(cameras: Dict[str, List[Dict[str, str]]]):
    logger.info("\n📷 DISCOVERED CAMERAS:")
    logger.info("=" * 60)
    if not cameras: logger.info("No RealSense or ZED cameras found!"); return
    
    for camera_type, camera_list in cameras.items():
        logger.info(f"\n{camera_type.upper()} Cameras ({len(camera_list)} found):")
        logger.info("-" * 30)
        for i, camera in enumerate(camera_list):
            log_str = f"  Camera {i + 1}:\n"
            log_str += f"    Name: {camera.get('name', 'N/A')}\n"
            log_str += f"    Serial: {camera.get('serial_number', 'N/A')}\n"
            if 'firmware' in camera: log_str += f"    Firmware: {camera['firmware']}\n"
            if 'usb_type' in camera: log_str += f"    USB Type: {camera['usb_type']}\n"
            if 'id' in camera: log_str += f"    ZED ID: {camera['id']}\n"
            logger.info(log_str.strip())

def generate_camera_config(cameras: Dict[str, List[Dict[str, str]]], 
                          config_dir: str = "configs/sensors") -> Optional[str]:
    """
    Generates a camera configuration file based on discovered cameras.
    If only one type of camera is found (RealSense or ZED), it generates a specific config for it.
    If multiple types or no cameras are found, it logs messages and might not generate a file.
    Returns the path to the generated file if one was created, otherwise None.
    """
    import yaml
    base_config_path = Path(config_dir)
    base_config_path.mkdir(parents=True, exist_ok=True)
    generated_config_path = None

    num_realsense = len(cameras.get('realsense', []))
    num_zed = len(cameras.get('zed', []))

    if num_realsense > 0 and num_zed == 0:
        # Only RealSense detected
        logger.info(f"Only RealSense cameras ({num_realsense}) detected. Generating realsense_cameras.yaml.")
        conf = {
            'global_settings': {
                'enable_sync': False, 'align_depth_to_color': True, 
                'test_settings': {'test_duration_sec': 2.0, 'min_depth_coverage_pct': 30.0}
            },
            'cameras': {}
        }
        for i, cam_data in enumerate(cameras['realsense']):
            cam_id = f"realsense_{cam_data['serial_number']}" # Use SN for unique ID
            conf['cameras'][cam_id] = {
                'enabled': True, 'type': 'realsense',
                'serial_number': cam_data['serial_number'],
                'device_id': cam_id, # For node/topic namespacing if desired
                'color': {'width': 640, 'height': 480, 'fps': 30, 'format': 'bgr8'},
                'depth': {'enabled': True, 'width': 640, 'height': 480, 'fps': 30},
                'transforms': { # Example static transform relative to base_link
                    'parent_frame': 'base_link', 
                    'camera_frame': f'{cam_id}_link', 
                    'optical_frame': f'{cam_id}_optical_frame',
                    'translation': [0.1, 0.0, 0.5], 'rotation_deg': [0,0,0]
                 },
                'description': cam_data['name']
            }
        generated_config_path = base_config_path / "realsense_cameras.yaml"
        with open(generated_config_path, 'w') as f: yaml.dump(conf, f, sort_keys=False)
        logger.info(f"✅ Generated RealSense config: {generated_config_path}")

    elif num_zed > 0 and num_realsense == 0:
        # Only ZED detected
        logger.info(f"Only ZED cameras ({num_zed}) detected. Generating zed_camera.yaml.")
        conf = {
            'global_settings': {
                'test_settings': {'test_duration_sec': 2.0, 'min_depth_coverage_pct': 30.0}
            },
            'cameras': {}
        }
        for i, cam_data in enumerate(cameras['zed']):
            cam_id = f"zed_{cam_data['serial_number']}"
            conf['cameras'][cam_id] = {
                'enabled': True, 'type': 'zed',
                'serial_number': cam_data['serial_number'],
                'device_id': cam_id,
                'color': {'resolution': 'HD720', 'fps': 30},
                'depth': {'enabled': True, 'mode': 'PERFORMANCE', 'min_distance_m': 0.3},
                'transforms': { 
                    'parent_frame': 'base_link', 
                    'camera_frame': f'{cam_id}_link', 
                    'optical_frame': f'{cam_id}_optical_frame',
                    'translation': [0.1, 0.0, 0.5], 'rotation_deg': [0,0,0]
                 },
                'description': cam_data['name']
            }
        generated_config_path = base_config_path / "zed_camera.yaml"
        with open(generated_config_path, 'w') as f: yaml.dump(conf, f, sort_keys=False)
        logger.info(f"✅ Generated ZED config: {generated_config_path}")
        
    elif num_realsense > 0 and num_zed > 0:
        logger.warning("Both RealSense and ZED cameras detected. Please create a specific configuration file manually or specify one at launch.")
        # Could optionally generate a combined file, but it might be confusing
        # generate_camera_config(cameras, output_path_str=str(base_config_path / "cameras_setup.yaml"), split_by_manufacturer=False)

    elif num_realsense == 0 and num_zed == 0:
        logger.warning("No RealSense or ZED cameras detected. No specific configuration file generated.")
        # Create a default empty cameras_setup.yaml if it doesn't exist, so node doesn't crash if this is the default
        default_setup_path = base_config_path / "cameras_setup.yaml"
        if not default_setup_path.exists():
            with open(default_setup_path, 'w') as f:
                yaml.dump({'global_settings': {}, 'cameras': {}}, f)
            logger.info(f"Created a default empty config: {default_setup_path}")
        generated_config_path = default_setup_path # Point to it so node can load something

    return str(generated_config_path) if generated_config_path else None


if __name__ == "__main__":
    import sys
    logger.setLevel(logging.INFO)
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(logging.Formatter('%(levelname)s: %(name)s - %(message)s'))
    logger.addHandler(handler)
    
    logger.info("🎥 Labelbox Robotics Camera Discovery Tool")
    logger.info("=" * 60)
    discovered_cams = discover_all_cameras(node_logger_instance=logger)
    print_camera_info(discovered_cams)
    if discovered_cams:
        logger.info("\n" + "=" * 60)
        response = input(f'Generate default camera configuration file(s) in {Path("configs/sensors").resolve()}/? (y/n): ')
        if response.lower() == 'y':
            generated_file = generate_camera_config(discovered_cams, config_dir=str(Path("configs/sensors")))
            if generated_file:
                logger.info(f"\n💡 Config file generated at: {generated_file}")
                logger.info("   Please review and edit this file to:")
                logger.info("   - Enable/disable specific cameras")
                logger.info("   - Adjust resolution, FPS, and transform settings")
                logger.info("   - Ensure serial numbers are correct if multiple cameras of the same type exist.")
            else:
                logger.info("No specific configuration file was generated (e.g., multiple camera types found or none).")
    else:
        logger.info("No supported cameras were discovered to generate a configuration for.") 