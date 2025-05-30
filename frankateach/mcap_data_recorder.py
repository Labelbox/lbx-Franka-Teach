#!/usr/bin/env python3
"""
MCAP Data Recorder for Labelbox Robotics Franka Teach System
Records teleoperation data in Labelbox Robotics MCAP format

Features:
- Records robot states, actions, and VR controller data
- Supports multiple camera streams (Intel RealSense/ZED)
- Compatible with Foxglove Studio visualization
- Automatic success/failure categorization
- Thread-safe recording with queues
- Labelbox Robotics schema format for data compatibility
"""

import os
import time
import json
import base64
import threading
import numpy as np
from pathlib import Path
from datetime import datetime
from queue import Queue, Empty
from typing import Dict, Optional, Any
import cv2

import mcap
from mcap.writer import Writer

from frankateach.utils import notify_component_start
from frankateach.constants import HOST, CAM_PORT, DEPTH_PORT_OFFSET
from frankateach.network import ZMQCameraSubscriber


class MCAPDataRecorder:
    """Records teleoperation data in MCAP format compatible with Labelbox Robotics"""
    
    def __init__(self, 
                 base_dir: str = None,
                 demo_name: str = None,
                 save_images: bool = True,
                 save_depth: bool = False,
                 camera_configs: Dict = None,
                 camera_manager=None):
        """
        Initialize MCAP data recorder
        
        Args:
            base_dir: Base directory for recordings (default: ~/recordings)
            demo_name: Name for this demonstration
            save_images: Whether to save camera images
            save_depth: Whether to save depth images
            camera_configs: Camera configuration dictionary (deprecated, use camera_manager)
            camera_manager: CameraManager instance for handling cameras
        """
        # Set up directories
        if base_dir is None:
            base_dir = Path.home() / "recordings"
        self.base_dir = Path(base_dir)
        self.base_dir.mkdir(parents=True, exist_ok=True)
        
        # Create success/failure directories (no date subfolders)
        self.success_dir = self.base_dir / "success"
        self.failure_dir = self.base_dir / "failure"
        self.success_dir.mkdir(parents=True, exist_ok=True)
        self.failure_dir.mkdir(parents=True, exist_ok=True)
        
        # Recording state
        self.recording = False
        self.demo_name = demo_name or datetime.now().strftime("%Y%m%d_%H%M%S")
        self.current_dir = None
        self.filepath = None
        self.start_timestamp = None
        self._has_written_data = False  # Track if we've written actual data
        
        # Configuration
        self.save_images = save_images
        self.save_depth = save_depth
        self.camera_configs = camera_configs or {}
        self.camera_manager = camera_manager
        
        # MCAP components
        self._writer = None
        self._mcap_file = None
        self._channels = {}
        self._schemas = {}
        
        # Data queues
        self._data_queue = Queue()
        self._writer_thread = None
        self._running = False
        
        # Camera recording thread
        self._camera_thread = None
        self._camera_recording = False
        
        # Recording metadata
        self.metadata = {
            "robot_type": "franka",
            "recording_software": "frankateach",
            "mcap_version": "1.0",
        }
        
    def _register_schemas(self):
        """Register all MCAP schemas for Labelbox Robotics data format"""
        
        # Robot state schema
        self._schemas["robot_state"] = self._writer.register_schema(
            name="labelbox_robotics.RobotState",
            encoding="jsonschema",
            data=json.dumps({
                "type": "object",
                "properties": {
                    "timestamp": {
                        "type": "object",
                        "properties": {
                            "sec": {"type": "integer"},
                            "nanosec": {"type": "integer"}
                        }
                    },
                    "joint_positions": {
                        "type": "array",
                        "items": {"type": "number"}
                    },
                    "joint_velocities": {
                        "type": "array",
                        "items": {"type": "number"}
                    },
                    "joint_efforts": {
                        "type": "array",
                        "items": {"type": "number"}
                    },
                    "cartesian_position": {
                        "type": "array",
                        "items": {"type": "number"}
                    },
                    "cartesian_velocity": {
                        "type": "array",
                        "items": {"type": "number"}
                    },
                    "gripper_position": {"type": "number"},
                    "gripper_velocity": {"type": "number"}
                }
            }).encode("utf-8")
        )
        
        # ROS2-style JointState schema for visualization
        # Using JSON encoding for simplicity and compatibility
        self._schemas["joint_state"] = self._writer.register_schema(
            name="sensor_msgs/msg/JointState",
            encoding="jsonschema",
            data=json.dumps({
                "type": "object",
                "properties": {
                    "header": {
                        "type": "object",
                        "properties": {
                            "stamp": {
                                "type": "object",
                                "properties": {
                                    "sec": {"type": "integer"},
                                    "nanosec": {"type": "integer"}
                                }
                            },
                            "frame_id": {"type": "string"}
                        }
                    },
                    "name": {
                        "type": "array",
                        "items": {"type": "string"}
                    },
                    "position": {
                        "type": "array",
                        "items": {"type": "number"}
                    },
                    "velocity": {
                        "type": "array",
                        "items": {"type": "number"}
                    },
                    "effort": {
                        "type": "array",
                        "items": {"type": "number"}
                    }
                }
            }).encode("utf-8")
        )
        
        # Action schema
        self._schemas["action"] = self._writer.register_schema(
            name="labelbox_robotics.Action",
            encoding="jsonschema",
            data=json.dumps({
                "type": "object",
                "properties": {
                    "timestamp": {
                        "type": "object",
                        "properties": {
                            "sec": {"type": "integer"},
                            "nanosec": {"type": "integer"}
                        }
                    },
                    "data": {
                        "type": "array",
                        "items": {"type": "number"}
                    }
                }
            }).encode("utf-8")
        )
        
        # VR Controller schema
        self._schemas["vr_controller"] = self._writer.register_schema(
            name="labelbox_robotics.VRController",
            encoding="jsonschema",
            data=json.dumps({
                "type": "object",
                "properties": {
                    "timestamp": {
                        "type": "object",
                        "properties": {
                            "sec": {"type": "integer"},
                            "nanosec": {"type": "integer"}
                        }
                    },
                    "poses": {
                        "type": "object",
                        "additionalProperties": {
                            "type": "array",
                            "items": {"type": "number"}
                        }
                    },
                    "buttons": {
                        "type": "object",
                        "additionalProperties": {"type": ["boolean", "array", "number"]}
                    },
                    "movement_enabled": {"type": "boolean"},
                    "controller_on": {"type": "boolean"},
                    "success": {"type": "boolean"},
                    "failure": {"type": "boolean"}
                }
            }).encode("utf-8")
        )
        
        # Compressed image schema (keeping foxglove standard)
        self._schemas["compressed_image"] = self._writer.register_schema(
            name="foxglove.CompressedImage",
            encoding="jsonschema",
            data=json.dumps({
                "type": "object",
                "properties": {
                    "timestamp": {
                        "type": "object",
                        "properties": {
                            "sec": {"type": "integer"},
                            "nanosec": {"type": "integer"}
                        }
                    },
                    "frame_id": {"type": "string"},
                    "data": {"type": "string", "contentEncoding": "base64"},
                    "format": {"type": "string"}
                }
            }).encode("utf-8")
        )
        
        # Raw image schema for depth data (proper Foxglove RawImage)
        self._schemas["raw_image"] = self._writer.register_schema(
            name="foxglove.RawImage",
            encoding="jsonschema",
            data=json.dumps({
                "type": "object",
                "properties": {
                    "timestamp": {
                        "type": "object",
                        "properties": {
                            "sec": {"type": "integer"},
                            "nanosec": {"type": "integer"}
                        }
                    },
                    "frame_id": {"type": "string"},
                    "width": {"type": "integer"},
                    "height": {"type": "integer"},
                    "encoding": {"type": "string"},
                    "step": {"type": "integer"},
                    "data": {"type": "string", "contentEncoding": "base64"}
                }
            }).encode("utf-8")
        )
        
        # Camera calibration schema (Foxglove standard)
        self._schemas["camera_calibration"] = self._writer.register_schema(
            name="foxglove.CameraCalibration",
            encoding="jsonschema",
            data=json.dumps({
                "type": "object",
                "properties": {
                    "timestamp": {
                        "type": "object",
                        "properties": {
                            "sec": {"type": "integer"},
                            "nanosec": {"type": "integer"}
                        }
                    },
                    "frame_id": {"type": "string"},
                    "width": {"type": "integer"},
                    "height": {"type": "integer"},
                    "distortion_model": {"type": "string"},
                    "D": {
                        "type": "array",
                        "items": {"type": "number"}
                    },
                    "K": {
                        "type": "array",
                        "items": {"type": "number"},
                        "minItems": 9,
                        "maxItems": 9
                    },
                    "R": {
                        "type": "array",
                        "items": {"type": "number"},
                        "minItems": 9,
                        "maxItems": 9
                    },
                    "P": {
                        "type": "array",
                        "items": {"type": "number"},
                        "minItems": 12,
                        "maxItems": 12
                    }
                }
            }).encode("utf-8")
        )
        
        # Transform schema for TF messages
        self._schemas["transform"] = self._writer.register_schema(
            name="tf2_msgs/msg/TFMessage",
            encoding="jsonschema",
            data=json.dumps({
                "type": "object",
                "properties": {
                    "transforms": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "header": {
                                    "type": "object",
                                    "properties": {
                                        "stamp": {
                                            "type": "object",
                                            "properties": {
                                                "sec": {"type": "integer"},
                                                "nanosec": {"type": "integer"}
                                            }
                                        },
                                        "frame_id": {"type": "string"}
                                    }
                                },
                                "child_frame_id": {"type": "string"},
                                "transform": {
                                    "type": "object",
                                    "properties": {
                                        "translation": {
                                            "type": "object",
                                            "properties": {
                                                "x": {"type": "number"},
                                                "y": {"type": "number"},
                                                "z": {"type": "number"}
                                            }
                                        },
                                        "rotation": {
                                            "type": "object",
                                            "properties": {
                                                "x": {"type": "number"},
                                                "y": {"type": "number"},
                                                "z": {"type": "number"},
                                                "w": {"type": "number"}
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }).encode("utf-8")
        )
        
    def start_recording(self, demo_name: str = None):
        """Start a new recording session"""
        if self.recording:
            print("⚠️  Recording already in progress!")
            return False
            
        # Generate timestamp-based filename with milliseconds to avoid collisions
        self.start_timestamp = datetime.now()
        timestamp_str = self.start_timestamp.strftime("%Y%m%d_%H%M%S_%f")[:-3]  # Include milliseconds
        
        # Create MCAP file directly in failure folder (will move on success)
        # Filename format: trajectory_YYYYMMDD_HHMMSS_mmm.mcap (duration added later)
        self.filepath = self.failure_dir / f"trajectory_{timestamp_str}.mcap"
        self._mcap_file = open(self.filepath, "wb")
        self._writer = Writer(self._mcap_file)
        self._writer.start("labelbox_robotics", library="frankateach")
        
        # Register schemas
        self._register_schemas()
        
        # Add initial metadata
        self.metadata["start_time"] = time.time()
        self.metadata["start_timestamp"] = timestamp_str  # This now includes milliseconds
        # MCAP writer expects name and data as positional arguments with string values
        metadata_str = {k: str(v) for k, v in self.metadata.items()}
        self._writer.add_metadata("recording_metadata", metadata_str)
        
        # Add robot model for visualization
        self._add_robot_model()
        
        # Also publish URDF as a topic for Foxglove compatibility
        self._publish_robot_description()
        
        # Write initial transforms
        self._write_initial_transforms()
        
        # Start writer thread
        self._running = True
        self._writer_thread = threading.Thread(target=self._write_worker, daemon=True)
        self._writer_thread.start()
        
        # Set recording flag before starting camera thread to avoid race condition
        self.recording = True
        
        # Initialize camera recording thread
        if self.save_images:
            self._init_camera_recording()
            
        notify_component_start("MCAP Data Recorder")
        print(f"📹 Started recording: {timestamp_str}")
        print(f"   Saving to: {self.filepath}")
        
        return True
        
    def _add_robot_model(self):
        """Add robot URDF model to MCAP for visualization"""
        # Add robot description as metadata
        robot_info = {
            "robot_type": "franka_fr3",
            "urdf_package": "franka_description",
            "urdf_path": "robots/fr3.urdf.xacro",
            "joint_names": [
                "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
                "fr3_joint5", "fr3_joint6", "fr3_joint7",
                "fr3_finger_joint1", "fr3_finger_joint2"
            ],
            "link_names": [
                "fr3_link0", "fr3_link1", "fr3_link2", "fr3_link3",
                "fr3_link4", "fr3_link5", "fr3_link6", "fr3_link7",
                "fr3_hand", "fr3_leftfinger", "fr3_rightfinger"
            ]
        }
        
        # Add as metadata
        robot_info_str = {k: str(v) for k, v in robot_info.items()}
        self._writer.add_metadata("robot_model", robot_info_str)
        
        # Get the FR3 URDF content
        fr3_urdf = self._get_fr3_urdf()
        
        # Write URDF as an attachment
        self._writer.add_attachment(
            create_time=0,  # Attachment created at start of recording
            log_time=0,
            name="robot_description",
            media_type="application/xml",
            data=fr3_urdf.encode("utf-8")
        )
        
    def _publish_robot_description(self):
        """Publish robot URDF as a topic message for Foxglove compatibility"""
        # Register schema for robot description
        if "robot_description" not in self._schemas:
            self._schemas["robot_description"] = self._writer.register_schema(
                name="std_msgs/msg/String",
                encoding="jsonschema",
                data=json.dumps({
                    "type": "object",
                    "properties": {
                        "data": {"type": "string"}
                    }
                }).encode("utf-8")
            )
        
        # Register channel
        if "robot_description" not in self._channels:
            self._channels["robot_description"] = self._writer.register_channel(
                topic="/robot_description",
                message_encoding="json",
                schema_id=self._schemas["robot_description"]
            )
        
        # Get the URDF content (same as in _add_robot_model)
        fr3_urdf = self._get_fr3_urdf()
        
        # Write the URDF as a message
        msg = {"data": fr3_urdf}
        
        # Use actual start time instead of 0
        start_time_ns = int(self.metadata["start_time"] * 1e9)
        
        self._writer.add_message(
            channel_id=self._channels["robot_description"],
            sequence=0,
            log_time=start_time_ns,
            publish_time=start_time_ns,
            data=json.dumps(msg).encode("utf-8")
        )
        
    def _get_fr3_urdf(self):
        """Get the FR3 URDF content - use modified URDF with snug hand fit"""
        # Use the modified URDF with hand offset from robot_urdf_models
        urdf_path = Path(__file__).parent.parent / "robot_urdf_models" / "fr3_franka_hand_snug.urdf"
        
        if urdf_path.exists():
            with open(urdf_path, 'r') as f:
                urdf_content = f.read()
            
            # Replace package:// references with GitHub raw URLs for web accessibility
            github_base_url = "https://raw.githubusercontent.com/frankaemika/franka_description/refs/heads/main"
            urdf_content = urdf_content.replace(
                "package://franka_description", 
                github_base_url
            )
            
            print(f"✅ Using modified FR3 URDF with snug hand fit (150mm closer)")
            print(f"   URDF path: {urdf_path.relative_to(Path(__file__).parent.parent)}")
            print(f"   Meshes will be loaded from: {github_base_url}")
            return urdf_content
        else:
            # Fallback to simplified URDF
            print(f"⚠️  Could not find modified URDF at {urdf_path}, using simplified version")
            return self._get_simplified_fr3_urdf()
    
    def _get_simplified_fr3_urdf(self):
        """Get simplified FR3 URDF as fallback"""
        return """<?xml version="1.0" ?>
<robot name="fr3">
  <!-- Base Link -->
  <link name="world"/>
  
  <link name="fr3_link0">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="white">
        <color rgba="0.9 0.9 0.9 1"/>
      </material>
    </visual>
  </link>
  
  <joint name="world_joint" type="fixed">
    <parent link="world"/>
    <child link="fr3_link0"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  
  <!-- Joint 1 -->
  <joint name="fr3_joint1" type="revolute">
    <parent link="fr3_link0"/>
    <child link="fr3_link1"/>
    <origin xyz="0 0 0.333" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2.8973" upper="2.8973" effort="87" velocity="2.175"/>
  </joint>
  
  <link name="fr3_link1">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.05"/>
      </geometry>
      <material name="white"/>
    </visual>
  </link>
  
  <!-- Joint 2 -->
  <joint name="fr3_joint2" type="revolute">
    <parent link="fr3_link1"/>
    <child link="fr3_link2"/>
    <origin xyz="0 0 0" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.7628" upper="1.7628" effort="87" velocity="2.175"/>
  </joint>
  
  <link name="fr3_link2">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.05"/>
      </geometry>
      <material name="white"/>
    </visual>
  </link>
  
  <!-- Joint 3 -->
  <joint name="fr3_joint3" type="revolute">
    <parent link="fr3_link2"/>
    <child link="fr3_link3"/>
    <origin xyz="0 -0.316 0" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2.8973" upper="2.8973" effort="87" velocity="2.175"/>
  </joint>
  
  <link name="fr3_link3">
    <visual>
      <geometry>
        <cylinder length="0.15" radius="0.05"/>
      </geometry>
      <material name="white"/>
    </visual>
  </link>
  
  <!-- Joint 4 -->
  <joint name="fr3_joint4" type="revolute">
    <parent link="fr3_link3"/>
    <child link="fr3_link4"/>
    <origin xyz="0.0825 0 0" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.0718" upper="-0.0698" effort="87" velocity="2.175"/>
  </joint>
  
  <link name="fr3_link4">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.05"/>
      </geometry>
      <material name="white"/>
    </visual>
  </link>
  
  <!-- Joint 5 -->
  <joint name="fr3_joint5" type="revolute">
    <parent link="fr3_link4"/>
    <child link="fr3_link5"/>
    <origin xyz="-0.0825 0.384 0" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2.8973" upper="2.8973" effort="12" velocity="2.61"/>
  </joint>
  
  <link name="fr3_link5">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="white"/>
    </visual>
  </link>
  
  <!-- Joint 6 -->
  <joint name="fr3_joint6" type="revolute">
    <parent link="fr3_link5"/>
    <child link="fr3_link6"/>
    <origin xyz="0 0 0" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.0175" upper="3.7525" effort="12" velocity="2.61"/>
  </joint>
  
  <link name="fr3_link6">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.04"/>
      </geometry>
      <material name="white"/>
    </visual>
  </link>
  
  <!-- Joint 7 -->
  <joint name="fr3_joint7" type="revolute">
    <parent link="fr3_link6"/>
    <child link="fr3_link7"/>
    <origin xyz="0.088 0 0" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2.8973" upper="2.8973" effort="12" velocity="2.61"/>
  </joint>
  
  <link name="fr3_link7">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.04"/>
      </geometry>
      <material name="white"/>
    </visual>
  </link>
  
  <!-- Hand - connected directly to link7, no link8 -->
  <joint name="fr3_hand_joint" type="fixed">
    <parent link="fr3_link7"/>
    <child link="fr3_hand"/>
    <origin xyz="0 0 0.107" rpy="0 0 0"/>
  </joint>
  
  <link name="fr3_hand">
    <visual>
      <geometry>
        <box size="0.08 0.08 0.05"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
  </link>
  
  <!-- Gripper fingers -->
  <joint name="fr3_finger_joint1" type="prismatic">
    <parent link="fr3_hand"/>
    <child link="fr3_leftfinger"/>
    <origin xyz="0 0.04 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="0.04" effort="20" velocity="0.2"/>
  </joint>
  
  <link name="fr3_leftfinger">
    <visual>
      <geometry>
        <box size="0.01 0.02 0.08"/>
      </geometry>
      <material name="grey"/>
    </visual>
  </link>
  
  <joint name="fr3_finger_joint2" type="prismatic">
    <parent link="fr3_hand"/>
    <child link="fr3_rightfinger"/>
    <origin xyz="0 -0.04 0.05" rpy="0 0 0"/>
    <axis xyz="0 -1 0"/>
    <limit lower="0" upper="0.04" effort="20" velocity="0.2"/>
  </joint>
  
  <link name="fr3_rightfinger">
    <visual>
      <geometry>
        <box size="0.01 0.02 0.08"/>
      </geometry>
      <material name="grey"/>
    </visual>
  </link>
</robot>
"""
        
    def stop_recording(self, success: bool = False):
        """Stop recording and categorize as success/failure
        
        Returns:
            Path to the saved MCAP file, or None if no recording was active
        """
        if not self.recording:
            print("⚠️  No recording in progress!")
            return None
            
        self.recording = False
        
        # Stop camera recording
        self._camera_recording = False
        if self._camera_thread:
            self._camera_thread.join(timeout=5.0)
            
        # Handle queue based on success/failure
        if success:
            # For successful recordings, wait for queue to empty to ensure all data is written
            print("⏳ Flushing data queue...")
            while not self._data_queue.empty():
                time.sleep(0.1)
        else:
            # For discarded recordings, clear the queue instantly
            print("🗑️  Clearing data queue...")
            # Clear the queue by getting all items without processing
            cleared_count = 0
            try:
                while True:
                    self._data_queue.get_nowait()
                    cleared_count += 1
            except Empty:
                pass
            if cleared_count > 0:
                print(f"   Cleared {cleared_count} pending items")
            
        # Stop writer thread
        self._running = False
        if self._writer_thread:
            self._writer_thread.join(timeout=5.0)
            
        # Calculate duration
        end_time = time.time()
        start_time = self.metadata.get("start_time", end_time)
        duration_seconds = int(end_time - start_time)
        duration_str = f"{duration_seconds//60:02d}m{duration_seconds%60:02d}s"
        
        # Add final metadata
        self.metadata["end_time"] = end_time
        self.metadata["duration"] = duration_seconds
        self.metadata["duration_str"] = duration_str
        self.metadata["success"] = success
        self.metadata["failure"] = not success
        # MCAP writer expects name and data as positional arguments with string values
        metadata_str = {k: str(v) for k, v in self.metadata.items()}
        self._writer.add_metadata("final_metadata", metadata_str)
        
        # Close MCAP file
        self._writer.finish()
        self._mcap_file.close()
        
        # Handle failure recordings - delete them
        if not success:
            # Delete the file instead of keeping it
            try:
                self.filepath.unlink()
                print(f"❌ Recording discarded (failure case)")
            except Exception as e:
                print(f"⚠️  Failed to delete failure recording: {e}")
            
            # Reset state
            self._channels.clear()
            self._schemas.clear()
            self._writer = None
            self._mcap_file = None
            self.filepath = None
            self.start_timestamp = None
            self._has_written_data = False
            
            return None
        
        # Only save successful recordings
        # Create new filename with duration
        timestamp_str = self.metadata.get("start_timestamp", "unknown")
        new_filename = f"trajectory_{timestamp_str}_{duration_str}.mcap"
        
        # Move to success directory with new name
        new_filepath = self.success_dir / new_filename
        self.filepath.rename(new_filepath)
        print(f"✅ Recording saved as SUCCESS: {new_filepath}")
            
        # Reset state
        self._channels.clear()
        self._schemas.clear()
        self._writer = None
        self._mcap_file = None
        self.filepath = None
        self.start_timestamp = None
        self._has_written_data = False
        
        return new_filepath
        
    def reset_recording(self):
        """Stop current recording without saving"""
        if self.recording:
            print("🔄 Stopping current recording...")
            # Stop current recording as failure (since it's being reset)
            self.stop_recording(success=False)
            return True
        return False
        
    def write_timestep(self, timestep: Dict[str, Any], timestamp: Optional[float] = None):
        """Queue timestep data for writing to MCAP"""
        if not self.recording:
            return
            
        # Add type field for the new data format
        timestep['type'] = 'timestep'
        self._data_queue.put(timestep)
        
    def write_robot_state(self, state: Dict[str, Any], timestamp: Optional[float] = None):
        """Deprecated - use write_timestep instead"""
        print("Warning: write_robot_state is deprecated, use write_timestep instead")
        
    def write_action(self, action: np.ndarray, timestamp: Optional[float] = None):
        """Deprecated - use write_timestep instead"""
        print("Warning: write_action is deprecated, use write_timestep instead")
        
    def write_vr_controller(self, controller_info: Dict[str, Any], timestamp: Optional[float] = None):
        """Deprecated - use write_timestep instead"""
        print("Warning: write_vr_controller is deprecated, use write_timestep instead")
        
    def write_camera_image(self, camera_id: str, image: np.ndarray, timestamp: Optional[float] = None):
        """Queue camera image for writing to MCAP"""
        if not self.recording:
            return
            
        if timestamp is None:
            timestamp = time.time()
            
        data = {
            "type": "camera_image",
            "camera_id": camera_id,
            "image": image,
            "timestamp": timestamp
        }
        
        self._data_queue.put(data)
        
    def write_depth_image(self, camera_id: str, depth_image: np.ndarray, timestamp: Optional[float] = None):
        """Queue depth image for writing to MCAP"""
        if not self.recording:
            return
            
        if timestamp is None:
            timestamp = time.time()
            
        data = {
            "type": "depth_image",
            "camera_id": camera_id,
            "depth_image": depth_image,
            "timestamp": timestamp
        }
        
        self._data_queue.put(data)
        
    def write_camera_calibration(self, camera_id: str, intrinsics: Dict, timestamp: Optional[float] = None):
        """Queue camera calibration for writing to MCAP"""
        if not self.recording:
            return
            
        if timestamp is None:
            timestamp = time.time()
            
        data = {
            "type": "camera_calibration",
            "camera_id": camera_id,
            "intrinsics": intrinsics,
            "timestamp": timestamp
        }
        
        self._data_queue.put(data)
        
    def _init_camera_recording(self):
        """Initialize camera recording thread if camera manager is available"""
        if not self.camera_manager:
            print("⚠️  No camera manager provided, camera recording disabled")
            return
            
        self._camera_recording = True
        self._camera_thread = threading.Thread(
            target=self._camera_recording_worker, 
            daemon=True,
            name="MCAP-Camera-Recorder"
        )
        self._camera_thread.start()
        print("📷 Started camera recording thread")
        
    def _camera_recording_worker(self):
        """Worker thread for recording camera frames to MCAP"""
        print("📷 Camera recording thread started")
        
        # Track which cameras have had their calibration written
        calibration_written = set()
        
        while self._camera_recording and self.recording:
            try:
                # Get frames from all cameras
                frames = self.camera_manager.get_all_frames(timeout=0.1)
                
                if frames:
                    # Process each camera frame
                    for cam_id, frame in frames.items():
                        # Use the frame's timestamp for better synchronization
                        timestamp = frame.timestamp
                        
                        # Write camera calibration once per camera
                        if cam_id not in calibration_written and frame.intrinsics:
                            self.write_camera_calibration(cam_id, frame.intrinsics, timestamp)
                            calibration_written.add(cam_id)
                        
                        # Write color image
                        if frame.color_image is not None:
                            self.write_camera_image(cam_id, frame.color_image, timestamp)
                            
                        # Write depth image if available and enabled
                        if self.save_depth and frame.depth_image is not None:
                            self.write_depth_image(cam_id, frame.depth_image, timestamp)
                            
                # Small sleep to prevent CPU spinning
                time.sleep(0.001)
                
            except Exception as e:
                print(f"❌ Error in camera recording: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(0.1)
                
        print("📷 Camera recording thread stopped")
        
    def _write_worker(self):
        """Worker thread for writing data to MCAP"""
        while self._running:
            try:
                data = self._data_queue.get(timeout=0.1)
                
                # Handle different data types
                if isinstance(data, dict) and 'type' in data:
                    data_type = data['type']
                    
                    if data_type == 'timestep':
                        # Original timestep format
                        self._write_timestep_to_mcap(data)
                    elif data_type == 'camera_image':
                        # Camera image data
                        timestamp = data['timestamp']
                        time_ns = int(timestamp * 1e9)
                        ts_sec = int(time_ns // 1_000_000_000)
                        ts_nsec = int(time_ns % 1_000_000_000)
                        self._write_camera_image_mcap(
                            data['camera_id'], 
                            data['image'], 
                            ts_sec, ts_nsec, time_ns
                        )
                    elif data_type == 'depth_image':
                        # Depth image data
                        timestamp = data['timestamp']
                        time_ns = int(timestamp * 1e9)
                        ts_sec = int(time_ns // 1_000_000_000)
                        ts_nsec = int(time_ns % 1_000_000_000)
                        self._write_depth_image_mcap(
                            data['camera_id'], 
                            data['depth_image'], 
                            ts_sec, ts_nsec, time_ns
                        )
                    elif data_type == 'camera_calibration':
                        # Camera calibration data
                        timestamp = data['timestamp']
                        time_ns = int(timestamp * 1e9)
                        ts_sec = int(time_ns // 1_000_000_000)
                        ts_nsec = int(time_ns % 1_000_000_000)
                        self._write_camera_calibration_mcap(
                            data['camera_id'], 
                            data['intrinsics'], 
                            ts_sec, ts_nsec, time_ns
                        )
                else:
                    # Assume it's a timestep for backward compatibility
                    self._write_timestep_to_mcap(data)
                    
            except Empty:
                continue
            except Exception as e:
                print(f"❌ Error in MCAP writer: {e}")
                import traceback
                traceback.print_exc()
                
    def _write_timestep_to_mcap(self, timestep: Dict[str, Any]):
        """Write a Labelbox Robotics format timestep to MCAP file"""
        # Check if this timestep has actual data
        has_robot_state = ("robot_state" in timestep.get("observation", {}) and 
                          timestep["observation"]["robot_state"])
        has_action = "action" in timestep and timestep["action"] is not None
        has_controller = ("controller_info" in timestep.get("observation", {}) and 
                         timestep["observation"]["controller_info"])
        has_images = ("image" in timestep.get("observation", {}) and 
                     timestep["observation"]["image"] is not None)
        
        # Skip empty timesteps at the beginning of recording
        if not self._has_written_data:
            if not (has_robot_state or has_action or has_controller or has_images):
                # Skip this empty timestep
                return
            else:
                # We have data, mark that we've started writing
                self._has_written_data = True
                print("📝 First data timestep received, starting MCAP recording")
        
        # Extract timestamp from the timestep data
        try:
            time_ns = timestep["observation"]["timestamp"]["robot_state"]["read_start"]
            if isinstance(time_ns, float):
                time_ns = int(time_ns)
        except (KeyError, TypeError):
            time_ns = int(time.time() * 1e9)
        
        # Ensure timestamp is valid integer
        time_ns = int(time_ns)
        ts_sec = int(time_ns // 1_000_000_000)
        ts_nsec = int(time_ns % 1_000_000_000)
        
        # Write robot state
        if "robot_state" in timestep["observation"]:
            self._write_robot_state_mcap(timestep["observation"]["robot_state"], ts_sec, ts_nsec, time_ns)
            
            # Also write joint state for visualization if we have cartesian data
            if "cartesian_position" in timestep["observation"]["robot_state"]:
                self._write_joint_state_for_visualization(timestep["observation"]["robot_state"], ts_sec, ts_nsec, time_ns)
                
                # Write transforms for all robot links
                joint_positions = timestep["observation"]["robot_state"].get("joint_positions", [])
                
                # If we have joint positions, append gripper data
                if joint_positions and len(joint_positions) >= 7:
                    joint_positions = list(joint_positions[:7])  # Get first 7 joints
                    
                    # Add gripper positions (same logic as in _write_joint_state_for_visualization)
                    gripper_pos = timestep["observation"]["robot_state"].get("gripper_position", 0.0)
                    finger_joint_pos = (1.0 - gripper_pos) * 0.04  # Invert: 0->0.04, 1->0.0
                    joint_positions_with_gripper = joint_positions + [finger_joint_pos, finger_joint_pos]
                    
                    self._write_robot_transforms(joint_positions_with_gripper, ts_sec, ts_nsec, time_ns)
                else:
                    # No joint data, just write base transforms
                    self._write_robot_transforms([], ts_sec, ts_nsec, time_ns)
        
        # Write action
        if "action" in timestep:
            self._write_action_mcap(timestep["action"], ts_sec, ts_nsec, time_ns)
        
        # Write VR controller info
        if "controller_info" in timestep["observation"]:
            self._write_vr_controller_mcap(timestep["observation"]["controller_info"], ts_sec, ts_nsec, time_ns)
        
        # Write camera images if present
        if "image" in timestep["observation"]:
            images = timestep["observation"]["image"]
            # Handle both single image and dict of images
            if isinstance(images, dict):
                # Multiple cameras
                for camera_id, image in images.items():
                    if image is not None:
                        self._write_camera_image_mcap(camera_id, image, ts_sec, ts_nsec, time_ns)
            elif images is not None:
                # Single image, assume camera_0
                self._write_camera_image_mcap("0", images, ts_sec, ts_nsec, time_ns)
            
    def _write_robot_state_mcap(self, state: Dict, ts_sec: int, ts_nsec: int, timestamp_ns: int):
        """Write robot state to MCAP"""
        if "robot_state" not in self._channels:
            self._channels["robot_state"] = self._writer.register_channel(
                topic="/robot_state",
                message_encoding="json",
                schema_id=self._schemas["robot_state"]
            )
        
        msg = {
            "timestamp": {"sec": ts_sec, "nanosec": ts_nsec},
            "joint_positions": state.get("joint_positions", []),
            "joint_velocities": state.get("joint_velocities", []),
            "joint_efforts": state.get("joint_efforts", []),
            "cartesian_position": state.get("cartesian_position", []),
            "cartesian_velocity": state.get("cartesian_velocity", []),
            "gripper_position": state.get("gripper_position", 0.0),
            "gripper_velocity": state.get("gripper_velocity", 0.0)
        }
        
        self._writer.add_message(
            channel_id=self._channels["robot_state"],
            sequence=0,
            log_time=timestamp_ns,
            publish_time=timestamp_ns,
            data=json.dumps(msg).encode("utf-8")
        )
        
    def _write_joint_state_for_visualization(self, state: Dict, ts_sec: int, ts_nsec: int, timestamp_ns: int):
        """Write ROS2-style joint state for robot visualization in Foxglove"""
        if "joint_state" not in self._channels:
            self._channels["joint_state"] = self._writer.register_channel(
                topic="/joint_states",
                message_encoding="json",
                schema_id=self._schemas["joint_state"]
            )
        
        # Franka FR3 has 7 joints + 2 finger joints
        joint_names = [
            "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
            "fr3_joint5", "fr3_joint6", "fr3_joint7",
            "fr3_finger_joint1", "fr3_finger_joint2"
        ]
        
        # Get joint positions if available
        joint_positions = state.get("joint_positions", [])
        
        # Ensure we have valid joint data
        if joint_positions and len(joint_positions) >= 7:
            # Use actual joint positions from robot
            joint_positions = list(joint_positions[:7])  # Take first 7 joints
        elif not joint_positions and "cartesian_position" in state:
            # Fallback: Try to estimate joint positions from cartesian data
            # This is a simplified approach - real IK would be more complex
            cart_pos = state["cartesian_position"]
            if len(cart_pos) >= 6:
                # Extract position and orientation
                pos = cart_pos[:3]
                euler = cart_pos[3:6] if len(cart_pos) >= 6 else [0, 0, 0]
                
                # Very rough approximation based on typical FR3 workspace
                # This is just for visualization - not accurate!
                j1 = np.arctan2(pos[1], pos[0])  # Base rotation
                j2 = -0.785 + pos[2] * 0.5  # Shoulder
                j3 = 0.0  # Elbow rotation
                j4 = -2.356 + pos[2] * 0.3  # Elbow
                j5 = 0.0  # Wrist rotation
                j6 = 1.571 + euler[1] * 0.5  # Wrist bend
                j7 = 0.785 + euler[2]  # Wrist rotation
                
                joint_positions = [j1, j2, j3, j4, j5, j6, j7]
            else:
                # Default home position for FR3
                joint_positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        else:
            # Default home position for FR3
            joint_positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        
        # Add gripper joints (convert 0-1 to joint angles)
        gripper_pos = state.get("gripper_position", 0.0)
        # FR3 gripper: 0 = open (0.04 rad per finger), 1 = closed (0.0 rad)
        # Note: gripper_position is 0 when open, 1 when closed
        # Each finger can move 0.04 meters (not radians)
        finger_joint_pos = (1.0 - gripper_pos) * 0.04  # Invert: 0->0.04, 1->0.0
        joint_positions = list(joint_positions) + [finger_joint_pos, finger_joint_pos]
        
        # Create ROS2 JointState message
        # ROS2 uses nanosec instead of nsec
        msg = {
            "header": {
                "stamp": {"sec": ts_sec, "nanosec": ts_nsec},
                "frame_id": "world"
            },
            "name": joint_names,
            "position": joint_positions,
            "velocity": [0.0] * len(joint_names),
            "effort": [0.0] * len(joint_names)
        }
        
        # Write as JSON
        self._writer.add_message(
            channel_id=self._channels["joint_state"],
            sequence=0,
            log_time=timestamp_ns,
            publish_time=timestamp_ns,
            data=json.dumps(msg).encode("utf-8")
        )
        
    def _write_action_mcap(self, action: np.ndarray, ts_sec: int, ts_nsec: int, timestamp_ns: int):
        """Write action to MCAP"""
        if "action" not in self._channels:
            self._channels["action"] = self._writer.register_channel(
                topic="/action",
                message_encoding="json",
                schema_id=self._schemas["action"]
            )
        
        msg = {
            "timestamp": {"sec": ts_sec, "nanosec": ts_nsec},
            "data": action.tolist() if isinstance(action, np.ndarray) else action
        }
        
        self._writer.add_message(
            channel_id=self._channels["action"],
            sequence=0,
            log_time=timestamp_ns,
            publish_time=timestamp_ns,
            data=json.dumps(msg).encode("utf-8")
        )
        
    def _write_vr_controller_mcap(self, controller_info: Dict, ts_sec: int, ts_nsec: int, timestamp_ns: int):
        """Write VR controller info to MCAP"""
        if "vr_controller" not in self._channels:
            self._channels["vr_controller"] = self._writer.register_channel(
                topic="/vr_controller",
                message_encoding="json",
                schema_id=self._schemas["vr_controller"]
            )
        
        # Extract controller info and convert numpy arrays to lists
        poses = controller_info.get("poses", {})
        # Convert any numpy arrays in poses to lists
        poses_serializable = {}
        for key, value in poses.items():
            if hasattr(value, 'tolist'):
                # It's a numpy array, convert to list
                # If it's a 4x4 matrix, flatten it to a 16-element list
                if hasattr(value, 'shape') and value.shape == (4, 4):
                    poses_serializable[key] = value.flatten().tolist()
                else:
                    poses_serializable[key] = value.tolist()
            elif isinstance(value, np.ndarray):
                # Another way to check for numpy array
                if value.shape == (4, 4):
                    poses_serializable[key] = value.flatten().tolist()
                else:
                    poses_serializable[key] = value.tolist()
            else:
                # Already serializable
                poses_serializable[key] = value
        
        buttons = controller_info.get("buttons", {})
        # Also check buttons for any numpy arrays
        buttons_serializable = {}
        for key, value in buttons.items():
            if hasattr(value, 'tolist'):
                buttons_serializable[key] = value.tolist()
            elif isinstance(value, np.ndarray):
                buttons_serializable[key] = value.tolist()
            elif isinstance(value, (list, tuple)) and len(value) > 0:
                # Check if it's a list/tuple containing numpy arrays
                if hasattr(value[0], 'tolist'):
                    buttons_serializable[key] = [v.tolist() if hasattr(v, 'tolist') else v for v in value]
                else:
                    buttons_serializable[key] = value
            else:
                buttons_serializable[key] = value
        
        msg = {
            "timestamp": {"sec": ts_sec, "nanosec": ts_nsec},
            "poses": poses_serializable,
            "buttons": buttons_serializable,
            "movement_enabled": controller_info.get("movement_enabled", False),
            "controller_on": controller_info.get("controller_on", False),
            "success": controller_info.get("success", False),
            "failure": controller_info.get("failure", False)
        }
        
        self._writer.add_message(
            channel_id=self._channels["vr_controller"],
            sequence=0,
            log_time=timestamp_ns,
            publish_time=timestamp_ns,
            data=json.dumps(msg).encode("utf-8")
        )
        
    def _write_camera_image_mcap(self, camera_id: str, image: np.ndarray, ts_sec: int, ts_nsec: int, timestamp_ns: int):
        """Write camera image to MCAP"""
        channel_name = f"camera_{camera_id}"
        if channel_name not in self._channels:
            self._channels[channel_name] = self._writer.register_channel(
                topic=f"/camera/{camera_id}/compressed",
                message_encoding="json",
                schema_id=self._schemas["compressed_image"]
            )
        
        # Compress image to JPEG
        success, buffer = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not success:
            print(f"⚠️  Failed to encode image from camera {camera_id}")
            return
            
        # Convert to base64
        image_data = base64.b64encode(buffer).decode('utf-8')
        
        msg = {
            "timestamp": {"sec": ts_sec, "nanosec": ts_nsec},
            "frame_id": f"camera_{camera_id}",
            "data": image_data,
            "format": "jpeg"
        }
        
        self._writer.add_message(
            channel_id=self._channels[channel_name],
            sequence=0,
            log_time=timestamp_ns,
            publish_time=timestamp_ns,
            data=json.dumps(msg).encode("utf-8")
        )
        
    def _write_depth_image_mcap(self, camera_id: str, depth_image: np.ndarray, ts_sec: int, ts_nsec: int, timestamp_ns: int):
        """Write depth image to MCAP"""
        channel_name = f"depth_{camera_id}"
        if channel_name not in self._channels:
            self._channels[channel_name] = self._writer.register_channel(
                topic=f"/camera/{camera_id}/depth",
                message_encoding="json",
                schema_id=self._schemas["raw_image"]
            )
        
        # Encode depth data
        # Convert to millimeters and clip to uint16 range
        # Get depth scale from camera manager config if available
        depth_scale = 0.001  # Default 1mm = 0.001m
        if self.camera_manager and hasattr(self.camera_manager, 'config'):
            depth_scale = self.camera_manager.config.get('camera_settings', {}).get('depth_scale', 0.001)
        
        # Ensure depth_image is float32 before conversion
        if depth_image.dtype != np.float32:
            depth_image = depth_image.astype(np.float32)
            
        # Convert meters to millimeters
        depth_mm = (depth_image / depth_scale).astype(np.uint16)
        
        # Get actual dimensions after any processing
        height, width = depth_mm.shape
        
        # Convert to bytes for proper encoding
        # Ensure we're using native byte order
        depth_bytes = depth_mm.tobytes()
        
        # Convert to base64
        depth_data = base64.b64encode(depth_bytes).decode('utf-8')
        
        msg = {
            "timestamp": {"sec": ts_sec, "nanosec": ts_nsec},
            "frame_id": f"camera_{camera_id}_depth",
            "width": width,
            "height": height,
            "encoding": "16UC1",  # 16-bit unsigned single channel
            "step": width * 2,  # 2 bytes per pixel for 16-bit
            "data": depth_data
        }
        
        self._writer.add_message(
            channel_id=self._channels[channel_name],
            sequence=0,
            log_time=timestamp_ns,
            publish_time=timestamp_ns,
            data=json.dumps(msg).encode("utf-8")
        )
        
    def _write_camera_calibration_mcap(self, camera_id: str, intrinsics: Dict, ts_sec: int, ts_nsec: int, timestamp_ns: int):
        """Write camera calibration to MCAP"""
        # Write color camera calibration
        channel_name = f"calibration_{camera_id}"
        if channel_name not in self._channels:
            self._channels[channel_name] = self._writer.register_channel(
                topic=f"/camera/{camera_id}/camera_info",
                message_encoding="json",
                schema_id=self._schemas["camera_calibration"]
            )
        
        # Convert intrinsics to camera calibration format
        # K matrix (3x3 camera intrinsic matrix)
        fx = intrinsics.get('fx', 0.0)
        fy = intrinsics.get('fy', 0.0)
        cx = intrinsics.get('cx', 0.0)
        cy = intrinsics.get('cy', 0.0)
        
        K = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0
        ]
        
        # R matrix (3x3 rectification matrix) - identity for single camera
        R = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]
        
        # P matrix (3x4 projection matrix)
        P = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        # Distortion coefficients
        D = intrinsics.get('coeffs', [0.0, 0.0, 0.0, 0.0, 0.0])
        
        # Distortion model
        model = intrinsics.get('model', 'plumb_bob')
        if model == 'distortion.brown_conrady':
            model = 'plumb_bob'  # Convert RealSense model name to ROS standard
        elif model == 'distortion.inverse_brown_conrady':
            model = 'plumb_bob'  # Convert RealSense inverse model to ROS standard
        elif 'brown_conrady' in str(model).lower():
            model = 'plumb_bob'  # Handle any brown_conrady variant
        
        msg = {
            "timestamp": {"sec": ts_sec, "nanosec": ts_nsec},
            "frame_id": f"camera_{camera_id}",
            "width": intrinsics.get('width', 0),
            "height": intrinsics.get('height', 0),
            "distortion_model": model,
            "D": D,
            "K": K,
            "R": R,
            "P": P
        }
        
        self._writer.add_message(
            channel_id=self._channels[channel_name],
            sequence=0,
            log_time=timestamp_ns,
            publish_time=timestamp_ns,
            data=json.dumps(msg).encode("utf-8")
        )
        
        # Write depth camera calibration if available
        depth_intrinsics = intrinsics.get('depth_intrinsics')
        if depth_intrinsics:
            depth_channel_name = f"calibration_depth_{camera_id}"
            if depth_channel_name not in self._channels:
                self._channels[depth_channel_name] = self._writer.register_channel(
                    topic=f"/camera/{camera_id}/depth/camera_info",
                    message_encoding="json",
                    schema_id=self._schemas["camera_calibration"]
                )
            
            # Convert depth intrinsics
            d_fx = depth_intrinsics.get('fx', 0.0)
            d_fy = depth_intrinsics.get('fy', 0.0)
            d_cx = depth_intrinsics.get('cx', 0.0)
            d_cy = depth_intrinsics.get('cy', 0.0)
            
            d_K = [
                d_fx, 0.0, d_cx,
                0.0, d_fy, d_cy,
                0.0, 0.0, 1.0
            ]
            
            d_P = [
                d_fx, 0.0, d_cx, 0.0,
                0.0, d_fy, d_cy, 0.0,
                0.0, 0.0, 1.0, 0.0
            ]
            
            # Depth distortion
            d_D = depth_intrinsics.get('coeffs', [0.0, 0.0, 0.0, 0.0, 0.0])
            d_model = depth_intrinsics.get('model', 'plumb_bob')
            if 'brown_conrady' in str(d_model).lower():
                d_model = 'plumb_bob'
            
            depth_msg = {
                "timestamp": {"sec": ts_sec, "nanosec": ts_nsec},
                "frame_id": f"camera_{camera_id}_depth",
                "width": depth_intrinsics.get('width', 0),
                "height": depth_intrinsics.get('height', 0),
                "distortion_model": d_model,
                "D": d_D,
                "K": d_K,
                "R": R,  # Same rectification
                "P": d_P
            }
            
            self._writer.add_message(
                channel_id=self._channels[depth_channel_name],
                sequence=0,
                log_time=timestamp_ns,
                publish_time=timestamp_ns,
                data=json.dumps(depth_msg).encode("utf-8")
            )
        
    def _write_robot_transforms(self, joint_positions: list, ts_sec: int, ts_nsec: int, timestamp_ns: int):
        """Write transforms for all robot links based on joint positions"""
        if "transform" not in self._channels:
            self._channels["transform"] = self._writer.register_channel(
                topic="/tf",
                message_encoding="json",
                schema_id=self._schemas["transform"]
            )
        
        # Start with base transforms
        transforms = [
            # World to base
            {
                "header": {
                    "stamp": {"sec": ts_sec, "nanosec": ts_nsec},
                    "frame_id": "world"
                },
                "child_frame_id": "base",
                "transform": {
                    "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                }
            },
            # Base to fr3_link0
            {
                "header": {
                    "stamp": {"sec": ts_sec, "nanosec": ts_nsec},
                    "frame_id": "base"
                },
                "child_frame_id": "fr3_link0",
                "transform": {
                    "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                }
            }
        ]
        
        # If we have joint positions, compute and publish transforms for all links
        if joint_positions and len(joint_positions) >= 7:
            # FR3 DH parameters and joint configurations
            # These are simplified - for exact values, we'd need to parse the URDF
            
            # Link 0 to Link 1 (Joint 1 - Z rotation)
            transforms.append({
                "header": {
                    "stamp": {"sec": ts_sec, "nanosec": ts_nsec},
                    "frame_id": "fr3_link0"
                },
                "child_frame_id": "fr3_link1",
                "transform": {
                    "translation": {"x": 0.0, "y": 0.0, "z": 0.333},
                    "rotation": self._axis_angle_to_quaternion([0, 0, 1], joint_positions[0])
                }
            })
            
            # Link 1 to Link 2 (Joint 2 - Y rotation, rotated -90 deg)
            transforms.append({
                "header": {
                    "stamp": {"sec": ts_sec, "nanosec": ts_nsec},
                    "frame_id": "fr3_link1"
                },
                "child_frame_id": "fr3_link2",
                "transform": {
                    "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "rotation": self._combine_rotations(
                        self._axis_angle_to_quaternion([1, 0, 0], -np.pi/2),
                        self._axis_angle_to_quaternion([0, 0, 1], joint_positions[1])
                    )
                }
            })
            
            # Link 2 to Link 3 (Joint 3 - Z rotation, rotated 90 deg)
            transforms.append({
                "header": {
                    "stamp": {"sec": ts_sec, "nanosec": ts_nsec},
                    "frame_id": "fr3_link2"
                },
                "child_frame_id": "fr3_link3",
                "transform": {
                    "translation": {"x": 0.0, "y": -0.316, "z": 0.0},
                    "rotation": self._combine_rotations(
                        self._axis_angle_to_quaternion([1, 0, 0], np.pi/2),
                        self._axis_angle_to_quaternion([0, 0, 1], joint_positions[2])
                    )
                }
            })
            
            # Link 3 to Link 4 (Joint 4 - Z rotation, rotated 90 deg)
            transforms.append({
                "header": {
                    "stamp": {"sec": ts_sec, "nanosec": ts_nsec},
                    "frame_id": "fr3_link3"
                },
                "child_frame_id": "fr3_link4",
                "transform": {
                    "translation": {"x": 0.0825, "y": 0.0, "z": 0.0},
                    "rotation": self._combine_rotations(
                        self._axis_angle_to_quaternion([1, 0, 0], np.pi/2),
                        self._axis_angle_to_quaternion([0, 0, 1], joint_positions[3])
                    )
                }
            })
            
            # Link 4 to Link 5 (Joint 5 - Z rotation, rotated -90 deg)
            transforms.append({
                "header": {
                    "stamp": {"sec": ts_sec, "nanosec": ts_nsec},
                    "frame_id": "fr3_link4"
                },
                "child_frame_id": "fr3_link5",
                "transform": {
                    "translation": {"x": -0.0825, "y": 0.384, "z": 0.0},
                    "rotation": self._combine_rotations(
                        self._axis_angle_to_quaternion([1, 0, 0], -np.pi/2),
                        self._axis_angle_to_quaternion([0, 0, 1], joint_positions[4])
                    )
                }
            })
            
            # Link 5 to Link 6 (Joint 6 - Z rotation, rotated 90 deg)
            transforms.append({
                "header": {
                    "stamp": {"sec": ts_sec, "nanosec": ts_nsec},
                    "frame_id": "fr3_link5"
                },
                "child_frame_id": "fr3_link6",
                "transform": {
                    "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "rotation": self._combine_rotations(
                        self._axis_angle_to_quaternion([1, 0, 0], np.pi/2),
                        self._axis_angle_to_quaternion([0, 0, 1], joint_positions[5])
                    )
                }
            })
            
            # Link 6 to Link 7 (Joint 7 - Z rotation, rotated 90 deg)
            transforms.append({
                "header": {
                    "stamp": {"sec": ts_sec, "nanosec": ts_nsec},
                    "frame_id": "fr3_link6"
                },
                "child_frame_id": "fr3_link7",
                "transform": {
                    "translation": {"x": 0.088, "y": 0.0, "z": 0.0},
                    "rotation": self._combine_rotations(
                        self._axis_angle_to_quaternion([1, 0, 0], np.pi/2),
                        self._axis_angle_to_quaternion([0, 0, 1], joint_positions[6])
                    )
                }
            })
            
            # Link 7 to Hand (fixed)
            transforms.append({
                "header": {
                    "stamp": {"sec": ts_sec, "nanosec": ts_nsec},
                    "frame_id": "fr3_link7"
                },
                "child_frame_id": "fr3_hand",
                "transform": {
                    "translation": {"x": 0.0, "y": 0.0, "z": 0.107},
                    "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                }
            })
            
            # Add gripper finger transforms if we have enough joint data
            if len(joint_positions) >= 9:
                # Left finger (fr3_finger_joint1)
                finger_pos = joint_positions[7]  # This is the linear position in meters
                transforms.append({
                    "header": {
                        "stamp": {"sec": ts_sec, "nanosec": ts_nsec},
                        "frame_id": "fr3_hand"
                    },
                    "child_frame_id": "fr3_leftfinger",
                    "transform": {
                        "translation": {"x": 0.0, "y": 0.04 - finger_pos, "z": 0.05},  # Move inward as it closes
                        "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                    }
                })
                
                # Right finger (fr3_finger_joint2)
                finger_pos = joint_positions[8]  # This is the linear position in meters
                transforms.append({
                    "header": {
                        "stamp": {"sec": ts_sec, "nanosec": ts_nsec},
                        "frame_id": "fr3_hand"
                    },
                    "child_frame_id": "fr3_rightfinger",
                    "transform": {
                        "translation": {"x": 0.0, "y": -0.04 + finger_pos, "z": 0.05},  # Move inward as it closes
                        "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                    }
                })
            else:
                # Default open position for fingers
                transforms.append({
                    "header": {
                        "stamp": {"sec": ts_sec, "nanosec": ts_nsec},
                        "frame_id": "fr3_hand"
                    },
                    "child_frame_id": "fr3_leftfinger",
                    "transform": {
                        "translation": {"x": 0.0, "y": 0.04, "z": 0.05},
                        "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                    }
                })
                
                transforms.append({
                    "header": {
                        "stamp": {"sec": ts_sec, "nanosec": ts_nsec},
                        "frame_id": "fr3_hand"
                    },
                    "child_frame_id": "fr3_rightfinger",
                    "transform": {
                        "translation": {"x": 0.0, "y": -0.04, "z": 0.05},
                        "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                    }
                })
        
        # Add dynamic camera transforms
        if hasattr(self, 'dynamic_camera_configs'):
            for camera_id, config in self.dynamic_camera_configs.items():
                parent_frame = config['parent_frame']
                trans = config['translation']
                rot = config['rotation']
                
                # Camera RGB frame
                transforms.append({
                    "header": {
                        "stamp": {"sec": ts_sec, "nanosec": ts_nsec},
                        "frame_id": parent_frame
                    },
                    "child_frame_id": f"camera_{camera_id}",
                    "transform": {
                        "translation": {"x": trans['x'], "y": trans['y'], "z": trans['z']},
                        "rotation": {"x": rot['x'], "y": rot['y'], "z": rot['z'], "w": rot['w']}
                    }
                })
                
                # Camera depth frame (same position as RGB)
                transforms.append({
                    "header": {
                        "stamp": {"sec": ts_sec, "nanosec": ts_nsec},
                        "frame_id": parent_frame
                    },
                    "child_frame_id": f"camera_{camera_id}_depth",
                    "transform": {
                        "translation": {"x": trans['x'], "y": trans['y'], "z": trans['z']},
                        "rotation": {"x": rot['x'], "y": rot['y'], "z": rot['z'], "w": rot['w']}
                    }
                })
        
        # Write all transforms in a single TFMessage
        msg = {"transforms": transforms}
        
        self._writer.add_message(
            channel_id=self._channels["transform"],
            sequence=0,
            log_time=timestamp_ns,
            publish_time=timestamp_ns,
            data=json.dumps(msg).encode("utf-8")
        )
        
    def _write_initial_transforms(self):
        """Write initial transforms to establish coordinate frames"""
        # Register transform channels if not already done
        if "transform" not in self._channels:
            self._channels["transform"] = self._writer.register_channel(
                topic="/tf",
                message_encoding="json",
                schema_id=self._schemas["transform"]
            )
        
        # Also register /tf_static for static transforms (Foxglove prefers this)
        if "tf_static" not in self._channels:
            self._channels["tf_static"] = self._writer.register_channel(
                topic="/tf_static",
                message_encoding="json",
                schema_id=self._schemas["transform"]
            )
        
        # Use actual start time for initial transforms
        initial_time_ns = int(self.metadata["start_time"] * 1e9)
        ts_sec = initial_time_ns // 1_000_000_000
        ts_nsec = initial_time_ns % 1_000_000_000
        
        static_transforms = {
            "transforms": [
                {
                    "header": {
                        "stamp": {"sec": ts_sec, "nanosec": ts_nsec},
                        "frame_id": "world"
                    },
                    "child_frame_id": "base",
                    "transform": {
                        "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                    }
                },
                {
                    "header": {
                        "stamp": {"sec": ts_sec, "nanosec": ts_nsec},
                        "frame_id": "base"
                    },
                    "child_frame_id": "fr3_link0",
                    "transform": {
                        "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                    }
                }
            ]
        }
        
        # Write to both /tf and /tf_static
        self._writer.add_message(
            channel_id=self._channels["transform"],
            sequence=0,
            log_time=initial_time_ns,
            publish_time=initial_time_ns,
            data=json.dumps(static_transforms).encode("utf-8")
        )
        
        self._writer.add_message(
            channel_id=self._channels["tf_static"],
            sequence=0,
            log_time=initial_time_ns,
            publish_time=initial_time_ns,
            data=json.dumps(static_transforms).encode("utf-8")
        )
        
        # Write camera transforms
        self._write_camera_transforms()
        
    def _write_camera_transforms(self):
        """Write static transforms for camera frames"""
        if not self.camera_manager:
            return
            
        # Try to load camera transform config
        camera_transforms_config = {}
        try:
            transform_config_path = Path(__file__).parent.parent / "configs" / "camera_transforms.yaml"
            if transform_config_path.exists():
                import yaml
                with open(transform_config_path, 'r') as f:
                    camera_transforms_config = yaml.safe_load(f)
        except Exception as e:
            print(f"⚠️  Could not load camera transforms config: {e}")
            
        # Use actual start time for transforms
        initial_time_ns = int(self.metadata["start_time"] * 1e9)
        ts_sec = initial_time_ns // 1_000_000_000
        ts_nsec = initial_time_ns % 1_000_000_000
        
        static_camera_transforms = []
        self.dynamic_camera_configs = {}  # Store configs for dynamic cameras
        
        # Get configured transforms
        transforms_data = camera_transforms_config.get('camera_transforms', {})
        default_transform = camera_transforms_config.get('default_camera_transform', {
            'parent_frame': 'base',
            'translation': {'x': 0.0, 'y': 0.3, 'z': 0.5},
            'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        })
        
        # Add transforms for each camera
        for camera_id in self.camera_manager.cameras.keys():
            # Get transform for this camera or use default
            transform = transforms_data.get(camera_id, default_transform)
            parent_frame = transform.get('parent_frame', default_transform.get('parent_frame', 'base'))
            trans = transform.get('translation', default_transform['translation'])
            rot = transform.get('rotation', default_transform['rotation'])
            
            # Check if this is a static transform (parent is base or world)
            if parent_frame in ['base', 'world', 'fr3_link0']:
                # Static transform - write once at start
                # Camera RGB frame
                static_camera_transforms.append({
                    "header": {
                        "stamp": {"sec": ts_sec, "nanosec": ts_nsec},
                        "frame_id": parent_frame
                    },
                    "child_frame_id": f"camera_{camera_id}",
                    "transform": {
                        "translation": {"x": trans['x'], "y": trans['y'], "z": trans['z']},
                        "rotation": {"x": rot['x'], "y": rot['y'], "z": rot['z'], "w": rot['w']}
                    }
                })
                
                # Camera depth frame (same position as RGB)
                static_camera_transforms.append({
                    "header": {
                        "stamp": {"sec": ts_sec, "nanosec": ts_nsec},
                        "frame_id": parent_frame
                    },
                    "child_frame_id": f"camera_{camera_id}_depth",
                    "transform": {
                        "translation": {"x": trans['x'], "y": trans['y'], "z": trans['z']},
                        "rotation": {"x": rot['x'], "y": rot['y'], "z": rot['z'], "w": rot['w']}
                    }
                })
            else:
                # Dynamic transform - store config for later updates
                self.dynamic_camera_configs[camera_id] = {
                    'parent_frame': parent_frame,
                    'translation': trans,
                    'rotation': rot
                }
                print(f"📷 Camera {camera_id} is mounted on {parent_frame} (dynamic transform)")
        
        if static_camera_transforms:
            msg = {"transforms": static_camera_transforms}
            
            # Write to /tf_static
            self._writer.add_message(
                channel_id=self._channels["tf_static"],
                sequence=0,
                log_time=initial_time_ns,
                publish_time=initial_time_ns,
                data=json.dumps(msg).encode("utf-8")
            )
        
    def _axis_angle_to_quaternion(self, axis: list, angle: float) -> dict:
        """Convert axis-angle representation to quaternion"""
        # Normalize axis
        axis = np.array(axis)
        axis = axis / np.linalg.norm(axis)
        
        # Convert to quaternion
        half_angle = angle / 2.0
        sin_half = np.sin(half_angle)
        cos_half = np.cos(half_angle)
        
        return {
            "x": axis[0] * sin_half,
            "y": axis[1] * sin_half,
            "z": axis[2] * sin_half,
            "w": cos_half
        }
    
    def _combine_rotations(self, q1: dict, q2: dict) -> dict:
        """Combine two quaternions (q1 * q2)"""
        # Extract components
        x1, y1, z1, w1 = q1["x"], q1["y"], q1["z"], q1["w"]
        x2, y2, z2, w2 = q2["x"], q2["y"], q2["z"], q2["w"]
        
        # Quaternion multiplication
        return {
            "x": w1*x2 + x1*w2 + y1*z2 - z1*y2,
            "y": w1*y2 - x1*z2 + y1*w2 + z1*x2,
            "z": w1*z2 + x1*y2 - y1*x2 + z1*w2,
            "w": w1*w2 - x1*x2 - y1*y2 - z1*z2
        } 