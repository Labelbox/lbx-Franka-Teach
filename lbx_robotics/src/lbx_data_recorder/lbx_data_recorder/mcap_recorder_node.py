#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import os
import time
import json
import base64
import threading
import numpy as np
from pathlib import Path
from datetime import datetime
from queue import Queue, Empty, Full
from typing import Dict, Optional, Any, List, Tuple
import cv2 # For image encoding
import sys

import mcap # Added import
from mcap.writer import Writer as BaseMcapWriter # Keep base writer for specific metadata/attachment if needed
from mcap_ros2.writer import Writer as McapRos2NativeWriter # For writing ROS2 messages with CDR

from diagnostic_updater import Updater, DiagnosticTask, DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

# Assuming standard messages for now. If custom messages are needed (e.g. from lbx_interfaces),
# they need to be imported and handled.
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, JointState, Joy
from geometry_msgs.msg import PoseStamped, TransformStamped # For VR controller and TF
from std_msgs.msg import String # For robot_description
from tf2_msgs.msg import TFMessage # For /tf and /tf_static
from cv_bridge import CvBridge # For image conversion

# Custom Services for recording control (define these in an srv file in your interfaces package)
from lbx_interfaces.srv import StartRecording, StopRecording

# Placeholder for actual robot state message if it's custom
# from lbx_interfaces.msg import RobotFullState # Example

# Define schemas as JSON strings (similar to original MCAPDataRecorder)
# These are simplified for brevity; ensure they match your exact required format.
SCHEMA_ROBOT_STATE_STR = json.dumps({
    "type": "object", "title": "labelbox_robotics.RobotState", "properties": {
        "timestamp": {"type": "object", "properties": {"sec": {"type": "integer"}, "nanosec": {"type": "integer"}}},
        "joint_positions": {"type": "array", "items": {"type": "number"}},
        "joint_velocities": {"type": "array", "items": {"type": "number"}, "default": []},
        "joint_efforts": {"type": "array", "items": {"type": "number"}, "default": []},
        "cartesian_position": {"type": "array", "items": {"type": "number"}, "description": "[x,y,z,qx,qy,qz,qw] or [x,y,z,roll,pitch,yaw]"},
        "cartesian_velocity": {"type": "array", "items": {"type": "number"}, "default": []},
        "gripper_position": {"type": "number"},
        "gripper_velocity": {"type": "number", "default": 0.0}
    },
    "required": ["timestamp", "joint_positions", "cartesian_position", "gripper_position"]
})

SCHEMA_ACTION_STR = json.dumps({
    "type": "object", "title": "labelbox_robotics.Action", "properties": {
        "timestamp": {"type": "object", "properties": {"sec": {"type": "integer"}, "nanosec": {"type": "integer"}}},
        "data": {"type": "array", "items": {"type": "number"}, "description": "[lin_x,lin_y,lin_z,ang_x,ang_y,ang_z,gripper_vel]"}
    },
    "required": ["timestamp", "data"]
})

SCHEMA_VR_CONTROLLER_STR = json.dumps({
    "type": "object",
    "properties": {
        "timestamp": {"type": "object", "properties": {"sec": {"type": "integer"}, "nanosec": {"type": "integer"}}},
        "left_joystick": {"type": "array", "items": {"type": "number"}, "minItems": 2, "maxItems": 2},
        "right_joystick": {"type": "array", "items": {"type": "number"}, "minItems": 2, "maxItems": 2},
        "left_buttons": {"type": "object", "additionalProperties": {"type": "boolean"}},
        "right_buttons": {"type": "object", "additionalProperties": {"type": "boolean"}},
        "left_analog_triggers": {"type": "object", "properties": {"trigger": {"type": "number"}, "grip": {"type": "number"}}},
        "right_analog_triggers": {"type": "object", "properties": {"trigger": {"type": "number"}, "grip": {"type": "number"}}}
    },
    "required": ["timestamp"]
})

# Fix MCAP WellKnownSchema compatibility issue
try:
    SCHEMA_COMPRESSED_IMAGE_STR = json.dumps(mcap.WellKnownSchema.foxglove_CompressedImage.json_schema)
    SCHEMA_CAMERA_CALIBRATION_STR = json.dumps(mcap.WellKnownSchema.foxglove_CameraCalibration.json_schema)
except AttributeError:
    # Fallback for older mcap versions without WellKnownSchema
    SCHEMA_COMPRESSED_IMAGE_STR = json.dumps({
        "type": "object",
        "properties": {
            "timestamp": {"type": "object", "properties": {"sec": {"type": "integer"}, "nanosec": {"type": "integer"}}},
            "frame_id": {"type": "string"},
            "data": {"type": "string", "description": "Base64 encoded image data"},
            "format": {"type": "string", "description": "Image format (e.g., jpeg, png)"}
        },
        "required": ["timestamp", "frame_id", "data", "format"]
    })
    SCHEMA_CAMERA_CALIBRATION_STR = json.dumps({
        "type": "object", 
        "properties": {
            "timestamp": {"type": "object", "properties": {"sec": {"type": "integer"}, "nanosec": {"type": "integer"}}},
            "frame_id": {"type": "string"},
            "width": {"type": "integer"},
            "height": {"type": "integer"},
            "distortion_model": {"type": "string"},
            "D": {"type": "array", "items": {"type": "number"}},
            "K": {"type": "array", "items": {"type": "number"}, "minItems": 9, "maxItems": 9},
            "R": {"type": "array", "items": {"type": "number"}, "minItems": 9, "maxItems": 9},
            "P": {"type": "array", "items": {"type": "number"}, "minItems": 12, "maxItems": 12}
        },
        "required": ["timestamp", "frame_id", "width", "height"]
    })

# These will be handled by McapRos2NativeWriter using .msg definitions
# SCHEMA_TF_MESSAGE_STR = json.dumps(mcap.WellKnownSchema.ros2_tf2_msgs_TFMessage.json_schema)
# SCHEMA_STD_STRING_STR = json.dumps(mcap.WellKnownSchema.ros2_std_msgs_String.json_schema)
# SCHEMA_SENSOR_JOINTSTATE_STR = json.dumps(mcap.WellKnownSchema.ros2_sensor_msgs_JointState.json_schema)

class MCAPRecorderNode(Node):
    def __init__(self):
        super().__init__('mcap_recorder_node')

        self.declare_parameter('base_directory', str(Path.home() / "recordings"))
        self.declare_parameter('default_robot_name', 'franka_fr3') # Used in metadata
        self.declare_parameter('auto_start_recording', False)
        self.declare_parameter('auto_start_filename_prefix', 'auto_trajectory')
        self.declare_parameter('topics_to_record', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('topic_types_map', rclpy.Parameter.Type.STRING_ARRAY, []) # topic_name:MsgType format
        self.declare_parameter('recording_frequency_hz', 20.0) # How often to bundle data for MCAP
        self.declare_parameter('image_jpeg_quality', 85)
        self.declare_parameter('mcap_queue_size', 1000)
        self.declare_parameter('diagnostics_publish_rate_hz', 0.2) # 5 seconds
        self.declare_parameter('mcap_write_chunk_size_kb', 1024) # New: MCAP chunk size
        self.declare_parameter('urdf_path', str(Path(__file__).parent.parent.parent / 'robot_urdf_models' / 'fr3_franka_hand_snug.urdf'))

        self.base_dir = Path(self.get_parameter('base_directory').get_parameter_value().string_value)
        self.robot_name = self.get_parameter('default_robot_name').get_parameter_value().string_value
        self.auto_start = self.get_parameter('auto_start_recording').get_parameter_value().bool_value
        self.auto_start_prefix = self.get_parameter('auto_start_filename_prefix').get_parameter_value().string_value
        self.configured_topics = self.get_parameter('topics_to_record').get_parameter_value().string_array_value
        self.topic_types_map_param = self.get_parameter('topic_types_map').get_parameter_value().string_array_value
        self.recording_hz = self.get_parameter('recording_frequency_hz').get_parameter_value().double_value
        self.jpeg_quality = self.get_parameter('image_jpeg_quality').get_parameter_value().integer_value
        mcap_queue_size = self.get_parameter('mcap_queue_size').get_parameter_value().integer_value
        diagnostics_rate = self.get_parameter('diagnostics_publish_rate_hz').get_parameter_value().double_value
        self.mcap_chunk_size = self.get_parameter('mcap_write_chunk_size_kb').get_parameter_value().integer_value * 1024
        self.urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value

        self.base_dir.mkdir(parents=True, exist_ok=True)
        self.success_dir = self.base_dir / "success"
        self.failure_dir = self.base_dir / "failure"
        self.success_dir.mkdir(parents=True, exist_ok=True)
        self.failure_dir.mkdir(parents=True, exist_ok=True)

        self.is_recording = False
        self.mcap_writer = None
        self.mcap_file_io = None
        self.current_mcap_path: Optional[Path] = None
        self.start_time_ns = 0
        self.message_sequence_counts: Dict[int, int] = {}
        self.registered_schemas_by_name: Dict[str, int] = {}
        self.registered_channels_by_topic: Dict[str, int] = {}

        self.data_buffer: Dict[str, Dict[str, Any]] = {}
        self.buffer_lock = threading.Lock()
        
        self.mcap_write_queue = Queue(maxsize=mcap_queue_size)
        self.writer_thread: Optional[threading.Thread] = None
        self._writer_thread_stop_event = threading.Event()

        self.subscribers: List[rclpy.subscription.Subscription] = []
        self.callback_group = ReentrantCallbackGroup() # Allow parallel callbacks

        self.topic_to_msg_type_name: Dict[str, str] = {}
        self.topic_to_msg_class: Dict[str, Any] = {}
        self.topic_to_schema_name: Dict[str, str] = {}
        self.cv_bridge = CvBridge()

        self._parse_topic_types_map()
        self._setup_subscribers()
        
        # Services for start/stop (using placeholder types for now)
        self.start_service = self.create_service(StartRecording, '~/start_recording', self.handle_start_recording)
        self.stop_service = self.create_service(StopRecording, '~/stop_recording', self.handle_stop_recording)

        if self.auto_start:
            # Create a dummy request for auto-start if using Trigger service
            dummy_request = StartRecording.Request()
            # If StartRecording has a filename_prefix field, set it:
            # dummy_request.filename_prefix = self.auto_start_prefix 
            self.handle_start_recording(dummy_request, StartRecording.Response()) 

        # Main timer for bundling and queueing data for MCAP
        if self.recording_hz > 0:
            self.bundling_timer = self.create_timer(1.0 / self.recording_hz, self.bundle_and_queue_data, callback_group=self.callback_group)
        
        self.diagnostic_updater = Updater(self, period=1.0/diagnostics_rate)
        self.diagnostic_updater.setHardwareID("mcap_data_recorder_lbx")
        self.diagnostic_updater.add(RecorderStatusTask("MCAP Recorder Status", self))
        self.get_logger().info("MCAP Recorder Node initialized.")
        if self.auto_start: self.get_logger().info(f"Auto-started recording to: {self.current_mcap_path}")

    def _parse_topic_types_map(self):
        for item in self.topic_types_map_param:
            parts = item.split(':')
            if len(parts) == 2:
                topic_name, msg_type_str = parts[0].strip(), parts[1].strip()
                self.topic_to_msg_type_name[topic_name] = msg_type_str
            else:
                self.get_logger().warn(f"Invalid format in topic_types_map: '{item}'. Expected 'topic_name:package.msg.Type'.")

    def _setup_subscribers(self):
        qos_profile = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        for topic_path in self.configured_topics:
            msg_type_str = self.topic_to_msg_type_name.get(topic_path)
            msg_type_class = None
            schema_name_for_topic = ""

            if msg_type_str:
                msg_type_class = self._get_ros2_message_class(msg_type_str)
                if msg_type_class:
                    # Attempt to map to a well-known schema or custom one
                    schema_name_for_topic = msg_type_str.replace('.msg.','/') # e.g. sensor_msgs/msg/Image -> sensor_msgs/Image
                    self.topic_to_schema_name[topic_path] = schema_name_for_topic
            
            if not msg_type_class: # Fallback to basic inference if not in map or map failed
                self.get_logger().warn(f"Msg type for '{topic_path}' not in topic_types_map or failed to load. Attempting inference.")
                msg_type_class, schema_name_for_topic = self._infer_message_type_and_schema(topic_path)
                if msg_type_class and schema_name_for_topic:
                    self.topic_to_msg_class[topic_path] = msg_type_class # Store the class now
                    self.topic_to_schema_name[topic_path] = schema_name_for_topic
            
            if msg_type_class:
                try:
                    self.subscribers.append(self.create_subscription(
                        msg_type_class, topic_path, 
                        lambda msg, tp=topic_path: self._topic_callback(msg, tp),
                        qos_profile, callback_group=self.callback_group
                    ))
                    self.get_logger().info(f"Subscribed to {topic_path} ({msg_type_class.__name__}, schema: {schema_name_for_topic or 'Default'})")
                except Exception as e: self.get_logger().error(f"Sub fail {topic_path} ({msg_type_class.__name__}): {e}")
            else: self.get_logger().warn(f"Cannot sub to '{topic_path}'. Unknown type.")

    def _infer_message_type_and_schema(self, topic_path: str) -> Tuple[Optional[Any], Optional[str]]:
        # Returns (message_class, schema_name_for_mcap_library)
        if "image_raw" in topic_path: return Image, "sensor_msgs/Image"
        if "camera_info" in topic_path: return CameraInfo, "sensor_msgs/CameraInfo"
        if "points" in topic_path: return PointCloud2, "sensor_msgs/PointCloud2"
        if "joint_states" in topic_path: return JointState, "sensor_msgs/JointState"
        if "joy" in topic_path: return Joy, "sensor_msgs/Joy"
        if "pose" in topic_path and "oculus" in topic_path : return PoseStamped, "geometry_msgs/PoseStamped"
        # Custom messages - these would need to be imported from your lbx_interfaces typically
        if "/robot_state" == topic_path: return String, "labelbox_robotics.RobotState" # Placeholder, using custom schema name
        if "/action" == topic_path: return String, "labelbox_robotics.Action"       # Placeholder
        if "/vr_controller" == topic_path: return String, "labelbox_robotics.VRController" # Placeholder
        if "/tf" == topic_path or "/tf_static" == topic_path: return TransformStamped, "tf2_msgs/TFMessage"
        if "/robot_description" == topic_path: return String, "std_msgs/String"
        return None, None

    def _topic_callback(self, msg: Any, topic_path: str):
        if not self.is_recording: return
        with self.buffer_lock:
            self.data_buffer[topic_path] = {'msg': msg, 'timestamp': self.get_clock().now().nanoseconds}

    def bundle_and_queue_data(self):
        if not self.is_recording or not self.data_buffer: return
        
        current_bundle_time_ns = self.get_clock().now().nanoseconds
        bundle = {'type': 'serialized_mcap_messages', 'log_time_ns': current_bundle_time_ns, 'messages_to_write': []}

        with self.buffer_lock:
            for topic_path, data_entry in self.data_buffer.items():
                bundle['messages_to_write'].append({
                    'topic': topic_path,
                    'data': data_entry['msg'], # The actual ROS message object
                    'schema_name': self.topic_to_schema_name.get(topic_path, ""), # The schema the data conforms to
                    'log_time_ns': current_bundle_time_ns, # Time of bundling
                    'publish_time_ns': data_entry['timestamp'] # When ROS callback received it
                })
            self.data_buffer.clear() # Clear buffer after bundling
        
        if bundle['messages_to_write']:
            try:
                self.mcap_write_queue.put_nowait(bundle)
            except Full:
                self.get_logger().warn("MCAP write queue is full. Data dropped for this interval!")

    def _get_ros2_message_class(self, full_type_str: str) -> Optional[Any]:
        try:
            parts = full_type_str.split('.msg.')
            if len(parts) != 2:
                # Try splitting by / for ros2 topic types like sensor_msgs/Image
                parts = full_type_str.split('/')
                if len(parts) < 2 : # e.g. sensor_msgs/Image or sensor_msgs/msg/Image
                    self.get_logger().error(f"Cannot parse message type string: {full_type_str}")
                    return None
                module_name = parts[0] + '.msg' if len(parts) == 2 else parts[0] + '.' + parts[1]
                class_name = parts[-1]
            else:
                module_name = parts[0] + '.msg'
                class_name = parts[1]
            
            module = __import__(module_name, fromlist=[class_name])
            return getattr(module, class_name)
        except Exception as e:
            self.get_logger().error(f"Failed to import message type {full_type_str}: {e}")
            return None

    def _get_or_register_schema(self, schema_name: str, schema_content: bytes, encoding: str = "jsonschema") -> int:
        if schema_name not in self.registered_schemas_by_name:
            if not self.mcap_writer: raise Exception("MCAP writer not initialized for schema reg")
            try:
                schema_id = self.mcap_writer.register_schema(name=schema_name, encoding=encoding, data=schema_content)
                self.registered_schemas_by_name[schema_name] = schema_id
                self.get_logger().info(f"Registered schema: {schema_name} (ID: {schema_id})")
                return schema_id
            except Exception as e:
                self.get_logger().error(f"Failed to register schema {schema_name}: {e}"); raise
        return self.registered_schemas_by_name[schema_name]

    def _get_or_register_channel(self, topic: str, schema_name: str, msg_encoding="json") -> int: # Default to json for custom schemas
        # This method is primarily for channels that use custom JSON serialization.
        # For standard ROS messages written with McapRos2NativeWriter, channels are handled by write_message().
        if topic not in self.registered_channels_by_topic:
            if not self.mcap_writer: raise Exception("MCAP writer not initialized for channel reg")
            
            schema_id_to_use = self.registered_schemas_by_name.get(schema_name, 0)
            if schema_id_to_use == 0:
                 self.get_logger().error(f"Schema '{schema_name}' (for JSON encoding) not pre-registered for topic '{topic}'. MCAP may be invalid.")
                 # It's critical that schema_name here maps to a JSON schema registered in _get_or_register_schema
                 # If this channel is for a standard ROS type meant for CDR, this manual registration path shouldn't be hit.
            
            try:
                # Use the base writer's register_channel for explicit JSON channel registration
                # McapRos2NativeWriter inherits from mcap.writer.Writer, so this is valid.
                channel_id = self.mcap_writer.register_channel(topic=topic, message_encoding=msg_encoding, schema_id=schema_id_to_use)
                self.registered_channels_by_topic[topic] = channel_id
                self.get_logger().info(f"Registered JSON channel for topic: {topic} (Schema: {schema_name}, Encoding: {msg_encoding}, ID: {channel_id})")
                return channel_id
            except Exception as e:
                self.get_logger().error(f"Failed to register JSON channel for {topic}: {e}"); raise
        return self.registered_channels_by_topic[topic]

    def _serialize_ros_message_to_custom_json(self, msg: Any, schema_name: str, timestamp_ns: int) -> bytes:
        # This is where you transform ROS messages to your custom Labelbox JSON format
        # For brevity, this is a placeholder.
        # You need to implement the actual transformation based on msg type and schema_name.
        # Example for a simplified RobotState:
        data_dict = {"timestamp": {"sec": int(timestamp_ns // 1e9), "nanosec": int(timestamp_ns % 1e9)}}
        if schema_name == "labelbox_robotics.RobotState":
            # Assuming msg is a dict-like or object with these attributes
            data_dict["joint_positions"] = msg.get("joint_positions", []) if isinstance(msg, dict) else getattr(msg, "joint_positions", [])
            data_dict["cartesian_position"] = msg.get("cartesian_position", []) if isinstance(msg, dict) else getattr(msg, "cartesian_position", [])
            data_dict["gripper_position"] = msg.get("gripper_position", 0.0) if isinstance(msg, dict) else getattr(msg, "gripper_position", 0.0)
        elif schema_name == "labelbox_robotics.Action":
            data_dict["data"] = msg.get("data", []) if isinstance(msg, dict) else getattr(msg, "data", [])
        # Add more transformations for VRController, etc.
        # For images, you'd handle JPEG encoding and base64 here if writing to CompressedImage schema
        return json.dumps(data_dict).encode('utf-8')

    def _writer_thread_main(self):
        self.get_logger().info(f"MCAP writer thread started for {Path(self.mcap_writer.output.name).name if hasattr(self.mcap_writer.output, 'name') else 'unknown_file'}")
        while not self._writer_thread_stop_event.is_set() or not self.mcap_write_queue.empty():
            try:
                item = self.mcap_write_queue.get(timeout=0.1)
                if item is None: break
                if item['type'] == 'serialized_mcap_messages' and self.mcap_writer:
                    log_time_ns = item['log_time_ns']
                    for msg_entry in item['messages_to_write']:
                        topic, ros_msg_object, schema_name_hint = msg_entry['topic'], msg_entry['data'], msg_entry['schema_name']
                        ros_pub_time_ns = msg_entry['publish_time_ns']
                        
                        # Check if this topic is intended for custom JSON serialization
                        # This relies on schema_name_hint being one of your custom JSON schema names.
                        is_custom_json_topic = schema_name_hint in ["labelbox_robotics.RobotState", 
                                                               "labelbox_robotics.Action", 
                                                               "labelbox_robotics.VRController", 
                                                               "foxglove.CompressedImage", 
                                                               "foxglove.CameraCalibration"]
                                                               # Add other custom JSON schema names if any

                        if is_custom_json_topic:
                            # Custom JSON serialization path
                            channel_id = self.registered_channels_by_topic.get(topic)
                            if not channel_id:
                                # This implies a custom JSON topic was not pre-registered with _get_or_register_channel
                                # which should have happened in start_new_recording or _setup_subscribers if specific topics are known.
                                self.get_logger().error(f"Channel for custom JSON topic '{topic}' (schema: '{schema_name_hint}') not registered. Skipping.")
                                continue
                            
                            serialized_data = b''
                            if schema_name_hint in ["labelbox_robotics.RobotState", "labelbox_robotics.Action", "labelbox_robotics.VRController"]:
                                serialized_data = self._serialize_ros_message_to_custom_json(ros_msg_object, schema_name_hint, ros_pub_time_ns)
                            elif schema_name_hint == "foxglove.CompressedImage" and isinstance(ros_msg_object, Image):
                                # Assuming ros_msg_object is a sensor_msgs/Image for compressed path
                                try:
                                    _, buffer = cv2.imencode('.jpg', self.cv_bridge.imgmsg_to_cv2(ros_msg_object, "bgr8"), [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
                                    img_data_b64 = base64.b64encode(buffer).decode('utf-8')
                                    json_payload = {"timestamp": {"sec": int(ros_pub_time_ns//1e9), "nanosec": int(ros_pub_time_ns%1e9)},
                                                    "frame_id": ros_msg_object.header.frame_id, "data": img_data_b64, "format": "jpeg"}
                                    serialized_data = json.dumps(json_payload).encode('utf-8')
                                except Exception as e:
                                    self.get_logger().error(f"Error compressing image for {topic}: {e}"); continue
                            # Add other custom JSON serializations if needed (e.g. CameraCalibration)
                            else:
                                self.get_logger().warn(f"Unhandled custom JSON schema '{schema_name_hint}' for topic '{topic}'. Skipping.")
                                continue
                            
                            if serialized_data:
                                seq = self.message_sequence_counts.get(channel_id, 0)
                                try:
                                    # Use add_message for pre-serialized JSON data
                                    self.mcap_writer.add_message(channel_id=channel_id, log_time=log_time_ns, data=serialized_data, publish_time=ros_pub_time_ns, sequence=seq)
                                    self.message_sequence_counts[channel_id] = seq + 1
                                except Exception as e: self.get_logger().error(f"MCAP JSON write error {topic}: {e}")
                        
                        else:
                            # Standard ROS2 message path (use McapRos2NativeWriter.write_message for CDR)
                            try:
                                # McapRos2NativeWriter.write_message handles schema and channel registration for ROS types
                                # It also handles sequence numbers internally per topic.
                                self.mcap_writer.write_message(topic=topic, message=ros_msg_object, log_time=log_time_ns, publish_time=ros_pub_time_ns)
                                # self.get_logger().info(f"Wrote ROS msg to {topic} ({type(ros_msg_object).__name__})")
                            except Exception as e:
                                self.get_logger().error(f"MCAP ROS2 native write error for topic '{topic}' (type: {type(ros_msg_object).__name__}): {e}")

                self.mcap_write_queue.task_done()
            except Empty: 
                if self._writer_thread_stop_event.is_set(): break
                continue
            except Exception as e: self.get_logger().error(f"MCAP writer thread exception: {e}")
        self.get_logger().info(f"MCAP writer thread finished for {self.current_mcap_path.name if self.current_mcap_path else 'N/A'}")

    def start_new_recording(self, filename_prefix: str = "trajectory", extra_metadata: Optional[Dict[str, str]] = None) -> bool:
        if self.is_recording:
            self.get_logger().warn("Recording already in progress. Stop current one first.")
            return False

        self.start_time_ns = self.get_clock().now().nanoseconds
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        # File initially goes to failure, moved on success.
        self.current_mcap_path = self.failure_dir / f"{filename_prefix}_{timestamp_str}.mcap"

        try:
            self.mcap_file_io = open(self.current_mcap_path, "wb")
            # Use McapRos2NativeWriter, which handles ROS2 specific message writing (CDR, schema)
            self.mcap_writer = McapRos2NativeWriter(output=self.mcap_file_io, chunk_size=self.mcap_chunk_size)
            self.mcap_writer.start() # McapRos2NativeWriter.start() doesn't take profile/library, it calls super().start()
            
            # Pre-register custom JSON schemas
            self.registered_schemas_by_name.clear(); self.registered_channels_by_topic.clear()
            self._get_or_register_schema("labelbox_robotics.RobotState", SCHEMA_ROBOT_STATE_STR.encode('utf-8'))
            self._get_or_register_schema("labelbox_robotics.Action", SCHEMA_ACTION_STR.encode('utf-8'))
            self._get_or_register_schema("labelbox_robotics.VRController", SCHEMA_VR_CONTROLLER_STR.encode('utf-8'))
            self._get_or_register_schema("foxglove.CompressedImage", SCHEMA_COMPRESSED_IMAGE_STR.encode('utf-8'))
            self._get_or_register_schema("foxglove.CameraCalibration", SCHEMA_CAMERA_CALIBRATION_STR.encode('utf-8'))
            # Do NOT register JSON schemas for standard ROS types like TFMessage, String, JointState here if McapRos2NativeWriter handles them

            # Register channels for topics that will use custom JSON serialization
            # Example: If /robot_state_custom_json is a topic meant for custom JSON
            # self._get_or_register_channel("/robot_state_custom_json", "labelbox_robotics.RobotState", "json")

            initial_meta = {"robot_type": self.robot_name, "recording_software": "lbx_data_recorder", 
                              "start_time_iso": datetime.now().isoformat(),
                              "ros_distro": os.environ.get("ROS_DISTRO", "unknown")}
            if extra_metadata: initial_meta.update(extra_metadata)
            self.mcap_writer.add_metadata("recording_initial_metadata", initial_meta)

            self.get_logger().info(f"MCAP recording started: {self.current_mcap_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to start MCAP writer: {e}")
            if self.mcap_file_io: self.mcap_file_io.close()
            return False

        self.is_recording = True
        self.message_sequence_counts.clear()
        self._writer_thread_stop_event.clear()
        self.writer_thread = threading.Thread(target=self._writer_thread_main, daemon=True)
        self.writer_thread.start()
        return True

    def stop_recording(self, success: bool = False, final_metadata: Optional[Dict[str, str]] = None) -> Optional[str]:
        if not self.is_recording:
            self.get_logger().warn("No recording in progress to stop.")
            return None

        self.get_logger().info(f"Stopping recording for {self.current_mcap_path.name} (Success: {success})...")
        self.is_recording = False # Signal bundling timer and callbacks to stop queueing
        
        # Signal writer thread to finish processing queue and then stop
        if self.writer_thread and self.writer_thread.is_alive():
            self.get_logger().info("Signaling MCAP writer thread to stop and flush queue...")
            self._writer_thread_stop_event.set()
            if not self.mcap_write_queue.empty(): # Put a final None to ensure it wakes up if waiting
                try: self.mcap_write_queue.put_nowait(None) 
                except Full: pass # If full, it will process existing items then stop
            self.writer_thread.join(timeout=10.0) # Generous timeout for flushing
            if self.writer_thread.is_alive():
                self.get_logger().warn("MCAP writer thread did not stop in time. Some data might be lost.")
        
        final_path = None
        if self.mcap_writer:
            try:
                end_time_ns = self.get_clock().now().nanoseconds
                duration_sec = (end_time_ns - self.start_time_ns) / 1e9
                duration_str = f"{int(duration_sec//60):02d}m{int(duration_sec%60):02d}s"
                new_filename = f"{self.current_mcap_path.stem}_{duration_str}.mcap"
                final_path = self.success_dir / new_filename
                try:
                    self.current_mcap_path.rename(final_path)
                    self.get_logger().info(f"Recording saved successfully: {final_path}")
                except Exception as e:
                    self.get_logger().error(f"Failed to move successful recording: {e}")
                    final_path = str(self.current_mcap_path) # Report original path if move fails
            except Exception as e: self.get_logger().error(f"Error finishing MCAP: {e}")
        if self.mcap_file_io:
            self.mcap_file_io.close()
            self.mcap_file_io = None

        if self.current_mcap_path and self.current_mcap_path.exists():
            if success:
                end_time_ns = self.get_clock().now().nanoseconds
                duration_sec = (end_time_ns - self.start_time_ns) / 1e9
                meta_to_write = {"success": str(success), "duration_seconds": f"{duration_sec:.3f}"}
                if final_metadata: meta_to_write.update(final_metadata)
                self.mcap_writer.add_metadata("recording_final_status", meta_to_write)
                self.mcap_writer.finish()
            else:
                try:
                    self.current_mcap_path.unlink() # Delete failed/discarded recording
                    self.get_logger().info(f"Recording discarded (not successful): {self.current_mcap_path.name}")
                except Exception as e:
                    self.get_logger().error(f"Failed to delete unsuccessful recording: {e}")
        
        self.mcap_writer = None
        self.current_mcap_path = None
        self.start_time_ns = 0
        # Clear queue again in case stop was abrupt
        while not self.mcap_write_queue.empty():
            try: self.mcap_write_queue.get_nowait()
            except Empty: break

        return str(final_path) if final_path else None

    def destroy_node(self):
        self.get_logger().info("Shutting down MCAP Recorder Node...")
        if self.is_recording:
            self.stop_recording(success=False) # Stop any active recording as failure on shutdown
        else: # Ensure writer thread is stopped even if not recording (e.g. if it was started then stopped before destroy)
            self._writer_thread_stop_event.set()
            if self.writer_thread and self.writer_thread.is_alive():
                if not self.mcap_write_queue.empty(): 
                    try: self.mcap_write_queue.put_nowait(None) 
                    except Full: pass
                self.writer_thread.join(timeout=1.0)
        if hasattr(self, 'bundling_timer') and self.bundling_timer: self.bundling_timer.cancel()
        if hasattr(self, 'diagnostic_updater'): self.diagnostic_updater.destroy()
        for sub in self.subscribers: self.destroy_subscription(sub)
        super().destroy_node()
        self.get_logger().info("MCAP Recorder Node shutdown complete.")

class RecorderStatusTask(DiagnosticTask):
    def __init__(self, name, node: MCAPRecorderNode):
        super().__init__(name)
        self.node = node

    def run(self, stat: DiagnosticStatus):
        if self.node.is_recording and self.node.current_mcap_path:
            stat.summary(DiagnosticStatus.OK, f"Recording to {self.node.current_mcap_path.name}")
            stat.add("File Path", str(self.node.current_mcap_path))
            stat.add("Queue Size", str(self.node.mcap_write_queue.qsize()))
            if self.node.start_time_ns > 0:
                duration = (self.node.get_clock().now().nanoseconds - self.node.start_time_ns) / 1e9
                stat.add("Duration (s)", f"{duration:.2f}")
        else:
            stat.summary(DiagnosticStatus.OK, "Idle. Not Recording.")
            stat.add("Queue Size", str(self.node.mcap_write_queue.qsize()))
        
        stat.add("Configured Topics", str(self.node.configured_topics))
        stat.add("Subscribed Topics Count", str(len(self.node.subscribers)))
        return stat

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MCAPRecorderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info('Keyboard interrupt, shutting down recorder.')
    except Exception as e:
        if node: node.get_logger().error(f"Unhandled exception in MCAP Recorder: {e}", exc_info=True)
        else: print(f"Unhandled exception during MCAP Recorder init: {e}", file=sys.stderr)
    finally:
        if node and rclpy.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main() 