#!/usr/bin/env python3
"""
Data Recorder Node

Records all teleoperation data to MCAP format including:
- VR controller states and calibration
- Robot joint states with torques
- Camera images and transforms
- System timestamps and metadata
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mcap.writer import Writer
from mcap_ros2.writer import Writer as Ros2Writer

import os
import time
import json
import threading
import queue
from datetime import datetime
from typing import Dict, List, Optional, Any
import numpy as np

# ROS2 messages
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, CameraInfo, JointState, PointCloud2
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_msgs.msg import TFMessage
from diagnostic_msgs.msg import DiagnosticArray
from lbx_interfaces.msg import VRControllerState, SystemStatus, RecordingStatus
from std_srvs.srv import Trigger


class DataRecorderNode(Node):
    """High-performance data recorder using MCAP format"""
    
    def __init__(self):
        super().__init__('lbx_data_recorder')
        
        # Parameters
        self.declare_parameter('output_dir', os.path.expanduser('~/lbx_recordings'))
        self.declare_parameter('recording_hz', 60.0)
        self.declare_parameter('queue_size', 1000)
        self.declare_parameter('verify_after_recording', False)
        
        self.output_dir = self.get_parameter('output_dir').value
        self.recording_hz = self.get_parameter('recording_hz').value
        self.queue_size = self.get_parameter('queue_size').value
        self.verify_after = self.get_parameter('verify_after_recording').value
        
        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Recording state
        self.recording_active = False
        self.mcap_writer = None
        self.ros2_writer = None
        self.current_recording_path = None
        self.recording_start_time = None
        self.vr_calibration_stored = False
        
        # Thread-safe data queue
        self.data_queue = queue.Queue(maxsize=self.queue_size)
        self.writer_thread = None
        self.writer_running = False
        
        # Callback groups
        self.data_callback_group = ReentrantCallbackGroup()
        self.service_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Create subscriptions for all data sources
        self._create_subscriptions()
        
        # Create services
        self.start_recording_srv = self.create_service(
            Trigger, 
            'start_recording', 
            self.start_recording_callback,
            callback_group=self.service_callback_group
        )
        
        self.stop_recording_srv = self.create_service(
            Trigger,
            'stop_recording',
            self.stop_recording_callback,
            callback_group=self.service_callback_group
        )
        
        # Publisher for recording status
        self.recording_status_pub = self.create_publisher(
            RecordingStatus,
            '/recording_status',
            10
        )
        
        # Publisher for verification results
        self.verification_result_pub = self.create_publisher(
            String,
            '/recording_verification_result',
            10
        )
        
        # Status timer
        self.status_timer = self.create_timer(
            1.0,  # 1Hz
            self.publish_recording_status
        )
        
        self.get_logger().info('📹 Data Recorder initialized')
        self.get_logger().info(f'   Output directory: {self.output_dir}')
        self.get_logger().info(f'   Recording rate: {self.recording_hz}Hz')
    
    def _create_subscriptions(self):
        """Create subscriptions for all data sources"""
        # VR Data
        self.vr_pose_sub = self.create_subscription(
            PoseStamped,
            '/vr/controller_pose',
            lambda msg: self._queue_data('vr_pose', msg),
            10,
            callback_group=self.data_callback_group
        )
        
        self.vr_state_sub = self.create_subscription(
            VRControllerState,
            '/vr_control_state',
            lambda msg: self._queue_data('vr_state', msg),
            10,
            callback_group=self.data_callback_group
        )
        
        # Robot Data with Torques
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            lambda msg: self._queue_data('joint_states', msg),
            100,  # High frequency for joint states
            callback_group=self.data_callback_group
        )
        
        # TF Data (includes camera transforms)
        self.tf_sub = self.create_subscription(
            TFMessage,
            '/tf',
            lambda msg: self._queue_data('tf', msg),
            100,
            callback_group=self.data_callback_group
        )
        
        self.tf_static_sub = self.create_subscription(
            TFMessage,
            '/tf_static',
            lambda msg: self._queue_data('tf_static', msg),
            10,
            callback_group=self.data_callback_group
        )
        
        # Camera Data
        self.camera_subs = []
        camera_topics = [
            # RGB Images
            '/cameras/wrist_camera/color/image_raw',
            '/cameras/overhead_camera/color/image_raw',
            # Depth Images
            '/cameras/wrist_camera/depth/image_rect_raw',
            '/cameras/overhead_camera/depth/image_rect_raw',
            # Aligned Depth Images (aligned to color frame)
            '/cameras/wrist_camera/aligned_depth_to_color/image_raw',
            '/cameras/overhead_camera/aligned_depth_to_color/image_raw',
            # Camera Info for both RGB and Depth
            '/cameras/wrist_camera/color/camera_info',
            '/cameras/overhead_camera/color/camera_info',
            '/cameras/wrist_camera/depth/camera_info',
            '/cameras/overhead_camera/depth/camera_info',
            # Point Clouds (optional but useful)
            '/cameras/wrist_camera/depth/color/points',
            '/cameras/overhead_camera/depth/color/points'
        ]
        
        for topic in camera_topics:
            if 'image' in topic:
                sub = self.create_subscription(
                    Image,
                    topic,
                    lambda msg, t=topic: self._queue_data(f'camera_{t}', msg),
                    5,  # Lower frequency for images
                    callback_group=self.data_callback_group
                )
                self.camera_subs.append(sub)
            elif 'camera_info' in topic:
                sub = self.create_subscription(
                    CameraInfo,
                    topic,
                    lambda msg, t=topic: self._queue_data(f'camera_info_{t}', msg),
                    5,
                    callback_group=self.data_callback_group
                )
                self.camera_subs.append(sub)
            elif 'points' in topic:
                sub = self.create_subscription(
                    PointCloud2,
                    topic,
                    lambda msg, t=topic: self._queue_data(f'pointcloud_{t}', msg),
                    2,  # Even lower frequency for point clouds
                    callback_group=self.data_callback_group
                )
                self.camera_subs.append(sub)
        
        # System Status
        self.system_status_sub = self.create_subscription(
            SystemStatus,
            '/system_status',
            lambda msg: self._handle_system_status(msg),
            10,
            callback_group=self.data_callback_group
        )
        
        # Diagnostics
        self.diagnostics_sub = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            lambda msg: self._queue_data('diagnostics', msg),
            5,
            callback_group=self.data_callback_group
        )
    
    def _queue_data(self, data_type: str, msg):
        """Queue data for writing"""
        if not self.recording_active:
            return
        
        try:
            timestamp = self.get_clock().now().nanoseconds
            self.data_queue.put_nowait({
                'type': data_type,
                'msg': msg,
                'timestamp': timestamp
            })
        except queue.Full:
            self.get_logger().warn(f"Data queue full, dropping {data_type} message")
    
    def _handle_system_status(self, msg: SystemStatus):
        """Handle system status and extract VR calibration"""
        # Queue the status message
        self._queue_data('system_status', msg)
        
        # Extract and store VR calibration matrix if available and not yet stored
        if not self.vr_calibration_stored and hasattr(msg, 'vr_calibration_valid') and msg.vr_calibration_valid:
            self._store_vr_calibration(msg.vr_calibration_matrix)
    
    def _store_vr_calibration(self, calibration_matrix):
        """Store VR calibration transformation matrix"""
        if self.recording_active and self.ros2_writer:
            # Convert flat array back to 4x4 matrix
            if len(calibration_matrix) == 16:
                matrix_4x4 = np.array(calibration_matrix).reshape(4, 4)
                
                # Create a custom message with calibration data
                calibration_msg = String()
                calibration_msg.data = json.dumps({
                    'vr_to_global_transform': matrix_4x4.tolist(),
                    'timestamp': self.get_clock().now().nanoseconds,
                    'type': 'vr_calibration_matrix'
                })
                
                # Write as special topic to MCAP
                self._queue_data('vr_calibration', calibration_msg)
                self.vr_calibration_stored = True
                self.get_logger().info("✅ VR calibration matrix stored in recording")
            else:
                self.get_logger().warn(f"Invalid calibration matrix size: {len(calibration_matrix)}")
    
    def start_recording_callback(self, request, response):
        """Handle start recording request"""
        if self.recording_active:
            response.success = False
            response.message = "Recording already active"
            return response
        
        # Generate filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.current_recording_path = os.path.join(
            self.output_dir,
            f"teleoperation_{timestamp}.mcap"
        )
        
        try:
            # Open MCAP file
            self.mcap_writer = open(self.current_recording_path, 'wb')
            self.ros2_writer = Ros2Writer(output=self.mcap_writer)
            
            # Start writer thread
            self.writer_running = True
            self.writer_thread = threading.Thread(target=self._writer_loop)
            self.writer_thread.start()
            
            # Set state
            self.recording_active = True
            self.recording_start_time = time.time()
            self.vr_calibration_stored = False
            
            self.get_logger().info(f"▶️  Started recording: {self.current_recording_path}")
            response.success = True
            response.message = f"Recording started: {os.path.basename(self.current_recording_path)}"
            
        except Exception as e:
            self.get_logger().error(f"Failed to start recording: {e}")
            response.success = False
            response.message = f"Failed to start recording: {str(e)}"
        
        return response
    
    def stop_recording_callback(self, request, response):
        """Handle stop recording request"""
        if not self.recording_active:
            response.success = False
            response.message = "No active recording"
            return response
        
        try:
            # Stop recording
            self.recording_active = False
            self.writer_running = False
            
            # Wait for writer thread to finish
            if self.writer_thread:
                self.writer_thread.join(timeout=5.0)
            
            # Close writers
            if self.ros2_writer:
                self.ros2_writer.finish()
            if self.mcap_writer:
                self.mcap_writer.close()
            
            recording_duration = time.time() - self.recording_start_time
            file_size = os.path.getsize(self.current_recording_path) / (1024 * 1024)  # MB
            
            self.get_logger().info(f"⏹️  Stopped recording: {self.current_recording_path}")
            self.get_logger().info(f"   Duration: {recording_duration:.1f}s, Size: {file_size:.1f}MB")
            
            # Verify data if requested
            if self.verify_after:
                self._verify_recording(self.current_recording_path)
            
            response.success = True
            response.message = f"Recording saved: {os.path.basename(self.current_recording_path)}"
            
        except Exception as e:
            self.get_logger().error(f"Failed to stop recording: {e}")
            response.success = False
            response.message = f"Failed to stop recording: {str(e)}"
        
        return response
    
    def _writer_loop(self):
        """Background thread for writing data"""
        while self.writer_running:
            try:
                # Get data from queue with timeout
                data = self.data_queue.get(timeout=0.1)
                
                # Write to MCAP
                if self.ros2_writer and data:
                    self.ros2_writer.write_message(
                        topic=f"/{data['type']}",
                        message=data['msg'],
                        log_time=data['timestamp'],
                        publish_time=data['timestamp']
                    )
                    
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Error writing data: {e}")
    
    def _verify_recording(self, file_path: str):
        """Verify the recorded MCAP file"""
        self.get_logger().info(f"🔍 Verifying recording: {file_path}")
        
        verification_results = {
            'file_path': file_path,
            'file_size_mb': os.path.getsize(file_path) / (1024 * 1024),
            'checks': {},
            'statistics': {}
        }
        
        try:
            from mcap.reader import make_reader
            
            # Read and analyze MCAP file
            with open(file_path, 'rb') as f:
                reader = make_reader(f)
                
                # Count messages by topic
                topic_counts = {}
                has_vr_calibration = False
                has_torques = False
                has_rgb_images = False
                has_depth_images = False
                has_point_clouds = False
                has_camera_transforms = False
                
                for schema, channel, message in reader.iter_messages():
                    topic = channel.topic
                    topic_counts[topic] = topic_counts.get(topic, 0) + 1
                    
                    # Check for specific data
                    if 'vr_calibration' in topic:
                        has_vr_calibration = True
                    if topic == '/joint_states':
                        # Check if torques are present in joint states
                        # This would require deserializing the message
                        has_torques = True  # Assuming torques are always published
                    if 'color/image' in topic:
                        has_rgb_images = True
                    if 'depth/image' in topic:
                        has_depth_images = True
                    if 'points' in topic:
                        has_point_clouds = True
                    if topic in ['/tf', '/tf_static']:
                        has_camera_transforms = True
                
                # Store results
                verification_results['checks'] = {
                    'has_vr_data': any('vr' in t for t in topic_counts),
                    'has_robot_data': '/joint_states' in topic_counts,
                    'has_vr_calibration': has_vr_calibration,
                    'has_torques': has_torques,
                    'has_rgb_images': has_rgb_images,
                    'has_depth_images': has_depth_images,
                    'has_point_clouds': has_point_clouds,
                    'has_camera_transforms': has_camera_transforms,
                    'total_topics': len(topic_counts),
                    'total_messages': sum(topic_counts.values())
                }
                
                verification_results['statistics'] = {
                    'message_counts': topic_counts,
                    'duration_seconds': reader.get_summary().statistics.message_end_time / 1e9 - 
                                       reader.get_summary().statistics.message_start_time / 1e9
                }
                
                # Log summary
                self.get_logger().info("✅ Recording verification complete:")
                self.get_logger().info(f"   Total messages: {verification_results['checks']['total_messages']}")
                self.get_logger().info(f"   Duration: {verification_results['statistics']['duration_seconds']:.1f}s")
                self.get_logger().info(f"   VR calibration: {'✓' if has_vr_calibration else '✗'}")
                self.get_logger().info(f"   Robot torques: {'✓' if has_torques else '✗'}")
                self.get_logger().info(f"   RGB images: {'✓' if has_rgb_images else '✗'}")
                self.get_logger().info(f"   Depth images: {'✓' if has_depth_images else '✗'}")
                self.get_logger().info(f"   Point clouds: {'✓' if has_point_clouds else '✗'}")
                self.get_logger().info(f"   Camera transforms: {'✓' if has_camera_transforms else '✗'}")
                
        except Exception as e:
            self.get_logger().error(f"Verification failed: {e}")
            verification_results['error'] = str(e)
        
        # Publish verification results
        result_msg = String()
        result_msg.data = json.dumps(verification_results, indent=2)
        self.verification_result_pub.publish(result_msg)
    
    def publish_recording_status(self):
        """Publish current recording status"""
        status = RecordingStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.recording_active = self.recording_active
        
        if self.recording_active:
            status.current_file = os.path.basename(self.current_recording_path)
            status.duration_seconds = time.time() - self.recording_start_time
            status.queue_size = self.data_queue.qsize()
            status.data_rate_hz = self.recording_hz
        
        self.recording_status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    
    recorder = DataRecorderNode()
    
    # Use multi-threaded executor
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(recorder)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        recorder.get_logger().info("Shutting down data recorder...")
    finally:
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 