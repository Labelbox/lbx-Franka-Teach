import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Joy
import tf2_ros
from transformations import quaternion_from_matrix
import numpy as np
import time
import os
from ament_index_python.packages import get_package_share_directory
import threading
import queue
import sys # For stderr in main

# ROS 2 Diagnostics
from diagnostic_updater import Updater, DiagnosticTask, DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

# Enhanced VR connection handling
VR_CONNECTION_STATUS = {
    'DISCONNECTED': 0,
    'CONNECTING': 1,
    'CONNECTED': 2,
    'ERROR': 3
}

class VRConnectionManager:
    """Handles VR connection with graceful fallback and clear user feedback"""
    
    def __init__(self, logger, connection_mode='usb', ip_address=None, port=5555):
        self.logger = logger
        self.connection_mode = connection_mode
        self.ip_address = ip_address
        self.port = port
        self.status = VR_CONNECTION_STATUS['DISCONNECTED']
        self.oculus_reader = None
        self.last_connection_attempt = 0
        self.connection_retry_interval = 5.0  # seconds
        self.max_retries = 3
        self.retry_count = 0
        
    def print_connection_banner(self):
        """Print VR connection status banner"""
        print("\n" + "═" * 70)
        print("🎮 VR CONNECTION STATUS")
        print("═" * 70)
        
        if self.connection_mode == 'network':
            print(f"📡 Mode: Network ({self.ip_address}:{self.port})")
        else:
            print("🔌 Mode: USB")
            
        print(f"🔍 Status: {self._get_status_text()}")
        
        if self.status == VR_CONNECTION_STATUS['DISCONNECTED']:
            print("\n📋 VR Setup Instructions:")
            if self.connection_mode == 'usb':
                print("   1. Connect Meta Quest via USB cable")
                print("   2. Enable Developer Mode on Quest")
                print("   3. Allow USB debugging when prompted")
                print("   4. Run 'adb devices' to verify connection")
            else:
                print("   1. Enable Developer Mode on Quest")
                print("   2. Connect Quest to same network")
                print("   3. Enable ADB over network in Developer settings")
                print(f"   4. Verify Quest IP address is {self.ip_address}")
            
            print("\n⚠️  VR Control Disabled: System will continue without VR input")
            print("   - Robot control via other interfaces remains available")
            print("   - Recording and other features work normally")
            
        print("═" * 70 + "\n")
    
    def _get_status_text(self):
        """Get human-readable status text with colors"""
        status_map = {
            VR_CONNECTION_STATUS['DISCONNECTED']: "❌ DISCONNECTED",
            VR_CONNECTION_STATUS['CONNECTING']: "🔄 CONNECTING...",
            VR_CONNECTION_STATUS['CONNECTED']: "✅ CONNECTED",
            VR_CONNECTION_STATUS['ERROR']: "⚠️  ERROR"
        }
        return status_map.get(self.status, "❓ UNKNOWN")
    
    def attempt_connection(self):
        """Attempt to connect to VR device with error handling"""
        current_time = time.time()
        
        # Rate limit connection attempts
        if current_time - self.last_connection_attempt < self.connection_retry_interval:
            return False
            
        if self.retry_count >= self.max_retries:
            self.logger.warn("Max VR connection retries reached. VR input disabled.")
            self.status = VR_CONNECTION_STATUS['ERROR']
            return False
            
        self.last_connection_attempt = current_time
        self.retry_count += 1
        self.status = VR_CONNECTION_STATUS['CONNECTING']
        
        try:
            self.logger.info(f"Attempting VR connection ({self.retry_count}/{self.max_retries})...")
            
            # Import OculusReader here to catch import errors gracefully
            from .oculus_reader.reader import OculusReader
            
            self.oculus_reader = OculusReader(
                ip_address=self.ip_address,
                port=self.port,
                print_FPS=False,
                run=True
            )
            
            # Test the connection by trying to get data
            time.sleep(1.0)  # Give it a moment to initialize
            transforms, buttons = self.oculus_reader.get_transformations_and_buttons()
            
            self.status = VR_CONNECTION_STATUS['CONNECTED']
            self.logger.info("✅ VR connection successful!")
            self.retry_count = 0  # Reset retry count on success
            return True
            
        except ImportError as e:
            self.logger.error(f"VR import error: {e}")
            self.logger.error("Check that 'pure-python-adb' is installed: pip install pure-python-adb")
            self.status = VR_CONNECTION_STATUS['ERROR']
            return False
            
        except ConnectionError as e:
            self.logger.warn(f"VR connection failed: {e}")
            self.status = VR_CONNECTION_STATUS['DISCONNECTED']
            return False
            
        except Exception as e:
            self.logger.error(f"Unexpected VR error: {e}")
            self.status = VR_CONNECTION_STATUS['ERROR']
            return False
    
    def get_vr_data(self):
        """Get VR data with connection monitoring"""
        if not self.oculus_reader or self.status != VR_CONNECTION_STATUS['CONNECTED']:
            return None, None
            
        try:
            transforms, buttons = self.oculus_reader.get_transformations_and_buttons()
            
            # Check if we're getting valid data
            if not transforms and not buttons:
                # No data might indicate connection loss
                self.status = VR_CONNECTION_STATUS['DISCONNECTED']
                self.logger.warn("VR data stream lost - connection may have dropped")
                return None, None
                
            return transforms, buttons
            
        except Exception as e:
            self.logger.error(f"Error getting VR data: {e}")
            self.status = VR_CONNECTION_STATUS['ERROR']
            return None, None
    
    def is_connected(self):
        """Check if VR is currently connected"""
        return self.status == VR_CONNECTION_STATUS['CONNECTED']
    
    def cleanup(self):
        """Clean up VR connection"""
        if self.oculus_reader:
            try:
                self.oculus_reader.stop()
            except:
                pass
            self.oculus_reader = None
        self.status = VR_CONNECTION_STATUS['DISCONNECTED']

# Helper function to convert 4x4 matrix to PoseStamped
def matrix_to_pose_stamped(matrix, stamp, frame_id):
    pose = PoseStamped()
    pose.header.stamp = stamp
    pose.header.frame_id = frame_id

    # Translation
    pose.pose.position.x = matrix[0, 3]
    pose.pose.position.y = matrix[1, 3]
    pose.pose.position.z = matrix[2, 3]

    # Orientation (quaternion)
    q = quaternion_from_matrix(matrix)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose

# Helper function to convert 4x4 matrix to TransformStamped
def matrix_to_transform_stamped(matrix, stamp, parent_frame_id, child_frame_id):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent_frame_id
    t.child_frame_id = child_frame_id

    # Translation
    t.transform.translation.x = matrix[0, 3]
    t.transform.translation.y = matrix[1, 3]
    t.transform.translation.z = matrix[2, 3]

    # Orientation
    q = quaternion_from_matrix(matrix)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    return t


class OculusInputNode(Node):
    def __init__(self):
        super().__init__('oculus_input_node')

        # Declare parameters
        self.declare_parameter('connection_mode', 'usb')
        self.declare_parameter('oculus_ip_address', '192.168.0.100')
        self.declare_parameter('oculus_port', 5555)
        self.declare_parameter('publish_rate_hz', 60.0)
        self.declare_parameter('poll_rate_hz', 60.0) # New parameter for Oculus polling speed
        self.declare_parameter('base_frame_id', 'oculus_world')
        self.declare_parameter('left_controller_frame_id', 'oculus_left_controller')
        self.declare_parameter('right_controller_frame_id', 'oculus_right_controller')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('print_fps', False)
        self.declare_parameter('queue_size', 10) # Max size for the internal queue
        self.declare_parameter('enable_graceful_fallback', True)  # New parameter

        # Get parameters
        self.connection_mode = self.get_parameter('connection_mode').get_parameter_value().string_value
        self.oculus_ip = self.get_parameter('oculus_ip_address').get_parameter_value().string_value
        self.oculus_port = self.get_parameter('oculus_port').get_parameter_value().integer_value
        self.publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.poll_rate_hz = self.get_parameter('poll_rate_hz').get_parameter_value().double_value
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self.left_controller_frame_id = self.get_parameter('left_controller_frame_id').get_parameter_value().string_value
        self.right_controller_frame_id = self.get_parameter('right_controller_frame_id').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.print_oculus_fps = self.get_parameter('print_fps').get_parameter_value().bool_value
        self.enable_graceful_fallback = self.get_parameter('enable_graceful_fallback').get_parameter_value().bool_value
        queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1, # Keep depth 1 for publishers, queue handles buffering
            durability=DurabilityPolicy.VOLATILE
        )

        self.left_pose_pub = self.create_publisher(PoseStamped, '~/left_controller_pose', qos_profile)
        self.right_pose_pub = self.create_publisher(PoseStamped, '~/right_controller_pose', qos_profile)
        self.left_joy_pub = self.create_publisher(Joy, '~/left_controller_joy', qos_profile)
        self.right_joy_pub = self.create_publisher(Joy, '~/right_controller_joy', qos_profile)

        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initialize VR connection manager
        reader_ip = self.oculus_ip if self.connection_mode == 'network' else None
        self.vr_manager = VRConnectionManager(
            logger=self.get_logger(),
            connection_mode=self.connection_mode,
            ip_address=reader_ip,
            port=self.oculus_port
        )
        
        # Print connection status banner
        self.vr_manager.print_connection_banner()
        
        # Attempt initial connection
        self.vr_connected = self.vr_manager.attempt_connection()
        
        if not self.vr_connected and not self.enable_graceful_fallback:
            self.get_logger().error("VR connection failed and graceful fallback disabled. Node will exit.")
            raise RuntimeError("VR connection required but failed")
        
        # Setup for dedicated polling thread
        self.data_queue = queue.Queue(maxsize=queue_size)
        self.polling_thread_stop_event = threading.Event()
        self.polling_interval = 1.0 / self.poll_rate_hz if self.poll_rate_hz > 0 else 0.001 # Avoid division by zero
        
        self.actual_poll_rate = 0.0
        self.actual_publish_rate = 0.0
        self._last_poll_time = time.monotonic()
        self._poll_count = 0
        self._last_publish_time = time.monotonic()
        self._publish_count = 0

        self.polling_thread = threading.Thread(target=self._poll_oculus_data_loop, daemon=True)
        self.polling_thread.start()
        
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_data_from_queue)
        
        connection_status = "CONNECTED" if self.vr_connected else "DISCONNECTED (Graceful Fallback)"
        self.get_logger().info(f'Oculus input node started. VR Status: {connection_status}')
        self.get_logger().info(f'ROS Publish Rate: {self.publish_rate} Hz, Polling Rate: {self.poll_rate_hz} Hz')

        # Initialize Diagnostic Updater
        self.diagnostic_updater = Updater(self)
        self.diagnostic_updater.setHardwareID(f"oculus_quest_{self.connection_mode}_{self.oculus_ip if self.connection_mode == 'network' else 'usb'}")
        self.diagnostic_updater.add(OculusConnectionTask("Oculus Connection", self))
        self.diagnostic_updater.add(OculusDataRateTask("Oculus Data Rates", self))

    def _poll_oculus_data_loop(self):
        self.get_logger().info("Oculus polling thread started.")
        poll_start_time = time.monotonic()
        last_connection_check = time.time()
        
        while not self.polling_thread_stop_event.is_set() and rclpy.ok():
            try:
                current_time = time.time()
                
                # Periodically attempt reconnection if disconnected
                if not self.vr_manager.is_connected() and current_time - last_connection_check > 10.0:
                    self.get_logger().info("Attempting VR reconnection...")
                    self.vr_connected = self.vr_manager.attempt_connection()
                    last_connection_check = current_time
                    
                    if self.vr_connected:
                        self.get_logger().info("🎮 VR reconnected successfully!")
                
                # Try to get VR data if connected
                if self.vr_manager.is_connected():
                    transforms, buttons = self.vr_manager.get_vr_data()
                    
                    if transforms is not None or buttons is not None:
                        try:
                            self.data_queue.put_nowait((transforms, buttons))
                        except queue.Full:
                            try: 
                                self.data_queue.get_nowait()
                            except queue.Empty: 
                                pass
                            try: 
                                self.data_queue.put_nowait((transforms, buttons))
                            except queue.Full:
                                self.get_logger().warn("Internal Oculus data queue still full. Data may be lost.")
                        self._poll_count += 1
                    
                    # Update connection status
                    if not self.vr_manager.is_connected():
                        self.vr_connected = False
                        self.get_logger().warn("VR connection lost during polling")

                current_time = time.monotonic()
                if current_time - poll_start_time >= 1.0: # Calculate rate every second
                    self.actual_poll_rate = self._poll_count / (current_time - poll_start_time)
                    self._poll_count = 0
                    poll_start_time = current_time

                time.sleep(self.polling_interval)
                
            except Exception as e:
                self.get_logger().warn(f"Exception in Oculus polling thread: {e}")
                self.vr_connected = False
                time.sleep(1.0) 
        
        self.get_logger().info("Oculus polling thread stopped.")

    def publish_data_from_queue(self):
        try:
            transforms, buttons = self.data_queue.get_nowait()
            self.data_queue.task_done() # Signal that item was processed
        except queue.Empty:
            # Publish empty/default data to indicate VR is disconnected
            if not self.vr_connected:
                self._publish_disconnected_status()
            return # No new data to publish

        current_time_msg = self.get_clock().now().to_msg()

        if transforms: # Check if transforms dict is not None and not empty
            if 'l' in transforms:
                left_matrix = transforms['l']
                left_pose_msg = matrix_to_pose_stamped(left_matrix, current_time_msg, self.base_frame_id)
                self.left_pose_pub.publish(left_pose_msg)
                if self.publish_tf:
                    left_tf_msg = matrix_to_transform_stamped(left_matrix, current_time_msg, self.base_frame_id, self.left_controller_frame_id)
                    self.tf_broadcaster.sendTransform(left_tf_msg)
            
            if 'r' in transforms:
                right_matrix = transforms['r']
                right_pose_msg = matrix_to_pose_stamped(right_matrix, current_time_msg, self.base_frame_id)
                self.right_pose_pub.publish(right_pose_msg)
                if self.publish_tf:
                    right_tf_msg = matrix_to_transform_stamped(right_matrix, current_time_msg, self.base_frame_id, self.right_controller_frame_id)
                    self.tf_broadcaster.sendTransform(right_tf_msg)

        if buttons: # Check if buttons dict is not None and not empty
            if any(key.startswith('L') or key == 'leftJS' or key == 'leftGrip' or key == 'leftTrig' for key in buttons.keys()):
                left_joy_msg = self.create_joy_message(buttons, 'L', current_time_msg)
                self.left_joy_pub.publish(left_joy_msg)

            if any(key.startswith('R') or key == 'rightJS' or key == 'rightGrip' or key == 'rightTrig' for key in buttons.keys()):
                right_joy_msg = self.create_joy_message(buttons, 'R', current_time_msg)
                self.right_joy_pub.publish(right_joy_msg)
        
        self._publish_count += 1
        current_time = time.monotonic()
        if current_time - self._last_publish_time >= 1.0:
            self.actual_publish_rate = self._publish_count / (current_time - self._last_publish_time)
            self._publish_count = 0
            self._last_publish_time = current_time

    def _publish_disconnected_status(self):
        """Publish default/empty data to indicate VR disconnection"""
        current_time_msg = self.get_clock().now().to_msg()
        
        # Publish zero pose for controllers
        zero_pose = PoseStamped()
        zero_pose.header.stamp = current_time_msg
        zero_pose.header.frame_id = self.base_frame_id
        # Position and orientation remain at default (0,0,0) and (0,0,0,1)
        zero_pose.pose.orientation.w = 1.0  # Valid quaternion
        
        self.left_pose_pub.publish(zero_pose)
        self.right_pose_pub.publish(zero_pose)
        
        # Publish empty joy messages
        empty_joy = Joy()
        empty_joy.header.stamp = current_time_msg
        empty_joy.header.frame_id = self.left_controller_frame_id
        empty_joy.axes = [0.0, 0.0, 0.0, 0.0]
        empty_joy.buttons = [0, 0, 0, 0, 0, 0]
        
        self.left_joy_pub.publish(empty_joy)
        
        empty_joy.header.frame_id = self.right_controller_frame_id
        self.right_joy_pub.publish(empty_joy)

    def create_joy_message(self, buttons_data, prefix, stamp):
        joy_msg = Joy()
        joy_msg.header.stamp = stamp
        joy_msg.header.frame_id = self.left_controller_frame_id if prefix == 'L' else self.right_controller_frame_id

        axes = []
        buttons = []

        js_key = 'leftJS' if prefix == 'L' else 'rightJS'
        if js_key in buttons_data and isinstance(buttons_data[js_key], (list, tuple)) and len(buttons_data[js_key]) == 2:
            axes.extend(buttons_data[js_key])
        else:
            axes.extend([0.0, 0.0]) 

        grip_key = 'leftGrip' if prefix == 'L' else 'rightGrip'
        trig_key = 'leftTrig' if prefix == 'L' else 'rightTrig'
        
        axes.append(float(buttons_data.get(grip_key, 0.0)))
        axes.append(float(buttons_data.get(trig_key, 0.0)))

        joy_msg.axes = axes
        
        button_map_l = ['X', 'Y', 'LJ', 'LThU', 'LG', 'LTr']
        button_map_r = ['A', 'B', 'RJ', 'RThU', 'RG', 'RTr']
        button_map = button_map_l if prefix == 'L' else button_map_r

        for btn_key in button_map:
            buttons.append(1 if buttons_data.get(btn_key, False) else 0)
            
        joy_msg.buttons = buttons
        return joy_msg

    def destroy_node(self):
        self.get_logger().info("Shutting down Oculus input node...")
        self.polling_thread_stop_event.set() # Signal polling thread to stop
        
        if hasattr(self, 'polling_thread') and self.polling_thread.is_alive():
            self.get_logger().info("Waiting for Oculus polling thread to join...")
            self.polling_thread.join(timeout=1.0) # Wait for the thread with a timeout
            if self.polling_thread.is_alive():
                self.get_logger().warn("Oculus polling thread did not join in time.")

        # Clean up VR connection
        self.vr_manager.cleanup()
        
        if hasattr(self, 'timer') and self.timer:
            self.timer.cancel()
        
        # Ensure queue is empty to allow task_done() calls to unblock any potential .join() on queue
        if hasattr(self, 'data_queue'):
            while not self.data_queue.empty():
                try:
                    self.data_queue.get_nowait()
                    self.data_queue.task_done()
                except queue.Empty:
                    break
        
        super().destroy_node()
        self.get_logger().info("Oculus input node shutdown complete.")

# Diagnostic Tasks (updated for graceful handling)
class OculusConnectionTask(DiagnosticTask):
    def __init__(self, name, node_instance):
        super().__init__(name)
        self.node = node_instance

    def run(self, stat: DiagnosticStatus):
        if self.node.vr_manager.is_connected():
            stat.summary(DiagnosticStatus.OK, "Oculus device connected and responding.")
            stat.add("Device IP", str(self.node.oculus_ip) if self.node.connection_mode == 'network' else "USB")
            stat.add("Device Port", str(self.node.oculus_port) if self.node.connection_mode == 'network' else "N/A")
            stat.add("Retry Count", "0")
        elif self.node.vr_manager.status == VR_CONNECTION_STATUS['CONNECTING']:
            stat.summary(DiagnosticStatus.WARN, "Attempting to connect to Oculus device...")
            stat.add("Status", "Connecting")
            stat.add("Retry Count", str(self.node.vr_manager.retry_count))
        elif self.node.vr_manager.status == VR_CONNECTION_STATUS['ERROR']:
            stat.summary(DiagnosticStatus.ERROR, "Oculus device connection error.")
            stat.add("Status", "Error - check device and ADB setup")
            stat.add("Retry Count", str(self.node.vr_manager.retry_count))
        else:
            stat.summary(DiagnosticStatus.WARN, "Oculus device disconnected (graceful fallback active).")
            stat.add("Status", "Disconnected - will retry connection")
            stat.add("Graceful Fallback", "Enabled" if self.node.enable_graceful_fallback else "Disabled")
        return stat

class OculusDataRateTask(DiagnosticTask):
    def __init__(self, name, node_instance):
        super().__init__(name)
        self.node = node_instance

    def run(self, stat: DiagnosticStatus):
        stat.summary(DiagnosticStatus.OK, "Reporting data rates.")
        stat.add("Target Poll Rate (Hz)", f"{self.node.poll_rate_hz:.2f}")
        stat.add("Actual Poll Rate (Hz)", f"{self.node.actual_poll_rate:.2f}")
        stat.add("Target Publish Rate (Hz)", f"{self.node.publish_rate:.2f}")
        stat.add("Actual Publish Rate (Hz)", f"{self.node.actual_publish_rate:.2f}")
        stat.add("Data Queue Size", str(self.node.data_queue.qsize()))
        stat.add("VR Connected", "Yes" if self.node.vr_manager.is_connected() else "No")
        
        if not self.node.vr_manager.is_connected():
            stat.mergeSummary(DiagnosticStatus.WARN, "VR device not connected - publishing default data.")
        elif abs(self.node.actual_poll_rate - self.node.poll_rate_hz) > self.node.poll_rate_hz * 0.2: # More than 20% deviation
            stat.mergeSummary(DiagnosticStatus.WARN, "Polling rate significantly different from target.")
        elif abs(self.node.actual_publish_rate - self.node.publish_rate) > self.node.publish_rate * 0.2:
            stat.mergeSummary(DiagnosticStatus.WARN, "Publish rate significantly different from target.")
        return stat

def main(args=None):
    rclpy.init(args=args)
    node = None # Initialize node to None for robust error handling in finally block
    try:
        node = OculusInputNode()
        
        if rclpy.ok():
            rclpy.spin(node)
    except KeyboardInterrupt:
        if node: 
            node.get_logger().info('Keyboard interrupt, shutting down.')
        else: 
            print('Keyboard interrupt before node initialization.')
    except Exception as e:
        if node:
            node.get_logger().error(f"Unhandled exception in main: {e}", exc_info=True)
        else:
            print(f"Unhandled exception before node initialization: {e}", file=sys.stderr)
    finally:
        if node and rclpy.ok(): # Check if node was successfully initialized enough to have destroy_node
            if hasattr(node, 'destroy_node'): # Ensure destroy_node method exists
                 node.destroy_node()
        if rclpy.ok(): # Ensure rclpy is still initialized before shutdown
             rclpy.shutdown()

if __name__ == '__main__':
    main() 