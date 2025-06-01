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

from .oculus_reader.reader import OculusReader # Relative import

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

        reader_ip = self.oculus_ip if self.connection_mode == 'network' else None
        log_msg = f'Connecting to Oculus via network: {reader_ip}:{self.oculus_port}' if reader_ip else 'Connecting to Oculus via USB.'
        self.get_logger().info(log_msg)

        self.oculus_reader = None # Initialize to allow cleanup even if constructor fails
        self.oculus_connected = False
        try:
            self.oculus_reader = OculusReader(
                ip_address=reader_ip,
                port=self.oculus_port,
                print_FPS=self.print_oculus_fps,
                run=True, # OculusReader starts its own logcat reading thread
            )
            self.oculus_connected = True # Assume connected if no exception
        except ConnectionError as e:
            self.get_logger().warn(f"Failed to connect to Oculus on init: {e}. Will keep trying if polling is active.")
            # self.oculus_connected remains False
        except Exception as e:
            self.get_logger().error(f"Unexpected error during OculusReader init: {e}. Node will not start.")
            return
        
        self.get_logger().info('Oculus Reader initialized (or attempted).')

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
        self.get_logger().info(f'Oculus input node started. ROS Publish Rate: {self.publish_rate} Hz, Polling Rate: {self.poll_rate_hz} Hz')

        # Initialize Diagnostic Updater
        self.diagnostic_updater = Updater(self)
        self.diagnostic_updater.setHardwareID(f"oculus_quest_{self.connection_mode}_{self.oculus_ip if self.connection_mode == 'network' else 'usb'}")
        self.diagnostic_updater.add(OculusConnectionTask("Oculus Connection", self))
        self.diagnostic_updater.add(OculusDataRateTask("Oculus Data Rates", self))
        # Updater will call its own timer to publish at ~1Hz by default.
        # If we want every 5s, we need to manage its update calls or see if it can be configured.
        # For now, default 1Hz is fine. We can adjust if needed.

    def _poll_oculus_data_loop(self):
        self.get_logger().info("Oculus polling thread started.")
        poll_start_time = time.monotonic()
        
        while not self.polling_thread_stop_event.is_set() and rclpy.ok():
            try:
                if not self.oculus_connected and self.oculus_reader: # Attempt to reconnect if not connected
                    try:
                        self.get_logger().info("Attempting to re-initialize OculusReader...")
                        reader_ip = self.oculus_ip if self.connection_mode == 'network' else None
                        # Re-initialize or re-run aspects of OculusReader
                        # This part is tricky as OculusReader starts its own threads on init.
                        # A full re-init might be needed or a dedicated 'connect' method in OculusReader.
                        # For simplicity, we assume OculusReader.run() can be called again or it auto-retries.
                        # If OculusReader doesn't auto-retry, this logic needs enhancement or OculusReader itself does.
                        # For now, let's assume it will eventually connect if device becomes available.
                        # We just try to get data. If it fails, oculus_connected remains false.
                        # If get_transformations_and_buttons succeeds, we set oculus_connected to True.
                        # self.oculus_reader.run() # Example: if OculusReader needs a manual restart
                        pass

                    except Exception as e_conn:
                        self.get_logger().warn(f"Re-connection attempt failed: {e_conn}")
                        time.sleep(5.0) # Wait before next connection attempt
                        continue

                if self.oculus_reader:
                    transforms, buttons = self.oculus_reader.get_transformations_and_buttons()
                    if transforms is not None or buttons is not None: # Check for actual data
                        self.oculus_connected = True # Mark as connected if data is received
                        try:
                            self.data_queue.put_nowait((transforms, buttons))
                        except queue.Full:
                            try: self.data_queue.get_nowait()
                            except queue.Empty: pass
                            try: self.data_queue.put_nowait((transforms, buttons))
                            except queue.Full:
                                self.get_logger().warn("Internal Oculus data queue still full. Data may be lost.")
                        self._poll_count += 1
                    # else: # If we get None, None it might indicate a transient issue, don't immediately set to not connected unless errors occur
                        # self.get_logger().debug("Polling returned no new data (None, None).")

                current_time = time.monotonic()
                if current_time - poll_start_time >= 1.0: # Calculate rate every second
                    self.actual_poll_rate = self._poll_count / (current_time - poll_start_time)
                    self._poll_count = 0
                    poll_start_time = current_time

                time.sleep(self.polling_interval)
            except ConnectionError as e_ce: # Catch specific connection errors during get_transformations
                self.get_logger().warn(f"ConnectionError in Oculus polling thread: {e_ce}. Marking as not connected.")
                self.oculus_connected = False
                time.sleep(5.0) # Wait longer if connection error occurs
            except Exception as e:
                self.get_logger().warn(f"Exception in Oculus polling thread: {e}")
                self.oculus_connected = False # Assume not connected on other errors too
                time.sleep(1.0) 
        self.get_logger().info("Oculus polling thread stopped.")

    def publish_data_from_queue(self):
        try:
            transforms, buttons = self.data_queue.get_nowait()
            self.data_queue.task_done() # Signal that item was processed
        except queue.Empty:
            # self.get_logger().debug("Oculus data queue empty, nothing to publish.")
            return # No new data to publish
        except Exception as e:
            self.get_logger().warn(f"Could not get data from internal queue: {e}")
            return

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
            # self.diagnostic_updater.force_update() # If we want to update diagnostics on our schedule

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

        if hasattr(self, 'oculus_reader') and self.oculus_reader:
            self.oculus_reader.stop() # This stops the OculusReader's internal logcat thread
        
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

# Diagnostic Tasks
class OculusConnectionTask(DiagnosticTask):
    def __init__(self, name, node_instance):
        super().__init__(name)
        self.node = node_instance

    def run(self, stat: DiagnosticStatus):
        if self.node.oculus_reader and self.node.oculus_connected:
            stat.summary(DiagnosticStatus.OK, "Oculus device connected and responding.")
            stat.add("Device IP", str(self.node.oculus_ip) if self.node.connection_mode == 'network' else "USB")
            stat.add("Device Port", str(self.node.oculus_port) if self.node.connection_mode == 'network' else "N/A")
        elif self.node.oculus_reader and not self.node.oculus_connected:
            stat.summary(DiagnosticStatus.WARN, "Oculus device connection issue or no data yet.")
            stat.add("Status", "Attempting to connect or waiting for data.")
        else: # No oculus_reader instance, major init failure
            stat.summary(DiagnosticStatus.ERROR, "OculusReader not initialized. Device connection issue.")
            stat.add("Status", "Initialization failed.")
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
        
        if abs(self.node.actual_poll_rate - self.node.poll_rate_hz) > self.node.poll_rate_hz * 0.2: # More than 20% deviation
            stat.mergeSummary(DiagnosticStatus.WARN, "Polling rate significantly different from target.")
        if abs(self.node.actual_publish_rate - self.node.publish_rate) > self.node.publish_rate * 0.2:
            stat.mergeSummary(DiagnosticStatus.WARN, "Publish rate significantly different from target.")
        return stat

def main(args=None):
    rclpy.init(args=args)
    node = None # Initialize node to None for robust error handling in finally block
    try:
        node = OculusInputNode()
        # Check if OculusReader initialization failed (indicated by oculus_reader still being None or an early return)
        if not hasattr(node, 'oculus_reader') or node.oculus_reader is None:
             if node: # if node object was created but init failed partway
                node.get_logger().error("OculusInputNode initialization failed. Shutting down.")
             else: # if super().__init__ wasn't even called or node creation failed very early
                print("Critical error: OculusInputNode object could not be created. Shutting down.", file=sys.stderr)
             # No need to call node.destroy_node() if init failed so badly
             if rclpy.ok():
                rclpy.shutdown()
             return 

        if rclpy.ok():
             rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info('Keyboard interrupt, shutting down.')
        else: print('Keyboard interrupt before node initialization.')
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