import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Joy
import tf2_ros
from tf_transformations import quaternion_from_matrix, euler_from_matrix
import numpy as np
import time
import os
from ament_index_python.packages import get_package_share_directory

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
        self.declare_parameter('connection_mode', 'usb')  # 'usb' or 'network'
        self.declare_parameter('oculus_ip_address', '192.168.0.100') # Example IP
        self.declare_parameter('oculus_port', 5555)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('base_frame_id', 'oculus_base') # Base frame for Oculus data
        self.declare_parameter('left_controller_frame_id', 'oculus_left_controller')
        self.declare_parameter('right_controller_frame_id', 'oculus_right_controller')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('print_fps', False) # For OculusReader internal FPS counter

        # Get parameters
        self.connection_mode = self.get_parameter('connection_mode').get_parameter_value().string_value
        self.oculus_ip = self.get_parameter('oculus_ip_address').get_parameter_value().string_value
        self.oculus_port = self.get_parameter('oculus_port').get_parameter_value().integer_value
        self.publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self.left_controller_frame_id = self.get_parameter('left_controller_frame_id').get_parameter_value().string_value
        self.right_controller_frame_id = self.get_parameter('right_controller_frame_id').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.print_oculus_fps = self.get_parameter('print_fps').get_parameter_value().bool_value
        
        # Construct full path to APK within the package share directory
        self.apk_path = os.path.join(
            get_package_share_directory('lbx_input_oculus'),
            'oculus_reader', 'APK', 'teleop-debug.apk'
        )
        self.get_logger().info(f"Expected APK path: {self.apk_path}")


        # Setup QoS profile for publishers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE # Oculus data is transient
        )

        # Publishers for controller poses
        self.left_pose_pub = self.create_publisher(PoseStamped, '~/left_controller_pose', qos_profile)
        self.right_pose_pub = self.create_publisher(PoseStamped, '~/right_controller_pose', qos_profile)

        # Publishers for controller buttons and joysticks (Joy messages)
        self.left_joy_pub = self.create_publisher(Joy, '~/left_controller_joy', qos_profile)
        self.right_joy_pub = self.create_publisher(Joy, '~/right_controller_joy', qos_profile)

        # TF broadcaster if enabled
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initialize OculusReader
        reader_ip = None
        if self.connection_mode == 'network':
            reader_ip = self.oculus_ip
            self.get_logger().info(f'Connecting to Oculus via network: {reader_ip}:{self.oculus_port}')
        else:
            self.get_logger().info('Connecting to Oculus via USB.')

        try:
            self.oculus_reader = OculusReader(
                ip_address=reader_ip,
                port=self.oculus_port,
                print_FPS=self.print_oculus_fps,
                run=True, # Start reading immediately
                apk_path_override=self.apk_path
            )
        except FileNotFoundError as e:
            self.get_logger().error(f"Oculus APK file not found: {e}")
            rclpy.shutdown() # Or handle more gracefully
            return
        except ConnectionError as e:
            self.get_logger().error(f"Failed to connect to Oculus: {e}")
            rclpy.shutdown()
            return
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred during OculusReader initialization: {e}")
            rclpy.shutdown()
            return


        self.get_logger().info('Oculus Reader initialized successfully.')
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_data)
        self.get_logger().info(f'Oculus input node started. Publishing rate: {self.publish_rate} Hz')

    def publish_data(self):
        try:
            transforms, buttons = self.oculus_reader.get_transformations_and_buttons()
        except Exception as e:
            self.get_logger().warn(f"Could not get data from Oculus: {e}")
            return

        current_time = self.get_clock().now().to_msg()

        if transforms:
            if 'l' in transforms:
                left_matrix = transforms['l']
                left_pose_msg = matrix_to_pose_stamped(left_matrix, current_time, self.base_frame_id)
                self.left_pose_pub.publish(left_pose_msg)
                if self.publish_tf:
                    left_tf_msg = matrix_to_transform_stamped(left_matrix, current_time, self.base_frame_id, self.left_controller_frame_id)
                    self.tf_broadcaster.sendTransform(left_tf_msg)
            
            if 'r' in transforms:
                right_matrix = transforms['r']
                right_pose_msg = matrix_to_pose_stamped(right_matrix, current_time, self.base_frame_id)
                self.right_pose_pub.publish(right_pose_msg)
                if self.publish_tf:
                    right_tf_msg = matrix_to_transform_stamped(right_matrix, current_time, self.base_frame_id, self.right_controller_frame_id)
                    self.tf_broadcaster.sendTransform(right_tf_msg)

        if buttons:
            # Process and publish Joy messages for left controller
            if any(key.startswith('L') or key == 'leftJS' or key == 'leftGrip' or key == 'leftTrig' for key in buttons.keys()):
                left_joy_msg = self.create_joy_message(buttons, 'L', current_time)
                self.left_joy_pub.publish(left_joy_msg)

            # Process and publish Joy messages for right controller
            if any(key.startswith('R') or key == 'rightJS' or key == 'rightGrip' or key == 'rightTrig' for key in buttons.keys()):
                right_joy_msg = self.create_joy_message(buttons, 'R', current_time)
                self.right_joy_pub.publish(right_joy_msg)

    def create_joy_message(self, buttons_data, prefix, stamp):
        joy_msg = Joy()
        joy_msg.header.stamp = stamp
        joy_msg.header.frame_id = self.left_controller_frame_id if prefix == 'L' else self.right_controller_frame_id

        axes = []
        buttons = []

        # Joysticks (typically 2 axes: x, y)
        js_key = 'leftJS' if prefix == 'L' else 'rightJS'
        if js_key in buttons_data:
            axes.extend(buttons_data[js_key]) # Should be a tuple (x, y)
        else:
            axes.extend([0.0, 0.0]) # Default if no joystick data

        # Triggers (float values, add as axes)
        grip_key = 'leftGrip' if prefix == 'L' else 'rightGrip'
        trig_key = 'leftTrig' if prefix == 'L' else 'rightTrig'
        
        axes.append(buttons_data.get(grip_key, 0.0)) # Grip trigger value
        axes.append(buttons_data.get(trig_key, 0.0)) # Index trigger value

        joy_msg.axes = axes

        # Boolean buttons (order matters for consistency if mapping to specific joy indices)
        # Left Controller: X, Y, LJ (Joystick Press), LThU (Thumb Up), LG (Grip Bool), LTr (Trigger Bool)
        # Right Controller: A, B, RJ (Joystick Press), RThU (Thumb Up), RG (Grip Bool), RTr (Trigger Bool)
        
        if prefix == 'L':
            buttons.append(1 if buttons_data.get('X', False) else 0)
            buttons.append(1 if buttons_data.get('Y', False) else 0)
            buttons.append(1 if buttons_data.get('LJ', False) else 0)
            buttons.append(1 if buttons_data.get('LThU', False) else 0) # Thumb rest / up
            buttons.append(1 if buttons_data.get('LG', False) else 0)   # Grip button (boolean)
            buttons.append(1 if buttons_data.get('LTr', False) else 0)  # Trigger button (boolean)
        elif prefix == 'R':
            buttons.append(1 if buttons_data.get('A', False) else 0)
            buttons.append(1 if buttons_data.get('B', False) else 0)
            buttons.append(1 if buttons_data.get('RJ', False) else 0)
            buttons.append(1 if buttons_data.get('RThU', False) else 0) # Thumb rest / up
            buttons.append(1 if buttons_data.get('RG', False) else 0)   # Grip button (boolean)
            buttons.append(1 if buttons_data.get('RTr', False) else 0)  # Trigger button (boolean)
            
        joy_msg.buttons = buttons
        return joy_msg


    def destroy_node(self):
        self.get_logger().info("Shutting down Oculus input node...")
        if hasattr(self, 'oculus_reader') and self.oculus_reader:
            self.oculus_reader.stop()
        if hasattr(self, 'timer') and self.timer: # Ensure timer exists before trying to cancel
            self.timer.cancel()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OculusInputNode()
        if rclpy.ok(): # Check if node initialization was successful
             rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    except Exception as e:
        if node:
            node.get_logger().error(f"Unhandled exception in main: {e}")
        else:
            print(f"Unhandled exception before node initialization: {e}")
    finally:
        if node and rclpy.ok() and node.oculus_reader: # Ensure node and reader exist
             node.destroy_node() # Calls reader.stop()
        if rclpy.ok():
             rclpy.shutdown()

if __name__ == '__main__':
    main() 