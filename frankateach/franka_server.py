import os
from pathlib import Path
import pickle
import time
import numpy as np
import zmq

from deoxys.utils import YamlConfig
from deoxys.utils import transform_utils
from deoxys.utils.config_utils import (
    get_default_controller_config,
    verify_controller_config,
)

# Import protobuf messages
import deoxys.proto.franka_interface.franka_robot_state_pb2 as franka_robot_state_pb2

from frankateach.utils import notify_component_start
from frankateach.network import create_response_socket
from frankateach.messages import FrankaAction, FrankaState
from frankateach.constants import (
    CONTROL_PORT,
    HOST,
    CONTROL_FREQ,
)

CONFIG_ROOT = Path(__file__).parent / "configs"


class FrankaServer:
    def __init__(self, cfg):
        self._robot = Robot(cfg, CONTROL_FREQ)
        # Action REQ/REP
        self.action_socket = create_response_socket(HOST, CONTROL_PORT)

    def init_server(self):
        # connect to robot
        print("Starting Franka server...")
        self._robot.reset_robot()
        self.control_daemon()

    def get_state(self):
        quat, pos = self._robot.last_eef_quat_and_pos
        gripper = self._robot.last_gripper_action
        joint_positions = self._robot.last_q  # Get joint positions
        
        if quat is not None and pos is not None and gripper is not None:
            state = FrankaState(
                pos=pos.flatten().astype(np.float32),
                quat=quat.flatten().astype(np.float32),
                gripper=gripper,
                timestamp=time.time(),
                joint_positions=np.array(joint_positions).astype(np.float32) if joint_positions is not None else None,
            )
            return bytes(pickle.dumps(state, protocol=-1))
        else:
            return b"state_error"

    def control_daemon(self):
        notify_component_start(component_name="Franka Control Subscriber")
        try:
            while True:
                command = self.action_socket.recv()
                if command == b"get_state":
                    self.action_socket.send(self.get_state())
                else:
                    franka_control: FrankaAction = pickle.loads(command)
                    if franka_control.reset:
                        self._robot.reset_joints(gripper_open=franka_control.gripper)
                        time.sleep(1)
                    else:
                        self._robot.osc_move(
                            franka_control.pos,
                            franka_control.quat,
                            franka_control.gripper,
                        )
                    self.action_socket.send(self.get_state())
        except KeyboardInterrupt:
            pass
        finally:
            self._robot.close()
            self.action_socket.close()


class Robot:
    def __init__(self, cfg, control_freq):
        # Load configuration
        self.config = YamlConfig(os.path.join(CONFIG_ROOT, cfg)).as_easydict()
        self.control_freq = control_freq
        
        # Initialize ZMQ subscribers for deoxys services
        self.context = zmq.Context()
        
        # Arm state subscriber (connects to deoxys franka-interface on port 5570)
        self.arm_subscriber = self.context.socket(zmq.SUB)
        self.arm_subscriber.setsockopt(zmq.CONFLATE, 1)
        self.arm_subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
        self.arm_subscriber.connect(f"tcp://localhost:{self.config.NUC.PUB_PORT}")
        
        # Gripper state subscriber (connects to deoxys gripper-interface on port 5572)
        self.gripper_subscriber = self.context.socket(zmq.SUB)
        self.gripper_subscriber.setsockopt(zmq.CONFLATE, 1)
        self.gripper_subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
        self.gripper_subscriber.connect(f"tcp://localhost:{self.config.NUC.GRIPPER_PUB_PORT}")
        
        # Control publisher (publishes to deoxys on port 5571)
        self.arm_publisher = self.context.socket(zmq.PUB)
        self.arm_publisher.bind(f"tcp://*:{self.config.NUC.SUB_PORT}")
        
        # Gripper control publisher (publishes to deoxys on port 5573)
        self.gripper_publisher = self.context.socket(zmq.PUB)
        self.gripper_publisher.bind(f"tcp://*:{self.config.NUC.GRIPPER_SUB_PORT}")
        
        # State variables
        self.last_eef_quat_and_pos = (None, None)
        self.last_gripper_action = None
        self.last_q = None
        self.received_states = False
        
        # Controller config
        self.velocity_controller_cfg = verify_controller_config(
            YamlConfig(
                os.path.join(CONFIG_ROOT, "osc-pose-controller.yml")
            ).as_easydict()
        )
        self.last_gripper_dim = 6

    def reset_robot(self):
        print("Waiting for the robot to connect...")
        
        # Start state receiving threads
        import threading
        
        def arm_state_receiver():
            while True:
                try:
                    message = self.arm_subscriber.recv(flags=zmq.NOBLOCK)
                    robot_state = franka_robot_state_pb2.FrankaRobotStateMessage()
                    robot_state.ParseFromString(message)
                    
                    # Extract end-effector pose from 4x4 transformation matrix
                    # O_T_EE is a flattened 4x4 matrix: [r11,r12,r13,tx, r21,r22,r23,ty, r31,r32,r33,tz, 0,0,0,1]
                    ee_pos = np.array([
                        robot_state.O_T_EE[3],   # tx
                        robot_state.O_T_EE[7],   # ty  
                        robot_state.O_T_EE[11],  # tz
                    ]).reshape(3, 1)
                    
                    # Extract 3x3 rotation matrix and convert to quaternion
                    rot_mat = np.array([
                        [robot_state.O_T_EE[0], robot_state.O_T_EE[1], robot_state.O_T_EE[2]],
                        [robot_state.O_T_EE[4], robot_state.O_T_EE[5], robot_state.O_T_EE[6]],
                        [robot_state.O_T_EE[8], robot_state.O_T_EE[9], robot_state.O_T_EE[10]]
                    ])
                    ee_quat = transform_utils.mat2quat(rot_mat).reshape(4, 1)
                    
                    # Extract joint positions
                    joint_pos = list(robot_state.q)
                    
                    self.last_eef_quat_and_pos = (ee_quat, ee_pos)
                    self.last_q = joint_pos
                    self.received_states = True
                    
                except zmq.Again:
                    time.sleep(0.001)
                except Exception as e:
                    print(f"Error receiving arm state: {e}")
                    time.sleep(0.001)
        
        def gripper_state_receiver():
            while True:
                try:
                    message = self.gripper_subscriber.recv(flags=zmq.NOBLOCK)
                    gripper_state = franka_robot_state_pb2.FrankaGripperStateMessage()
                    gripper_state.ParseFromString(message)
                    
                    # Convert gripper width to action (-1 to 1 range)
                    gripper_width = gripper_state.width
                    max_width = gripper_state.max_width
                    self.last_gripper_action = (gripper_width / max_width) * 2 - 1  # Scale to [-1, 1]
                    
                except zmq.Again:
                    time.sleep(0.001)
                except Exception as e:
                    print(f"Error receiving gripper state: {e}")
                    time.sleep(0.001)
        
        # Start receiver threads
        arm_thread = threading.Thread(target=arm_state_receiver, daemon=True)
        gripper_thread = threading.Thread(target=gripper_state_receiver, daemon=True)
        arm_thread.start()
        gripper_thread.start()
        
        # Wait for initial state
        while not self.received_states or self.last_gripper_action is None:
            time.sleep(0.01)

        print("Franka is connected")

    def check_nonzero_configuration(self):
        """Check if the robot is in a valid configuration"""
        if self.last_q is None:
            return False
        return not all(abs(q) < 1e-6 for q in self.last_q)

    def close(self):
        """Close ZMQ connections"""
        self.arm_subscriber.close()
        self.gripper_subscriber.close()
        self.arm_publisher.close()
        self.gripper_publisher.close()
        self.context.term()

    def osc_move(self, target_pos, target_quat, gripper_state):
        """Send OSC control command to the robot"""
        # This would need to be implemented to send control commands
        # to the deoxys services via ZMQ publishers
        # For now, just updating the target internally
        pass

    def reset_joints(self, timeout=7, gripper_open=False):
        """Reset robot to initial joint configuration"""
        # This would need to be implemented to send joint position commands
        # to the deoxys services via ZMQ publishers
        # For now, just a placeholder
        return True

    def control(self, controller_type, action, controller_cfg):
        """Send control command to deoxys services"""
        # This would need to be implemented to send control commands
        # to the deoxys services via ZMQ publishers
        # For now, just a placeholder
        pass
