#!/usr/bin/env python3
"""
Complete Debug Teleop - Includes both teleoperator and oculus_stick components
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from multiprocessing import Process
import time

def start_debug_teleop():
    """Debug version of the teleoperator"""
    print("🤖 Starting DEBUG teleoperator process...")
    
    import pickle
    import numpy as np
    from frankateach.utils import notify_component_start
    from frankateach.network import (
        ZMQKeypointSubscriber,
        create_request_socket,
        ZMQKeypointPublisher,
    )
    from frankateach.constants import (
        COMMANDED_STATE_PORT,
        CONTROL_PORT,
        HOST,
        STATE_PORT,
        VR_CONTROLLER_STATE_PORT,
        GRIPPER_OPEN,
    )
    from frankateach.messages import FrankaAction, FrankaState
    from deoxys.utils import transform_utils

    class DebugFrankaOperator:
        def __init__(self):
            print("🔧 [TELEOP] Initializing DebugFrankaOperator...")
            
            # Subscribe controller state
            self._controller_state_subscriber = ZMQKeypointSubscriber(
                host=HOST, port=VR_CONTROLLER_STATE_PORT, topic="controller_state"
            )
            self.action_socket = create_request_socket(HOST, CONTROL_PORT)
            self.state_socket = ZMQKeypointPublisher(HOST, STATE_PORT)
            self.commanded_state_socket = ZMQKeypointPublisher(HOST, COMMANDED_STATE_PORT)

            # Class variables
            self.is_first_frame = True
            self.gripper_state = GRIPPER_OPEN
            self.start_teleop = False
            self.init_affine = None
            self.teleop_mode = "human"
            self.home_offset = np.array([-0.22, 0.0, 0.1])
            
            print("✅ [TELEOP] DebugFrankaOperator initialized")

        def debug_apply_retargeted_angles(self):
            loop_start = time.time()
            print(f"\n🔄 [TELEOP] Frame start (first: {self.is_first_frame})")
            
            # Receive controller state
            print("📡 [TELEOP] Waiting for controller state...")
            try:
                recv_start = time.time()
                self.controller_state = self._controller_state_subscriber.recv_keypoints()
                recv_time = time.time() - recv_start
                print(f"✅ [TELEOP] Received controller state in {recv_time:.3f}s")
                print(f"   Right A: {self.controller_state.right_a}")
                print(f"   Right B: {self.controller_state.right_b}")
                print(f"   Position: {self.controller_state.right_local_position}")
            except Exception as e:
                print(f"❌ [TELEOP] Error receiving controller state: {e}")
                return

            if self.is_first_frame:
                print("🏠 [TELEOP] First frame - performing reset...")
                
                # Reset robot
                action = FrankaAction(
                    pos=np.zeros(3),
                    quat=np.zeros(4),
                    gripper=self.gripper_state,
                    reset=True,
                    timestamp=time.time(),
                )
                self.action_socket.send(bytes(pickle.dumps(action, protocol=-1)))
                robot_state = pickle.loads(self.action_socket.recv())
                print(f"✅ [TELEOP] Reset complete")

                # Move to offset position
                target_pos = robot_state.pos + self.home_offset
                target_quat = robot_state.quat
                action = FrankaAction(
                    pos=target_pos.flatten().astype(np.float32),
                    quat=target_quat.flatten().astype(np.float32),
                    gripper=self.gripper_state,
                    reset=False,
                    timestamp=time.time(),
                )
                self.action_socket.send(bytes(pickle.dumps(action, protocol=-1)))
                robot_state = pickle.loads(self.action_socket.recv())
                print(f"✅ [TELEOP] Moved to home position")

                self.home_rot, self.home_pos = (
                    transform_utils.quat2mat(robot_state.quat),
                    robot_state.pos,
                )
                
                self.is_first_frame = False
                print("✅ [TELEOP] First frame complete")

            # Check button states
            if self.controller_state.right_a:
                if not self.start_teleop:
                    print("🟢 [TELEOP] A button pressed - Starting teleop!")
                    self.start_teleop = True
                    self.init_affine = self.controller_state.right_affine
                
            if self.controller_state.right_b:
                if self.start_teleop:
                    print("🔴 [TELEOP] B button pressed - Stopping teleop!")
                    self.start_teleop = False
                    self.init_affine = None

            # In human mode, always send get_state
            print(f"📊 [TELEOP] Teleop status: {self.start_teleop}")
            self.action_socket.send(b"get_state")
            robot_state = pickle.loads(self.action_socket.recv())
            robot_state.start_teleop = self.start_teleop
            
            # Publish states
            self.state_socket.pub_keypoints(robot_state, "robot_state")
            
            dummy_action = FrankaAction(
                pos=self.home_pos.flatten().astype(np.float32),
                quat=transform_utils.mat2quat(self.home_rot).flatten().astype(np.float32),
                gripper=self.gripper_state,
                reset=False,
                timestamp=time.time(),
            )
            self.commanded_state_socket.pub_keypoints(dummy_action, "commanded_robot_state")
            
            loop_time = time.time() - loop_start
            print(f"✅ [TELEOP] Frame complete in {loop_time:.3f}s")

        def stream(self):
            notify_component_start("Debug Franka teleoperator control")
            print("🚀 [TELEOP] Starting debug stream...")

            loop_count = 0
            try:
                while True:
                    loop_count += 1
                    if loop_count % 10 == 1:  # Print every 10th loop
                        print(f"\n{'='*40}")
                        print(f"🔄 [TELEOP] LOOP {loop_count}")
                        print(f"{'='*40}")
                    
                    self.debug_apply_retargeted_angles()
                    
            except KeyboardInterrupt:
                print("\n🛑 [TELEOP] Stopped by user")
            finally:
                self._controller_state_subscriber.stop()
                self.action_socket.close()

    operator = DebugFrankaOperator()
    operator.stream()


def start_debug_oculus_stick():
    """Debug version of the oculus stick detector"""
    print("👁️ Starting DEBUG oculus stick process...")
    
    from frankateach.constants import (
        VR_CONTROLLER_STATE_PORT,
        VR_FREQ,
        VR_TCP_HOST,
        VR_TCP_PORT,
    )
    from frankateach.utils import FrequencyTimer
    from frankateach.network import create_subscriber_socket, ZMQKeypointPublisher
    from frankateach.utils import parse_controller_state, notify_component_start

    class DebugOculusVRStickDetector:
        def __init__(self, host, controller_state_pub_port):
            print("🔧 [VR] Initializing DebugOculusVRStickDetector...")
            notify_component_start("debug vr detector")

            # Create a subscriber socket
            self.stick_socket = create_subscriber_socket(
                VR_TCP_HOST, VR_TCP_PORT, b"", conflate=True
            )

            # Create a publisher for the controller state
            self.controller_state_publisher = ZMQKeypointPublisher(
                host=host, port=controller_state_pub_port
            )
            self.timer = FrequencyTimer(VR_FREQ)
            print("✅ [VR] DebugOculusVRStickDetector initialized")

        def _publish_controller_state(self, controller_state):
            self.controller_state_publisher.pub_keypoints(
                keypoint_array=controller_state, topic_name="controller_state"
            )

        def stream(self):
            print("👁️ [VR] Starting oculus stick stream...")
            message_count = 0
            
            while True:
                try:
                    self.timer.start_loop()

                    message = self.stick_socket.recv_string()
                    if message == "oculus_controller":
                        continue

                    message_count += 1
                    controller_state = parse_controller_state(message)

                    # Debug output every 50 messages
                    if message_count % 50 == 0:
                        print(f"📡 [VR] Processed {message_count} messages")
                        print(f"   Right A: {controller_state.right_a}")
                        print(f"   Right B: {controller_state.right_b}")
                        print(f"   Position: {controller_state.right_local_position}")

                    # Publish message
                    self._publish_controller_state(controller_state)

                    self.timer.end_loop()

                except KeyboardInterrupt:
                    break
                except Exception as e:
                    print(f"❌ [VR] Error: {e}")

            self.controller_state_publisher.stop()
            print("🛑 [VR] Stopping the oculus keypoint extraction process.")

    from frankateach.constants import HOST
    detector = DebugOculusVRStickDetector(HOST, VR_CONTROLLER_STATE_PORT)
    detector.stream()


def main():
    print("🐛 Starting COMPLETE Debug Teleop")
    print("=" * 50)
    print("This includes both teleoperator and oculus_stick components")
    print("=" * 50)
    
    # Start both processes (same as original teleop.py)
    teleop_process = Process(target=start_debug_teleop)
    oculus_stick_process = Process(target=start_debug_oculus_stick)

    print("🚀 Starting processes...")
    teleop_process.start()
    oculus_stick_process.start()

    try:
        teleop_process.join()
        oculus_stick_process.join()
    except KeyboardInterrupt:
        print("\n🛑 Stopping all processes...")
        teleop_process.terminate()
        oculus_stick_process.terminate()
        teleop_process.join()
        oculus_stick_process.join()

if __name__ == "__main__":
    main() 