#!/usr/bin/env python3
"""
Debug script to test what messages are being published by deoxys services
"""
import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import zmq
import time
from pathlib import Path
from deoxys.utils import YamlConfig
import deoxys.proto.franka_interface.franka_robot_state_pb2 as franka_robot_state_pb2

CONFIG_ROOT = Path(__file__).parent / "frankateach" / "configs"

def test_deoxys_messages():
    # Load configuration
    config = YamlConfig(CONFIG_ROOT / "deoxys_right.yml").as_easydict()
    
    print(f"🔍 Testing deoxys message reception...")
    print(f"Config: NUC IP = {config.NUC.IP}")
    print(f"Arm port: {config.NUC.PUB_PORT}")
    print(f"Gripper port: {config.NUC.GRIPPER_PUB_PORT}")
    
    # Initialize ZMQ context
    context = zmq.Context()
    
    # Test arm state subscriber
    print(f"\n📡 Testing arm state on tcp://localhost:{config.NUC.PUB_PORT}")
    arm_subscriber = context.socket(zmq.SUB)
    arm_subscriber.setsockopt(zmq.CONFLATE, 1)
    arm_subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
    arm_subscriber.connect(f"tcp://localhost:{config.NUC.PUB_PORT}")
    
    # Test gripper state subscriber
    print(f"📡 Testing gripper state on tcp://localhost:{config.NUC.GRIPPER_PUB_PORT}")
    gripper_subscriber = context.socket(zmq.SUB)
    gripper_subscriber.setsockopt(zmq.CONFLATE, 1)
    gripper_subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
    gripper_subscriber.connect(f"tcp://localhost:{config.NUC.GRIPPER_PUB_PORT}")
    
    arm_received = False
    gripper_received = False
    
    print(f"\n⏱️ Waiting for messages (timeout: 10 seconds)...")
    start_time = time.time()
    
    while time.time() - start_time < 10:
        # Test arm messages
        if not arm_received:
            try:
                message = arm_subscriber.recv(flags=zmq.NOBLOCK)
                print(f"✅ ARM: Received message ({len(message)} bytes)")
                
                try:
                    robot_state = franka_robot_state_pb2.FrankaRobotStateMessage()
                    robot_state.ParseFromString(message)
                    print(f"   ✅ ARM: Successfully parsed protobuf")
                    print(f"   📊 ARM: Frame number: {robot_state.frame}")
                    print(f"   🤖 ARM: Joint positions: {robot_state.q[:3]}...")
                    
                    # Debug: print available attributes
                    attrs = [attr for attr in dir(robot_state) if not attr.startswith('_')]
                    print(f"   🔍 ARM: Available attributes: {[a for a in attrs if 'o_t_ee' in a.lower() or 'ee' in a.lower()]}")
                    
                    # Try different possible attribute names
                    if hasattr(robot_state, 'o_t_ee'):
                        print(f"   📍 ARM: o_t_ee: {robot_state.o_t_ee[:4]}...")
                    elif hasattr(robot_state, 'O_T_EE'):
                        print(f"   📍 ARM: O_T_EE: {robot_state.O_T_EE[:4]}...")
                    else:
                        print(f"   ❌ ARM: No o_t_ee or O_T_EE attribute found")
                    
                    arm_received = True
                except Exception as e:
                    print(f"   ❌ ARM: Failed to parse protobuf: {e}")
                    
            except zmq.Again:
                pass
        
        # Test gripper messages  
        if not gripper_received:
            try:
                message = gripper_subscriber.recv(flags=zmq.NOBLOCK)
                print(f"✅ GRIPPER: Received message ({len(message)} bytes)")
                
                try:
                    gripper_state = franka_robot_state_pb2.FrankaGripperStateMessage()
                    gripper_state.ParseFromString(message)
                    print(f"   ✅ GRIPPER: Successfully parsed protobuf")
                    print(f"   📏 GRIPPER: Width: {gripper_state.width}")
                    print(f"   📏 GRIPPER: Max width: {gripper_state.max_width}")
                    print(f"   🤏 GRIPPER: Is grasped: {gripper_state.is_grasped}")
                    gripper_received = True
                except Exception as e:
                    print(f"   ❌ GRIPPER: Failed to parse protobuf: {e}")
                    
            except zmq.Again:
                pass
        
        if arm_received and gripper_received:
            break
            
        time.sleep(0.01)
    
    # Results
    print(f"\n📋 RESULTS:")
    print(f"   ARM messages: {'✅ Received' if arm_received else '❌ Not received'}")
    print(f"   GRIPPER messages: {'✅ Received' if gripper_received else '❌ Not received'}")
    
    if arm_received and gripper_received:
        print(f"   🎉 SUCCESS: Both message types are working!")
    else:
        print(f"   ⚠️  Some message types are not working")
    
    # Cleanup
    arm_subscriber.close()
    gripper_subscriber.close()
    context.term()

if __name__ == "__main__":
    test_deoxys_messages() 