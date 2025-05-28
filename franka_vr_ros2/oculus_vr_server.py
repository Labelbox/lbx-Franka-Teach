#!/usr/bin/env python3
"""
Oculus VR Server - ROS2/MoveIt Implementation
Preserves all features from the original Deoxys implementation
"""

import asyncio
import argparse
import signal
import sys
import os
import subprocess
from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import numpy as np
import time

from franka_vr_ros2.core.vr_input import VRInputHandler
from franka_vr_ros2.core.transform import LabelboxTransform
from franka_vr_ros2.core.motion_filter import MotionFilter
from franka_vr_ros2.strategies import get_strategy
from franka_vr_ros2.recording.mcap_recorder import MCAPRecorder
from franka_vr_ros2.recording.camera_manager import CameraManager


class OculusVRServer:
    """Main VR teleoperation server - single entry point"""
    
    def __init__(self, args):
        self.args = args
        self.running = True
        
        # Initialize ROS2
        rclpy.init()
        
        # Create main node
        self.node = Node('oculus_vr_server')
        
        # Initialize components (all async-compatible)
        self.vr_input = VRInputHandler(
            node=self.node,
            right_controller=not args.left_controller,
            ip_address=args.ip
        )
        
        self.transform = LabelboxTransform(
            node=self.node,
            position_reorder=[-3, -1, 2, 4],  # Preserved from original
            rotation_mode='labelbox'
        )
        
        self.motion_filter = MotionFilter(
            node=self.node,
            enable_prediction=not args.no_prediction
        )
        
        # Control strategy (modular)
        self.control_strategy = get_strategy(
            args.control_strategy,
            self.node,
            robot_ip=args.robot_ip
        )
        
        # Recording components (preserved from original)
        self.mcap_recorder = None
        self.camera_manager = None
        
        if not args.no_recording:
            self.mcap_recorder = MCAPRecorder(
                node=self.node,
                save_dir=os.path.expanduser("~/recordings/success")
            )
            
        if args.enable_cameras and args.camera_config:
            self.camera_manager = CameraManager(args.camera_config)
            if self.mcap_recorder:
                self.mcap_recorder.set_camera_manager(self.camera_manager)
                
        # Async event loop for high-performance operation
        self.loop = asyncio.new_event_loop()
        self.executor = MultiThreadedExecutor()
        
        # Button state tracking
        self.prev_a_button = False
        
        # Origin calibration state
        self.reset_origin = True
        self.robot_origin = None
        self.vr_origin = None
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
    async def run(self):
        """Main async run loop"""
        # Start all components
        await self.vr_input.start()
        
        if self.camera_manager:
            self.camera_manager.start()
            
        # Initialize control strategy
        await self.control_strategy.initialize()
        
        # Launch ROS2 nodes in subprocess for better isolation
        launch_process = self.start_ros2_nodes()
        
        # Main control loop
        control_task = asyncio.create_task(self.control_loop())
        recording_task = asyncio.create_task(self.recording_loop()) if self.mcap_recorder else None
        
        # Start ROS2 executor in separate thread
        executor_thread = threading.Thread(target=self.run_ros2_executor)
        executor_thread.daemon = True
        executor_thread.start()
        
        # Print startup message
        self.print_startup_message()
        
        try:
            # Run until shutdown
            tasks = [control_task]
            if recording_task:
                tasks.append(recording_task)
            await asyncio.gather(*tasks)
        finally:
            # Cleanup
            if launch_process:
                launch_process.terminate()
            await self.cleanup()
            
    def run_ros2_executor(self):
        """Run ROS2 executor in separate thread"""
        self.executor.add_node(self.node)
        self.executor.spin()
        
    async def control_loop(self):
        """Main control loop - coordinates all components"""
        last_time = self.node.get_clock().now()
        
        while self.running:
            try:
                # Get VR state (async, non-blocking)
                vr_state = await self.vr_input.get_state()
                
                if vr_state:
                    # Update transform calibration if needed
                    if hasattr(self.vr_input, 'vr_to_global_mat'):
                        self.transform.set_calibration(
                            self.vr_input.vr_to_global_mat,
                            self.vr_input.vr_neutral_pose
                        )
                    
                    # Handle origin calibration
                    if self.reset_origin and vr_state.movement_enabled:
                        robot_state = self.control_strategy.get_robot_state()
                        if robot_state:
                            self.robot_origin = {
                                'pos': robot_state.pos,
                                'quat': robot_state.quat
                            }
                            self.vr_origin = vr_state.pose
                            self.reset_origin = False
                            self.node.get_logger().info("Origin calibrated")
                    
                    # Reset origin when grip is released
                    if not vr_state.movement_enabled:
                        self.reset_origin = True
                    
                    if vr_state.movement_enabled and self.robot_origin and self.vr_origin:
                        # Apply coordinate transform
                        robot_pose = self.transform.transform_pose(vr_state.pose)
                        
                        # Apply motion filtering
                        filtered_pose = self.motion_filter.filter_pose(robot_pose)
                        
                        # Apply origin offset
                        # This maintains the relative motion from the grip point
                        filtered_pose.position.x += self.robot_origin['pos'][0] - self.vr_origin.position.x
                        filtered_pose.position.y += self.robot_origin['pos'][1] - self.vr_origin.position.y
                        filtered_pose.position.z += self.robot_origin['pos'][2] - self.vr_origin.position.z
                        
                        # Send to control strategy
                        await self.control_strategy.send_command(filtered_pose)
                        
                # Handle button inputs
                self.handle_buttons(vr_state)
                
                # Maintain control rate
                await self.rate_limit(last_time, self.args.control_rate)
                last_time = self.node.get_clock().now()
                
            except Exception as e:
                self.node.get_logger().error(f"Control loop error: {e}")
                import traceback
                traceback.print_exc()
                
    async def rate_limit(self, last_time, target_rate):
        """Maintain target control rate"""
        current_time = self.node.get_clock().now()
        elapsed = (current_time - last_time).nanoseconds / 1e9
        sleep_time = (1.0 / target_rate) - elapsed
        if sleep_time > 0:
            await asyncio.sleep(sleep_time)
            
    async def recording_loop(self):
        """Async recording loop - handles MCAP and cameras"""
        while self.running:
            if self.mcap_recorder.is_recording():
                # Get latest states
                vr_state = self.vr_input.get_latest_state()
                robot_state = self.control_strategy.get_robot_state()
                
                if vr_state and robot_state:
                    # Record timestep
                    await self.mcap_recorder.record_timestep(
                        vr_state=vr_state,
                        robot_state=robot_state,
                        timestamp=self.node.get_clock().now()
                    )
                    
            await asyncio.sleep(1.0 / self.args.recording_rate)
            
    def handle_buttons(self, vr_state):
        """Handle VR controller buttons - preserved from original"""
        if not vr_state:
            return
            
        # A button: Start/stop recording
        current_a_button = vr_state.buttons.get('A' if self.args.left_controller else 'A', False)
        if current_a_button and not self.prev_a_button:  # Rising edge
            if self.mcap_recorder:
                if self.mcap_recorder.is_recording():
                    self.mcap_recorder.stop_recording(success=False)
                    print("📹 Recording stopped")
                else:
                    self.mcap_recorder.start_recording()
                    print("📹 Recording started")
                    
        # B button: Save recording as successful
        if vr_state.buttons.get('B' if self.args.left_controller else 'B', False) and self.mcap_recorder and self.mcap_recorder.is_recording():
            filepath = self.mcap_recorder.stop_recording(success=True)
            print(f"✅ Recording saved: {filepath}")
            
        self.prev_a_button = current_a_button
        
    def start_ros2_nodes(self):
        """Launch ROS2 nodes for robot control"""
        if self.args.debug:
            return None
            
        # For now, we'll assume the robot nodes are launched separately
        # In a full implementation, this would launch the robot driver and MoveIt
        self.node.get_logger().info("Robot control nodes should be launched separately")
        return None
        
    def print_startup_message(self):
        """Print startup information"""
        print("\n🎮 Oculus VR Server - ROS2/MoveIt Edition")
        print(f"   Control Strategy: {self.args.control_strategy}")
        print(f"   Control Rate: {self.args.control_rate}Hz")
        print(f"   Robot IP: {self.args.robot_ip}")
        print(f"   Recording: {'Enabled' if self.mcap_recorder else 'Disabled'}")
        print(f"   Cameras: {'Enabled' if self.camera_manager else 'Disabled'}")
        print("\n📋 Controls (preserved from original):")
        print("   - HOLD grip: Enable teleoperation")
        print("   - Trigger: Close/open gripper")
        print("   - A button: Start/stop recording")
        print("   - B button: Save recording as successful")
        print("   - Joystick: Calibrate forward direction")
        print("\nPress Ctrl+C to exit\n")
        
    def signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        print("\n🛑 Shutting down...")
        self.running = False
        
    async def cleanup(self):
        """Clean shutdown of all components"""
        if self.mcap_recorder and self.mcap_recorder.is_recording():
            self.mcap_recorder.stop_recording(success=False)
            
        if self.camera_manager:
            self.camera_manager.stop()
            
        await self.vr_input.stop()
        
        # Shutdown ROS2
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Oculus VR Server - ROS2/MoveIt Implementation')
    
    # Preserved arguments from original
    parser.add_argument('--debug', action='store_true', help='Debug mode')
    parser.add_argument('--left-controller', action='store_true', help='Use left controller')
    parser.add_argument('--ip', type=str, default=None, help='Quest IP address')
    parser.add_argument('--simulation', action='store_true', help='Use simulated robot')
    parser.add_argument('--no-recording', action='store_true', help='Disable MCAP recording')
    parser.add_argument('--camera-config', type=str, help='Camera configuration file')
    parser.add_argument('--enable-cameras', action='store_true', help='Enable camera recording')
    parser.add_argument('--hot-reload', action='store_true', help='Enable hot reload mode')
    
    # New arguments for ROS2 implementation
    parser.add_argument('--robot-ip', type=str, default='192.168.1.59',  # Same as Deoxys
                        help='Robot IP address')
    parser.add_argument('--control-strategy', type=str, default='moveit_servo',
                        choices=['moveit_servo', 'direct_ik', 'cartesian_pose'],
                        help='Control strategy to use')
    parser.add_argument('--control-rate', type=int, default=250, help='Control loop rate (Hz)')
    parser.add_argument('--recording-rate', type=int, default=30, help='Recording rate (Hz)')
    parser.add_argument('--no-prediction', action='store_true', help='Disable motion prediction')
    
    args = parser.parse_args()
    
    # If hot reload is requested, launch the hot reload wrapper instead
    if args.hot_reload:
        # Remove --hot-reload from args and pass the rest to the wrapper
        new_args = [arg for arg in sys.argv[1:] if arg != '--hot-reload']
        
        print("🔥 Launching in hot reload mode...")
        
        # Check if hot reload script exists
        hot_reload_script = os.path.join(os.path.dirname(__file__), 'oculus_vr_server_hotreload.py')
        if not os.path.exists(hot_reload_script):
            print("❌ Hot reload script not found!")
            print("   Make sure oculus_vr_server_hotreload.py is in the same directory")
            sys.exit(1)
        
        # Launch the hot reload wrapper
        try:
            subprocess.run([sys.executable, hot_reload_script] + new_args)
        except KeyboardInterrupt:
            print("\n✅ Hot reload stopped")
        sys.exit(0)
    
    # Create and run server
    server = OculusVRServer(args)
    
    # Run async main loop
    try:
        asyncio.run(server.run())
    except KeyboardInterrupt:
        pass
    finally:
        print("✅ Server stopped")


if __name__ == "__main__":
    main() 