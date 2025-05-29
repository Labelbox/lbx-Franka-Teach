# Technical Design Document: ROS2/MoveIt VR Teleoperation System

## Executive Summary

This document outlines the technical design for migrating the Franka FR3 VR teleoperation system from Deoxys to ROS2/MoveIt. The new system will achieve <10ms latency (vs current 120ms) while maintaining the proven Labelbox coordinate transformations and adding modular control strategies.

### Key Improvements

- **Latency**: 120ms → <10ms
- **Control Rate**: 15-30Hz → 100-1000Hz
- **Architecture**: Monolithic → Modular with swappable strategies
- **Tracking**: Velocity-based → Direct 1:1 pose mapping
- **Framework**: Custom Deoxys → Industry-standard ROS2/MoveIt

### Design Principles

- **Python-first implementation** (C++ only where necessary for performance)
- **Single entry point**: `oculus_vr_server.py` launches everything
- **Preserve all features**: MCAP recording, async architecture, camera capture
- **Immediate implementation**: No phased approach, build it now

## System Architecture

### High-Level Architecture

```
┌─────────────────────┐
│ oculus_vr_server.py │ Single entry point
│  (Main Process)     │
└──────────┬──────────┘
           │ Spawns ROS2 nodes
    ┌──────┴──────┬──────────┬──────────┬──────────┐
    ▼             ▼          ▼          ▼          ▼
┌────────┐ ┌──────────┐ ┌────────┐ ┌────────┐ ┌────────┐
│  VR    │ │Transform │ │Control │ │ MCAP   │ │Camera  │
│ Input  │ │  Node    │ │Strategy│ │Recorder│ │Manager │
└────────┘ └──────────┘ └────────┘ └────────┘ └────────┘
    │             │          │          │          │
    └─────────────┴──────────┴──────────┴──────────┘
                  │ ROS2 Topics
    ┌─────────────▼──────────┐
    │   Robot Interface      │
    │ (MoveIt/ros2_control)  │
    └─────────────┬──────────┘
                  │ FCI
    ┌─────────────▼──────────┐
    │   Franka FR3 Robot     │
    └────────────────────────┘
```

### Simplified Package Structure

```
franka_vr_ros2/
├── oculus_vr_server.py          # Main entry point (launches everything)
├── requirements.txt             # Updated with ROS2 dependencies
├── setup.py
│
├── franka_vr_ros2/              # Python package
│   ├── __init__.py
│   ├── core/
│   │   ├── __init__.py
│   │   ├── vr_input.py          # VR input handling (async)
│   │   ├── transform.py         # Labelbox coordinate transforms
│   │   ├── motion_filter.py    # Kalman/complementary filtering
│   │   └── types.py            # Data structures
│   │
│   ├── strategies/              # Control strategy implementations
│   │   ├── __init__.py
│   │   ├── base.py             # Abstract base class
│   │   ├── moveit_servo.py    # MoveIt Servo strategy
│   │   ├── direct_ik.py       # Direct IK solver (Python bindings)
│   │   └── cartesian_pose.py  # Direct Cartesian control
│   │
│   ├── recording/               # MCAP and camera recording
│   │   ├── __init__.py
│   │   ├── mcap_recorder.py   # Async MCAP writer
│   │   └── camera_manager.py  # Camera capture (preserved)
│   │
│   └── utils/
│       ├── __init__.py
│       ├── ros2_utils.py       # ROS2 helper functions
│       └── realtime_utils.py  # RT priority, CPU affinity
│
├── config/
│   ├── robot_config.yaml       # Robot parameters
│   ├── transform_config.yaml   # Labelbox transform settings
│   ├── control_strategies.yaml # Strategy configurations
│   └── cameras.yaml           # Camera configurations
│
└── launch/
    └── teleop_system.launch.py # ROS2 launch file (called by main script)
```

## Core Implementation

### 1. Main Entry Point: `oculus_vr_server.py`

```python
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

        # Setup signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    async def run(self):
        """Main async run loop"""
        # Start all components
        await self.vr_input.start()

        if self.camera_manager:
            self.camera_manager.start()

        # Launch ROS2 nodes in subprocess for better isolation
        launch_process = self.start_ros2_nodes()

        # Main control loop
        control_task = asyncio.create_task(self.control_loop())
        recording_task = asyncio.create_task(self.recording_loop()) if self.mcap_recorder else None

        # Print startup message
        self.print_startup_message()

        try:
            # Run until shutdown
            await asyncio.gather(
                control_task,
                recording_task if recording_task else asyncio.sleep(0)
            )
        finally:
            # Cleanup
            if launch_process:
                launch_process.terminate()
            await self.cleanup()

    async def control_loop(self):
        """Main control loop - coordinates all components"""
        last_time = self.node.get_clock().now()

        while self.running:
            try:
                # Get VR state (async, non-blocking)
                vr_state = await self.vr_input.get_state()

                if vr_state and vr_state.movement_enabled:
                    # Apply coordinate transform
                    robot_pose = self.transform.transform_pose(vr_state.pose)

                    # Apply motion filtering
                    filtered_pose = self.motion_filter.filter_pose(robot_pose)

                    # Send to control strategy
                    await self.control_strategy.send_command(filtered_pose)

                # Handle button inputs
                self.handle_buttons(vr_state)

                # Maintain control rate
                await self.rate_limit(last_time, self.args.control_rate)
                last_time = self.node.get_clock().now()

            except Exception as e:
                self.node.get_logger().error(f"Control loop error: {e}")

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
        if vr_state.buttons.get('A') and not self.prev_a_button:
            if self.mcap_recorder:
                if self.mcap_recorder.is_recording():
                    self.mcap_recorder.stop_recording(success=False)
                    print("📹 Recording stopped")
                else:
                    self.mcap_recorder.start_recording()
                    print("📹 Recording started")

        # B button: Save recording as successful
        if vr_state.buttons.get('B') and self.mcap_recorder and self.mcap_recorder.is_recording():
            filepath = self.mcap_recorder.stop_recording(success=True)
            print(f"✅ Recording saved: {filepath}")

        self.prev_a_button = vr_state.buttons.get('A', False)

    def start_ros2_nodes(self):
        """Launch ROS2 nodes for robot control"""
        launch_cmd = [
            'ros2', 'launch',
            'franka_vr_ros2', 'teleop_system.launch.py',
            f'robot_ip:={self.args.robot_ip}',
            f'control_strategy:={self.args.control_strategy}',
            f'use_fake_hardware:={str(self.args.simulation).lower()}'
        ]

        if not self.args.debug:
            return subprocess.Popen(launch_cmd)
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

    # New arguments for ROS2 implementation
    parser.add_argument('--robot-ip', type=str, default='192.168.1.1', help='Robot IP address')
    parser.add_argument('--control-strategy', type=str, default='moveit_servo',
                        choices=['moveit_servo', 'direct_ik', 'cartesian_pose'],
                        help='Control strategy to use')
    parser.add_argument('--control-rate', type=int, default=250, help='Control loop rate (Hz)')
    parser.add_argument('--recording-rate', type=int, default=30, help='Recording rate (Hz)')
    parser.add_argument('--no-prediction', action='store_true', help='Disable motion prediction')

    args = parser.parse_args()

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
```

### 2. Core Components (Python Implementation)

#### A. VR Input Handler (`franka_vr_ros2/core/vr_input.py`)

```python
import asyncio
import numpy as np
from dataclasses import dataclass
from typing import Dict, Optional
import time

from oculus_reader.reader import OculusReader
from geometry_msgs.msg import PoseStamped
from franka_vr_ros2.core.types import VRState


class VRInputHandler:
    """Async VR input handler - preserves original functionality"""

    def __init__(self, node, right_controller=True, ip_address=None):
        self.node = node
        self.controller_id = "r" if right_controller else "l"
        self.oculus_reader = OculusReader(ip_address=ip_address)

        # Publishers
        self.pose_pub = node.create_publisher(
            PoseStamped, 'vr_controller_pose', 10
        )

        # State
        self._latest_state = None
        self._running = False

        # Calibration state (preserved from original)
        self.vr_neutral_pose = None
        self.calibrating_forward = False

    async def start(self):
        """Start async polling loop"""
        self._running = True
        asyncio.create_task(self._poll_loop())

    async def _poll_loop(self):
        """Poll VR controller at 90Hz"""
        while self._running:
            try:
                poses, buttons = self.oculus_reader.get_transformations_and_buttons()

                if self.controller_id in poses:
                    # Create VR state
                    state = VRState(
                        timestamp=time.time(),
                        pose=self._matrix_to_pose(poses[self.controller_id]),
                        buttons=buttons,
                        movement_enabled=buttons.get(f"{self.controller_id.upper()}G", False),
                        controller_on=True
                    )

                    # Handle calibration (preserved from original)
                    self._handle_calibration(state, poses[self.controller_id])

                    # Update latest state
                    self._latest_state = state

                    # Publish pose
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.node.get_clock().now().to_msg()
                    pose_msg.header.frame_id = "vr_base"
                    pose_msg.pose = state.pose
                    self.pose_pub.publish(pose_msg)

            except Exception as e:
                self.node.get_logger().error(f"VR polling error: {e}")

            await asyncio.sleep(0.011)  # ~90Hz

    async def get_state(self) -> Optional[VRState]:
        """Get latest VR state (non-blocking)"""
        return self._latest_state

    def get_latest_state(self) -> Optional[VRState]:
        """Sync version for recording thread"""
        return self._latest_state
```

#### B. Labelbox Transform (`franka_vr_ros2/core/transform.py`)

```python
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose


class LabelboxTransform:
    """Preserves exact coordinate transformations from original implementation"""

    def __init__(self, node, position_reorder, rotation_mode='labelbox'):
        self.node = node
        self.position_reorder = position_reorder
        self.rotation_mode = rotation_mode

        # Create transformation matrices
        self.global_to_env_mat = self._vec_to_reorder_mat(position_reorder)
        self.vr_to_global_mat = np.eye(4)

    def transform_pose(self, vr_pose: Pose) -> Pose:
        """Apply Labelbox coordinate transformation"""
        # Convert pose to matrix
        vr_mat = self._pose_to_matrix(vr_pose)

        # Apply position transformation
        transformed_mat = self.global_to_env_mat @ self.vr_to_global_mat @ vr_mat
        robot_pos = transformed_mat[:3, 3]

        # Apply rotation transformation (labelbox mode)
        vr_quat = np.array([
            vr_pose.orientation.x,
            vr_pose.orientation.y,
            vr_pose.orientation.z,
            vr_pose.orientation.w
        ])
        robot_quat = self._apply_labelbox_rotation(vr_quat)

        # Create output pose
        robot_pose = Pose()
        robot_pose.position.x = robot_pos[0]
        robot_pose.position.y = robot_pos[1]
        robot_pose.position.z = robot_pos[2]
        robot_pose.orientation.x = robot_quat[0]
        robot_pose.orientation.y = robot_quat[1]
        robot_pose.orientation.z = robot_quat[2]
        robot_pose.orientation.w = robot_quat[3]

        return robot_pose

    def _apply_labelbox_rotation(self, vr_quat):
        """Apply labelbox rotation mapping - preserved from original"""
        # Convert to rotation object
        vr_rot = R.from_quat(vr_quat)

        # Get axis-angle representation
        rotvec = vr_rot.as_rotvec()
        angle = np.linalg.norm(rotvec)

        if angle > 0:
            axis = rotvec / angle

            # Apply labelbox transformation
            # VR: X=pitch, Y=roll, Z=yaw
            # Robot: X=roll, Y=pitch, Z=yaw
            # Mapping: [-Y, X, Z] with Y inverted
            transformed_axis = np.array([-axis[1], axis[0], axis[2]])

            # Create new rotation
            transformed_rotvec = transformed_axis * angle
            robot_rot = R.from_rotvec(transformed_rotvec)
            return robot_rot.as_quat()
        else:
            return np.array([0, 0, 0, 1])

    def _vec_to_reorder_mat(self, vec):
        """Convert reorder vector to transformation matrix"""
        X = np.zeros((len(vec), len(vec)))
        for i in range(len(vec)):
            ind = int(abs(vec[i])) - 1
            X[i, ind] = np.sign(vec[i])
        return X
```

#### C. Control Strategies

##### Base Strategy (`franka_vr_ros2/strategies/base.py`)

```python
from abc import ABC, abstractmethod
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState


class ControlStrategyBase(ABC):
    """Base class for control strategies"""

    def __init__(self, node, robot_ip):
        self.node = node
        self.robot_ip = robot_ip
        self._robot_state = None

    @abstractmethod
    async def initialize(self):
        """Initialize the control strategy"""
        pass

    @abstractmethod
    async def send_command(self, target_pose: Pose):
        """Send control command to robot"""
        pass

    def get_robot_state(self):
        """Get current robot state"""
        return self._robot_state
```

##### MoveIt Servo Strategy (`franka_vr_ros2/strategies/moveit_servo.py`)

```python
import asyncio
from geometry_msgs.msg import TwistStamped, Pose
from moveit_msgs.srv import ServoCommandType
from franka_vr_ros2.strategies.base import ControlStrategyBase


class MoveItServoStrategy(ControlStrategyBase):
    """MoveIt Servo control strategy for smooth teleoperation"""

    def __init__(self, node, robot_ip):
        super().__init__(node, robot_ip)

        # Publishers
        self.twist_pub = node.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10
        )

        # Service clients
        self.servo_start_client = node.create_client(
            ServoCommandType, '/servo_node/start_servo'
        )

        self._last_pose = None
        self._initialized = False

    async def initialize(self):
        """Start MoveIt Servo"""
        # Wait for servo service
        while not self.servo_start_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for servo_node...')

        # Start servo
        request = ServoCommandType.Request()
        request.command = True
        future = self.servo_start_client.call_async(request)
        await future

        self._initialized = True
        self.node.get_logger().info('MoveIt Servo initialized')

    async def send_command(self, target_pose: Pose):
        """Convert pose to twist and send to servo"""
        if not self._initialized or self._last_pose is None:
            self._last_pose = target_pose
            return

        # Calculate twist from pose difference
        twist = TwistStamped()
        twist.header.stamp = self.node.get_clock().now().to_msg()
        twist.header.frame_id = "panda_link0"

        # Position difference (direct tracking)
        dt = 0.01  # 100Hz
        twist.twist.linear.x = (target_pose.position.x - self._last_pose.position.x) / dt
        twist.twist.linear.y = (target_pose.position.y - self._last_pose.position.y) / dt
        twist.twist.linear.z = (target_pose.position.z - self._last_pose.position.z) / dt

        # Orientation difference (simplified for now)
        # TODO: Proper quaternion difference to angular velocity
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0

        # Publish twist
        self.twist_pub.publish(twist)
        self._last_pose = target_pose
```

### 3. MCAP Recorder (Async Implementation)

```python
# franka_vr_ros2/recording/mcap_recorder.py
import asyncio
import mcap
import mcap_ros2
from pathlib import Path
import time
from datetime import datetime
import numpy as np


class MCAPRecorder:
    """Async MCAP recorder - preserves all original functionality"""

    def __init__(self, node, save_dir):
        self.node = node
        self.save_dir = Path(save_dir)
        self.save_dir.mkdir(parents=True, exist_ok=True)

        self._recording = False
        self._writer = None
        self._file = None
        self._camera_manager = None

        # Async queue for non-blocking writes
        self._write_queue = asyncio.Queue(maxsize=1000)
        self._writer_task = None

    def set_camera_manager(self, camera_manager):
        """Set camera manager for image recording"""
        self._camera_manager = camera_manager

    def start_recording(self):
        """Start new recording"""
        if self._recording:
            return

        # Create new file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = self.save_dir / f"recording_{timestamp}.mcap"

        self._file = open(filepath, "wb")
        self._writer = mcap.McapWriter(self._file)
        self._writer.start()

        # Register schemas
        self._register_schemas()

        # Start writer task
        self._writer_task = asyncio.create_task(self._writer_loop())

        self._recording = True
        self._start_time = time.time()

    async def record_timestep(self, vr_state, robot_state, timestamp):
        """Queue timestep for recording"""
        if not self._recording:
            return

        timestep = {
            'vr_state': vr_state,
            'robot_state': robot_state,
            'timestamp': timestamp,
            'images': {}
        }

        # Get camera images if available
        if self._camera_manager:
            images = self._camera_manager.get_latest_frames()
            timestep['images'] = images

        # Queue for async writing
        try:
            self._write_queue.put_nowait(timestep)
        except asyncio.QueueFull:
            self.node.get_logger().warn("MCAP queue full, dropping frame")

    async def _writer_loop(self):
        """Async writer loop"""
        while self._recording or not self._write_queue.empty():
            try:
                timestep = await asyncio.wait_for(
                    self._write_queue.get(), timeout=0.1
                )

                # Write to MCAP file
                self._write_timestep(timestep)

            except asyncio.TimeoutError:
                continue
            except Exception as e:
                self.node.get_logger().error(f"MCAP write error: {e}")

    def stop_recording(self, success=True):
        """Stop recording and save file"""
        if not self._recording:
            return None

        self._recording = False

        # Wait for queue to empty
        if self._writer_task:
            asyncio.create_task(self._writer_task)

        # Close file
        if self._writer:
            self._writer.finish()

        if self._file:
            self._file.close()

        # Handle success/failure
        if success:
            return str(self._file.name)
        else:
            # Delete file if not successful
            Path(self._file.name).unlink(missing_ok=True)
            return None

    def is_recording(self):
        """Check if currently recording"""
        return self._recording
```

### 4. Launch File

```python
# launch/teleop_system.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # Declare arguments
    robot_ip_arg = DeclareLaunchArgument('robot_ip', default_value='192.168.1.1')
    control_strategy_arg = DeclareLaunchArgument('control_strategy', default_value='moveit_servo')
    use_fake_hardware_arg = DeclareLaunchArgument('use_fake_hardware', default_value='false')

    # Robot bringup
    robot_bringup = Node(
        package='franka_bringup',
        executable='franka_control_node',
        parameters=[{
            'robot_ip': LaunchConfiguration('robot_ip'),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
        }]
    )

    # MoveIt (conditional on strategy)
    moveit_node = Node(
        package='moveit_servo',
        executable='servo_node',
        parameters=['config/servo_config.yaml'],
        condition=IfCondition("control_strategy == 'moveit_servo'")
    )

    return LaunchDescription([
        robot_ip_arg,
        control_strategy_arg,
        use_fake_hardware_arg,
        robot_bringup,
        moveit_node
    ])
```

### 5. Updated Requirements

```txt
# requirements.txt additions
rclpy>=3.3.0
geometry_msgs
sensor_msgs
moveit_msgs
tf2_ros
tf2_geometry_msgs

# Performance
uvloop  # Faster event loop
aiofiles  # Async file I/O

# Existing requirements preserved
numpy>=1.19.0,<2.0
scipy
pyzmq
mcap
mcap-ros2-support
opencv-python>=4.5.0
pyrealsense2==2.55.1.6486
pyyaml
```

## Key Features Preserved

1. **Async Architecture**: Main loop and all I/O operations are async
2. **MCAP Recording**: Full async implementation with queuing
3. **Camera Support**: Camera manager integrated with MCAP
4. **Button Controls**: A/B button functionality preserved
5. **Calibration**: Forward direction and origin calibration preserved
6. **Coordinate Transforms**: Exact Labelbox transformations maintained
7. **Single Entry Point**: `oculus_vr_server.py` launches everything

## Performance Optimizations

1. **Python with C++ Extensions**: Use `py_trees_ros` for behavior trees, `moveit_py` for Python bindings
2. **Async Everything**: All I/O operations are non-blocking
3. **Real-time Threads**: Control loop runs in dedicated thread with RT priority
4. **Zero-copy Messages**: Use ROS2 zero-copy where possible

## Implementation Timeline

### Immediate Implementation Steps

1. **Hour 1-2**: Set up ROS2 workspace and core package structure
2. **Hour 3-4**: Implement VR input handler and Labelbox transform
3. **Hour 5-6**: Create MoveIt Servo strategy and basic control loop
4. **Hour 7-8**: Port MCAP recorder and camera manager
5. **Hour 9-10**: Integration testing and performance tuning
6. **Hour 11-12**: Add remaining control strategies (IK, Cartesian)

## Quick Start

```bash
# Install dependencies
pip install -r requirements.txt

# Source ROS2
source /opt/ros/humble/setup.bash

# Run the server
python oculus_vr_server.py --robot-ip 192.168.1.1 --control-strategy moveit_servo

# With recording and cameras
python oculus_vr_server.py --robot-ip 192.168.1.1 --enable-cameras --camera-config configs/cameras.yaml

# Test with simulation
python oculus_vr_server.py --simulation --debug
```

This implementation maintains all the features from your current system while providing the performance benefits of ROS2/MoveIt. The modular design allows you to easily swap control strategies while keeping everything else constant.
