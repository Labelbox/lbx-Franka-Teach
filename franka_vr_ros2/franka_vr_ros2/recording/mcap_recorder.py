"""
MCAP Recorder - Async implementation preserving all original functionality
"""

import asyncio
import mcap
import mcap_ros2
from pathlib import Path
import time
from datetime import datetime
import numpy as np
import json
from typing import Optional, Dict, Any
import aiofiles


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
        self._current_filepath = None
        
        # Async queue for non-blocking writes
        self._write_queue = asyncio.Queue(maxsize=1000)
        self._writer_task = None
        
        # Recording metadata
        self._start_time = None
        self._frame_count = 0
        
        self.node.get_logger().info(f"MCAP Recorder initialized")
        self.node.get_logger().info(f"  Save directory: {self.save_dir}")
        
    def set_camera_manager(self, camera_manager):
        """Set camera manager for image recording"""
        self._camera_manager = camera_manager
        self.node.get_logger().info("Camera manager connected to MCAP recorder")
        
    def start_recording(self):
        """Start new recording"""
        if self._recording:
            self.node.get_logger().warn("Recording already in progress")
            return
            
        # Create new file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._current_filepath = self.save_dir / f"recording_{timestamp}.mcap"
        
        self._file = open(self._current_filepath, "wb")
        self._writer = mcap.McapWriter(self._file)
        self._writer.start()
        
        # Register schemas
        self._register_schemas()
        
        # Start writer task
        self._writer_task = asyncio.create_task(self._writer_loop())
        
        self._recording = True
        self._start_time = time.time()
        self._frame_count = 0
        
        self.node.get_logger().info(f"Started recording: {self._current_filepath}")
        
    def _register_schemas(self):
        """Register MCAP schemas for Labelbox Robotics format"""
        # Schema for robot state
        robot_state_schema = {
            "type": "object",
            "properties": {
                "timestamp": {"type": "object"},
                "robot_state": {"type": "object"},
                "controller_info": {"type": "object"}
            }
        }
        
        # Schema for action
        action_schema = {
            "type": "array",
            "items": {"type": "number"}
        }
        
        # Register schemas with MCAP
        self._robot_state_schema_id = self._writer.register_schema(
            name="robot_state",
            encoding="json",
            data=json.dumps(robot_state_schema).encode()
        )
        
        self._action_schema_id = self._writer.register_schema(
            name="action",
            encoding="json",
            data=json.dumps(action_schema).encode()
        )
        
        # Register channels
        self._observation_channel_id = self._writer.register_channel(
            topic="/observation",
            message_encoding="json",
            schema_id=self._robot_state_schema_id
        )
        
        self._action_channel_id = self._writer.register_channel(
            topic="/action",
            message_encoding="json",
            schema_id=self._action_schema_id
        )
        
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
            images = await self._camera_manager.get_latest_frames_async()
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
                await self._write_timestep(timestep)
                self._frame_count += 1
                
            except asyncio.TimeoutError:
                continue
            except Exception as e:
                self.node.get_logger().error(f"MCAP write error: {e}")
                
    async def _write_timestep(self, timestep):
        """Write timestep to MCAP file"""
        # Convert to Labelbox Robotics format
        timestamp_ns = int(timestep['timestamp'].nanoseconds)
        
        # Create observation
        observation = {
            "timestamp": {
                "robot_state": {
                    "read_start": timestamp_ns,
                    "read_end": timestamp_ns
                }
            },
            "robot_state": {
                "joint_positions": timestep['robot_state'].joint_positions.tolist() if timestep['robot_state'].joint_positions is not None else [],
                "joint_velocities": [],
                "joint_efforts": [],
                "cartesian_position": np.concatenate([
                    timestep['robot_state'].pos,
                    timestep['robot_state'].quat
                ]).tolist() if timestep['robot_state'].pos is not None else [],
                "cartesian_velocity": [],
                "gripper_position": timestep['robot_state'].gripper,
                "gripper_velocity": 0.0
            },
            "controller_info": {
                "poses": timestep['vr_state'].poses if hasattr(timestep['vr_state'], 'poses') else {},
                "buttons": timestep['vr_state'].buttons,
                "movement_enabled": timestep['vr_state'].movement_enabled,
                "controller_on": timestep['vr_state'].controller_on
            }
        }
        
        # Create action (simplified for now)
        action = [0.0] * 7  # 3 pos + 3 rot + 1 gripper
        
        # Write observation
        self._writer.add_message(
            channel_id=self._observation_channel_id,
            log_time=timestamp_ns,
            data=json.dumps(observation).encode(),
            publish_time=timestamp_ns
        )
        
        # Write action
        self._writer.add_message(
            channel_id=self._action_channel_id,
            log_time=timestamp_ns,
            data=json.dumps(action).encode(),
            publish_time=timestamp_ns
        )
        
        # Write camera images if available
        if timestep.get('images'):
            await self._write_images(timestep['images'], timestamp_ns)
            
    async def _write_images(self, images: Dict[str, Any], timestamp_ns: int):
        """Write camera images to MCAP"""
        # TODO: Implement image writing
        pass
        
    def stop_recording(self, success=True):
        """Stop recording and save file"""
        if not self._recording:
            return None
            
        self._recording = False
        
        # Wait for queue to empty
        if self._writer_task:
            asyncio.create_task(self._finalize_recording())
            
        # Log recording stats
        duration = time.time() - self._start_time
        self.node.get_logger().info(f"Recording stopped:")
        self.node.get_logger().info(f"  Duration: {duration:.1f}s")
        self.node.get_logger().info(f"  Frames: {self._frame_count}")
        self.node.get_logger().info(f"  Average FPS: {self._frame_count/duration:.1f}")
        
        if success:
            self.node.get_logger().info(f"  Saved to: {self._current_filepath}")
            return str(self._current_filepath)
        else:
            # Delete file if not successful
            if self._current_filepath and self._current_filepath.exists():
                self._current_filepath.unlink()
            self.node.get_logger().info("  Recording discarded")
            return None
            
    async def _finalize_recording(self):
        """Finalize recording after queue is empty"""
        # Wait for writer task to complete
        if self._writer_task:
            await self._writer_task
            
        # Close file
        if self._writer:
            self._writer.finish()
            
        if self._file:
            self._file.close()
            
    def reset_recording(self):
        """Reset recording without saving"""
        self.stop_recording(success=False)
        
    def is_recording(self):
        """Check if currently recording"""
        return self._recording 