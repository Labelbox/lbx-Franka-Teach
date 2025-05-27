"""
FR3 Simulated Robot Controller
Provides the same interface as the real robot but runs in simulation
"""

import numpy as np
import time
import threading
from typing import Tuple, Optional
from scipy.spatial.transform import Rotation as R

from .fr3_robot_model import FR3RobotModel
from .fr3_pybullet_visualizer import FR3PyBulletVisualizer


class FR3SimController:
    """
    Simulated FR3 robot controller that mimics the real robot interface
    """
    
    def __init__(self, visualize: bool = True, update_rate: float = 50.0):
        """
        Initialize simulated robot controller
        
        Args:
            visualize: Whether to show 3D visualization
            update_rate: Simulation update rate in Hz
        """
        self.robot_model = FR3RobotModel()
        self.visualize = visualize
        self.update_rate = update_rate
        self.update_interval = 1.0 / update_rate
        
        # Initialize visualizer if requested
        self.visualizer = None
        if self.visualize:
            self.visualizer = FR3PyBulletVisualizer(self.robot_model, gui=True)
        
        # Robot state
        self.joint_angles = self.robot_model.rest_pose.copy()
        self.joint_velocities = np.zeros(7)
        self.target_joint_angles = self.joint_angles.copy()
        self.gripper_state = 0.0  # 0 = open, 1 = closed
        self.target_gripper_state = 0.0
        
        # Cartesian state (computed from forward kinematics)
        self.ee_pos, self.ee_quat = self.robot_model.forward_kinematics(self.joint_angles)
        
        # Control parameters
        self.position_gain = 10.0  # P gain for position control
        self.velocity_damping = 2.0  # D gain for velocity damping
        self.max_joint_velocity = 2.0  # rad/s
        self.gripper_speed = 5.0  # gripper units/s
        
        # Simulation thread
        self.running = False
        self.sim_thread = None
        
        # Thread lock for state access
        self.state_lock = threading.Lock()
        
        # Trajectory recording
        self.record_trajectory = False
        self.trajectory_history = []
        
    def start(self):
        """Start the simulation thread"""
        if not self.running:
            self.running = True
            self.sim_thread = threading.Thread(target=self._simulation_loop)
            self.sim_thread.daemon = True
            self.sim_thread.start()
            print("🤖 FR3 Simulation started")
    
    def stop(self):
        """Stop the simulation thread"""
        if self.running:
            self.running = False
            if self.sim_thread:
                self.sim_thread.join(timeout=1.0)
            if self.visualizer:
                self.visualizer.close()
            print("🛑 FR3 Simulation stopped")
    
    def _simulation_loop(self):
        """Main simulation loop"""
        last_time = time.time()
        
        while self.running:
            current_time = time.time()
            dt = current_time - last_time
            
            if dt >= self.update_interval:
                # Update robot state
                self._update_robot_state(dt)
                
                # Update visualization
                if self.visualizer:
                    self.visualizer.update_robot_pose(
                        self.joint_angles,
                        gripper_state=self.gripper_state,
                        show_trajectory=self.record_trajectory
                    )
                
                last_time = current_time
            
            # Small sleep to prevent CPU spinning
            time.sleep(0.001)
    
    def _update_robot_state(self, dt: float):
        """Update robot state based on control inputs"""
        with self.state_lock:
            # Compute joint errors
            joint_errors = self.target_joint_angles - self.joint_angles
            
            # Simple PD control for joints
            desired_velocities = self.position_gain * joint_errors - self.velocity_damping * self.joint_velocities
            
            # Limit velocities
            desired_velocities = np.clip(desired_velocities, -self.max_joint_velocity, self.max_joint_velocity)
            
            # Update joint velocities and positions
            self.joint_velocities = desired_velocities
            self.joint_angles += self.joint_velocities * dt
            
            # Ensure joint limits
            self.joint_angles = self.robot_model.clip_joint_angles(self.joint_angles)
            
            # Update gripper
            gripper_error = self.target_gripper_state - self.gripper_state
            gripper_velocity = np.clip(self.gripper_speed * gripper_error, -self.gripper_speed, self.gripper_speed)
            self.gripper_state += gripper_velocity * dt
            self.gripper_state = np.clip(self.gripper_state, 0.0, 1.0)
            
            # Update Cartesian state
            self.ee_pos, self.ee_quat = self.robot_model.forward_kinematics(self.joint_angles)
            
            # Record trajectory if enabled
            if self.record_trajectory:
                self.trajectory_history.append({
                    'time': time.time(),
                    'joint_angles': self.joint_angles.copy(),
                    'ee_pos': self.ee_pos.copy(),
                    'ee_quat': self.ee_quat.copy(),
                    'gripper': self.gripper_state
                })
    
    def get_state(self) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        Get current robot state
        
        Returns:
            ee_pos: End-effector position
            ee_quat: End-effector quaternion (x,y,z,w)
            gripper: Gripper state (0-1)
        """
        with self.state_lock:
            return self.ee_pos.copy(), self.ee_quat.copy(), self.gripper_state
    
    def set_target_pose(self, pos: np.ndarray, quat: np.ndarray, gripper: float):
        """
        Set target end-effector pose (simplified - uses current joint config)
        
        Args:
            pos: Target position
            quat: Target quaternion (x,y,z,w)
            gripper: Target gripper state (0-1)
        """
        with self.state_lock:
            # For simulation, we'll use a simplified approach
            # In reality, this would use inverse kinematics
            
            # Compute position error
            pos_error = pos - self.ee_pos
            
            # Compute orientation error
            current_rot = R.from_quat(self.ee_quat)
            target_rot = R.from_quat(quat)
            rot_error = target_rot * current_rot.inv()
            axis_angle = rot_error.as_rotvec()
            
            # Use Jacobian to compute joint velocities
            J = self.robot_model.jacobian(self.joint_angles)
            
            # Stack position and orientation errors
            cartesian_error = np.concatenate([pos_error, axis_angle])
            
            # Compute joint space error using pseudo-inverse
            try:
                J_pinv = np.linalg.pinv(J)
                joint_error = J_pinv @ cartesian_error
                
                # Scale and add to current joint angles
                self.target_joint_angles = self.joint_angles + 0.1 * joint_error
                self.target_joint_angles = self.robot_model.clip_joint_angles(self.target_joint_angles)
            except:
                # If Jacobian is singular, maintain current position
                pass
            
            # Set gripper target
            self.target_gripper_state = np.clip(gripper, 0.0, 1.0)
            
            # Show target marker in visualizer
            if self.visualizer:
                self.visualizer.set_target_marker(pos, quat)
    
    def reset_to_home(self):
        """Reset robot to home position"""
        with self.state_lock:
            self.target_joint_angles = self.robot_model.rest_pose.copy()
            self.target_gripper_state = 0.0
            
        # Wait for robot to reach home position
        print("🏠 Resetting to home position...")
        time.sleep(2.0)  # Give time for movement
        
        with self.state_lock:
            return self.ee_pos.copy(), self.ee_quat.copy()
    
    def set_trajectory_recording(self, enable: bool):
        """Enable/disable trajectory recording"""
        self.record_trajectory = enable
        if not enable and self.trajectory_history:
            print(f"📊 Recorded {len(self.trajectory_history)} trajectory points")
    
    def get_trajectory_history(self):
        """Get recorded trajectory history"""
        return self.trajectory_history.copy()
    
    def clear_trajectory_history(self):
        """Clear trajectory history"""
        self.trajectory_history.clear()
        if self.visualizer:
            self.visualizer.clear_trajectory() 