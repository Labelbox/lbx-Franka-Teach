#!/usr/bin/env python3
"""
Ultimate robot testing and diagnostic tool for Franka robot system.
Combines system diagnostics, health monitoring, and comprehensive robot testing.
"""

import time
import pickle
import numpy as np
import subprocess
import psutil
import socket
import sys
from frankateach.network import create_request_socket
from frankateach.constants import (
    HOST, CONTROL_PORT, GRIPPER_OPEN, GRIPPER_CLOSE, VR_CONTROLLER_STATE_PORT,
    ROBOT_WORKSPACE_MIN, ROBOT_WORKSPACE_MAX
)
from frankateach.messages import FrankaAction, FrankaState
from deoxys.utils import transform_utils


class UltimateRobotTester:
    def __init__(self):
        """Initialize the ultimate robot tester."""
        print(f"🤖 ULTIMATE ROBOT TESTER")
        print(f"System diagnostics + comprehensive robot testing")
        print(f"Connecting to robot server...")
        
        try:
            self.action_socket = create_request_socket(HOST, CONTROL_PORT)
            print(f"✅ Connected to robot server at {HOST}:{CONTROL_PORT}")
            self.connected = True
        except Exception as e:
            print(f"❌ Failed to connect to robot server: {e}")
            self.connected = False
    
    # ==================== SYSTEM DIAGNOSTICS ====================
    
    def check_network_services(self):
        """Check if required network services are running."""
        print("\n📡 NETWORK SERVICES")
        print("=" * 40)
        
        services = [
            (HOST, CONTROL_PORT, "Franka Server"),
            (HOST, VR_CONTROLLER_STATE_PORT, "VR Controller Service"),
        ]
        
        all_ok = True
        for host, port, name in services:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(3)
                result = sock.connect_ex((host, port))
                sock.close()
                
                if result == 0:
                    print(f"✅ {name} running on {port}")
                else:
                    print(f"❌ {name} NOT running on {port}")
                    if name == "Franka Server":  # Critical service
                        all_ok = False
            except Exception as e:
                print(f"❌ Error checking {name}: {e}")
                all_ok = False
        
        return all_ok
    
    def check_processes(self):
        """Check if required processes are running."""
        print("\n🔄 PROCESSES")
        print("=" * 40)
        
        processes = [
            ("franka_server.py", "Franka Server", True),
            ("franka-interface", "Deoxys Interface", True),
            ("auto_arm.sh", "Auto Arm Script", False),  # Optional
        ]
        
        all_ok = True
        for pattern, name, required in processes:
            try:
                result = subprocess.run(['pgrep', '-f', pattern], 
                                      capture_output=True, text=True)
                if result.returncode == 0:
                    pids = result.stdout.strip().split('\n')
                    print(f"✅ {name} running (PID: {pids[0]})")
                else:
                    print(f"❌ {name} NOT running")
                    if required:
                        all_ok = False
            except Exception as e:
                print(f"❌ Error checking {name}: {e}")
                if required:
                    all_ok = False
        
        return all_ok
    
    def check_system_health(self):
        """Check system resource health."""
        print("\n🖥️  SYSTEM HEALTH")
        print("=" * 40)
        
        try:
            cpu = psutil.cpu_percent(interval=1)
            memory = psutil.virtual_memory()
            
            print(f"CPU: {cpu:.1f}%")
            print(f"Memory: {memory.percent:.1f}%")
            
            if cpu > 80 or memory.percent > 90:
                print(f"⚠️  High resource usage detected")
                return False
            else:
                print(f"✅ System resources healthy")
                return True
        except:
            print(f"❓ Could not check system resources")
            return True
    
    # ==================== ROBOT TESTING ====================
    
    def get_robot_state(self):
        """Get current robot state."""
        if not self.connected:
            return None
            
        try:
            self.action_socket.send(b"get_state")
            response = self.action_socket.recv()
            robot_state = pickle.loads(response)
            
            if robot_state == b"state_error":
                return None
            return robot_state
        except Exception as e:
            print(f"Error getting robot state: {e}")
            return None
    
    def check_position_in_bounds(self, pos):
        """Check if position is within workspace bounds."""
        in_bounds = np.all(pos >= ROBOT_WORKSPACE_MIN) and np.all(pos <= ROBOT_WORKSPACE_MAX)
        return in_bounds
    
    def move_to_safe_position(self):
        """Move robot to a safe position within workspace bounds."""
        print("\n🏠 MOVING TO SAFE POSITION")
        print("=" * 40)
        
        # Get current state
        current_state = self.get_robot_state()
        if current_state is None:
            print("❌ Cannot get robot state")
            return False
        
        current_pos = current_state.pos.copy()
        print(f"Current position: {current_pos}")
        print(f"Workspace bounds: X=[{ROBOT_WORKSPACE_MIN[0]:.2f}, {ROBOT_WORKSPACE_MAX[0]:.2f}], "
              f"Y=[{ROBOT_WORKSPACE_MIN[1]:.2f}, {ROBOT_WORKSPACE_MAX[1]:.2f}], "
              f"Z=[{ROBOT_WORKSPACE_MIN[2]:.2f}, {ROBOT_WORKSPACE_MAX[2]:.2f}]")
        
        # Check if current position is in bounds
        if self.check_position_in_bounds(current_pos):
            print("✅ Robot is already within workspace bounds")
            return True
        
        print("⚠️  Robot is outside workspace bounds!")
        
        # Define a safe center position
        safe_pos = np.array([0.4, 0.0, 0.3])  # Center of workspace
        print(f"Moving to safe position: {safe_pos}")
        
        # Create action to move to safe position
        action = FrankaAction(
            pos=safe_pos.flatten().astype(np.float32),
            quat=current_state.quat.flatten().astype(np.float32),
            gripper=current_state.gripper,
            reset=False,
            timestamp=time.time(),
        )
        
        try:
            self.action_socket.send(bytes(pickle.dumps(action, protocol=-1)))
            response = self.action_socket.recv()
            result_state = pickle.loads(response)
            
            if result_state != b"state_error":
                new_pos = result_state.pos
                movement = np.linalg.norm(new_pos - current_pos)
                print(f"New position: {new_pos}")
                print(f"Movement: {movement*1000:.2f}mm")
                
                if self.check_position_in_bounds(new_pos):
                    print("✅ Successfully moved to safe position")
                    return True
                else:
                    print("❌ Still outside bounds after movement")
                    return False
            else:
                print("❌ Error moving to safe position")
                return False
                
        except Exception as e:
            print(f"❌ Failed to move to safe position: {e}")
            return False
    
    def reset_robot(self):
        """Reset robot to initial position."""
        print("Resetting robot...")
        
        action = FrankaAction(
            pos=np.zeros(3),
            quat=np.zeros(4),
            gripper=GRIPPER_OPEN,
            reset=True,
            timestamp=time.time(),
        )
        
        try:
            self.action_socket.send(bytes(pickle.dumps(action, protocol=-1)))
            response = self.action_socket.recv()
            robot_state = pickle.loads(response)
            print(f"✅ Robot reset to: {robot_state.pos}")
            return robot_state
        except Exception as e:
            print(f"❌ Reset failed: {e}")
            return None
    
    def test_communication(self):
        """Test basic robot communication."""
        print("\n🔗 COMMUNICATION TEST")
        print("=" * 40)
        
        if not self.connected:
            print("❌ Not connected to robot server")
            return False
        
        state = self.get_robot_state()
        if state is not None:
            print(f"✅ Communication successful")
            print(f"   Position: {state.pos}")
            print(f"   Gripper: {state.gripper}")
            
            # Check if position is in bounds
            if not self.check_position_in_bounds(state.pos):
                print(f"   ⚠️  WARNING: Robot is outside workspace bounds!")
            
            return True
        else:
            print(f"❌ Communication failed")
            return False
    
    def test_movement(self, detailed=True):
        """Test robot movement capability."""
        print(f"\n🎯 MOVEMENT TEST {'(DETAILED)' if detailed else '(QUICK)'}")
        print("=" * 40)
        
        if not self.connected:
            print("❌ Not connected")
            return False
        
        # First ensure robot is in safe position
        current_state = self.get_robot_state()
        if current_state is None:
            print("❌ Cannot get robot state")
            return False
        
        if not self.check_position_in_bounds(current_state.pos):
            print("⚠️  Robot outside bounds - moving to safe position first...")
            if not self.move_to_safe_position():
                print("❌ Cannot move to safe position - aborting test")
                return False
            # Get updated state after moving to safe position
            current_state = self.get_robot_state()
            if current_state is None:
                return False
        
        start_pos = current_state.pos.copy()
        start_quat = current_state.quat.copy()
        
        print(f"Starting position: {start_pos}")
        print("Using VR-style incremental movements...")
        
        # Define test movements (smaller, incremental movements)
        if detailed:
            movements = [
                ("2mm X (incremental)", np.array([0.002, 0.0, 0.0])),
                ("2mm Y (incremental)", np.array([0.0, 0.002, 0.0])),
                ("2mm Z (incremental)", np.array([0.0, 0.0, 0.002])),
            ]
        else:
            movements = [
                ("1mm X (incremental)", np.array([0.001, 0.0, 0.0])),
            ]
        
        success_count = 0
        
        # VR-style movement: send small increments multiple times
        for description, offset in movements:
            print(f"\n   Testing {description} movement...")
            
            # Get fresh state before each movement
            current_state = self.get_robot_state()
            if current_state is None:
                continue
                
            current_pos = current_state.pos.copy()
            
            # Send multiple small steps (like VR controller would)
            num_steps = 5
            step_size = offset / num_steps
            
            for step in range(num_steps):
                # Calculate incremental target
                target_pos = current_pos + step_size
                
                # Ensure target is within bounds
                if not self.check_position_in_bounds(target_pos):
                    print(f"   ⚠️  Step {step+1} would go outside bounds, stopping")
                    break
                
                action = FrankaAction(
                    pos=target_pos.flatten().astype(np.float32),
                    quat=current_state.quat.flatten().astype(np.float32),
                    gripper=current_state.gripper,
                    reset=False,
                    timestamp=time.time(),
                )
                
                try:
                    self.action_socket.send(bytes(pickle.dumps(action, protocol=-1)))
                    response = self.action_socket.recv()
                    result_state = pickle.loads(response)
                    
                    # Update current position for next step
                    current_pos = result_state.pos.copy()
                    
                    # Small delay between steps (like VR would have)
                    time.sleep(0.05)  # 50ms, similar to VR update rate
                    
                except Exception as e:
                    print(f"   ❌ Step {step+1} failed: {e}")
                    break
            
            # Check total movement
            final_state = self.get_robot_state()
            if final_state:
                actual_movement = np.linalg.norm(final_state.pos - start_pos)
                expected_movement = np.linalg.norm(offset)
                
                print(f"   Expected total: {expected_movement*1000:.1f}mm")
                print(f"   Actual total: {actual_movement*1000:.1f}mm")
                
                if actual_movement > 0.0005:  # More than 0.5mm
                    print(f"   ✅ Robot moved!")
                    success_count += 1
                    start_pos = final_state.pos.copy()  # Update for next test
                else:
                    print(f"   ❌ No significant movement detected")
            
            time.sleep(0.5)
        
        success = success_count > 0
        print(f"\n📊 Movement Result: {success_count}/{len(movements)} successful")
        
        if success:
            print(f"✅ Robot movement is working!")
        else:
            print(f"❌ Robot not moving")
            print(f"   Possible causes:")
            print(f"   - Deoxys OSC controller issue")
            print(f"   - Movement commands too small")
            print(f"   - Pose interpolator expecting different format")
        
        return success
    
    def reset_to_home(self):
        """Reset robot to home position (like teleoperator does)."""
        print("\n🏠 RESETTING TO HOME POSITION")
        print("=" * 40)
        
        # First reset to default joints
        print("Step 1: Resetting to default joint configuration...")
        reset_action = FrankaAction(
            pos=np.zeros(3),
            quat=np.zeros(4),
            gripper=GRIPPER_OPEN,
            reset=True,
            timestamp=time.time(),
        )
        
        try:
            self.action_socket.send(bytes(pickle.dumps(reset_action, protocol=-1)))
            response = self.action_socket.recv()
            robot_state = pickle.loads(response)
            
            if robot_state == b"state_error":
                print("❌ Reset failed")
                return None
                
            print(f"✅ Reset complete. Position: {robot_state.pos}")
            print(f"   Quaternion: {robot_state.quat}")
            
            # Wait for robot to settle
            time.sleep(2)
            
            return robot_state
            
        except Exception as e:
            print(f"❌ Reset failed: {e}")
            return None
    
    def test_large_movement(self, distance_cm=20, direction='x'):
        """Test large robot movement using incremental steps."""
        print(f"\n🚀 LARGE MOVEMENT TEST ({distance_cm}cm in {direction.upper()})")
        print("=" * 40)
        
        if not self.connected:
            print("❌ Not connected")
            return False
        
        # Always reset to home first
        reset_state = self.reset_to_home()
        if reset_state is None:
            print("❌ Failed to reset to home position")
            return False
        
        # Get current state after reset
        current_state = self.get_robot_state()
        if current_state is None:
            print("❌ Cannot get robot state")
            return False
        
        start_pos = current_state.pos.copy()
        print(f"Starting position after reset: {start_pos}")
        
        # Calculate target based on direction
        distance_m = distance_cm / 100.0
        if direction.lower() == 'x':
            offset_vector = np.array([distance_m, 0.0, 0.0])
        elif direction.lower() == 'y':
            offset_vector = np.array([0.0, distance_m, 0.0])
        elif direction.lower() == 'z':
            offset_vector = np.array([0.0, 0.0, distance_m])
        else:
            print(f"❌ Invalid direction: {direction}")
            return False
        
        final_target = start_pos + offset_vector
        
        # Check if final target is within bounds
        if not self.check_position_in_bounds(final_target):
            print(f"❌ Target position {final_target} is outside workspace bounds!")
            print(f"   Workspace: X=[{ROBOT_WORKSPACE_MIN[0]:.2f}, {ROBOT_WORKSPACE_MAX[0]:.2f}], "
                  f"Y=[{ROBOT_WORKSPACE_MIN[1]:.2f}, {ROBOT_WORKSPACE_MAX[1]:.2f}], "
                  f"Z=[{ROBOT_WORKSPACE_MIN[2]:.2f}, {ROBOT_WORKSPACE_MAX[2]:.2f}]")
            
            # Calculate maximum safe distance
            if direction.lower() == 'x':
                max_safe = ROBOT_WORKSPACE_MAX[0] - start_pos[0] if distance_m > 0 else start_pos[0] - ROBOT_WORKSPACE_MIN[0]
            elif direction.lower() == 'y':
                max_safe = ROBOT_WORKSPACE_MAX[1] - start_pos[1] if distance_m > 0 else start_pos[1] - ROBOT_WORKSPACE_MIN[1]
            else:  # z
                max_safe = ROBOT_WORKSPACE_MAX[2] - start_pos[2] if distance_m > 0 else start_pos[2] - ROBOT_WORKSPACE_MIN[2]
            
            max_safe_cm = max_safe * 100
            print(f"   Maximum safe movement: {max_safe_cm:.1f}cm")
            
            if max_safe_cm < 1:
                print("   ❌ No safe movement possible in this direction")
                return False
            
            # Adjust target to stay within bounds
            distance_m = max_safe * 0.95  # Leave 5% margin
            distance_cm = distance_m * 100
            offset_vector = offset_vector / np.linalg.norm(offset_vector) * distance_m
            final_target = start_pos + offset_vector
            print(f"   Adjusted to safe movement: {distance_cm:.1f}cm")
        
        print(f"Target position: {final_target}")
        print(f"Total distance: {distance_cm:.1f}cm")
        
        # Break into small steps (2mm each for safety)
        step_size_m = 0.002  # 2mm per step
        num_steps = int(distance_m / step_size_m)
        
        print(f"Breaking into {num_steps} steps of {step_size_m*1000:.1f}mm each")
        print("Progress: ", end='', flush=True)
        
        current_pos = start_pos.copy()
        success_steps = 0
        
        # Execute movement
        for i in range(num_steps):
            # Calculate next position
            progress = (i + 1) / num_steps
            target_pos = start_pos + offset_vector * progress
            
            # Ensure still within bounds
            if not self.check_position_in_bounds(target_pos):
                print(f"\n⚠️  Step {i+1} would go outside bounds, stopping")
                break
            
            action = FrankaAction(
                pos=target_pos.flatten().astype(np.float32),
                quat=current_state.quat.flatten().astype(np.float32),
                gripper=current_state.gripper,
                reset=False,
                timestamp=time.time(),
            )
            
            try:
                self.action_socket.send(bytes(pickle.dumps(action, protocol=-1)))
                response = self.action_socket.recv()
                result_state = pickle.loads(response)
                
                success_steps += 1
                
                # Print progress every 10%
                if (i + 1) % max(1, num_steps // 10) == 0:
                    print(f"{int(progress * 100)}%", end='', flush=True)
                else:
                    print(".", end='', flush=True)
                
                # Small delay between steps
                time.sleep(0.05)  # 50ms, ~20Hz
                
            except Exception as e:
                print(f"\n❌ Error at step {i+1}: {e}")
                break
        
        print()  # New line after progress
        
        # Check final position
        final_state = self.get_robot_state()
        if final_state:
            actual_movement = np.linalg.norm(final_state.pos - start_pos)
            actual_cm = actual_movement * 100
            
            print(f"\n📊 Results:")
            print(f"   Target movement: {distance_cm:.1f}cm")
            print(f"   Actual movement: {actual_cm:.1f}cm")
            print(f"   Final position: {final_state.pos}")
            print(f"   Success rate: {success_steps}/{num_steps} steps")
            
            if actual_cm > distance_cm * 0.8:  # At least 80% of target
                print(f"   ✅ Large movement successful!")
                return True
            else:
                print(f"   ⚠️  Movement incomplete ({actual_cm/distance_cm*100:.1f}% of target)")
                return actual_cm > 5  # Success if moved at least 5cm
        
        return False
    
    def test_gripper(self):
        """Test gripper functionality with better debugging."""
        print(f"\n🤏 GRIPPER TEST (ENHANCED)")
        print("=" * 40)
        
        if not self.connected:
            print("❌ Not connected")
            return False
        
        current_state = self.get_robot_state()
        if current_state is None:
            return False
        
        pos = current_state.pos.copy()
        quat = current_state.quat.copy()
        
        print(f"Current gripper state: {current_state.gripper}")
        print(f"GRIPPER_OPEN value: {GRIPPER_OPEN}")
        print(f"GRIPPER_CLOSE value: {GRIPPER_CLOSE}")
        
        # Test close
        print("\n Testing gripper CLOSE...")
        action = FrankaAction(pos=pos, quat=quat, gripper=GRIPPER_CLOSE, reset=False, timestamp=time.time())
        
        try:
            print(f"   Sending gripper={GRIPPER_CLOSE}")
            self.action_socket.send(bytes(pickle.dumps(action, protocol=-1)))
            response = self.action_socket.recv()
            result = pickle.loads(response)
            
            print(f"   Response gripper state: {result.gripper}")
            if abs(result.gripper - GRIPPER_CLOSE) < 0.1:
                print("   ✅ Gripper reports closed (software)")
                close_ok = True
            else:
                print("   ❌ Gripper close failed (software)")
                close_ok = False
            
            print("   ⚠️  Check if physical gripper actually closed!")
            time.sleep(2)
            
            # Test open
            print("\n Testing gripper OPEN...")
            action.gripper = GRIPPER_OPEN
            print(f"   Sending gripper={GRIPPER_OPEN}")
            self.action_socket.send(bytes(pickle.dumps(action, protocol=-1)))
            response = self.action_socket.recv()
            result = pickle.loads(response)
            
            print(f"   Response gripper state: {result.gripper}")
            if abs(result.gripper - GRIPPER_OPEN) < 0.1:
                print("   ✅ Gripper reports opened (software)")
                open_ok = True
            else:
                print("   ❌ Gripper open failed (software)")
                open_ok = False
            
            print("   ⚠️  Check if physical gripper actually opened!")
            
            print("\n📝 Gripper Debug Info:")
            print("   - Software reports success but physical gripper not moving?")
            print("   - Check if Franka Hand is powered on (LEDs on gripper)")
            print("   - Check deoxys terminal for gripper errors")
            print("   - Gripper might need separate initialization process")
            
            return close_ok and open_ok
            
        except Exception as e:
            print(f"   ❌ Gripper test failed: {e}")
            return False
    
    def test_reset_and_move(self):
        """Test robot reset to valid position then movement."""
        print("\n🔄 RESET AND MOVEMENT TEST")
        print("=" * 40)
        
        if not self.connected:
            print("❌ Not connected")
            return False
        
        # First, try to reset robot to default joint configuration
        print("Attempting to reset robot to default joint configuration...")
        print("This should move robot to a valid position within workspace bounds")
        
        reset_state = self.reset_robot()
        if reset_state is None:
            print("❌ Reset failed - deoxys may not be connected to robot")
            return False
        
        print(f"Reset complete. New position: {reset_state.pos}")
        
        # Wait for robot to settle
        print("Waiting 3 seconds for robot to settle...")
        time.sleep(3)
        
        # Get current state after reset
        current_state = self.get_robot_state()
        if current_state is None:
            print("❌ Cannot get robot state after reset")
            return False
        
        print(f"Current position after reset: {current_state.pos}")
        
        # Check if now in bounds
        if self.check_position_in_bounds(current_state.pos):
            print("✅ Robot is now within workspace bounds!")
            
            # Try a small movement
            print("\nTesting small movement from reset position...")
            offset = np.array([0.01, 0.0, 0.0])  # 10mm in X
            target_pos = current_state.pos + offset
            
            action = FrankaAction(
                pos=target_pos.flatten().astype(np.float32),
                quat=current_state.quat.flatten().astype(np.float32),
                gripper=current_state.gripper,
                reset=False,
                timestamp=time.time(),
            )
            
            self.action_socket.send(bytes(pickle.dumps(action, protocol=-1)))
            response = self.action_socket.recv()
            result_state = pickle.loads(response)
            
            movement = np.linalg.norm(result_state.pos - current_state.pos)
            print(f"Movement after reset: {movement*1000:.2f}mm")
            
            if movement > 0.005:  # More than 5mm
                print("✅ Robot successfully moved after reset!")
                return True
            else:
                print("❌ Robot still not moving after reset")
                return False
        else:
            print("❌ Robot still outside bounds after reset")
            print("   Reset position may be invalid or deoxys not connected")
            return False
    
    # ==================== TEST MODES ====================
    
    def run_quick_check(self):
        """Quick system status check."""
        print("⚡ QUICK STATUS CHECK")
        print("=" * 50)
        
        network_ok = self.check_network_services()
        comm_ok = self.test_communication()
        
        if network_ok and comm_ok:
            print(f"\n✅ QUICK CHECK: System operational")
            return True
        else:
            print(f"\n❌ QUICK CHECK: Issues detected")
            return False
    
    def run_diagnostics(self):
        """Full system diagnostics."""
        print("🔍 SYSTEM DIAGNOSTICS")
        print("=" * 50)
        
        results = {}
        results['network'] = self.check_network_services()
        results['processes'] = self.check_processes()
        results['health'] = self.check_system_health()
        results['communication'] = self.test_communication()
        results['movement'] = self.test_movement(detailed=False)
        
        # Summary
        print(f"\n{'='*50}")
        print(f"DIAGNOSTIC SUMMARY")
        print(f"{'='*50}")
        
        for check, status in results.items():
            icon = "✅" if status else "❌"
            print(f"{icon} {check.title()}: {'PASS' if status else 'FAIL'}")
        
        passed = sum(results.values())
        total = len(results)
        
        print(f"\n📊 Score: {passed}/{total} checks passed")
        return passed == total
    
    def run_comprehensive_test(self):
        """Full comprehensive robot testing."""
        print("🤖 COMPREHENSIVE ROBOT TEST")
        print("=" * 50)
        
        # Quick system check first
        if not self.run_quick_check():
            print(f"\n❌ System issues detected - fix before robot testing")
            return False
        
        print(f"\n" + "="*50)
        print(f"ROBOT FUNCTIONALITY TESTS")
        print(f"="*50)
        
        # Robot tests
        results = {}
        results['communication'] = self.test_communication()
        
        # Try reset first if robot is out of bounds
        state = self.get_robot_state()
        if state and not self.check_position_in_bounds(state.pos):
            print("\n⚠️  Robot outside bounds - attempting reset first...")
            results['reset'] = self.test_reset_and_move()
            if results['reset']:
                results['movement'] = self.test_movement(detailed=True)
            else:
                results['movement'] = False
                print("❌ Skipping movement test - reset failed")
        else:
            results['movement'] = self.test_movement(detailed=True)
        
        # Test large movement if small movements work
        if results.get('movement', False):
            results['large_movement'] = self.test_large_movement(distance_cm=20, direction='x')
        
        results['gripper'] = self.test_gripper()
        
        # Summary
        print(f"\n{'='*50}")
        print(f"ROBOT TEST SUMMARY")
        print(f"{'='*50}")
        
        for test, status in results.items():
            icon = "✅" if status else "❌"
            print(f"{icon} {test.replace('_', ' ').title()}: {'PASS' if status else 'FAIL'}")
        
        passed = sum(results.values())
        total = len(results)
        
        print(f"\n📊 Robot Score: {passed}/{total} tests passed")
        
        if passed == total:
            print(f"\n🎉 ALL TESTS PASSED!")
            print(f"✅ Robot system fully functional")
            print(f"✅ Robot can perform large movements!")
        elif results['communication']:
            print(f"\n⚠️  PARTIAL SUCCESS")
            print(f"✅ Robot responds to commands")
            if not results.get('movement', False) and not results.get('reset', False):
                print(f"❌ Movement blocked - check:")
                print(f"   - CPU performance mode")
                print(f"   - Real-time permissions")
                print(f"   - Robot workspace bounds")
                print(f"   - Deoxys connection to robot")
                print(f"\n🔧 CRITICAL: Neither arm nor gripper moving indicates")
                print(f"   deoxys is NOT connected to the physical robot!")
                print(f"\n   Run deoxys with sudo in another terminal:")
                print(f"   cd ~/projects/lbx-Franka-Teach")
                print(f"   ./run_arm_sudo.sh")
        else:
            print(f"\n❌ ROBOT SYSTEM ISSUES")
            print(f"🔧 Core robot system needs attention")
        
        return passed == total
    
    def close(self):
        """Clean up resources."""
        if hasattr(self, 'action_socket') and self.connected:
            self.action_socket.close()


def show_help():
    """Show usage help."""
    print("🤖 ULTIMATE ROBOT TESTER")
    print("=" * 40)
    print("Usage:")
    print("  python robot_test.py              # Full comprehensive test")
    print("  python robot_test.py --quick      # Quick status check")
    print("  python robot_test.py --diagnostic # System diagnostics")
    print("  python robot_test.py --move20     # Test 20cm movement")
    print("  python robot_test.py --gripper    # Test gripper open/close only")
    print("  python robot_test.py --help       # Show this help")
    print()
    print("Modes:")
    print("  --quick      : Fast system status check")
    print("  --diagnostic : Full system diagnostics")
    print("  --move20     : Test 20cm movement in X direction")
    print("  --gripper    : Test gripper open/close only")
    print("  (default)    : Comprehensive robot testing")


def main():
    """Main function."""
    # Parse arguments
    if '--help' in sys.argv or '-h' in sys.argv:
        show_help()
        return
    
    quick_mode = '--quick' in sys.argv or '-q' in sys.argv
    diagnostic_mode = '--diagnostic' in sys.argv or '-d' in sys.argv
    move20_mode = '--move20' in sys.argv
    gripper_mode = '--gripper' in sys.argv
    
    if quick_mode:
        print("⚡ QUICK MODE")
        need_input = False
    elif diagnostic_mode:
        print("🔍 DIAGNOSTIC MODE")
        need_input = False
    elif move20_mode:
        print("🚀 20CM MOVEMENT MODE")
        need_input = False
    elif gripper_mode:
        print("🤏 GRIPPER TEST MODE")
        need_input = False
    else:
        print("🤖 COMPREHENSIVE TEST MODE")
        need_input = True
    
    # Get user confirmation for comprehensive mode
    if need_input:
        try:
            input("\nPress Enter to start comprehensive testing (Ctrl+C to cancel)...")
        except KeyboardInterrupt:
            print("\nTest cancelled")
            return
    
    # Run tests
    tester = UltimateRobotTester()
    
    try:
        if quick_mode:
            success = tester.run_quick_check()
        elif diagnostic_mode:
            success = tester.run_diagnostics()
        elif move20_mode:
            # Quick check first
            if tester.test_communication():
                success = tester.test_large_movement(distance_cm=20, direction='x')
            else:
                print("❌ Cannot communicate with robot")
                success = False
        elif gripper_mode:
            # Quick check first
            if tester.test_communication():
                success = tester.test_gripper()
            else:
                print("❌ Cannot communicate with robot")
                success = False
        else:
            success = tester.run_comprehensive_test()
        
        if success:
            print(f"\n🎯 CONCLUSION: Test successful")
        else:
            print(f"\n🔧 CONCLUSION: Test failed or incomplete")
            
    except KeyboardInterrupt:
        print(f"\n🛑 Test interrupted")
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
    finally:
        tester.close()
        print("Testing finished.")


if __name__ == "__main__":
    main() 