#!/usr/bin/env python3
"""
Interactive VR Controller Calibration Tool
Guides user through various orientations and captures state data
"""

import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import argparse
import sys

# Import the Oculus Reader
from oculus_reader.reader import OculusReader


class VRCalibrationTool:
    def __init__(self, right_controller=True, ip_address=None):
        self.right_controller = right_controller
        self.controller_id = "r" if right_controller else "l"
        
        # Initialize Oculus Reader
        print("🎮 Initializing Oculus Reader...")
        try:
            self.oculus_reader = OculusReader(
                ip_address=ip_address,
                print_FPS=False
            )
            print("✅ Oculus Reader initialized successfully")
        except Exception as e:
            print(f"❌ Failed to initialize Oculus Reader: {e}")
            sys.exit(1)
        
        # Calibration data storage
        self.calibration_data = {}
        
        # List of orientations to test - only 90 degree rotations
        self.orientations = [
            ("neutral", "Hold controller in comfortable neutral position"),
            ("roll_left_90", "Roll controller 90° to the LEFT (tilt left)"),
            ("roll_right_90", "Roll controller 90° to the RIGHT (tilt right)"),
            ("pitch_up_90", "Pitch controller 90° UP (point up)"),
            ("pitch_down_90", "Pitch controller 90° DOWN (point down)"),
            ("yaw_left_90", "Yaw controller 90° LEFT (turn left)"),
            ("yaw_right_90", "Yaw controller 90° RIGHT (turn right)"),
        ]
    
    def capture_state(self, orientation_name, instruction):
        """Capture controller state for a given orientation"""
        print(f"\n{'='*60}")
        print(f"📐 ORIENTATION: {orientation_name.upper().replace('_', ' ')}")
        print(f"{'='*60}")
        print(f"\n📋 Instructions: {instruction}")
        print("\nPress ENTER when ready to capture (or 'q' to quit)...")
        
        user_input = input().strip().lower()
        if user_input == 'q':
            return False
        
        # Capture multiple samples
        print("📸 Capturing data... Hold position steady!")
        samples = []
        sample_count = 10
        
        for i in range(sample_count):
            poses, buttons = self.oculus_reader.get_transformations_and_buttons()
            if self.controller_id in poses:
                samples.append(poses[self.controller_id])
                print(f"   Sample {i+1}/{sample_count} captured", end='\r')
                time.sleep(0.1)
            else:
                print(f"\n⚠️  Controller not detected! Make sure {self.controller_id.upper()} controller is on.")
                return True
        
        if not samples:
            print("\n❌ No data captured!")
            return True
        
        # Average the samples
        avg_pose = np.mean(samples, axis=0)
        
        # Extract rotation and position
        rotation_matrix = avg_pose[:3, :3]
        position = avg_pose[:3, 3]
        
        # Calculate various representations
        rot = R.from_matrix(rotation_matrix)
        quat = rot.as_quat()  # [x, y, z, w]
        euler = rot.as_euler('xyz', degrees=True)
        
        # Store only the averaged data in a clean format
        self.calibration_data[orientation_name] = {
            'position': position.tolist(),
            'quaternion': quat.tolist(),
            'euler': euler.tolist(),
        }
        
        # Print captured data
        print(f"\n\n✅ Data captured for {orientation_name}:")
        print(f"   Position: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]")
        print(f"   Quaternion: [{quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f}]")
        print(f"   Euler (XYZ): [R:{euler[0]:6.1f}, P:{euler[1]:6.1f}, Y:{euler[2]:6.1f}]")
        
        # If we have neutral, show relative rotation
        if 'neutral' in self.calibration_data and orientation_name != 'neutral':
            neutral_rot = R.from_quat(self.calibration_data['neutral']['quaternion'])
            relative_rot = neutral_rot.inv() * rot
            rel_rotvec = relative_rot.as_rotvec()
            rel_angle = np.linalg.norm(rel_rotvec)
            rel_axis = rel_rotvec / rel_angle if rel_angle > 0 else np.array([0, 0, 0])
            
            print(f"\n   📊 Relative to neutral:")
            print(f"   Rotation: {np.degrees(rel_angle):.1f}° around [{rel_axis[0]:.2f}, {rel_axis[1]:.2f}, {rel_axis[2]:.2f}]")
            
            # Euler difference
            neutral_euler = np.array(self.calibration_data['neutral']['euler'])
            euler_diff = euler - neutral_euler
            print(f"   Euler change: [ΔR:{euler_diff[0]:6.1f}, ΔP:{euler_diff[1]:6.1f}, ΔY:{euler_diff[2]:6.1f}]")
            
            # Identify dominant axis
            abs_axis = np.abs(rel_axis)
            if rel_angle > 0:
                dominant_idx = np.argmax(abs_axis)
                axis_names = ['X', 'Y', 'Z']
                print(f"   Dominant axis: {axis_names[dominant_idx]} ({abs_axis[dominant_idx]:.2f})")
        
        return True
    
    def analyze_results(self):
        """Analyze calibration results and suggest axis mappings"""
        print(f"\n\n{'='*60}")
        print("📊 CALIBRATION ANALYSIS")
        print(f"{'='*60}")
        
        if 'neutral' not in self.calibration_data:
            print("❌ No neutral pose captured!")
            return
        
        neutral_rot = R.from_quat(self.calibration_data['neutral']['quaternion'])
        
        # Analyze each 90-degree rotation
        rotations_90 = ['roll_left_90', 'roll_right_90', 'pitch_up_90', 
                        'pitch_down_90', 'yaw_left_90', 'yaw_right_90']
        
        axis_mapping = {'roll': [], 'pitch': [], 'yaw': []}
        
        print("\n🔍 Detailed Axis Analysis:")
        print("-" * 60)
        
        for rot_name in rotations_90:
            if rot_name not in self.calibration_data:
                continue
            
            rot = R.from_quat(self.calibration_data[rot_name]['quaternion'])
            relative_rot = neutral_rot.inv() * rot
            rotvec = relative_rot.as_rotvec()
            angle = np.linalg.norm(rotvec)
            
            if angle > 0:
                axis = rotvec / angle
                abs_axis = np.abs(axis)
                dominant_idx = np.argmax(abs_axis)
                
                # Print detailed info for each rotation
                axis_names = ['X', 'Y', 'Z']
                print(f"\n{rot_name.upper().replace('_', ' ')}:")
                print(f"  Rotation: {np.degrees(angle):.1f}° around [{axis[0]:.3f}, {axis[1]:.3f}, {axis[2]:.3f}]")
                print(f"  Dominant axis: {axis_names[dominant_idx]}-axis ({abs_axis[dominant_idx]:.3f})")
                print(f"  Sign: {'positive' if axis[dominant_idx] > 0 else 'negative'}")
                
                # Determine motion type
                if 'roll' in rot_name:
                    axis_mapping['roll'].append((dominant_idx, axis[dominant_idx], rot_name))
                elif 'pitch' in rot_name:
                    axis_mapping['pitch'].append((dominant_idx, axis[dominant_idx], rot_name))
                elif 'yaw' in rot_name:
                    axis_mapping['yaw'].append((dominant_idx, axis[dominant_idx], rot_name))
        
        print("\n\n🎯 Detected Axis Mappings:")
        print("-" * 60)
        axis_names = ['X', 'Y', 'Z']
        
        for motion, data in axis_mapping.items():
            if data:
                # Find most common axis
                axes = [d[0] for d in data]
                dominant_axis = max(set(axes), key=axes.count)
                
                # Check consistency
                consistent = all(d[0] == dominant_axis for d in data)
                
                print(f"\n{motion.upper()}:")
                print(f"  Primary axis: {axis_names[dominant_axis]}-axis")
                print(f"  Consistency: {'✅ Good' if consistent else '⚠️  Mixed axes detected'}")
                
                # Show details for each direction
                for idx, sign, name in data:
                    direction = name.split('_')[1].upper()
                    print(f"    {direction}: {axis_names[idx]}-axis ({sign:+.3f})")
        
        # Check for euler angle vs axis-angle discrepancies
        print("\n\n⚠️  Euler Angle Analysis (can be misleading due to gimbal lock):")
        print("-" * 60)
        
        # Generate summary table
        print(f"{'Orientation':<20} {'Euler [R, P, Y]':<30} {'Dominant Change':<30}")
        print("-" * 80)
        
        neutral_euler = np.array(self.calibration_data['neutral']['euler'])
        
        for name in self.orientations:
            orientation_name = name[0]
            if orientation_name in self.calibration_data:
                euler = np.array(self.calibration_data[orientation_name]['euler'])
                
                if orientation_name == 'neutral':
                    print(f"{orientation_name:<20} [{euler[0]:6.1f}, {euler[1]:6.1f}, {euler[2]:6.1f}]")
                else:
                    euler_diff = euler - neutral_euler
                    # Find which changed most
                    abs_diff = np.abs(euler_diff)
                    max_idx = np.argmax(abs_diff)
                    change_names = ['Roll', 'Pitch', 'Yaw']
                    
                    print(f"{orientation_name:<20} [{euler[0]:6.1f}, {euler[1]:6.1f}, {euler[2]:6.1f}]   "
                          f"{change_names[max_idx]}: {euler_diff[max_idx]:+.1f}°")
        
        print("\n💡 Note: The axis-angle representation (top section) is more reliable than")
        print("   euler angles for determining the true rotation axes. Euler angles can")
        print("   show misleading results due to gimbal lock and order-dependent coupling.")
        
        # Generate transformation recommendation
        print("\n\n🔧 Recommended Axis Transformation:")
        print("-" * 60)
        
        # Determine the actual axes for each motion
        vr_axes = {}
        for motion, data in axis_mapping.items():
            if data:
                axes = [d[0] for d in data]
                dominant_axis = max(set(axes), key=axes.count)
                vr_axes[motion] = axis_names[dominant_axis]
        
        if len(vr_axes) == 3:
            print(f"\nVR Controller Axes:")
            print(f"  Roll:  {vr_axes.get('roll', '?')}-axis")
            print(f"  Pitch: {vr_axes.get('pitch', '?')}-axis")
            print(f"  Yaw:   {vr_axes.get('yaw', '?')}-axis")
            
            print(f"\nFor standard robot mapping (Roll=X, Pitch=Y, Yaw=Z):")
            print(f"  Transformation needed: VR [{vr_axes.get('pitch', '?')}, {vr_axes.get('yaw', '?')}, {vr_axes.get('roll', '?')}] → Robot [X, Y, Z]")
        else:
            print("\n⚠️  Incomplete data - capture all orientations for full analysis")
    
    def save_results(self, filename="vr_calibration_data.txt"):
        """Save calibration results to file in a clean format"""
        import json
        
        # Save as JSON for easy parsing
        json_filename = filename.replace('.txt', '.json')
        with open(json_filename, 'w') as f:
            json.dump(self.calibration_data, f, indent=2)
        print(f"\n💾 JSON data saved to {json_filename}")
        
        # Also save human-readable format
        with open(filename, 'w') as f:
            f.write("VR Controller Calibration Data\n")
            f.write("=" * 60 + "\n")
            f.write(f"Controller: {'Right' if self.right_controller else 'Left'}\n")
            f.write(f"Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write("=" * 60 + "\n\n")
            
            # Print in a clean table format
            f.write("Averaged Data Per Orientation:\n")
            f.write("-" * 80 + "\n")
            
            for name, _ in self.orientations:
                if name in self.calibration_data:
                    data = self.calibration_data[name]
                    euler = data['euler']
                    quat = data['quaternion']
                    
                    f.write(f"\n{name.upper().replace('_', ' ')}:\n")
                    f.write(f"  Euler (degrees): R={euler[0]:7.2f}, P={euler[1]:7.2f}, Y={euler[2]:7.2f}\n")
                    f.write(f"  Quaternion:      x={quat[0]:7.4f}, y={quat[1]:7.4f}, z={quat[2]:7.4f}, w={quat[3]:7.4f}\n")
                    
                    if name != 'neutral' and 'neutral' in self.calibration_data:
                        neutral_euler = self.calibration_data['neutral']['euler']
                        euler_diff = np.array(euler) - np.array(neutral_euler)
                        f.write(f"  Change from neutral: ΔR={euler_diff[0]:7.2f}, ΔP={euler_diff[1]:7.2f}, ΔY={euler_diff[2]:7.2f}\n")
        
        print(f"💾 Human-readable data saved to {filename}")
    
    def run(self):
        """Run the calibration process"""
        print("\n🎮 VR Controller Calibration Tool")
        print(f"   Using {'RIGHT' if self.right_controller else 'LEFT'} controller")
        print("\n📋 Instructions:")
        print("   - Follow the prompts to orient your controller")
        print("   - Hold each position steady while capturing")
        print("   - Press ENTER to capture each orientation")
        print("   - Type 'q' and ENTER to quit at any time")
        print("\n💡 Tips:")
        print("   - Start with a comfortable neutral position")
        print("   - For 90° rotations, aim for perpendicular orientations")
        print("   - Keep the controller as steady as possible during capture")
        
        # Capture all orientations
        for name, instruction in self.orientations:
            if not self.capture_state(name, instruction):
                print("\n👋 Calibration cancelled by user")
                break
        
        # Analyze results
        if self.calibration_data:
            self.analyze_results()
            self.save_results()
        
        print("\n✅ Calibration complete!")
    
    def cleanup(self):
        """Clean up resources"""
        if hasattr(self, 'oculus_reader'):
            try:
                self.oculus_reader.stop()
            except:
                pass


def main():
    parser = argparse.ArgumentParser(
        description='Interactive VR Controller Calibration Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('--left-controller', action='store_true',
                        help='Use left controller instead of right (default: right)')
    parser.add_argument('--ip', type=str, default=None,
                        help='IP address of Quest device (default: USB connection)')
    
    args = parser.parse_args()
    
    # Create and run calibration tool
    tool = VRCalibrationTool(
        right_controller=not args.left_controller,
        ip_address=args.ip
    )
    
    try:
        tool.run()
    except KeyboardInterrupt:
        print("\n\n🛑 Calibration interrupted")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        tool.cleanup()


if __name__ == "__main__":
    main() 