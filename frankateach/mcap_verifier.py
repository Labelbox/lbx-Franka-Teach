#!/usr/bin/env python3
"""
MCAP Data Verifier for Franka Teach System
Verifies the integrity and completeness of recorded MCAP files
"""

import json
import time
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import numpy as np

import mcap
from mcap.reader import make_reader


class MCAPVerifier:
    """Verifies MCAP recordings for data integrity and completeness"""
    
    def __init__(self, filepath: str):
        """
        Initialize MCAP verifier
        
        Args:
            filepath: Path to MCAP file to verify
        """
        self.filepath = Path(filepath)
        if not self.filepath.exists():
            raise FileNotFoundError(f"MCAP file not found: {filepath}")
            
        self.verification_results = {
            "file_info": {},
            "data_streams": {},
            "errors": [],
            "warnings": [],
            "summary": {}
        }
        
    def verify(self, verbose: bool = True) -> Dict:
        """
        Perform comprehensive verification of MCAP file
        
        Args:
            verbose: Print detailed results during verification
            
        Returns:
            Dictionary with verification results
        """
        if verbose:
            print(f"\n🔍 Verifying MCAP file: {self.filepath.name}")
            print("=" * 60)
        
        # Basic file info
        self._verify_file_info()
        
        # Read and analyze data
        self._verify_data_streams(verbose)
        
        # Generate summary
        self._generate_summary()
        
        # Print results if verbose
        if verbose:
            self._print_results()
            
        return self.verification_results
    
    def _verify_file_info(self):
        """Verify basic file information"""
        stat = self.filepath.stat()
        self.verification_results["file_info"] = {
            "filename": self.filepath.name,
            "size_mb": stat.st_size / (1024 * 1024),
            "created": time.ctime(stat.st_ctime),
            "modified": time.ctime(stat.st_mtime)
        }
        
    def _verify_data_streams(self, verbose: bool):
        """Verify all data streams in the MCAP file"""
        with open(self.filepath, "rb") as f:
            reader = make_reader(f)
            
            # Get summary
            summary = reader.get_summary()
            if summary is None:
                self.verification_results["errors"].append("Failed to read MCAP summary")
                return
                
            # Calculate duration
            duration_ns = summary.statistics.message_end_time - summary.statistics.message_start_time
            duration_sec = duration_ns / 1e9
            self.verification_results["file_info"]["duration_sec"] = duration_sec
            self.verification_results["file_info"]["duration_str"] = f"{int(duration_sec//60):02d}m{int(duration_sec%60):02d}s"
            
            # Track data by topic
            topic_data = {}
            message_counts = {}
            
            # Read all messages
            for schema, channel, message in reader.iter_messages():
                topic = channel.topic
                
                if topic not in topic_data:
                    topic_data[topic] = {
                        "schema": schema.name if schema else "unknown",
                        "encoding": schema.encoding if schema else "unknown",
                        "messages": [],
                        "timestamps": [],
                        "first_timestamp": None,
                        "last_timestamp": None
                    }
                    message_counts[topic] = 0
                
                # Track message count and timestamps
                message_counts[topic] += 1
                timestamp_sec = message.log_time / 1e9
                topic_data[topic]["timestamps"].append(timestamp_sec)
                
                if topic_data[topic]["first_timestamp"] is None:
                    topic_data[topic]["first_timestamp"] = timestamp_sec
                topic_data[topic]["last_timestamp"] = timestamp_sec
                
                # Parse and verify message content
                try:
                    if channel.message_encoding == "json":
                        msg_data = json.loads(message.data)
                        
                        # Verify specific message types
                        if topic == "/robot_state":
                            self._verify_robot_state(msg_data, topic_data[topic])
                        elif topic == "/action":
                            self._verify_action(msg_data, topic_data[topic])
                        elif topic == "/vr_controller":
                            self._verify_vr_controller(msg_data, topic_data[topic])
                        elif topic == "/joint_states":
                            self._verify_joint_states(msg_data, topic_data[topic])
                            
                except Exception as e:
                    self.verification_results["errors"].append(f"Error parsing {topic}: {str(e)}")
            
            # Analyze each topic
            for topic, data in topic_data.items():
                count = message_counts[topic]
                
                # Calculate frequency
                if len(data["timestamps"]) > 1:
                    time_diffs = np.diff(data["timestamps"])
                    avg_freq = 1.0 / np.mean(time_diffs)
                    std_freq = np.std(1.0 / time_diffs) if len(time_diffs) > 1 else 0
                else:
                    avg_freq = 0
                    std_freq = 0
                
                self.verification_results["data_streams"][topic] = {
                    "schema": data["schema"],
                    "encoding": data["encoding"],
                    "message_count": count,
                    "average_frequency_hz": round(avg_freq, 2),
                    "frequency_std_hz": round(std_freq, 2),
                    "duration_sec": data["last_timestamp"] - data["first_timestamp"] if data["first_timestamp"] else 0,
                    "has_joint_data": data.get("has_joint_data", False),
                    "has_cartesian_data": data.get("has_cartesian_data", False),
                    "joint_data_valid": data.get("joint_data_valid", False),
                    "action_range": data.get("action_range", {}),
                    "gripper_range": data.get("gripper_range", {})
                }
                
    def _verify_robot_state(self, msg_data: Dict, topic_info: Dict):
        """Verify robot state message"""
        # Check for required fields
        required_fields = ["joint_positions", "cartesian_position", "gripper_position"]
        for field in required_fields:
            if field not in msg_data:
                self.verification_results["warnings"].append(f"Missing {field} in robot_state")
                
        # Check joint positions
        joint_positions = msg_data.get("joint_positions", [])
        if joint_positions:
            topic_info["has_joint_data"] = True
            if len(joint_positions) == 7:
                topic_info["joint_data_valid"] = True
                # Check if joints are within reasonable bounds
                for i, pos in enumerate(joint_positions):
                    if abs(pos) > 2 * np.pi:  # More than 360 degrees
                        self.verification_results["warnings"].append(
                            f"Joint {i+1} position out of range: {pos:.3f} rad"
                        )
            else:
                self.verification_results["warnings"].append(
                    f"Invalid joint count: {len(joint_positions)} (expected 7)"
                )
                
        # Check cartesian position
        cart_pos = msg_data.get("cartesian_position", [])
        if cart_pos:
            topic_info["has_cartesian_data"] = True
            if len(cart_pos) != 6:
                self.verification_results["warnings"].append(
                    f"Invalid cartesian position size: {len(cart_pos)} (expected 6)"
                )
                
        # Track gripper range
        gripper_pos = msg_data.get("gripper_position", 0)
        if "gripper_range" not in topic_info:
            topic_info["gripper_range"] = {"min": gripper_pos, "max": gripper_pos}
        else:
            topic_info["gripper_range"]["min"] = min(topic_info["gripper_range"]["min"], gripper_pos)
            topic_info["gripper_range"]["max"] = max(topic_info["gripper_range"]["max"], gripper_pos)
            
    def _verify_action(self, msg_data: Dict, topic_info: Dict):
        """Verify action message"""
        action_data = msg_data.get("data", [])
        if len(action_data) != 7:
            self.verification_results["warnings"].append(
                f"Invalid action size: {len(action_data)} (expected 7)"
            )
            
        # Track action ranges
        if action_data and "action_range" not in topic_info:
            topic_info["action_range"] = {
                "min": list(action_data),
                "max": list(action_data)
            }
        elif action_data:
            for i, val in enumerate(action_data):
                topic_info["action_range"]["min"][i] = min(topic_info["action_range"]["min"][i], val)
                topic_info["action_range"]["max"][i] = max(topic_info["action_range"]["max"][i], val)
                
    def _verify_vr_controller(self, msg_data: Dict, topic_info: Dict):
        """Verify VR controller message"""
        # Check for required fields
        required_fields = ["poses", "buttons", "movement_enabled"]
        for field in required_fields:
            if field not in msg_data:
                self.verification_results["warnings"].append(f"Missing {field} in vr_controller")
                
    def _verify_joint_states(self, msg_data: Dict, topic_info: Dict):
        """Verify joint states message for visualization"""
        # Check joint names
        joint_names = msg_data.get("name", [])
        expected_names = [
            "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
            "fr3_joint5", "fr3_joint6", "fr3_joint7",
            "fr3_finger_joint1", "fr3_finger_joint2"
        ]
        
        if joint_names != expected_names:
            self.verification_results["warnings"].append(
                f"Unexpected joint names in /joint_states"
            )
            
        # Check positions
        positions = msg_data.get("position", [])
        if len(positions) != 9:  # 7 arm + 2 finger
            self.verification_results["warnings"].append(
                f"Invalid joint state position count: {len(positions)} (expected 9)"
            )
            
    def _generate_summary(self):
        """Generate verification summary"""
        # Count totals
        total_messages = sum(
            stream["message_count"] 
            for stream in self.verification_results["data_streams"].values()
        )
        
        # Check data quality
        has_robot_state = "/robot_state" in self.verification_results["data_streams"]
        has_actions = "/action" in self.verification_results["data_streams"]
        has_vr_data = "/vr_controller" in self.verification_results["data_streams"]
        has_joint_viz = "/joint_states" in self.verification_results["data_streams"]
        
        # Check joint data
        has_joint_data = False
        joint_data_valid = False
        if has_robot_state:
            robot_state = self.verification_results["data_streams"]["/robot_state"]
            has_joint_data = robot_state.get("has_joint_data", False)
            joint_data_valid = robot_state.get("joint_data_valid", False)
            
        # Overall status
        is_valid = (
            has_robot_state and 
            has_actions and 
            has_vr_data and
            len(self.verification_results["errors"]) == 0
        )
        
        self.verification_results["summary"] = {
            "is_valid": is_valid,
            "total_messages": total_messages,
            "has_robot_state": has_robot_state,
            "has_actions": has_actions,
            "has_vr_controller": has_vr_data,
            "has_joint_visualization": has_joint_viz,
            "has_joint_data": has_joint_data,
            "joint_data_valid": joint_data_valid,
            "error_count": len(self.verification_results["errors"]),
            "warning_count": len(self.verification_results["warnings"])
        }
        
    def _print_results(self):
        """Print verification results"""
        results = self.verification_results
        
        # File info
        print(f"\n📁 File Information:")
        print(f"   Size: {results['file_info']['size_mb']:.2f} MB")
        print(f"   Duration: {results['file_info']['duration_str']}")
        
        # Data streams
        print(f"\n📊 Data Streams:")
        for topic, info in results["data_streams"].items():
            print(f"\n   {topic}:")
            print(f"      Schema: {info['schema']}")
            print(f"      Messages: {info['message_count']}")
            print(f"      Frequency: {info['average_frequency_hz']:.1f} Hz (±{info['frequency_std_hz']:.1f})")
            
            if topic == "/robot_state":
                print(f"      Has joint data: {'✅' if info['has_joint_data'] else '❌'}")
                if info['has_joint_data']:
                    print(f"      Joint data valid: {'✅' if info['joint_data_valid'] else '❌'}")
                print(f"      Gripper range: [{info['gripper_range'].get('min', 0):.3f}, {info['gripper_range'].get('max', 0):.3f}]")
                
            elif topic == "/action" and info.get('action_range'):
                action_range = info['action_range']
                if action_range:
                    print(f"      Linear velocity range: [{action_range['min'][0]:.3f}, {action_range['max'][0]:.3f}]")
                    print(f"      Angular velocity range: [{action_range['min'][3]:.3f}, {action_range['max'][3]:.3f}]")
        
        # Errors and warnings
        if results["errors"]:
            print(f"\n❌ Errors ({len(results['errors'])}):")
            for error in results["errors"]:
                print(f"   - {error}")
                
        if results["warnings"]:
            print(f"\n⚠️  Warnings ({len(results['warnings'])}):")
            for warning in results["warnings"][:5]:  # Show first 5
                print(f"   - {warning}")
            if len(results["warnings"]) > 5:
                print(f"   ... and {len(results['warnings']) - 5} more")
        
        # Summary
        summary = results["summary"]
        print(f"\n📋 Summary:")
        print(f"   Total messages: {summary['total_messages']}")
        print(f"   Data streams present: {sum([summary['has_robot_state'], summary['has_actions'], summary['has_vr_controller']])}/3")
        print(f"   Joint data: {'✅ Valid' if summary['joint_data_valid'] else '❌ Missing or Invalid'}")
        print(f"   Visualization ready: {'✅' if summary['has_joint_visualization'] else '❌'}")
        
        # Final verdict
        print(f"\n{'✅ VERIFICATION PASSED' if summary['is_valid'] else '❌ VERIFICATION FAILED'}")
        if not summary['is_valid']:
            if not summary['has_robot_state']:
                print("   - Missing robot state data")
            if not summary['has_actions']:
                print("   - Missing action data")
            if not summary['has_vr_controller']:
                print("   - Missing VR controller data")
            if summary['error_count'] > 0:
                print(f"   - {summary['error_count']} errors found") 