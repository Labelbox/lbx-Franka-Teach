#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray
import sys
import time

class DiagnosticListener(Node):
    def __init__(self):
        super().__init__('diagnostic_listener')
        
        self.subscription = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            10)
        
        self.camera_diagnostics_received = False
        self.diagnostics_count = 0
        
        print("🔍 Listening for camera diagnostics...")
        print("Will show camera-related diagnostics when received...")
    
    def diagnostics_callback(self, msg):
        self.diagnostics_count += 1
        
        for status in msg.status:
            if 'vision_camera_node' in status.name or 'camera' in status.name.lower() or 'realsense' in status.name.lower():
                if not self.camera_diagnostics_received:
                    print(f"\n📊 CAMERA DIAGNOSTICS FOUND (Message #{self.diagnostics_count}):")
                    print("=" * 60)
                    self.camera_diagnostics_received = True
                
                print(f"\n🏷️  Diagnostic: {status.name}")
                print(f"   Level: {status.level} ({'OK' if status.level == 0 else 'WARN' if status.level == 1 else 'ERROR' if status.level == 2 else 'STALE'})")
                print(f"   Message: {status.message}")
                print(f"   Hardware ID: {status.hardware_id}")
                
                if status.values:
                    print("   Values:")
                    for value in status.values:
                        print(f"     {value.key}: {value.value}")
                
                print("-" * 40)

def main():
    rclpy.init()
    node = DiagnosticListener()
    
    print("Starting diagnostic listener...")
    print("Press Ctrl+C to stop")
    
    try:
        # Run for 30 seconds to capture diagnostics
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time < 30):
            rclpy.spin_once(node, timeout_sec=1.0)
            
            if node.camera_diagnostics_received and node.diagnostics_count > 5:
                print(f"\n✅ Captured camera diagnostics after {node.diagnostics_count} messages")
                break
        
        if not node.camera_diagnostics_received:
            print("\n❌ No camera diagnostics received")
            print("Make sure the camera node is running: ros2 launch lbx_vision_camera camera.launch.py")
            
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 