#!/usr/bin/env python3
"""
Mouse VR Server - Simulates VR hand movement for human teleoperation mode
"""

import zmq
import time
import threading
import tkinter as tk
from tkinter import ttk
import math
import numpy as np
import signal
import sys

class MouseVRServer:
    def __init__(self):
        # ZMQ publisher for teleop system
        self.context = zmq.Context()
        self.zmq_publisher = self.context.socket(zmq.PUB)
        # Bind to the specific IP that oculus_stick.py expects to connect to
        self.zmq_publisher.bind("tcp://192.168.1.54:5555")
        
        # Mouse state
        self.mouse_x = 0.0
        self.mouse_y = 0.0
        self.left_click_held = False
        self.right_click = False
        self.middle_click = False
        
        # Hand simulation state
        self.base_hand_pos = [0.0, 0.0, 0.3]  # Base hand position in 3D space
        self.movement_scale = 0.1  # Reduced scale for more precise robot control
        self.running = True
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        print("Mouse VR Server for Robot Control starting...")
        print("Publishing hand tracking data on tcp://192.168.1.54:5555")
        print("Simulating VR hand movement for robot control:")
        print("  - Hold LEFT CLICK + move mouse: Control robot hand in X/Y plane")
        print("  - Mouse X movement: Robot left/right (Y axis)")
        print("  - Mouse Y movement: Robot forward/backward (X axis)")
        print("  - Right click: A button (start/stop robot following)")
        print("  - Middle click: B button")
        print("\nUse with: python3 teleop.py teleop_mode=robot")
        print("Press Ctrl+C to exit gracefully")
        
    def signal_handler(self, signum, frame):
        """Handle Ctrl+C and other termination signals"""
        print(f"\n🛑 Received signal {signum}, shutting down gracefully...")
        self.stop_server()
        
    def create_controller_message(self):
        """Create controller message simulating VR hand tracking"""
        
        # Only update hand position when left-click is held (active tracking)
        if self.left_click_held:
            # Map mouse movement to robot coordinate system
            # Mouse X -> Robot Y (left/right movement)
            # Mouse Y -> Robot X (forward/backward movement) 
            # Keep Z constant for 2D movement
            hand_x = self.base_hand_pos[0] + (self.mouse_y * self.movement_scale)  # Forward/back
            hand_y = self.base_hand_pos[1] + (self.mouse_x * self.movement_scale)  # Left/right  
            hand_z = self.base_hand_pos[2]  # Keep Z constant
        else:
            # When not actively tracking, return to base position
            hand_x = self.base_hand_pos[0]
            hand_y = self.base_hand_pos[1] 
            hand_z = self.base_hand_pos[2]
        
        # Create identity rotation (no rotation for simplicity)
        quat_w = 1.0
        quat_x = 0.0
        quat_y = 0.0
        quat_z = 0.0
        
        # Create controller format string
        # For robot mode, the right hand position controls robot movement
        controller_text = (
            f"left;"
            f"x:false;"
            f"y:false;"
            f"menu:false;"
            f"thumbstick:false;"
            f"index_trigger:0.0;"
            f"hand_trigger:0.0;"
            f"thumbstick_axes:0.0,0.0;"
            f"position:0.0,0.0,0.0;"
            f"rotation:1.0,0.0,0.0,0.0;"
            f"|"
            f"right;"
            f"a:{str(self.right_click).lower()};"
            f"b:{str(self.middle_click).lower()};"
            f"menu:false;"
            f"thumbstick:false;"
            f"index_trigger:0.0;"
            f"hand_trigger:0.0;"
            f"thumbstick_axes:0.0,0.0;"
            f"position:{hand_x:.6f},{hand_y:.6f},{hand_z:.6f};"
            f"rotation:{quat_w:.6f},{quat_x:.6f},{quat_y:.6f},{quat_z:.6f};"
        )
        
        return controller_text
    
    def start_gui(self):
        """Start the GUI for mouse control"""
        try:
            self.root = tk.Tk()
            self.root.title("Mouse VR Hand Simulator")
            self.root.geometry("600x550")
            
            # Instructions
            instructions = tk.Label(self.root, text="""
Mouse VR Server for Robot Control

IMPORTANT: Run teleop with: python3 teleop.py teleop_mode=robot

Instructions:
• HOLD LEFT CLICK + move mouse: Control robot hand in X/Y plane
• Mouse X movement: Robot left/right (Y axis)  
• Mouse Y movement: Robot forward/backward (X axis)
• Right Click: A button (start/stop robot following)
• Middle Click: B button

Workflow:
1. Start this mouse server
2. Start teleop with teleop_mode=robot
3. Right-click to start robot following mode
4. Hold left-click and move mouse to control robot hand position
5. Right-click again to stop robot following
            """, justify=tk.LEFT, font=("Arial", 9))
            instructions.pack(pady=10)
            
            # Canvas for mouse tracking
            self.canvas = tk.Canvas(self.root, width=400, height=200, bg="lightblue", relief=tk.SUNKEN, bd=2)
            self.canvas.pack(pady=10)
            
            # Add center crosshair and grid
            self.canvas.create_line(200, 0, 200, 200, fill="blue", width=1)
            self.canvas.create_line(0, 100, 400, 100, fill="blue", width=1)
            
            # Add grid lines
            for i in range(0, 400, 50):
                self.canvas.create_line(i, 0, i, 200, fill="lightgray", width=1)
            for i in range(0, 200, 50):
                self.canvas.create_line(0, i, 400, i, fill="lightgray", width=1)
            
            # Bind mouse events
            self.canvas.bind("<Motion>", self.on_mouse_move)
            self.canvas.bind("<Button-1>", self.on_left_press)
            self.canvas.bind("<ButtonRelease-1>", self.on_left_release)
            self.canvas.bind("<Button-3>", self.on_right_click)
            self.canvas.bind("<ButtonRelease-3>", self.on_right_release)
            self.canvas.bind("<Button-2>", self.on_middle_click)
            self.canvas.bind("<ButtonRelease-2>", self.on_middle_release)
            
            # Status display
            self.status_var = tk.StringVar()
            self.status_label = tk.Label(self.root, textvariable=self.status_var, font=("Courier", 9), justify=tk.LEFT)
            self.status_label.pack(pady=5)
            
            # Control buttons
            button_frame = tk.Frame(self.root)
            button_frame.pack(pady=10)
            
            tk.Button(button_frame, text="Reset Position", command=self.reset_position).pack(side=tk.LEFT, padx=5)
            tk.Button(button_frame, text="Stop Server", command=self.stop_server).pack(side=tk.LEFT, padx=5)
            
            # Scale adjustment
            scale_frame = tk.Frame(self.root)
            scale_frame.pack(pady=5)
            tk.Label(scale_frame, text="Hand Movement Scale:").pack(side=tk.LEFT)
            self.scale_var = tk.DoubleVar(value=self.movement_scale)
            scale_slider = tk.Scale(scale_frame, from_=0.05, to=0.5, resolution=0.01, 
                                   orient=tk.HORIZONTAL, variable=self.scale_var,
                                   command=self.update_scale)
            scale_slider.pack(side=tk.LEFT, padx=5)
            
            # Start publishing thread
            self.publish_thread = threading.Thread(target=self.publish_loop, daemon=True)
            self.publish_thread.start()
            
            # Update status and canvas
            self.update_display()
            
            # Start GUI
            self.root.protocol("WM_DELETE_WINDOW", self.stop_server)
            self.root.mainloop()
            
        except Exception as e:
            print(f"❌ Error starting GUI: {e}")
            self.stop_server()
    
    def update_scale(self, value):
        self.movement_scale = float(value)
    
    def on_mouse_move(self, event):
        # Convert canvas coordinates to relative position (-1 to 1)
        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()
        
        self.mouse_x = (event.x - canvas_width/2) / (canvas_width/2)
        self.mouse_y = -(event.y - canvas_height/2) / (canvas_height/2)  # Invert Y
        
        # Clamp to [-1, 1]
        self.mouse_x = max(-1, min(1, self.mouse_x))
        self.mouse_y = max(-1, min(1, self.mouse_y))
        
        # Update canvas color based on hand tracking state
        if self.left_click_held:
            self.canvas.configure(bg="lightgreen")  # Green when tracking hand
        else:
            self.canvas.configure(bg="lightblue")   # Blue when not tracking
    
    def on_left_press(self, event):
        self.left_click_held = True
        print("🟢 Hand tracking STARTED - Move mouse to simulate hand movement")
        
    def on_left_release(self, event):
        self.left_click_held = False
        print("⚪ Hand tracking STOPPED")
        
    def on_right_click(self, event):
        self.right_click = True
        print("🔴 A button pressed - Start/Stop recording")
        
    def on_right_release(self, event):
        self.right_click = False
        print("⚪ A button released")
        
    def on_middle_click(self, event):
        self.middle_click = True
        print("🟡 B button pressed")
        
    def on_middle_release(self, event):
        self.middle_click = False
        print("⚪ B button released")
    
    def reset_position(self):
        self.mouse_x = 0.0
        self.mouse_y = 0.0
        self.left_click_held = False
        self.right_click = False
        self.middle_click = False
        print("🔄 Hand position reset")
    
    def update_display(self):
        if not self.running:
            return
        
        # Calculate current hand position
        if self.left_click_held:
            # Map mouse to robot coordinates
            hand_x = self.base_hand_pos[0] + (self.mouse_y * self.movement_scale)  # Forward/back
            hand_y = self.base_hand_pos[1] + (self.mouse_x * self.movement_scale)  # Left/right
            hand_z = self.base_hand_pos[2]
        else:
            hand_x = self.base_hand_pos[0]
            hand_y = self.base_hand_pos[1] 
            hand_z = self.base_hand_pos[2]
            
        status_text = f"""Mouse Position: X={self.mouse_x:+.3f}, Y={self.mouse_y:+.3f}
Robot Control: {'ACTIVE' if self.left_click_held else 'INACTIVE'}
Robot Following: {'ON' if self.right_click else 'OFF'}
Buttons: A={self.right_click}, B={self.middle_click}

Robot Hand Position: 
  X={hand_x:+.6f} (forward/back, mouse Y)
  Y={hand_y:+.6f} (left/right, mouse X)  
  Z={hand_z:+.6f} (height, fixed)
  
Movement Scale: {self.movement_scale:.3f} (adjust with slider)"""
        
        self.status_var.set(status_text)
        
        if self.running:
            self.root.after(100, self.update_display)  # Update every 100ms
    
    def publish_loop(self):
        """Continuously publish hand tracking data"""
        message_count = 0
        last_debug_time = time.time()
        
        while self.running:
            try:
                # Create and publish controller message
                controller_text = self.create_controller_message()
                
                # Publish topic message first
                self.zmq_publisher.send_string("oculus_controller")
                
                # Then publish controller data
                self.zmq_publisher.send_string(controller_text)
                
                message_count += 1
                
                # Debug logging every 2 seconds or when buttons change
                current_time = time.time()
                if (current_time - last_debug_time > 2.0 or 
                    self.right_click or self.middle_click or 
                    (message_count % 40 == 0)):  # Every 2 seconds at 20Hz
                    
                    print(f"📡 [{message_count:04d}] Publishing VR data:")
                    print(f"   Right A (recording): {self.right_click}")
                    print(f"   Right B: {self.middle_click}")
                    print(f"   Hand tracking active: {self.left_click_held}")
                    if self.left_click_held:
                        hand_x = self.base_hand_pos[0] + (self.mouse_x * self.movement_scale)
                        hand_y = self.base_hand_pos[1] + (self.mouse_y * self.movement_scale)
                        hand_z = self.base_hand_pos[2]
                        print(f"   Hand position: [{hand_x:.3f}, {hand_y:.3f}, {hand_z:.3f}]")
                        print(f"   Mouse offset: [{self.mouse_x:.3f}, {self.mouse_y:.3f}]")
                    
                    # Show a snippet of the controller text
                    if len(controller_text) > 100:
                        snippet = controller_text[:100] + "..."
                    else:
                        snippet = controller_text
                    print(f"   Data: {snippet}")
                    print()
                    
                    last_debug_time = current_time
                
                time.sleep(0.05)  # 20 Hz
                
            except Exception as e:
                if self.running:  # Only print error if we're still supposed to be running
                    print(f"❌ Error in publish loop: {e}")
                break
    
    def stop_server(self):
        """Gracefully stop the server"""
        if not self.running:
            return  # Already stopping
            
        print("🛑 Stopping Mouse VR Hand Simulator...")
        self.running = False
        
        # Close ZMQ resources
        try:
            self.zmq_publisher.close()
            self.context.term()
            print("✅ ZMQ resources closed")
        except Exception as e:
            print(f"⚠️  Error closing ZMQ: {e}")
        
        # Close GUI if it exists
        if hasattr(self, 'root'):
            try:
                self.root.quit()
                self.root.destroy()
                print("✅ GUI closed")
            except Exception as e:
                print(f"⚠️  Error closing GUI: {e}")
        
        print("✅ Server stopped gracefully")
        
        # Exit the program
        sys.exit(0)

if __name__ == "__main__":
    server = MouseVRServer()
    try:
        server.start_gui()
    except KeyboardInterrupt:
        print("\n🛑 Keyboard interrupt received")
        server.stop_server()
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        server.stop_server() 