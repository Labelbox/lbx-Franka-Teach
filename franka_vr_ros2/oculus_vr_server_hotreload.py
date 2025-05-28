#!/usr/bin/env python3
"""
Hot Reload Wrapper for Oculus VR Server ROS2
Automatically restarts the server when code changes are detected
"""

import os
import sys
import time
import subprocess
import signal
from pathlib import Path
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler


class CodeChangeHandler(FileSystemEventHandler):
    """Handler for file system events"""
    
    def __init__(self, restart_callback):
        self.restart_callback = restart_callback
        self.last_restart = 0
        self.restart_cooldown = 1.0  # Minimum seconds between restarts
        
    def on_modified(self, event):
        if event.is_directory:
            return
            
        # Check if it's a Python file
        if event.src_path.endswith('.py'):
            current_time = time.time()
            if current_time - self.last_restart > self.restart_cooldown:
                print(f"\n🔄 Detected change in {event.src_path}")
                self.last_restart = current_time
                self.restart_callback()


class HotReloadServer:
    """Hot reload wrapper for the VR server"""
    
    def __init__(self, args):
        self.args = args
        self.process = None
        self.running = True
        
        # Paths to watch
        self.watch_paths = [
            Path(__file__).parent / "franka_vr_ros2",
            Path(__file__).parent / "oculus_vr_server.py",
        ]
        
        # Setup file watcher
        self.observer = Observer()
        self.handler = CodeChangeHandler(self.restart_server)
        
        for path in self.watch_paths:
            if path.exists():
                if path.is_file():
                    self.observer.schedule(self.handler, str(path.parent), recursive=False)
                else:
                    self.observer.schedule(self.handler, str(path), recursive=True)
                print(f"👁️  Watching: {path}")
                
    def start(self):
        """Start the hot reload server"""
        print("🔥 Hot Reload Mode Active")
        print("   The server will restart automatically when you save changes")
        print("   Press Ctrl+C to stop\n")
        
        # Start file watcher
        self.observer.start()
        
        # Start initial server
        self.start_server()
        
        try:
            while self.running:
                time.sleep(0.1)
                
                # Check if process died unexpectedly
                if self.process and self.process.poll() is not None:
                    print("\n⚠️  Server process died unexpectedly, restarting...")
                    time.sleep(1)
                    self.start_server()
                    
        except KeyboardInterrupt:
            print("\n🛑 Stopping hot reload...")
            self.stop()
            
    def start_server(self):
        """Start the VR server process"""
        if self.process:
            self.stop_server()
            
        print("🚀 Starting VR server...")
        
        # Build command
        cmd = [sys.executable, "oculus_vr_server.py"] + self.args
        
        # Start process
        self.process = subprocess.Popen(
            cmd,
            cwd=Path(__file__).parent
        )
        
        print("✅ Server started (PID: {})".format(self.process.pid))
        
    def stop_server(self):
        """Stop the current server process"""
        if self.process:
            print("🛑 Stopping current server...")
            
            # Try graceful shutdown first
            self.process.terminate()
            
            # Wait up to 5 seconds for graceful shutdown
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                print("⚠️  Force killing server...")
                self.process.kill()
                self.process.wait()
                
            self.process = None
            print("✅ Server stopped")
            
    def restart_server(self):
        """Restart the server"""
        print("\n🔄 Restarting server...")
        self.stop_server()
        time.sleep(0.5)  # Brief pause
        self.start_server()
        print("✅ Restart complete\n")
        
    def stop(self):
        """Stop everything"""
        self.running = False
        self.observer.stop()
        self.observer.join()
        self.stop_server()
        print("✅ Hot reload stopped")


def main():
    """Main entry point"""
    # Get arguments (everything except the script name)
    args = sys.argv[1:]
    
    # Check if watchdog is installed
    try:
        import watchdog
    except ImportError:
        print("❌ Hot reload requires 'watchdog' package")
        print("   Install with: pip install watchdog")
        sys.exit(1)
    
    # Create and run hot reload server
    server = HotReloadServer(args)
    server.start()


if __name__ == "__main__":
    main() 