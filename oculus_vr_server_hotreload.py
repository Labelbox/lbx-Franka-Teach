#!/usr/bin/env python3
"""
Hot-reloading wrapper for Oculus VR Server
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


class ServerReloader(FileSystemEventHandler):
    """Handles file system events and manages server restart"""
    
    def __init__(self, script_args):
        self.script_args = script_args
        self.process = None
        self.restarting = False
        self.last_restart_time = 0
        self.restart_cooldown = 1.0  # Minimum seconds between restarts
        
        # Files to watch
        self.watch_patterns = {
            '*.py',  # Python files
            'configs/*.yaml',  # Config files
            'frankateach/*.py',  # Package files
            'simulation/*.py',  # Simulation files
        }
        
        # Files to ignore
        self.ignore_patterns = {
            '*__pycache__*',
            '*.pyc',
            '*.pyo',
            '*.swp',
            '*.swo',
            '*~',
            '.git/*',
            'logs/*',
            'outputs/*',
        }
        
    def should_reload(self, path):
        """Check if a file change should trigger reload"""
        path_str = str(path)
        
        # Check ignore patterns
        for pattern in self.ignore_patterns:
            if pattern.replace('*', '') in path_str:
                return False
        
        # Check if it's a Python file or config
        if path_str.endswith('.py') or path_str.endswith('.yaml'):
            return True
            
        return False
    
    def on_modified(self, event):
        """Handle file modification events"""
        if event.is_directory:
            return
            
        if self.should_reload(event.src_path):
            current_time = time.time()
            if current_time - self.last_restart_time > self.restart_cooldown:
                print(f"\n🔄 Detected change in {event.src_path}")
                self.restart_server()
                self.last_restart_time = current_time
    
    def start_server(self):
        """Start the VR server process"""
        if self.process and self.process.poll() is None:
            self.stop_server()
        
        print("\n🚀 Starting Oculus VR Server...")
        print(f"   Command: python3 oculus_vr_server.py {' '.join(self.script_args)}")
        
        # Start the server as a subprocess
        cmd = [sys.executable, 'oculus_vr_server.py'] + self.script_args
        self.process = subprocess.Popen(
            cmd,
            stdout=sys.stdout,
            stderr=sys.stderr,
            stdin=sys.stdin,
            preexec_fn=os.setsid  # Create new process group
        )
        
        print("✅ Server started (PID: {})".format(self.process.pid))
        print("\n👀 Watching for file changes...")
        print("   Press Ctrl+C to stop\n")
    
    def stop_server(self):
        """Stop the VR server process"""
        if self.process and self.process.poll() is None:
            print("\n🛑 Stopping server...")
            
            # Send SIGTERM to the process group
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                # Force kill if it doesn't stop gracefully
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                self.process.wait()
            except ProcessLookupError:
                # Process already dead
                pass
            
            print("✅ Server stopped")
    
    def restart_server(self):
        """Restart the VR server"""
        if self.restarting:
            return
            
        self.restarting = True
        print("\n♻️  Restarting server...")
        
        self.stop_server()
        time.sleep(0.5)  # Brief pause before restart
        self.start_server()
        
        self.restarting = False


def main():
    """Main hot reload runner"""
    # Check if watchdog is installed
    try:
        import watchdog
    except ImportError:
        print("❌ watchdog library not installed!")
        print("   Install it with: pip install watchdog")
        sys.exit(1)
    
    # Pass through command line arguments to the actual server
    script_args = sys.argv[1:]
    
    print("🔥 Oculus VR Server - Hot Reload Mode")
    print("=" * 50)
    
    # Create event handler and observer
    event_handler = ServerReloader(script_args)
    observer = Observer()
    
    # Watch current directory and subdirectories
    watch_path = Path.cwd()
    observer.schedule(event_handler, str(watch_path), recursive=True)
    
    # Start the server initially
    event_handler.start_server()
    
    # Start watching for changes
    observer.start()
    
    try:
        # Keep running until interrupted
        while True:
            time.sleep(1)
            
            # Check if server process died unexpectedly
            if event_handler.process and event_handler.process.poll() is not None:
                print("\n⚠️  Server process died unexpectedly!")
                print("   Exit code:", event_handler.process.returncode)
                print("   Restarting in 2 seconds...")
                time.sleep(2)
                event_handler.start_server()
                
    except KeyboardInterrupt:
        print("\n\n🛑 Shutting down hot reload...")
        event_handler.stop_server()
        observer.stop()
        observer.join()
        print("✅ Hot reload stopped")
        sys.exit(0)


if __name__ == "__main__":
    main() 