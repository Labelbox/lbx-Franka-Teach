"""Meta Quest VR controller reader implementation"""

import numpy as np
import threading
import time
import os
import sys
from typing import Dict, Tuple, Optional

from ..base import InputDeviceBase
from .buttons_parser import parse_buttons
from .fps_counter import FPSCounter

# Only import ppadb if we're actually using the device
try:
    from ppadb.client import Client as AdbClient
    PPADB_AVAILABLE = True
except ImportError:
    PPADB_AVAILABLE = False
    print("Warning: ppadb not available. Meta Quest reader will not work.")


def eprint(*args, **kwargs):
    """Print to stderr in red"""
    RED = "\033[1;31m"  
    sys.stderr.write(RED)
    print(*args, file=sys.stderr, **kwargs)
    RESET = "\033[0;0m"
    sys.stderr.write(RESET)


class MetaQuestReader(InputDeviceBase):
    """
    Reader for Meta Quest (Oculus) VR controllers
    
    This is a cleaned up version of the original OculusReader,
    structured to work with the new input device framework.
    """
    
    def __init__(self,
                 ip_address: Optional[str] = None,
                 port: int = 5555,
                 apk_name: str = 'com.rail.oculus.teleop',
                 print_fps: bool = False,
                 auto_start: bool = True):
        """
        Initialize Meta Quest reader
        
        Args:
            ip_address: IP address of Quest device (None for USB connection)
            port: ADB port (default 5555)
            apk_name: Package name of the companion app
            print_fps: Whether to print FPS statistics
            auto_start: Whether to automatically start reading
        """
        super().__init__()
        
        if not PPADB_AVAILABLE:
            raise ImportError("ppadb is required for Meta Quest reader. Install with: pip install pure-python-adb")
            
        self.ip_address = ip_address
        self.port = port
        self.apk_name = apk_name
        self.print_fps = print_fps
        self.tag = 'wE9ryARX'  # Log tag used by the companion app
        
        # Thread-safe storage for latest data
        self._lock = threading.Lock()
        self.last_transforms = {}
        self.last_buttons = {}
        
        # FPS counter
        if self.print_fps:
            self.fps_counter = FPSCounter()
            
        # Get ADB device connection
        self.device = self._get_device()
        
        # Install companion app if needed
        self._install_apk(verbose=False)
        
        # Auto start if requested
        if auto_start:
            self.start()
            
    def _get_device(self):
        """Get ADB device connection"""
        client = AdbClient(host="127.0.0.1", port=5037)
        
        if self.ip_address is not None:
            return self._get_network_device(client)
        else:
            return self._get_usb_device(client)
            
    def _get_network_device(self, client, retry=0):
        """Connect to device over network"""
        try:
            client.remote_connect(self.ip_address, self.port)
        except RuntimeError:
            os.system('adb devices')
            client.remote_connect(self.ip_address, self.port)
            
        device = client.device(f"{self.ip_address}:{self.port}")
        
        if device is None:
            if retry == 1:
                os.system(f'adb tcpip {self.port}')
            if retry == 2:
                eprint(f'Make sure device is available at IP: {self.ip_address}')
                eprint('Run `adb shell ip route` to verify the IP address.')
                raise ConnectionError(f"Cannot connect to device at {self.ip_address}:{self.port}")
            else:
                return self._get_network_device(client, retry=retry+1)
                
        return device
        
    def _get_usb_device(self, client):
        """Connect to device over USB"""
        try:
            devices = client.devices()
        except RuntimeError:
            os.system('adb devices')
            devices = client.devices()
            
        for device in devices:
            # USB devices don't have dots in their serial
            if device.serial.count('.') < 3:
                return device
                
        eprint('Device not found. Make sure device is connected over USB.')
        eprint('Run `adb devices` to verify that the device is visible.')
        raise ConnectionError("No USB device found")
        
    def _install_apk(self, apk_path=None, verbose=True, reinstall=False):
        """Install companion APK on device if needed"""
        try:
            installed = self.device.is_installed(self.apk_name)
            if not installed or reinstall:
                if apk_path is None:
                    # Look for APK in the package directory
                    apk_path = os.path.join(
                        os.path.dirname(os.path.realpath(__file__)),
                        'apk', 'teleop-debug.apk'
                    )
                    
                if not os.path.exists(apk_path):
                    eprint(f"APK not found at {apk_path}")
                    eprint("Please provide the companion APK or disable auto-install")
                    return
                    
                success = self.device.install(apk_path, test=True, reinstall=reinstall)
                if success:
                    print('APK installed successfully.')
                else:
                    eprint('APK install failed.')
            elif verbose:
                print('APK is already installed.')
        except RuntimeError as e:
            eprint(f'Device access error: {e}')
            eprint('If you see "no permissions", please allow access on the Quest.')
            
    def start(self):
        """Start reading from the device"""
        if self.running:
            return
            
        self.running = True
        
        # Start the companion app
        self.device.shell(
            f'am start -n "{self.apk_name}/{self.apk_name}.MainActivity" '
            '-a android.intent.action.MAIN -c android.intent.category.LAUNCHER'
        )
        
        # Start logcat reading thread
        self.thread = threading.Thread(
            target=self.device.shell,
            args=("logcat -T 0", self._read_logcat_by_line)
        )
        self.thread.daemon = True
        self.thread.start()
        
    def stop(self):
        """Stop reading from the device"""
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join(timeout=1.0)
            
    def _read_logcat_by_line(self, connection):
        """Read and process logcat output line by line"""
        file_obj = connection.socket.makefile()
        
        while self.running:
            try:
                line = file_obj.readline().strip()
                data = self._extract_data(line)
                
                if data:
                    transforms, buttons = self._process_data(data)
                    if transforms is not None and buttons is not None:
                        with self._lock:
                            self.last_transforms = transforms
                            self.last_buttons = buttons
                            
                        if self.print_fps:
                            self.fps_counter.get_and_print_fps()
                            
            except UnicodeDecodeError:
                pass
            except Exception as e:
                if self.running:
                    eprint(f"Error reading logcat: {e}")
                    
        file_obj.close()
        connection.close()
        
    def _extract_data(self, line: str) -> str:
        """Extract controller data from logcat line"""
        if self.tag in line:
            try:
                return line.split(f'{self.tag}: ')[1]
            except (ValueError, IndexError):
                pass
        return ''
        
    def _process_data(self, data_string: str) -> Tuple[Optional[Dict], Optional[Dict]]:
        """Process raw data string into transforms and buttons"""
        try:
            transforms_string, buttons_string = data_string.split('&')
        except ValueError:
            return None, None
            
        # Process transforms
        transforms = {}
        for pair_string in transforms_string.split('|'):
            parts = pair_string.split(':')
            if len(parts) != 2:
                continue
                
            controller_id = parts[0]  # 'r' or 'l'
            values = parts[1].split(' ')
            
            # Build 4x4 transformation matrix
            transform = np.zeros((4, 4))
            idx = 0
            for value in values:
                if value and idx < 16:
                    row = idx // 4
                    col = idx % 4
                    transform[row, col] = float(value)
                    idx += 1
                    
            if idx == 16:
                transforms[controller_id] = transform
                
        # Process buttons
        buttons = parse_buttons(buttons_string)
        
        return transforms, buttons
        
    def get_transformations_and_buttons(self) -> Tuple[Dict[str, np.ndarray], Dict[str, any]]:
        """Get current controller transformations and button states"""
        with self._lock:
            return self.last_transforms.copy(), self.last_buttons.copy()
            
    def is_connected(self) -> bool:
        """Check if the device is connected and working"""
        try:
            # Check if we have recent data
            if not self.last_transforms:
                return False
                
            # Try a simple shell command to verify connection
            result = self.device.shell("echo test")
            return result.strip() == "test"
        except:
            return False
            
    def __del__(self):
        """Cleanup on deletion"""
        self.stop() 