"""FPS counter utility for performance monitoring"""

import time
import numpy as np


class FPSCounter:
    """Simple FPS counter for monitoring input device performance"""
    
    def __init__(self, display_interval: float = 5.0, window_size: int = 50):
        """
        Initialize FPS counter
        
        Args:
            display_interval: How often to display FPS (seconds)
            window_size: Number of samples for moving average
        """
        current_time = time.time()
        self.start_time_for_display = current_time
        self.last_time = current_time
        self.display_interval = display_interval
        self.time_between_calls = []
        self.window_size = window_size
        
    def get_and_print_fps(self, print_fps: bool = True) -> float:
        """
        Calculate and optionally print current FPS
        
        Args:
            print_fps: Whether to print the FPS
            
        Returns:
            Current FPS estimate
        """
        current_time = time.time()
        
        # Calculate instantaneous FPS
        time_delta = current_time - self.last_time
        if time_delta > 0:
            instant_fps = 1.0 / time_delta
        else:
            instant_fps = 0.0
            
        # Update moving average
        self.time_between_calls.append(instant_fps)
        if len(self.time_between_calls) > self.window_size:
            self.time_between_calls.pop(0)
            
        self.last_time = current_time
        
        # Calculate average FPS
        if self.time_between_calls:
            avg_fps = np.mean(self.time_between_calls)
        else:
            avg_fps = 0.0
            
        # Print if requested and enough time has passed
        if print_fps and (current_time - self.start_time_for_display) > self.display_interval:
            print(f"Input Device FPS: {int(avg_fps)}Hz")
            self.start_time_for_display = current_time
            
        return avg_fps 