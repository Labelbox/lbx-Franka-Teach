"""Abstract base class for input devices"""

from abc import ABC, abstractmethod
from typing import Dict, Tuple, Optional
import numpy as np


class InputDeviceBase(ABC):
    """Abstract base class for all input devices (VR controllers, joysticks, etc.)"""
    
    def __init__(self):
        self.running = False
        
    @abstractmethod
    def start(self):
        """Start the input device reader"""
        pass
        
    @abstractmethod
    def stop(self):
        """Stop the input device reader"""
        pass
        
    @abstractmethod
    def get_transformations_and_buttons(self) -> Tuple[Dict[str, np.ndarray], Dict[str, any]]:
        """
        Get current controller transformations and button states
        
        Returns:
            transforms: Dict mapping controller IDs to 4x4 transformation matrices
            buttons: Dict mapping button names to their states (bool or float)
        """
        pass
        
    @abstractmethod
    def is_connected(self) -> bool:
        """Check if the device is connected and working"""
        pass 