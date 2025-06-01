#!/usr/bin/env python3
"""
Utility functions for the lbx_data_recorder package.
"""

import time

class FrequencyTimer:
    """Helper class to maintain a certain frequency for a loop."""
    def __init__(self, frequency_hz: float):
        if frequency_hz <= 0:
            raise ValueError("Frequency must be positive.")
        self.period_ns = 1e9 / frequency_hz
        self.start_time_ns = 0

    def start_loop(self):
        self.start_time_ns = time.time_ns()

    def sleep_for_remainder(self):
        """Sleeps for the remaining time in the loop to maintain frequency."""
        loop_duration_ns = time.time_ns() - self.start_time_ns
        wait_time_ns = self.period_ns - loop_duration_ns
        if wait_time_ns > 0:
            time.sleep(wait_time_ns / 1e9)

def notify_component_start(logger, component_name: str):
    """Logs a standardized component start message."""
    logger.info(f"\n***************************************************************\n"
                f"     Starting {component_name} component\n"
                f"***************************************************************")

# Add other general utilities here as needed. 