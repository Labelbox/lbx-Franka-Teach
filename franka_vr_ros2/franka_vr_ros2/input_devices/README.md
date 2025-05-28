# Input Devices Module

This module provides a modular framework for different input devices used in VR teleoperation.

## Structure

```
input_devices/
├── base/                    # Base classes and interfaces
│   └── input_device_base.py # Abstract base class for all input devices
├── meta_quest/              # Meta Quest (Oculus) implementation
│   ├── quest_reader.py      # Main reader implementation
│   ├── buttons_parser.py    # Button state parsing
│   ├── fps_counter.py       # Performance monitoring
│   └── apk/                 # Companion app for Quest
└── apple_vision/            # Apple Vision Pro (future)
```

## Supported Devices

### Meta Quest (Oculus Quest 2/3/Pro)

- Full 6DOF tracking for both controllers
- All buttons and analog inputs
- USB and wireless connectivity
- Requires companion app installation

### Future Support

- **Apple Vision Pro**: Hand tracking and spatial computing
- **HTC Vive**: SteamVR integration
- **Windows Mixed Reality**: OpenXR support

## Adding a New Input Device

1. Create a new directory under `input_devices/`
2. Implement the `InputDeviceBase` interface:

```python
from franka_vr_ros2.input_devices.base import InputDeviceBase

class MyDeviceReader(InputDeviceBase):
    def start(self):
        """Start reading from device"""
        pass

    def stop(self):
        """Stop reading from device"""
        pass

    def get_transformations_and_buttons(self):
        """Return current state"""
        return transforms_dict, buttons_dict

    def is_connected(self):
        """Check connection status"""
        return True
```

3. Update `__init__.py` to export your device
4. Update the main VR input handler to support device selection

## Meta Quest Setup

### USB Connection

1. Enable developer mode on Quest
2. Connect Quest to computer via USB
3. Allow USB debugging when prompted
4. Run: `adb devices` to verify connection

### Wireless Connection

1. Connect Quest to same network as computer
2. Get Quest IP: Settings → Network → Advanced
3. Run: `adb tcpip 5555`
4. Run: `adb connect <quest-ip>:5555`
5. Pass IP to reader: `MetaQuestReader(ip_address="<quest-ip>")`

### Troubleshooting

- "Device not found": Check USB connection and developer mode
- "No permissions": Accept USB debugging prompt on Quest
- "APK install failed": Manually uninstall old version first
- Low FPS: Use USB connection instead of wireless

## Button Mapping

### Meta Quest Controllers

**Right Controller:**

- A, B: Action buttons
- RG: Grip button (boolean)
- RTr: Trigger button (boolean)
- RJ: Joystick press
- rightGrip: Grip analog (0.0-1.0)
- rightTrig: Trigger analog (0.0-1.0)
- rightJS: Joystick position (x,y)

**Left Controller:**

- X, Y: Action buttons
- LG: Grip button (boolean)
- LTr: Trigger button (boolean)
- LJ: Joystick press
- leftGrip: Grip analog (0.0-1.0)
- leftTrig: Trigger analog (0.0-1.0)
- leftJS: Joystick position (x,y)
