# Mouse VR Control System

Quick setup commands to run the mouse-controlled VR teleoperation system.

## Prerequisites
- Robot connected and powered on
- Network configured (PC: 192.168.1.122, WiFi: 192.168.1.54)

## Startup Sequence

### 1. Start Deoxys (Robot Interface)
```bash
python3 run_deoxys_correct.py
```

### 2. Start Franka Server (Robot Control)
```bash
# In new terminal
python3 franka_server.py
```

### 3. Start Mouse VR Server (Hand Simulation)
```bash
# In new terminal
python3 mouse_vr_server.py
```

### 4. Start Teleop (Main Control)
```bash
# In new terminal
python3 teleop.py teleop_mode=robot
```

## Usage
1. Right-click in mouse VR GUI to start robot following mode
2. Hold left-click + move mouse to control robot hand position
3. Right-click again to stop robot following
4. Use Ctrl+C to stop all processes

## Troubleshooting
- Ensure all terminals are in the project directory
- Check network connectivity if VR connection fails
- Restart sequence if robot doesn't respond 