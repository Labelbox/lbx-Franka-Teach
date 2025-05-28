# Async Architecture & Performance Optimizations

## Overview

The Oculus VR Server uses a sophisticated asynchronous architecture to achieve high-frequency data recording (40Hz) while managing slower robot communication (6.6Hz). This document explains the architecture, optimizations, and performance characteristics.

## Architecture Diagram

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   VR Thread     │     │ Control Thread  │     │ Recording Thread│
│    (50Hz)       │────▶│    (40Hz)       │────▶│    (40Hz)       │
└─────────────────┘     └─────────────────┘     └─────────────────┘
         │                       │                        │
         ▼                       ▼                        ▼
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│ VR State Lock   │     │ Command Queue   │     │  MCAP Queue     │
└─────────────────┘     └─────────────────┘     └─────────────────┘
                                 │                        │
                                 ▼                        ▼
                        ┌─────────────────┐     ┌─────────────────┐
                        │Robot Comm Thread│     │ MCAP Writer     │
                        │   (Async I/O)   │     │    Thread       │
                        └─────────────────┘     └─────────────────┘
                                 │
                                 ▼
                        ┌─────────────────┐
                        │  Robot Hardware │
                        │    (6.6Hz)      │
                        └─────────────────┘
```

## Thread Architecture

### 1. **VR State Thread** (`_update_internal_state`)
- **Frequency**: 50Hz
- **Purpose**: Continuously polls Oculus VR controller
- **Features**:
  - Raw pose and button data capture
  - Forward direction calibration
  - Origin recalibration on grip press/release
  - Thread-safe state publishing via `_vr_state_lock`

### 2. **Robot Control Thread** (`_robot_control_worker`)
- **Frequency**: 40Hz (with performance mode)
- **Purpose**: Calculates robot commands at high frequency
- **Features**:
  - Velocity-based control with DROID-exact parameters
  - Predictive control when robot feedback is delayed
  - Non-blocking command queuing
  - Automatic mode switching (REAL-TIME ↔ PREDICTIVE)

### 3. **Data Recording Thread** (`_data_recording_worker`)
- **Frequency**: 40Hz (independent of robot speed)
- **Purpose**: Records data at consistent high frequency
- **Features**:
  - Decoupled from robot communication delays
  - Always uses latest available states
  - Non-blocking queue operations
  - Maintains data integrity at target frequency

### 4. **Robot Communication Thread** (`_robot_comm_worker`)
- **Frequency**: As fast as robot responds (~6.6Hz)
- **Purpose**: Handles slow robot I/O asynchronously
- **Features**:
  - Separate thread for blocking socket operations
  - Command buffering with small queue
  - Response queuing for control thread
  - Average latency tracking

### 5. **MCAP Writer Thread** (`_mcap_writer_worker`)
- **Frequency**: As fast as data arrives
- **Purpose**: Writes data to disk without blocking
- **Features**:
  - Large buffer (1000 items) for burst handling
  - Graceful shutdown with queue flushing
  - Error handling and recovery

## Key Optimizations

### 1. **Decoupled Architecture**
```python
# Traditional (Slow) - Everything sequential
while True:
    vr_data = read_vr()          # 20ms
    action = calculate_action()   # 5ms
    robot_response = send_robot() # 149ms - BLOCKS!
    record_data()                # 10ms
    # Total: 184ms = 5.4Hz

# Async (Fast) - Everything parallel
VR Thread:       read_vr() continuously at 50Hz
Control Thread:  calculate_action() at 40Hz
Robot Thread:    send_robot() asynchronously
Recording Thread: record_data() at 40Hz independently
```

### 2. **Predictive Control**
When robot feedback is delayed, the system automatically switches to predictive mode:
```python
if state_age > self.control_interval * 2:
    # Use predicted state based on commanded position
    new_robot_state = RobotState(
        pos=target_pos,  # Use target as prediction
        quat=target_quat,
        ...
    )
```

### 3. **Lock-Free Data Flow**
- Minimal lock contention with separate locks for different data
- Non-blocking queue operations with `put_nowait()` and `get_nowait()`
- Copy-on-read for thread safety

### 4. **Performance Mode**
Enabled with `--performance` flag:
- Control frequency: 20Hz → 40Hz
- Position gain: 5.0 → 10.0
- Rotation gain: 2.0 → 3.0
- Smaller deltas for higher frequency

## Performance Metrics

### Typical Performance
```
📊 Recording frequency: 39.2Hz (target: 40Hz)  ✅
⚡ Control frequency: 40.0Hz (target: 40Hz) - PREDICTIVE  ✅
📡 Avg robot comm: 149.0ms  (Limited by hardware)
```

### Data Quality
- **Recording Rate**: 39-40Hz consistently
- **Control Rate**: 40Hz (predictive mode)
- **Robot Response**: 6.6Hz (hardware limited)
- **VR Polling**: 50Hz (oversampled for accuracy)

## Configuration

### Control Frequency
Set in `frankateach/constants.py`:
```python
CONTROL_FREQ = 20  # Base frequency
# With performance mode: CONTROL_FREQ * 2 = 40Hz
```

### Thread Priorities
All worker threads are daemon threads with equal priority.

### Queue Sizes
```python
self.mcap_queue = queue.Queue(maxsize=1000)      # Large buffer
self._robot_command_queue = queue.Queue(maxsize=2) # Small, fresh commands
self._robot_response_queue = queue.Queue(maxsize=2) # Latest responses only
```

## Benefits

1. **6x Higher Recording Rate**: 40Hz vs 6.6Hz
2. **Smooth Control**: Commands sent at consistent 40Hz
3. **No Blocking**: Robot delays don't affect recording
4. **Predictive Control**: Maintains responsiveness
5. **Data Integrity**: All data timestamped accurately

## Usage

### Basic Usage
```bash
./run_server.sh
```

### With Performance Mode (Recommended)
```bash
./run_server.sh --performance
```

### With Hot Reload for Development
```bash
./run_server.sh --hot-reload --performance
```

### Debug Mode (No Robot)
```bash
./run_server.sh --debug --performance
```

## Troubleshooting

### Low Recording Frequency
- Check CPU usage - may need to reduce other processes
- Verify performance mode is enabled
- Check MCAP queue size in debug output

### Robot Communication Slow
- This is hardware limited (~149ms round trip)
- Predictive control compensates for this
- Check network/USB connection quality

### High CPU Usage
- Normal for 40Hz operation with multiple threads
- Consider reducing frequency if needed
- Use nice/renice for process priority

## Future Optimizations

1. **UDP Communication**: Could reduce latency vs TCP
2. **Shared Memory**: For local robot communication
3. **GPU Acceleration**: For complex transformations
4. **Adaptive Frequency**: Adjust based on system load

## Code Examples

### Adding a New Thread
```python
def _new_worker(self):
    """New worker thread template"""
    print("🆕 New worker started")
    
    while self.running:
        try:
            # Do work at specific frequency
            # Use locks for shared data
            # Use queues for communication
            pass
        except Exception as e:
            print(f"❌ Error: {e}")
            
    print("🆕 New worker stopped")
```

### Thread-Safe State Access
```python
# Writing state
with self._state_lock:
    self._shared_state = new_value

# Reading state  
with self._state_lock:
    local_copy = self._shared_state.copy()
```

## Summary

The async architecture achieves:
- **40Hz recording** despite 6.6Hz robot communication
- **Predictive control** for responsiveness
- **Non-blocking operation** throughout
- **Thread-safe** data handling
- **Graceful degradation** under load

This architecture ensures high-quality data collection for robot learning while maintaining smooth teleoperation. 