# Robust Franka Control System

A crash-proof ROS2 MoveIt implementation for Franka FR3 robots with automatic recovery and exception handling.

## 🚀 Features

- **Crash-Proof Operation**: Comprehensive exception handling for libfranka errors
- **Auto-Recovery**: Automatic restart and recovery from connection failures
- **Health Monitoring**: Real-time system health monitoring and diagnostics
- **Graceful Error Handling**: Smart error detection and recovery procedures
- **Production Ready**: Built for continuous operation in production environments
- **Comprehensive Logging**: Detailed error reporting and system status logging

## 🏗️ Architecture

The system consists of three main components:

1. **Robust Franka Control Node** (`robust_franka_control.py`)
   - Main control node with exception handling
   - State machine for robot status tracking
   - Automatic MoveIt component reinitialization
   - Thread-safe operation with recovery procedures

2. **System Health Monitor** (`system_health_monitor.py`)
   - Monitors system health and performance
   - Tracks process status and resource usage
   - Automatic restart of failed components
   - ROS diagnostics integration

3. **Robust Production Launch** (`franka_robust_production.launch.py`)
   - Orchestrates the entire system
   - Configurable launch parameters
   - Event handling and process monitoring
   - Graceful shutdown management

## 📦 Build Instructions

1. **Clear old build files and rebuild**:
   ```bash
   cd ros2_moveit_franka
   rm -rf build/ install/ log/
   colcon build --packages-select ros2_moveit_franka
   ```

2. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```

## 🚀 Usage

### Quick Start

The easiest way to run the system is using the provided launch script:

```bash
./run_robust_franka.sh
```

### Advanced Usage

#### Command-line Options

```bash
./run_robust_franka.sh [OPTIONS]

Options:
  --robot-ip IP             Robot IP address (default: 192.168.1.59)
  --fake-hardware           Use fake hardware for testing
  --no-rviz                 Disable RViz visualization
  --no-health-monitor       Disable health monitoring
  --no-auto-restart         Disable automatic restart
  --log-level LEVEL         Set log level (DEBUG, INFO, WARN, ERROR)
  --help                    Show help message
```

#### Examples

```bash
# Use defaults (robot at 192.168.1.59)
./run_robust_franka.sh

# Custom robot IP
./run_robust_franka.sh --robot-ip 192.168.1.100

# Test mode without robot hardware
./run_robust_franka.sh --fake-hardware --no-rviz

# Production mode with debug logging
./run_robust_franka.sh --log-level DEBUG
```

### Manual Launch

For more control, you can launch components manually:

#### 1. Launch the base MoveIt system:
```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=192.168.1.59
```

#### 2. Launch the robust control system:
```bash
ros2 launch ros2_moveit_franka franka_robust_production.launch.py robot_ip:=192.168.1.59
```

#### 3. Run individual components:
```bash
# Robust control node
ros2 run ros2_moveit_franka robust_franka_control

# Health monitor
ros2 run ros2_moveit_franka system_health_monitor
```

## 🔧 Configuration

### Robot IP Configuration

Update the default robot IP in multiple places:

1. **Launch script**: Edit `ROBOT_IP` in `run_robust_franka.sh`
2. **Health monitor**: Edit IP in `system_health_monitor.py` line 241
3. **Launch files**: Update default values in launch files

### Recovery Settings

Modify recovery behavior in `robust_franka_control.py`:

```python
@dataclass
class RecoveryConfig:
    max_retries: int = 5              # Max recovery attempts
    retry_delay: float = 2.0          # Delay between retries
    connection_timeout: float = 10.0   # Connection timeout
    emergency_stop_timeout: float = 1.0
    health_check_interval: float = 1.0 # Health check frequency
```

### Health Monitor Settings

Adjust monitoring parameters in `system_health_monitor.py`:

```python
self.monitor_interval = 2.0        # Health check interval
self.restart_threshold = 3         # Failures before restart
self.auto_restart_enabled = True   # Enable auto-restart
```

## 📊 Monitoring and Diagnostics

### ROS Topics

The system publishes several monitoring topics:

- `/robot_state` - Current robot state (initializing, ready, moving, error, etc.)
- `/robot_health` - Boolean health status
- `/robot_errors` - Error messages with timestamps
- `/system_health` - Overall system health status
- `/health_metrics` - Detailed JSON health metrics
- `/diagnostics` - ROS diagnostics messages

### Monitor System Status

```bash
# Watch robot state
ros2 topic echo /robot_state

# Monitor health
ros2 topic echo /robot_health

# View errors
ros2 topic echo /robot_errors

# Detailed metrics
ros2 topic echo /health_metrics
```

### View Diagnostics

```bash
# ROS diagnostics
ros2 topic echo /diagnostics

# System processes
ps aux | grep franka
```

## 🛠️ Troubleshooting

### Common Issues

1. **libfranka Connection Errors**
   - The system automatically detects and recovers from these
   - Check robot network connectivity
   - Verify robot is in the correct mode

2. **MoveIt Planning Failures**
   - System will retry with exponential backoff
   - Check joint limits and workspace constraints
   - Verify robot configuration

3. **Build Errors**
   - Ensure all dependencies are installed: `pip install psutil`
   - Clear build files: `rm -rf build/ install/ log/`
   - Check ROS2 environment: `source /opt/ros/humble/setup.bash`

### Recovery Procedures

The system implements several recovery mechanisms:

1. **Automatic Reinitialization**: MoveIt components are reinitialized on errors
2. **Process Restart**: Failed nodes are automatically restarted
3. **Emergency Stop**: Immediate robot stop on critical errors
4. **Health Monitoring**: Continuous monitoring with automated recovery

### Manual Recovery

If manual intervention is needed:

```bash
# Stop all processes
pkill -f robust_franka
pkill -f system_health_monitor

# Restart the system
./run_robust_franka.sh
```

## 🔒 Safety Features

- **Emergency Stop**: Immediate stop on any critical error
- **State Machine**: Prevents commands during error states
- **Connection Monitoring**: Continuous robot connectivity checks
- **Resource Monitoring**: CPU/Memory usage monitoring
- **Graceful Shutdown**: Clean process termination on exit

## 📈 Performance

The robust system is designed for:

- **Continuous Operation**: 24/7 production environments
- **Low Latency**: Minimal overhead from error handling
- **High Reliability**: Multiple failure modes handled gracefully
- **Resource Efficient**: Optimized for minimal system impact

## 🔄 Updates and Maintenance

To update the system:

1. **Pull latest changes**
2. **Rebuild package**: `colcon build --packages-select ros2_moveit_franka`
3. **Test with fake hardware**: `./run_robust_franka.sh --fake-hardware`
4. **Deploy to production**

## 📞 Support

For issues or questions:

1. Check the logs: `~/.ros/log/`
2. Monitor diagnostics: `ros2 topic echo /diagnostics`
3. Review error messages: `ros2 topic echo /robot_errors`

## 🏆 Features Comparison

| Feature | Standard MoveIt | Robust System |
|---------|----------------|---------------|
| Error Handling | Basic | Comprehensive |
| Auto Recovery | None | Full |
| Health Monitoring | None | Real-time |
| Process Restart | Manual | Automatic |
| Production Ready | No | Yes |
| Diagnostics | Limited | Extensive |

The robust system provides enterprise-grade reliability for Franka robot operations with minimal configuration required. 