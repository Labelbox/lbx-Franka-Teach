# Docker Integration with Official franka_ros2

This document explains how our `ros2_moveit_franka` package integrates with the [official franka_ros2 Docker setup](https://github.com/frankaemika/franka_ros2).

## 🐳 Docker Architecture

### Base Integration

Our Docker setup extends and complements the official franka_ros2 Docker environment:

```
Official franka_ros2 Docker
├── ROS 2 Humble base image
├── libfranka dependencies
├── franka_ros2 packages
└── MoveIt integration

Our ros2_moveit_franka Docker
├── Extends: Official franka_ros2 environment
├── Adds: Our MoveIt demonstration package
├── Adds: Development tools and VS Code integration
└── Adds: Management scripts for easy usage
```

### Key Benefits

1. **🔄 Compatibility**: 100% compatible with official franka_ros2 Docker
2. **📦 Dependencies**: Automatically includes all franka_ros2 packages
3. **🛠️ Development**: VS Code devcontainer support
4. **🚀 Deployment**: Production-ready containerization
5. **🔧 Management**: Easy-to-use scripts for common tasks

## 📁 Docker Files Overview

### Core Docker Files

| File                 | Purpose               | Description                                                    |
| -------------------- | --------------------- | -------------------------------------------------------------- |
| `Dockerfile`         | Container definition  | Builds on ROS 2 Humble, installs franka_ros2, adds our package |
| `docker-compose.yml` | Service orchestration | Defines development and simulation services                    |
| `.dockerignore`      | Build optimization    | Excludes unnecessary files from Docker build                   |

### Development Integration

| File                              | Purpose             | Description                                      |
| --------------------------------- | ------------------- | ------------------------------------------------ |
| `.devcontainer/devcontainer.json` | VS Code integration | Full IDE setup with extensions and configuration |
| `scripts/docker_run.sh`           | Management script   | Easy commands for build, run, demo, development  |

## 🔧 Usage Patterns

### Quick Start

```bash
# Build environment (includes franka_ros2)
./scripts/docker_run.sh build

# Test with simulation
./scripts/docker_run.sh sim

# Run with real robot
./scripts/docker_run.sh demo --robot-ip 192.168.1.59
```

### Development Workflow

```bash
# Start development container
./scripts/docker_run.sh run

# Or use VS Code devcontainer
code .  # Click "Reopen in Container"
```

### Production Deployment

```bash
# Run in production mode
docker-compose up ros2_moveit_franka
```

## 🌐 Network Configuration

### Robot Communication

- **Mode**: Host networking (`network_mode: host`)
- **Purpose**: Direct access to robot at `192.168.1.59`
- **Ports**: ROS 2 DDS ports (7400-7404) automatically exposed

### GUI Support

- **Linux**: X11 forwarding via `/tmp/.X11-unix` mount
- **macOS**: XQuartz integration with `DISPLAY=host.docker.internal:0`
- **Windows**: VcXsrv support with proper environment variables

## 🔒 Security Considerations

### Container Capabilities

```yaml
cap_add:
  - SYS_NICE # Real-time scheduling for robot control
  - NET_ADMIN # Network configuration for ROS communication
```

### Volume Mounts

```yaml
volumes:
  - .:/workspace/ros2_ws/src/ros2_moveit_franka:rw # Source code (development)
  - /tmp/.X11-unix:/tmp/.X11-unix:rw # X11 GUI support
  - ros2_moveit_franka_bash_history:/root/.bash_history # Persistent history
```

## 🔄 Integration Points

### With Official franka_ros2

Our Docker setup is designed to work seamlessly with the official repository:

1. **Same Base Image**: Uses `ros:humble-ros-base`
2. **Same Dependencies**: Automatically clones and builds franka_ros2
3. **Same Network**: Host networking for robot communication
4. **Same Environment**: Compatible ROS 2 and environment setup

### With Your Existing Deoxys System

The Docker environment can coexist with your current setup:

- **Robot IP**: Uses same IP (`192.168.1.59`) from your `franka_right.yml`
- **Isolation**: Containerized environment doesn't interfere with host
- **Switching**: Easy to switch between Docker and native execution
- **Development**: Can develop in Docker while testing natively

## 🚀 Advanced Usage

### Custom Robot Configuration

```bash
# Use different robot IP
export ROBOT_IP=192.168.1.100
./scripts/docker_run.sh demo --robot-ip $ROBOT_IP
```

### Development with Live Reload

```bash
# Start development container with code mounting
./scripts/docker_run.sh run

# Inside container, your code changes are immediately available
# No need to rebuild container for code changes
```

### Integration with Official Examples

```bash
# Our container includes all franka_ros2 packages
# You can run official examples alongside our demo

# In container:
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=192.168.1.59
ros2 run ros2_moveit_franka simple_arm_control
```

## 📊 Performance Considerations

### Build Time Optimization

- **Base Layer Caching**: ROS 2 and system dependencies cached
- **Incremental Builds**: Only our package rebuilds on changes
- **Multi-stage**: Optimized for both development and production

### Runtime Performance

- **Host Networking**: No network virtualization overhead
- **GPU Access**: Direct GPU access for visualization
- **Real-time**: Proper capabilities for real-time robot control

## 🔧 Customization

### Extending the Docker Environment

```dockerfile
# Create custom Dockerfile extending ours
FROM ros2_moveit_franka:latest

# Add your custom packages
RUN apt-get update && apt-get install -y your-custom-package

# Add your custom ROS packages
COPY your_package /workspace/ros2_ws/src/your_package
RUN colcon build --packages-select your_package
```

### Custom Docker Compose Override

```yaml
# docker-compose.override.yml
version: "3.8"
services:
  ros2_moveit_franka:
    environment:
      - CUSTOM_VAR=value
    volumes:
      - ./custom_config:/workspace/custom_config
```

## 🧪 Testing

### Validation Commands

```bash
# Test Docker environment
docker --version
docker-compose --version

# Test build
./scripts/docker_run.sh build

# Test simulation
./scripts/docker_run.sh sim

# Test robot connectivity (from container)
./scripts/docker_run.sh shell
# Inside: ping 192.168.1.59
```

### Continuous Integration

The Docker setup is designed for CI/CD pipelines:

```yaml
# Example GitHub Actions workflow
- name: Build Docker image
  run: docker build -t ros2_moveit_franka .

- name: Test simulation
  run: docker-compose run --rm ros2_moveit_franka_sim
```

## 📝 Migration Guide

### From Native to Docker

1. **Backup current setup**: Save your workspace
2. **Test simulation**: `./scripts/docker_run.sh sim`
3. **Verify robot connection**: `./scripts/docker_run.sh demo`
4. **Migrate custom code**: Copy to package and rebuild

### From Official franka_ros2 Docker

1. **Stop existing containers**: `docker-compose down`
2. **Clone our package**: `git clone ...`
3. **Build new environment**: `./scripts/docker_run.sh build`
4. **Test compatibility**: Run your existing launch files

## 🆘 Troubleshooting

### Common Docker Issues

| Issue              | Solution                                                           |
| ------------------ | ------------------------------------------------------------------ |
| GUI not working    | Set up X11 forwarding correctly for your OS                        |
| Build failures     | Check Docker daemon, clean up with `./scripts/docker_run.sh clean` |
| Robot unreachable  | Verify host networking and robot IP                                |
| Performance issues | Ensure proper capabilities and GPU access                          |

### Debugging Commands

```bash
# Container status
docker ps -a

# Container logs
./scripts/docker_run.sh logs

# Network debugging
docker network ls

# Volume debugging
docker volume ls
```

## 🎯 Conclusion

Our Docker integration provides:

✅ **Seamless compatibility** with official franka_ros2  
✅ **Easy development** with VS Code integration  
✅ **Production deployment** capabilities  
✅ **Cross-platform support** for Linux/macOS/Windows  
✅ **Isolated environment** without host contamination  
✅ **Standard tooling** with Docker/Docker Compose

The integration maintains full compatibility with the official franka_ros2 Docker setup while adding modern development tools and easier management for robot control tasks.
