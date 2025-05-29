# Getting Started with ROS 2 MoveIt Franka Control

## 🎯 What We've Created

This package provides a complete ROS 2 MoveIt integration for your Franka FR3 robot. It includes:

- **Simple Arm Controller**: Resets arm to home and moves 10cm in X direction
- **Launch Files**: Complete system startup with MoveIt and visualization
- **Safety Features**: Conservative limits and error handling
- **Integration**: Compatible with your existing Deoxys setup
- **🐳 Docker Support**: Full Docker integration with the [official franka_ros2](https://github.com/frankaemika/franka_ros2)

## 📁 Package Structure

```
ros2_moveit_franka/
├── package.xml                          # ROS 2 package manifest
├── setup.py                            # Python package setup
├── README.md                           # Complete documentation
├── GETTING_STARTED.md                  # This file
├── Dockerfile                          # Docker container definition
├── docker-compose.yml                 # Docker Compose configuration
├── .dockerignore                       # Docker build optimization
├── .devcontainer/                      # VS Code dev container
│   └── devcontainer.json              # Development environment config
├── launch/
│   └── franka_demo.launch.py          # Launch file for complete system
├── ros2_moveit_franka/
│   ├── __init__.py                     # Package init
│   └── simple_arm_control.py          # Main control script
├── scripts/
│   ├── quick_test.sh                   # Build and test script
│   └── docker_run.sh                  # Docker management script
└── resource/
    └── ros2_moveit_franka              # ROS 2 resource file
```

## 🚀 Quick Start (Choose Your Path)

### Path A: Docker (Recommended) 🐳

**Why Docker?** Consistent environment, no dependency conflicts, works on all platforms.

#### Step 1: Install Docker

```bash
# Linux
curl -fsSL https://get.docker.com -o get-docker.sh && sh get-docker.sh

# macOS
brew install --cask docker

# Windows: Install Docker Desktop from https://docker.com
```

#### Step 2: Setup GUI Support

```bash
# Linux (run once)
xhost +local:docker

# macOS: Install XQuartz
brew install --cask xquartz
open -a XQuartz

# Windows: Install VcXsrv from https://sourceforge.net/projects/vcxsrv/
```

#### Step 3: Build and Run

```bash
# Navigate to the package
cd ros2_moveit_franka

# Build Docker environment (includes franka_ros2)
./scripts/docker_run.sh build

# Test with simulation (safe)
./scripts/docker_run.sh sim

# Run with real robot (ensure robot is ready!)
./scripts/docker_run.sh demo --robot-ip 192.168.1.59
```

**🎉 That's it! You're controlling your Franka FR3 with Docker!**

### Path B: Local Installation

#### Step 1: Install Franka ROS 2 Dependencies

```bash
# Create workspace and install franka_ros2
mkdir -p ~/franka_ros2_ws/src && cd ~/franka_ros2_ws
git clone https://github.com/frankaemika/franka_ros2.git src
vcs import src < src/franka.repos --recursive --skip-existing
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "source ~/franka_ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### Step 2: Build This Package

```bash
# Copy to your ROS 2 workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
cp -r /path/to/this/ros2_moveit_franka .

# Build
cd ~/ros2_ws
colcon build --packages-select ros2_moveit_franka
source install/setup.bash
```

#### Step 3: Run the Demo

```bash
# Test in simulation first (safe)
ros2 launch ros2_moveit_franka franka_demo.launch.py use_fake_hardware:=true

# Then with real robot (ensure robot is ready!)
ros2 launch ros2_moveit_franka franka_demo.launch.py robot_ip:=192.168.1.59
```

## 🐳 Docker Commands Reference

```bash
# Essential commands
./scripts/docker_run.sh build    # Build Docker image
./scripts/docker_run.sh sim      # Run simulation demo
./scripts/docker_run.sh demo     # Run real robot demo
./scripts/docker_run.sh run      # Interactive development

# Development commands
./scripts/docker_run.sh shell    # Open shell in container
./scripts/docker_run.sh logs     # View container logs
./scripts/docker_run.sh stop     # Stop containers
./scripts/docker_run.sh clean    # Clean up everything
```

## 💻 VS Code Development

For the best development experience:

1. **Install VS Code Extensions**:

   - Docker
   - Dev Containers
   - Remote Development

2. **Open in Container**:

   ```bash
   code ros2_moveit_franka
   # Click "Reopen in Container" when prompted
   ```

3. **Automatic Setup**: Everything is configured automatically!

## 🤖 Robot Configuration Used

Based on your existing codebase:

- **Robot IP**: `192.168.1.59` (from `franka_right.yml`)
- **Model**: Franka FR3
- **Control**: MoveIt with hardware interface
- **Safety**: 30% velocity/acceleration limits

## 🔧 What the Demo Does

1. **Initialize**: Connects to robot and MoveIt planning
2. **Reset**: Moves robot to safe home position
3. **Move**: Translates end-effector 10cm in +X direction
4. **Return**: Returns to home position
5. **Monitor**: Prints positions and states throughout

## 📊 Expected Output

```
[INFO] [franka_arm_controller]: Franka FR3 Arm Controller Initialized
[INFO] [franka_arm_controller]: Planning frame: panda_link0
[INFO] [franka_arm_controller]: End effector link: panda_hand
[INFO] [franka_arm_controller]: Moving to home position...
[INFO] [franka_arm_controller]: ✅ Successfully moved to 'ready' position
[INFO] [franka_arm_controller]: Moving 10.0cm in +X direction...
[INFO] [franka_arm_controller]: ✅ Successfully moved in X direction
[INFO] [franka_arm_controller]: ✅ DEMO SEQUENCE COMPLETED SUCCESSFULLY!
```

## ⚠️ Safety Checklist

Before running with real robot:

- [ ] Robot is powered on and in programming mode
- [ ] Robot workspace is clear of obstacles
- [ ] Emergency stop is accessible
- [ ] Network connection to `192.168.1.59` is working
- [ ] Test in simulation mode first
- [ ] Only one control system active (not Deoxys simultaneously)

## 🔍 Quick Debugging

### Docker Issues

```bash
# Check Docker status
docker --version
docker-compose --version

# GUI not working?
# Linux: xhost +local:docker
# macOS: Ensure XQuartz is running
# Windows: Configure VcXsrv properly

# Container logs
./scripts/docker_run.sh logs
```

### General Issues

```bash
# Check robot connectivity
ping 192.168.1.59

# Verify environment
echo $ROS_DISTRO  # Should show "humble"

# Check if packages are available
ros2 pkg list | grep franka

# Test build
./scripts/docker_run.sh build
```

## 🚀 Advanced Docker Usage

### Custom Robot IP

```bash
# Use different robot IP
./scripts/docker_run.sh demo --robot-ip 192.168.1.100
```

### Development Workflow

```bash
# Start development container
./scripts/docker_run.sh run

# Inside container, modify code and test
ros2 launch ros2_moveit_franka franka_demo.launch.py use_fake_hardware:=true

# Code changes are automatically synced!
```

### Integration with Official franka_ros2 Docker

This package is fully compatible with the [official franka_ros2 Docker setup](https://github.com/frankaemika/franka_ros2):

- Uses the same base image and dependencies
- Follows the same conventions
- Can be used alongside official examples
- Includes all franka_ros2 packages automatically

## 📚 Next Steps

1. **Experiment**: Modify target positions in `simple_arm_control.py`
2. **Extend**: Add more complex movement patterns
3. **Integrate**: Combine with your existing Deoxys workflows
4. **Learn**: Explore MoveIt's advanced features (constraints, planning scenes)
5. **Develop**: Use VS Code devcontainer for seamless development

## 🔗 Compatibility

### With Official franka_ros2

- ✅ Same Docker base image
- ✅ Compatible launch files
- ✅ Shared dependencies
- ✅ Network configuration

### With Your Existing System

- ✅ Same robot IP configuration
- ✅ Compatible workspace limits
- ✅ Parallel operation (when needed)
- ✅ Shared configuration files

## 🆘 Need Help?

- **Package Issues**: Check the main `README.md`
- **Docker Issues**: See [Docker documentation](https://docs.docker.com/)
- **Franka ROS 2**: See [official docs](https://frankaemika.github.io/docs/franka_ros2.html)
- **MoveIt Help**: Visit [MoveIt tutorials](https://moveit.ros.org/documentation/tutorials/)

---

🎉 **You're ready to control your Franka FR3 with ROS 2 MoveIt using Docker!**

**Recommended first steps:**

1. `./scripts/docker_run.sh build`
2. `./scripts/docker_run.sh sim`
3. `./scripts/docker_run.sh demo`
