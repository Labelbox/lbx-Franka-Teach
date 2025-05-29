# ROS 2 MoveIt Franka FR3 Control

This package provides a simple demonstration of controlling a Franka FR3 robot arm using ROS 2 and MoveIt. The demo resets the arm to home position and then moves it 10cm in the X direction.

**🐳 Docker Support**: This package is fully compatible with the [official franka_ros2 Docker setup](https://github.com/frankaemika/franka_ros2) and includes its own Docker configuration for easy deployment.

## Prerequisites

### Option A: Docker Setup (Recommended) 🐳

**Advantages**: Consistent environment, no dependency conflicts, works on all platforms.

1. **Install Docker**:

   - **Linux**: Follow [Docker Engine installation](https://docs.docker.com/engine/install/)
   - **macOS**: Install [Docker Desktop](https://docs.docker.com/desktop/mac/)
   - **Windows**: Install [Docker Desktop](https://docs.docker.com/desktop/windows/)

2. **For GUI applications (RViz)**:
   - **Linux**: X11 forwarding is automatically configured
   - **macOS**: Install XQuartz: `brew install --cask xquartz` and run `open -a XQuartz`
   - **Windows**: Install [VcXsrv](https://sourceforge.net/projects/vcxsrv/)

### Option B: Local Installation

Refer to the local installation instructions in the later sections.

## Robot Configuration

The robot is configured with the following settings from your codebase:

- **Robot IP**: `192.168.1.59`
- **Robot Model**: Franka FR3
- **End Effector**: Franka Hand (gripper)

Make sure your robot is:

1. Connected to the network and accessible at the specified IP
2. In the correct mode (e.g., programming mode for external control)
3. E-stop is released and robot is ready for operation

## Quick Start with Docker 🚀

### 1. Build the Docker Environment

```bash
# Clone/copy the package to your workspace
cd ros2_moveit_franka

# Build the Docker image (includes franka_ros2 dependencies)
./scripts/docker_run.sh build
```

### 2. Run Simulation Demo (Safe Testing)

```bash
# Start simulation with GUI (RViz)
./scripts/docker_run.sh sim
```

### 3. Run with Real Robot

```bash
# Ensure robot is ready and accessible
ping 192.168.1.59

# Run with real robot
./scripts/docker_run.sh demo --robot-ip 192.168.1.59
```

### 4. Interactive Development

```bash
# Start interactive container for development
./scripts/docker_run.sh run

# Inside container:
ros2 launch ros2_moveit_franka franka_demo.launch.py use_fake_hardware:=true
```

## Docker Usage Guide 🐳

### Available Docker Commands

```bash
# Build Docker image
./scripts/docker_run.sh build

# Run interactive development container
./scripts/docker_run.sh run

# Run simulation demo
./scripts/docker_run.sh sim

# Run real robot demo
./scripts/docker_run.sh demo [--robot-ip IP]

# Open shell in running container
./scripts/docker_run.sh shell

# View container logs
./scripts/docker_run.sh logs

# Stop all containers
./scripts/docker_run.sh stop

# Clean up (remove containers and images)
./scripts/docker_run.sh clean
```

### VS Code Development Container

For integrated development experience:

1. **Install VS Code Extensions**:

   - Docker
   - Dev Containers
   - Remote Development

2. **Open in Container**:

   ```bash
   # Open the package directory in VS Code
   code ros2_moveit_franka

   # When prompted, click "Reopen in Container"
   # Or use Command Palette: "Dev Containers: Reopen in Container"
   ```

3. **Automatic Setup**: The devcontainer will automatically:
   - Build the Docker environment
   - Install franka_ros2 dependencies
   - Configure ROS 2 environment
   - Set up development tools

### Integration with Official franka_ros2 Docker

This package is designed to work seamlessly with the [official franka_ros2 Docker setup](https://github.com/frankaemika/franka_ros2):

- **Base Image**: Uses the same ROS 2 Humble base
- **Dependencies**: Automatically installs franka_ros2 from source
- **Configuration**: Compatible with official launch files and parameters
- **Network**: Uses host networking for real robot communication

## Local Installation (Alternative to Docker)

### 1. ROS 2 Humble Installation

Make sure you have ROS 2 Humble installed on your system. Follow the [official installation guide](https://docs.ros.org/en/humble/Installation.html).

### 2. Franka ROS 2 Dependencies

Install the official Franka ROS 2 packages:

```bash
# Create a ROS 2 workspace for Franka dependencies
mkdir -p ~/franka_ros2_ws/src
cd ~/franka_ros2_ws

# Clone the Franka ROS 2 repository
git clone https://github.com/frankaemika/franka_ros2.git src

# Install dependencies
vcs import src < src/franka.repos --recursive --skip-existing
rosdep install --from-paths src --ignore-src --rosdistro humble -y

# Build the workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash
```

### 3. Add to your ROS 2 environment

Add the Franka workspace to your ROS 2 environment:

```bash
echo "source ~/franka_ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 4. Install This Package

1. **Copy this package to your ROS 2 workspace:**

   ```bash
   # If you don't have a workspace yet
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src

   # Copy the package (assuming you're in the lbx-Franka-Teach directory)
   cp -r ros2_moveit_franka .
   ```

2. **Install dependencies for this package:**

   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src --rosdistro humble -y
   ```

3. **Build the package:**
   ```bash
   colcon build --packages-select ros2_moveit_franka
   source install/setup.bash
   ```

## Usage

### Option 1: Full Demo with Launch File (Recommended)

Start the complete system with MoveIt and visualization:

```bash
# For real robot (make sure robot is connected and ready)
ros2 launch ros2_moveit_franka franka_demo.launch.py robot_ip:=192.168.1.59

# For simulation/testing without real robot
ros2 launch ros2_moveit_franka franka_demo.launch.py robot_ip:=192.168.1.59 use_fake_hardware:=true
```

### Option 2: Manual Launch (Step by Step)

If you want to start components manually:

1. **Start the Franka MoveIt system:**

   ```bash
   # Terminal 1: Start MoveIt with real robot
   ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=192.168.1.59

   # OR for simulation
   ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=192.168.1.59 use_fake_hardware:=true
   ```

2. **Run the demo script:**

   ```bash
   # Terminal 2: Run the arm control demo
   ros2 run ros2_moveit_franka simple_arm_control
   ```

3. **Optional: Start RViz for visualization:**
   ```bash
   # Terminal 3: Launch RViz
   rviz2 -d $(ros2 pkg prefix franka_fr3_moveit_config)/share/franka_fr3_moveit_config/rviz/moveit.rviz
   ```

## Demo Sequence

The demo performs the following sequence:

1. **🔍 Initial State Check**: Prints current robot position and joint states
2. **🤏 Gripper Control**: Opens the gripper
3. **🏠 Home Position**: Moves the robot to a safe home/ready position
4. **➡️ X-Direction Movement**: Moves the end effector 10cm in the positive X direction
5. **🏠 Return Home**: Returns the robot to the home position

## Safety Notes

⚠️ **Important Safety Information:**

- Always ensure the robot workspace is clear before running the demo
- Keep the emergency stop within reach during operation
- The robot will move to predefined positions - ensure these are safe for your setup
- Start with simulation mode (`use_fake_hardware:=true`) to test before using real hardware
- The demo includes conservative velocity and acceleration limits for safety

## Troubleshooting

### Docker-Specific Issues:

1. **GUI applications (RViz) not displaying**:

   - **Linux**: Run `xhost +local:docker` before starting containers
   - **macOS**: Ensure XQuartz is running and `DISPLAY` is set correctly
   - **Windows**: Configure VcXsrv with proper settings

2. **Container build failures**:

   ```bash
   # Clean up and rebuild
   ./scripts/docker_run.sh clean
   ./scripts/docker_run.sh build
   ```

3. **Robot connection issues in Docker**:
   - Ensure network mode is set to `host` (default in docker-compose.yml)
   - Check robot IP accessibility from host: `ping 192.168.1.59`

### Common Issues:

1. **"Failed to connect to robot"**

   - Check robot IP address (should be `192.168.1.59`)
   - Ensure robot is powered on and in programming mode
   - Verify network connectivity: `ping 192.168.1.59`

2. **"Planning failed"**

   - Check if the target position is within robot workspace
   - Ensure no obstacles are blocking the path
   - Try increasing planning timeout or attempts

3. **"MoveGroup not available"**

   - Ensure the Franka MoveIt configuration is running
   - Check that all required ROS 2 nodes are active: `ros2 node list`

4. **Missing dependencies**
   - Make sure you installed the Franka ROS 2 packages
   - Run `rosdep install` again to check for missing dependencies

### Debug Commands:

```bash
# Check if robot is reachable
ping 192.168.1.59

# List active ROS 2 nodes
ros2 node list

# Check MoveIt planning groups
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene

# Monitor robot state
ros2 topic echo /joint_states

# Docker container status
docker ps
```

## Configuration

### Robot Settings

- **Planning Group**: `panda_arm` (7-DOF arm)
- **Gripper Group**: `panda_hand` (2-finger gripper)
- **End Effector Link**: `panda_hand`
- **Planning Frame**: `panda_link0`

### Safety Limits

- **Max Velocity Scale**: 30% of maximum
- **Max Acceleration Scale**: 30% of maximum
- **Planning Time**: 10 seconds
- **Planning Attempts**: 10

### Docker Configuration

- **Base Image**: `ros:humble-ros-base`
- **Network**: Host mode for robot communication
- **GUI Support**: X11 forwarding for RViz
- **Development**: Live code mounting for easy iteration

## Extending the Demo

To modify the demo for your needs:

1. **Edit the target positions** in `simple_arm_control.py`
2. **Add more movement sequences** to the `execute_demo_sequence()` method
3. **Adjust safety parameters** in the constructor
4. **Add custom named poses** by modifying the MoveIt configuration

## Integration with Existing System

This package is designed to work alongside your existing Deoxys-based control system:

- **Robot IP**: Uses the same robot (`192.168.1.59`) configured in your `franka_right.yml`
- **Workspace Limits**: Respects the workspace bounds defined in your constants
- **Safety**: Implements conservative limits compatible with your current setup
- **Docker**: Can run alongside or replace your current Docker setup

You can run this demo independently of your Deoxys system, but make sure only one control system is active at a time.

## Advanced Usage

### Custom Docker Builds

```bash
# Build with specific ROS distro
docker build --build-arg ROS_DISTRO=humble -t custom_franka .

# Run with custom configuration
docker-compose -f docker-compose.yml -f docker-compose.override.yml up
```

### Production Deployment

```bash
# For production use, disable development volumes
docker-compose -f docker-compose.yml up ros2_moveit_franka
```

## License

MIT License - Feel free to modify and extend for your research needs.

## Support

For issues related to:

- **This package**: Check the troubleshooting section above
- **Docker setup**: See [Docker documentation](https://docs.docker.com/)
- **Franka ROS 2**: See [official documentation](https://frankaemika.github.io/docs/franka_ros2.html)
- **MoveIt**: See [MoveIt documentation](https://moveit.ros.org/)

## References

- [Official Franka ROS 2 Repository](https://github.com/frankaemika/franka_ros2)
- [MoveIt 2 Documentation](https://moveit.ros.org/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Docker Documentation](https://docs.docker.com/)
