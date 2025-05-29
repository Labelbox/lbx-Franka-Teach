# ROS 2 MoveIt Franka FR3 Docker Image
# This image contains everything needed to run the Franka FR3 MoveIt demo

ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

# Avoid interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    # Build tools
    build-essential \
    cmake \
    git \
    wget \
    curl \
    # ROS 2 tools
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    # GUI support for RViz
    qtbase5-dev \
    qt5-qmake \
    # Utilities
    nano \
    vim \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /workspace

# Create a non-root user for development
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to the ros user
USER $USERNAME

# Set up Franka ROS 2 workspace
RUN mkdir -p /home/$USERNAME/franka_ros2_ws/src
WORKDIR /home/$USERNAME/franka_ros2_ws

# Clone and build Franka ROS 2 packages
RUN git clone https://github.com/frankaemika/franka_ros2.git src \
    && vcs import src < src/franka.repos --recursive --skip-existing \
    && sudo rosdep init || true \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y \
    && . /opt/ros/$ROS_DISTRO/setup.sh \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --packages-skip franka_ign_ros2_control franka_gazebo

# Apply the version fix
RUN sed -i '/param name="prefix"/a\          <param name="version">0.1.0</param>' \
    /home/$USERNAME/franka_ros2_ws/src/franka_description/robots/common/franka_arm.ros2_control.xacro \
    && . /opt/ros/$ROS_DISTRO/setup.sh \
    && . install/setup.bash \
    && colcon build --packages-select franka_description --symlink-install

# Copy the ros2_moveit_franka package
COPY --chown=$USERNAME:$USERNAME . /home/$USERNAME/ros2_moveit_franka/

# Build the package
WORKDIR /home/$USERNAME/ros2_moveit_franka
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && . /home/$USERNAME/franka_ros2_ws/install/setup.bash \
    && colcon build --symlink-install

# Set up environment in bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /home/$USERNAME/franka_ros2_ws/install/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /home/$USERNAME/ros2_moveit_franka/install/setup.bash" >> /home/$USERNAME/.bashrc

# Create convenience scripts
RUN echo '#!/bin/bash\n\
echo "🚀 Launching MoveIt for Franka FR3..."\n\
echo "Robot IP: ${ROBOT_IP:-192.168.1.59}"\n\
echo "Press Ctrl+C to stop"\n\
echo ""\n\
source /opt/ros/'$ROS_DISTRO'/setup.bash\n\
source ~/franka_ros2_ws/install/setup.bash\n\
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=${ROBOT_IP:-192.168.1.59} use_fake_hardware:=${USE_FAKE_HARDWARE:-false}' \
    > /home/$USERNAME/launch_moveit.sh && chmod +x /home/$USERNAME/launch_moveit.sh

RUN echo '#!/bin/bash\n\
echo "🎯 Running Franka FR3 Demo..."\n\
echo "Make sure MoveIt is running in another terminal!"\n\
echo ""\n\
source /opt/ros/'$ROS_DISTRO'/setup.bash\n\
source ~/franka_ros2_ws/install/setup.bash\n\
source ~/ros2_moveit_franka/install/setup.bash\n\
ros2 run ros2_moveit_franka simple_arm_control' \
    > /home/$USERNAME/run_demo.sh && chmod +x /home/$USERNAME/run_demo.sh

# Set environment variables
ENV ROBOT_IP=192.168.1.59
ENV USE_FAKE_HARDWARE=false
ENV ROS_DOMAIN_ID=0

# Expose ROS 2 ports
EXPOSE 11811
EXPOSE 7400-7500

# Set working directory
WORKDIR /home/$USERNAME

# Default command
CMD ["/bin/bash"]

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD source /opt/ros/$ROS_DISTRO/setup.bash && ros2 pkg list | grep -q franka_fr3_moveit_config || exit 1

# Labels
LABEL maintainer="ros2_moveit_franka"
LABEL description="ROS 2 MoveIt integration for Franka FR3 robot"
LABEL version="1.0" 