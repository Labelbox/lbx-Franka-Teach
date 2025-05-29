ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}

# Configure apt for better reliability
RUN echo 'Acquire::http::Timeout "300";' > /etc/apt/apt.conf.d/99timeout && \
    echo 'Acquire::Retries "3";' >> /etc/apt/apt.conf.d/99timeout && \
    echo 'Acquire::http::Pipeline-Depth "0";' >> /etc/apt/apt.conf.d/99timeout

# Update package lists with retry
RUN apt-get update || (sleep 5 && apt-get update) || (sleep 10 && apt-get update)

# Install system dependencies in smaller chunks
RUN apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    vim \
    nano \
    iputils-ping \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

# Install MoveIt dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-moveit-ros-planning-interface \
    ros-${ROS_DISTRO}-moveit-visual-tools \
    ros-${ROS_DISTRO}-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory
WORKDIR /workspace

# Clone and build franka_ros2 dependencies
RUN mkdir -p /workspace/franka_ros2_ws/src && \
    cd /workspace/franka_ros2_ws && \
    git clone https://github.com/frankaemika/franka_ros2.git src && \
    vcs import src < src/franka.repos --recursive --skip-existing && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y && \
    bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Create main workspace for our package
RUN mkdir -p /workspace/ros2_ws/src

# Copy our package into the container
COPY . /workspace/ros2_ws/src/ros2_moveit_franka

# Set up environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/franka_ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/ros2_ws/install/setup.bash" >> ~/.bashrc

# Build our package
WORKDIR /workspace/ros2_ws
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /workspace/franka_ros2_ws/install/setup.bash && \
    rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y && \
    colcon build --packages-select ros2_moveit_franka --symlink-install"

# Create entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# Source ROS 2 environment\n\
source /opt/ros/'${ROS_DISTRO}'/setup.bash\n\
source /workspace/franka_ros2_ws/install/setup.bash\n\
source /workspace/ros2_ws/install/setup.bash\n\
\n\
# Execute the command\n\
exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["bash"]

# Set working directory
WORKDIR /workspace/ros2_ws

# Expose common ROS 2 ports
EXPOSE 7400 7401 7402 7403 7404 