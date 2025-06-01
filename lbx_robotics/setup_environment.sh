#!/bin/bash
# Setup script for the lbx_robotics Conda environment with integrated franka_ros2

# --- Configuration ---
ENV_NAME="lbx_robotics_env"
ENV_FILE="environment.yaml"
REQ_FILE="requirements.txt"
# Get the directory of this script to ensure relative paths work
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"

# --- Colors for Output ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# --- Helper Functions ---
echo_error() {
    echo -e "${RED}${BOLD}ERROR:${NC}${RED} $1${NC}" >&2
}

echo_success() {
    echo -e "${GREEN}${BOLD}✅ SUCCESS:${NC}${GREEN} $1${NC}"
}

echo_info() {
    echo -e "${BLUE}INFO:${NC} $1"
}

echo_warn() {
    echo -e "${YELLOW}${BOLD}WARNING:${NC}${YELLOW} $1${NC}"
}

echo_step() {
    echo -e "${CYAN}${BOLD}STEP:${NC}${CYAN} $1${NC}"
}

# --- Pre-flight Checks ---
echo_info "Starting environment setup for '$ENV_NAME' with integrated franka_ros2..."
echo_info "This script will set up conda environment and integrate franka_ros2 into lbx_robotics workspace."
echo_info "Script location: $SCRIPT_DIR"

if ! command -v conda &> /dev/null; then
    echo_error "Conda is not installed or not in your PATH. Please install Miniconda or Anaconda."
    exit 1
fi
echo_success "Conda found."

if [ ! -f "$SCRIPT_DIR/$ENV_FILE" ]; then
    echo_error "Environment file '$ENV_FILE' not found in $SCRIPT_DIR/"
    echo_error "Please ensure you are running this script from the root of the 'lbx_robotics' workspace."
    exit 1
fi

if [ ! -f "$SCRIPT_DIR/$REQ_FILE" ]; then
    echo_warn "Requirements file '$REQ_FILE' not found in $SCRIPT_DIR/. Pip installation step will be skipped if environment is created from scratch."
fi

# --- Create or Update Conda Environment ---
echo_step "Setting up Conda environment..."
if conda env list | grep -q "^$ENV_NAME\s"; then
    echo_info "Conda environment '$ENV_NAME' already exists."
    echo_info "Removing existing environment to ensure clean setup..."
    conda env remove -n "$ENV_NAME" -y
    echo_success "Existing environment removed."
fi

echo_info "Creating Conda environment '$ENV_NAME' from $SCRIPT_DIR/$ENV_FILE..."
if conda env create -f "$SCRIPT_DIR/$ENV_FILE" -n "$ENV_NAME"; then
    echo_success "Conda environment '$ENV_NAME' created successfully."
else
    echo_error "Failed to create Conda environment '$ENV_NAME' from $SCRIPT_DIR/$ENV_FILE."
    exit 1
fi

# --- Install Pip Dependencies --- 
echo_step "Installing pip dependencies..."
if [ -f "$SCRIPT_DIR/$REQ_FILE" ]; then
    # Use conda run to execute pip install within the target environment
    # --no-capture-output and --live-stream allow seeing pip's output directly
    if conda run -n "$ENV_NAME" --no-capture-output --live-stream pip install -r "$SCRIPT_DIR/$REQ_FILE"; then
        echo_success "Pip packages installed/updated successfully from $SCRIPT_DIR/$REQ_FILE."
    else
        echo_error "Failed to install/update pip packages from $SCRIPT_DIR/$REQ_FILE."
        echo_info "You might need to activate the environment (\`conda activate $ENV_NAME\`) and run \`pip install -r $SCRIPT_DIR/$REQ_FILE\` manually to troubleshoot."
        exit 1 # Exit if pip install fails, as it might be critical
    fi
else
    echo_info "Pip requirements file '$REQ_FILE' not found in $SCRIPT_DIR/. Skipping pip installation."
fi

# --- Install additional Python packages needed for franka_ros2 in conda environment ---
echo_step "Installing additional Python packages for franka_ros2 integration..."
if conda run -n "$ENV_NAME" --no-capture-output --live-stream pip install catkin_pkg empy lark; then
    echo_success "Additional Python packages for franka_ros2 installed successfully."
else
    echo_warn "Some additional packages may not have installed correctly. Continuing anyway."
fi

# --- ROS 2 Humble Installation ---
echo_step "Installing ROS 2 Humble..."

# Check if ROS 2 Humble is already installed
if dpkg -l | grep -q "ros-humble-desktop"; then
    echo_success "ROS 2 Humble Desktop already installed."
elif dpkg -l | grep -q "ros-humble-ros-base"; then
    echo_success "ROS 2 Humble Base already installed."
else
    echo_info "Installing ROS 2 Humble Desktop and development tools..."
    
    # Update package index
    sudo apt update
    
    # Install ROS 2 Humble Desktop (includes visualization tools)
    if sudo apt install -y ros-humble-desktop ros-dev-tools; then
        echo_success "ROS 2 Humble Desktop and development tools installed successfully."
    else
        echo_error "Failed to install ROS 2 Humble. Please check your internet connection and system compatibility."
        exit 1
    fi
fi

# Source ROS 2 environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo_success "ROS 2 Humble environment sourced."
else
    echo_error "ROS 2 Humble setup.bash not found. Installation may have failed."
    exit 1
fi

# --- Install additional tools ---
echo_step "Installing additional ROS 2 tools..."
if sudo apt install -y python3-vcstool python3-rosdep python3-colcon-common-extensions; then
    echo_success "Additional ROS 2 tools installed successfully."
else
    echo_warn "Some additional tools may not have installed correctly. Continuing anyway."
fi

# --- Install system dependencies for libfranka and other packages ---
echo_step "Installing remaining system-level dependencies (if any)..."
# Most C++ deps like Poco, PCRE, Eigen, Boost are now handled by conda.
# This section will now focus on things not typically in conda-forge or specific OS packages.

# Install other build dependencies (some might still be needed system-wide for ROS tools)
echo_info "Installing essential build tools (cmake, git, pkg-config)..."
if sudo apt install -y \
    build-essential \
    cmake \
    git \
    pkg-config; then
    echo_success "Essential build tools installed successfully."
else
    echo_warn "Some essential build tools may not have installed correctly."
fi

# Install RealSense SDK dependencies (these often have specific driver/kernel interactions)
echo_info "Installing RealSense camera dependencies (system-level)..."
if sudo apt install -y \
    libusb-1.0-0-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev; then
    echo_success "RealSense dependencies installed successfully."
else
    echo_warn "Some RealSense dependencies may not have installed correctly."
fi

# Install ROS2 control packages (these are ROS-specific)
echo_info "Installing ROS2 control packages (system-level)..."
if sudo apt install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gripper-controllers \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-xacro; then
    echo_success "ROS2 control packages installed successfully."
else
    echo_warn "Some ROS2 control packages may not have installed correctly."
fi

# Install Gazebo for simulation (ROS-specific)
echo_info "Installing Gazebo simulation packages (system-level)..."
if sudo apt install -y \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-msgs \
    ros-humble-gazebo-plugins; then
    echo_success "Gazebo packages installed successfully."
else
    echo_warn "Gazebo packages installation failed. Simulation features may not work."
fi

# Pinocchio is handled by conda, apt install for ros-humble-pinocchio is a fallback.
echo_info "Checking for ROS2 Pinocchio package (fallback)..."
if sudo apt install -y ros-humble-pinocchio; then
    echo_success "ROS2 Pinocchio package (ros-humble-pinocchio) is present or installed."
else
    echo_warn "ROS2 Pinocchio package (ros-humble-pinocchio) not found. Main installation via conda."
fi

# Initialize rosdep if not already done
if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    echo_info "Initializing rosdep..."
    sudo rosdep init
fi

echo_info "Updating rosdep..."
rosdep update

# --- Integrate franka_ros2 into lbx_robotics workspace ---
echo_step "Integrating franka_ros2 packages into lbx_robotics workspace..."

# Check if libfranka is already installed system-wide
echo_info "Checking for system-wide libfranka installation..."
if pkg-config --exists libfranka 2>/dev/null; then
    echo_success "libfranka found system-wide at: $(pkg-config --variable=prefix libfranka)"
    echo_info "Will use system libfranka instead of building from source"
    SKIP_LIBFRANKA_BUILD=true
else
    echo_warn "System libfranka not found, will need to handle during build"
    SKIP_LIBFRANKA_BUILD=false
    
    # Install libfranka if available from apt
    echo_info "Attempting to install libfranka from apt..."
    if sudo apt install -y libfranka-dev; then
        echo_success "libfranka installed from apt"
        SKIP_LIBFRANKA_BUILD=true
    else
        echo_warn "libfranka not available from apt"
    fi
fi

# Ensure we're in the lbx_robotics directory
cd "$SCRIPT_DIR"

# Create src directory if it doesn't exist
if [ ! -d "$SCRIPT_DIR/src" ]; then
    echo_info "Creating src directory in lbx_robotics workspace..."
    mkdir -p "$SCRIPT_DIR/src"
fi

# Clone franka_ros2 repository into src if not already present
if [ ! -d "$SCRIPT_DIR/src/franka_ros2" ]; then
    echo_info "Cloning franka_ros2 repository into lbx_robotics/src/..."
    cd "$SCRIPT_DIR/src"
    if git clone https://github.com/frankaemika/franka_ros2.git; then
        echo_success "franka_ros2 repository cloned successfully into lbx_robotics workspace."
    else
        echo_error "Failed to clone franka_ros2 repository."
        exit 1
    fi
    cd "$SCRIPT_DIR"
else
    echo_info "franka_ros2 repository already present in lbx_robotics/src/."
fi

# Import franka_ros2 dependencies into the lbx_robotics workspace
echo_info "Importing franka_ros2 dependencies into lbx_robotics workspace..."
if [ -f "$SCRIPT_DIR/src/franka_ros2/franka.repos" ]; then
    cd "$SCRIPT_DIR"
    if vcs import src < src/franka_ros2/franka.repos --recursive --skip-existing; then
        echo_success "franka_ros2 dependencies imported successfully into lbx_robotics workspace."
    else
        echo_warn "Some dependencies may not have imported correctly. Continuing anyway."
    fi
else
    echo_warn "franka.repos file not found. Some dependencies may be missing."
fi

# Install dependencies with rosdep for the entire lbx_robotics workspace
echo_info "Installing workspace dependencies with rosdep..."
cd "$SCRIPT_DIR"
if rosdep install --from-paths src --ignore-src --rosdistro humble -y; then
    echo_success "Workspace dependencies installed successfully."
else
    echo_warn "Some dependencies may not have installed correctly. Continuing anyway."
fi

# --- Final Instructions / Activation ---
echo ""
echo_success "-------------------------------------------------------------"
echo_success " Environment setup complete for '$ENV_NAME' + franka_ros2! "
echo_success "-------------------------------------------------------------"
echo ""
echo_info "To use the complete environment, run these commands in order:"
echo ""
echo -e "${CYAN}${BOLD}1. Activate Conda environment:${NC}"
echo -e "   ${CYAN}conda activate $ENV_NAME${NC}"
echo ""
echo -e "${CYAN}${BOLD}2. Source ROS 2 environment:${NC}"
echo -e "   ${CYAN}source /opt/ros/humble/setup.bash${NC}"
echo ""
echo -e "${CYAN}${BOLD}3. Navigate to lbx_robotics workspace:${NC}"
echo -e "   ${CYAN}cd $SCRIPT_DIR${NC}"
echo ""
echo -e "${CYAN}${BOLD}4. Build the unified workspace:${NC}"
echo -e "   ${CYAN}colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release${NC}"
echo -e "   ${CYAN}# OR use the unified launch script:${NC}"
echo -e "   ${CYAN}./unified_launch.sh --clean-build${NC}"
echo ""
echo -e "${CYAN}${BOLD}5. Source the built workspace:${NC}"
echo -e "   ${CYAN}source install/setup.bash${NC}"
echo ""
echo_info "The unified workspace now includes:"
echo_info "  • Your custom lbx_robotics packages"
echo_info "  • Official franka_ros2 packages integrated"
echo_info "  • Conda environment with Python packages"
echo_info "  • ROS 2 Humble with development tools"
echo_info "  • Intel RealSense camera support"
echo_info "  • Oculus VR input capabilities"
echo_info "  • MCAP data recording"
echo_info "  • All system dependencies (Poco, Eigen3, etc.)"
echo_info "  • ROS2 control packages for robot control"
echo_info "  • Gazebo simulation support"
echo ""
echo -e "${GREEN}${BOLD}Test your setup:${NC}"
echo -e "  • Test franka_ros2: ${CYAN}ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true${NC}"
echo -e "  • List all packages: ${CYAN}ros2 pkg list | grep -E '(franka|lbx)'${NC}"
echo ""
echo_info "All packages are now in a single unified workspace at: $SCRIPT_DIR"
echo ""
echo_warn "Note: If you encounter CMake errors finding Poco libraries during build:"
echo_warn "  The unified_launch.sh script will automatically set up paths"
echo_warn "  OR manually export: CMAKE_PREFIX_PATH=\"/usr/lib/x86_64-linux-gnu/cmake:\$CMAKE_PREFIX_PATH\""
echo_warn "  OR use system cmake: conda deactivate && conda activate lbx_robotics_env --no-stack"
echo ""
echo_warn "Note: The build will skip libfranka if it has complex dependencies."
echo_warn "  This is normal - libfranka is only needed if building from source."
echo_warn "  Most functionality works with the pre-installed ROS2 packages." 