#!/bin/bash
# Setup script for the lbx_robotics Conda environment and franka_ros2 integration

# --- Configuration ---
ENV_NAME="lbx_robotics_env"
ENV_FILE="environment.yaml"
REQ_FILE="requirements.txt"
FRANKA_WS_DIR="$HOME/franka_ros2_ws"
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
echo_info "Starting environment setup for '$ENV_NAME' with franka_ros2 integration..."
echo_info "This script will set up both conda environment and ROS 2 franka_ros2 workspace."
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
    # Optionally, add logic here to ask if the user wants to update or remove & recreate.
    # For now, we'll proceed to ensure pip packages are installed/updated.
else
    echo_info "Conda environment '$ENV_NAME' not found. Creating it from $SCRIPT_DIR/$ENV_FILE..."
    if conda env create -f "$SCRIPT_DIR/$ENV_FILE" -n "$ENV_NAME"; then
        echo_success "Conda environment '$ENV_NAME' created successfully."
    else
        echo_error "Failed to create Conda environment '$ENV_NAME' from $SCRIPT_DIR/$ENV_FILE."
        exit 1
    fi
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

# Initialize rosdep if not already done
if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    echo_info "Initializing rosdep..."
    sudo rosdep init
fi

echo_info "Updating rosdep..."
rosdep update

# --- franka_ros2 Workspace Setup ---
echo_step "Setting up franka_ros2 workspace..."

# Create workspace directory
if [ ! -d "$FRANKA_WS_DIR" ]; then
    echo_info "Creating franka_ros2 workspace at $FRANKA_WS_DIR..."
    mkdir -p "$FRANKA_WS_DIR/src"
else
    echo_info "franka_ros2 workspace already exists at $FRANKA_WS_DIR."
fi

cd "$FRANKA_WS_DIR"

# Clone franka_ros2 repository if not already present
if [ ! -d "$FRANKA_WS_DIR/src/franka_ros2" ] && [ ! -d "$FRANKA_WS_DIR/src/.git" ]; then
    echo_info "Cloning franka_ros2 repository..."
    if git clone https://github.com/frankaemika/franka_ros2.git src; then
        echo_success "franka_ros2 repository cloned successfully."
    else
        echo_error "Failed to clone franka_ros2 repository."
        exit 1
    fi
else
    echo_info "franka_ros2 repository already present in workspace."
fi

# Import additional dependencies
echo_info "Importing franka_ros2 dependencies..."
if [ -f "$FRANKA_WS_DIR/src/franka.repos" ]; then
    if vcs import src < src/franka.repos --recursive --skip-existing; then
        echo_success "franka_ros2 dependencies imported successfully."
    else
        echo_warn "Some dependencies may not have imported correctly. Continuing anyway."
    fi
else
    echo_warn "franka.repos file not found. Some dependencies may be missing."
fi

# Install dependencies with rosdep
echo_info "Installing workspace dependencies with rosdep..."
if rosdep install --from-paths src --ignore-src --rosdistro humble -y; then
    echo_success "Workspace dependencies installed successfully."
else
    echo_warn "Some dependencies may not have installed correctly. Continuing anyway."
fi

# Build the workspace
echo_step "Building franka_ros2 workspace..."
if colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release; then
    echo_success "franka_ros2 workspace built successfully."
else
    echo_error "Failed to build franka_ros2 workspace."
    echo_info "You may need to resolve build errors manually."
    exit 1
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
echo -e "${CYAN}${BOLD}3. Source franka_ros2 workspace:${NC}"
echo -e "   ${CYAN}source $FRANKA_WS_DIR/install/setup.bash${NC}"
echo ""
echo -e "${CYAN}${BOLD}4. Navigate to your lbx_robotics workspace:${NC}"
echo -e "   ${CYAN}cd $SCRIPT_DIR${NC}"
echo ""
echo_info "The complete environment includes:"
echo_info "  • Conda environment with Python packages"
echo_info "  • ROS 2 Humble with development tools"
echo_info "  • franka_ros2 packages for Franka robot control"
echo_info "  • Intel RealSense camera support"
echo_info "  • Oculus VR input capabilities"
echo_info "  • MCAP data recording"
echo ""
echo -e "${GREEN}${BOLD}Test your setup:${NC}"
echo -e "  • Test franka_ros2: ${CYAN}ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true${NC}"
echo -e "  • Build lbx_robotics: ${CYAN}colcon build${NC} (from lbx_robotics directory)"
echo ""
echo_info "franka_ros2 workspace location: $FRANKA_WS_DIR" 