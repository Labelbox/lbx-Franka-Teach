#!/bin/bash
# Setup script for lbx_robotics on a System-Level ROS 2 Humble Environment

# --- Configuration ---
PIP_REQ_FILE="requirements.txt"
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
echo_error() { echo -e "${RED}${BOLD}ERROR:${NC}${RED} $1${NC}" >&2; }
echo_success() { echo -e "${GREEN}${BOLD}✅ SUCCESS:${NC}${GREEN} $1${NC}"; }
echo_info() { echo -e "${BLUE}INFO:${NC} $1"; }
echo_warn() { echo -e "${YELLOW}${BOLD}WARNING:${NC}${YELLOW} $1${NC}"; }
echo_step() { echo -e "${CYAN}${BOLD}STEP:${NC}${CYAN} $1${NC}"; }

# --- Main Setup --- 
echo_info "Starting system-level setup for lbx_robotics with ROS 2 Humble..."

# 0. Ensure no Conda environment is active
echo_step "Ensuring no Conda environment is active..."
if [ ! -z "$CONDA_DEFAULT_ENV" ] || [ ! -z "$CONDA_PREFIX" ]; then
    echo_warn "A Conda environment appears to be active ($CONDA_DEFAULT_ENV)."
    echo_warn "This script will proceed with system-level installations."
    echo_warn "It is STRONGLY recommended to run 'conda deactivate' until no environment is active before proceeding."
    read -p "Do you want to continue? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo_error "Setup aborted by user due to active Conda environment."
    exit 1
fi
else
    echo_success "No Conda environment active. Proceeding with system setup."
fi

# 1. Install System Build Tools & Essential Libraries
echo_step "Installing system build tools and essential C++ libraries..."
sudo apt update

# Ensure a recent CMake (>=3.22) is installed and used
echo_info "Checking and updating CMake version if necessary..."
# Remove any existing cmake to ensure a clean slate
sudo apt-get remove --purge -y cmake cmake-data > /dev/null 2>&1 || echo_info "No old CMake to purge or purge failed (continuing)."
# Add Kitware PPA for newer CMake versions
sudo apt install -y software-properties-common lsb-release wget apt-transport-https ca-certificates gpg
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null
if ! grep -q "kitware.list" /etc/apt/sources.list.d/*kitware.list 2>/dev/null ; then
    echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null
fi
sudo apt update
# Install a specific recent version or let apt choose the latest from Kitware
sudo apt install --reinstall --no-install-recommends -y cmake

# Verify CMake version
CMAKE_EXE=$(which cmake)
if [ -z "$CMAKE_EXE" ]; then CMAKE_EXE="/usr/bin/cmake"; fi # Fallback if not in PATH immediately

if $CMAKE_EXE --version | head -n1 | awk '{print $3}' | awk -F. '{exit !($1 > 3 || ($1 == 3 && $2 >= 22))}'; then
    echo_success "CMake version $($CMAKE_EXE --version | head -n1) is installed and sufficient."
    else
    echo_error "Failed to install CMake >= 3.22. Current version: $($CMAKE_EXE --version | head -n1 || echo 'Not Found'). Pinocchio build may fail. Please install CMake 3.22+ manually."
    # exit 1 # Optionally exit
fi

REQUIRED_PKGS=(
    build-essential git pkg-config libboost-all-dev 
    libpcre3 libpcre3-dev libpoco-dev libeigen3-dev libssl-dev libcurl4-openssl-dev 
    libusb-1.0-0-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
)
FAILED_PKGS=()
for pkg in "${REQUIRED_PKGS[@]}"; do
    echo_info "Installing $pkg (if not already covered by build-essential/cmake)..."
    if ! dpkg -l "$pkg" > /dev/null 2>&1 || ! sudo apt install -y "$pkg"; then
        # Check if already installed (especially for build-essential components)
        if ! dpkg -l "$pkg" > /dev/null 2>&1; then 
            echo_warn "Failed to install $pkg via apt."
            FAILED_PKGS+=("$pkg")
        else
            echo_info "$pkg is already installed."
        fi
    fi
done
if [ ${#FAILED_PKGS[@]} -ne 0 ]; then
    echo_error "Failed to install the following essential system packages: ${FAILED_PKGS[*]}. Aborting."
    exit 1
fi
echo_success "Essential system libraries and build tools checked/installed."

# 2. Install Pinocchio (from source, as it's often problematic with apt versions)
echo_step "Installing Pinocchio from source (v2.6.20)..."
if ! dpkg -l | grep -q libpinocchio-dev || ! pkg-config --exists pinocchio 2>/dev/null; then 
    TEMP_DIR_PINOCCHIO=$(mktemp -d)
    echo_info "Building Pinocchio in $TEMP_DIR_PINOCCHIO"
    (
        cd "$TEMP_DIR_PINOCCHIO"
        sudo apt install -y liburdfdom-dev libconsole-bridge-dev libassimp-dev liboctomap-dev # Pinocchio deps
        
        echo_info "Using CMake for Pinocchio: $($CMAKE_EXE --version | head -n1)"

        git clone --recursive https://github.com/stack-of-tasks/pinocchio.git
        cd pinocchio
        git checkout v2.6.20
        mkdir build && cd build
        # Explicitly use the cmake in PATH and set a policy version
        $CMAKE_EXE .. -DCMAKE_BUILD_TYPE=Release \
                      -DCMAKE_INSTALL_PREFIX=/usr/local \
                      -DBUILD_PYTHON_INTERFACE=OFF \
                      -DBUILD_UNIT_TESTS=OFF \
                      -DBUILD_WITH_COLLISION_SUPPORT=ON \
                      -DCMAKE_CXX_STANDARD=14 \
                      -DCMAKE_POLICY_VERSION=3.5 # Attempt to set policies for CMake 3.5 compatibility
        make -j$(nproc)
        sudo make install
        sudo ldconfig
    )
    rm -rf "$TEMP_DIR_PINOCCHIO"
    echo_success "Pinocchio installed from source to /usr/local."
else
    echo_success "Pinocchio already found or installed."
fi

# 3. Install ROS 2 Humble
echo_step "Installing/Verifying ROS 2 Humble Desktop..."
if ! dpkg -l | grep -q "ros-humble-desktop"; then
    sudo apt install -y ros-humble-desktop ros-dev-tools
fi
source "/opt/ros/humble/setup.bash"
echo_success "ROS 2 Humble sourced."

# 4. Install Additional ROS 2 Tools & Control Packages
echo_step "Installing additional ROS 2 tools and control packages..."
sudo apt install -y python3-vcstool python3-rosdep python3-colcon-common-extensions \
    ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gripper-controllers \
    ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller ros-humble-xacro \
    ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs ros-humble-gazebo-msgs ros-humble-gazebo-plugins
echo_success "Additional ROS 2 tools and control packages installed."

# 5. Install Pre-built Franka ROS 2 Packages (if available, to save build time)
echo_step "Attempting to install pre-built Franka ROS 2 packages..."
FRANKA_APT_PACKAGES=(
    ros-humble-libfranka 
    ros-humble-franka-hardware 
    ros-humble-franka-msgs 
    ros-humble-franka-gripper 
    ros-humble-franka-description 
    ros-humble-franka-fr3-moveit-config
)
ALL_FRANKA_APT_INSTALLED=true
for pkg in "${FRANKA_APT_PACKAGES[@]}"; do
    if ! sudo apt install -y "$pkg"; then 
        echo_warn "Could not install $pkg from apt. Will need to build from source."
        ALL_FRANKA_APT_INSTALLED=false
    fi
done
if $ALL_FRANKA_APT_INSTALLED; then
    echo_success "All recommended Franka ROS 2 packages installed via apt."
else
    echo_info "Some Franka packages will be built from source in the workspace."
fi

# 6. Install Python Dependencies using pip
echo_step "Installing Python dependencies from $PIP_REQ_FILE..."
if [ -f "$SCRIPT_DIR/$PIP_REQ_FILE" ]; then
    sudo apt install -y python3-pip # Ensure pip is available for system Python
    echo_info "Ensuring Python pip, setuptools, and wheel are up to date..."
    python3 -m pip install --upgrade pip setuptools wheel
    
    echo_info "Installing packages from $PIP_REQ_FILE..."
    # Use --user flag if system site-packages is not writable, or run with sudo if appropriate
    # For system-wide ROS setup, installing to system Python is often intended.
    # However, if non-root, pip might default to --user.
    if python3 -m pip install -r "$SCRIPT_DIR/$PIP_REQ_FILE"; then
        echo_success "Python dependencies installed successfully."
        if [[ ":$PATH:" != *":$HOME/.local/bin:"* ]]; then
            echo_warn "~/.local/bin is not on your PATH. You might want to add it by running:"
            echo_warn "  echo 'export PATH=\"$HOME/.local/bin:\$PATH\"' >> ~/.bashrc && source ~/.bashrc"
        fi
    else
        echo_error "Failed to install Python dependencies from $PIP_REQ_FILE. Check errors above."
        exit 1
    fi
else
    echo_warn "Python requirements file '$PIP_REQ_FILE' not found. Skipping."
fi

# 7. Initialize/Update rosdep
echo_step "Initializing/Updating rosdep..."
if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then sudo rosdep init; fi
rosdep update

# 8. Integrate franka_ros2 into lbx_robotics workspace (if not fully installed by apt)
echo_step "Integrating franka_ros2 packages into lbx_robotics workspace (if needed)..."
cd "$SCRIPT_DIR"
if [ ! -d "$SCRIPT_DIR/src" ]; then mkdir -p "$SCRIPT_DIR/src"; fi

if ! $ALL_FRANKA_APT_INSTALLED || [ ! -d "$SCRIPT_DIR/src/franka_ros2" ]; then
if [ ! -d "$SCRIPT_DIR/src/franka_ros2" ]; then
    echo_info "Cloning franka_ros2 repository into lbx_robotics/src/..."
    cd "$SCRIPT_DIR/src"
        git clone https://github.com/frankaemika/franka_ros2.git
    cd "$SCRIPT_DIR"
    fi
    echo_info "Importing franka_ros2 dependencies into workspace..."
    vcs import src < src/franka_ros2/franka.repos --recursive --skip-existing
echo_success "franka_ros2 source integration complete (if needed)."
else
    echo_info "franka_ros2 seems to be fully installed via apt or already present. Skipping source integration."
fi

# 9. Install workspace dependencies with rosdep
echo_step "Installing workspace dependencies with rosdep..."
cd "$SCRIPT_DIR"
rosdep install --from-paths src --ignore-src --rosdistro humble -y -r
echo_success "Workspace dependencies resolved."

# --- Final Instructions ---
echo ""
echo_success "--------------------------------------------------------------"
echo_success " System-level environment setup for lbx_robotics complete!  "
echo_success "--------------------------------------------------------------"
echo ""
echo_info "To build and run the system:"
echo -e "  1. ${CYAN}Ensure no Conda environment is active.${NC}"
echo -e "  2. ${CYAN}Open a new terminal or re-source your .bashrc (if ROS setup was added).${NC}"
echo -e "  3. ${CYAN}Source ROS 2 Humble: source /opt/ros/humble/setup.bash${NC}"
echo -e "  4. ${CYAN}Navigate to workspace: cd $SCRIPT_DIR${NC}"
echo -e "  5. ${CYAN}Build: ./unified_launch.sh --clean-build${NC} (this will use system libraries)"
echo -e "  6. ${CYAN}Source built workspace: source install/setup.bash${NC}" 