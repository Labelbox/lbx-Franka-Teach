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

# Function to fix CMake version issues in dependencies
fix_cmake_versions() {
    echo_info "Fixing CMake version requirements in dependencies..."
    
    # Find all CMakeLists.txt and .in template files in src directory
    find "$SCRIPT_DIR/src" \( -name "CMakeLists.txt" -o -name "*.cmake" -o -name "*.cmake.in" -o -name "CMakeLists.txt.in" \) -type f 2>/dev/null | while read -r cmake_file; do
        # Check if file contains old cmake_minimum_required
        if grep -E "cmake_minimum_required.*VERSION.*[0-2]\.|cmake_minimum_required.*VERSION.*3\.[0-4]" "$cmake_file" > /dev/null 2>&1; then
            
            # Get the relative path for display
            rel_path="${cmake_file#$SCRIPT_DIR/}"
            
            # Create backup
            cp "$cmake_file" "${cmake_file}.bak"
            
            # Update cmake_minimum_required to version 3.11
            # Use perl for better cross-platform compatibility
            if command -v perl &> /dev/null; then
                perl -i -pe 's/cmake_minimum_required\s*\(\s*VERSION\s+[0-9]+\.[0-9]+(?:\.[0-9]+)?\s*\)/cmake_minimum_required(VERSION 3.11)/gi' "$cmake_file"
            elif [[ "$OSTYPE" == "darwin"* ]]; then
                # macOS sed with backup
                sed -i '' -E 's/cmake_minimum_required[[:space:]]*\([[:space:]]*VERSION[[:space:]]+[0-9]+\.[0-9]+(\.[0-9]+)?[[:space:]]*\)/cmake_minimum_required(VERSION 3.11)/g' "$cmake_file"
            else
                # Linux sed
                sed -i -E 's/cmake_minimum_required[[:space:]]*\([[:space:]]*VERSION[[:space:]]+[0-9]+\.[0-9]+(\.[0-9]+)?[[:space:]]*\)/cmake_minimum_required(VERSION 3.11)/g' "$cmake_file"
            fi
            
            # Check if changes were made
            if ! diff -q "$cmake_file" "${cmake_file}.bak" > /dev/null 2>&1; then
                echo_success "  ✓ Updated CMake version in $rel_path"
            fi
            
            # Clean up backup
            rm -f "${cmake_file}.bak"
        fi
    done
    
    # Also check for any nested CMakeLists.txt files in subdirectories like 'common'
    # This is specifically for libfranka which has issues with submodules
    if [ -d "$SCRIPT_DIR/src/libfranka" ]; then
        echo_info "Checking libfranka submodules..."
        
        # Ensure submodules are initialized
        (cd "$SCRIPT_DIR/src/libfranka" && git submodule update --init --recursive 2>/dev/null || true)
        
        # Fix common/CMakeLists.txt specifically
        if [ -f "$SCRIPT_DIR/src/libfranka/common/CMakeLists.txt" ]; then
            cmake_file="$SCRIPT_DIR/src/libfranka/common/CMakeLists.txt"
            if grep -E "cmake_minimum_required.*VERSION.*3\.[0-4]" "$cmake_file" > /dev/null 2>&1; then
                echo_info "  Fixing libfranka/common/CMakeLists.txt..."
                if command -v perl &> /dev/null; then
                    perl -i -pe 's/cmake_minimum_required\s*\(\s*VERSION\s+[0-9]+\.[0-9]+(?:\.[0-9]+)?\s*\)/cmake_minimum_required(VERSION 3.11)/gi' "$cmake_file"
                elif [[ "$OSTYPE" == "darwin"* ]]; then
                    sed -i '' -E 's/cmake_minimum_required[[:space:]]*\([[:space:]]*VERSION[[:space:]]+[0-9]+\.[0-9]+(\.[0-9]+)?[[:space:]]*\)/cmake_minimum_required(VERSION 3.11)/g' "$cmake_file"
                else
                    sed -i -E 's/cmake_minimum_required[[:space:]]*\([[:space:]]*VERSION[[:space:]]+[0-9]+\.[0-9]+(\.[0-9]+)?[[:space:]]*\)/cmake_minimum_required(VERSION 3.11)/g' "$cmake_file"
                fi
                echo_success "  ✓ Fixed libfranka/common/CMakeLists.txt"
            fi
        fi
        
        # Also fix any other problematic files in build directory if they were already generated
        if [ -d "$SCRIPT_DIR/build" ]; then
            echo_info "  Checking for generated CMake files in build directory..."
            find "$SCRIPT_DIR/build" -name "CMakeLists.txt" -type f 2>/dev/null | while read -r cmake_file; do
                if grep -E "cmake_minimum_required.*VERSION.*[0-2]\.|cmake_minimum_required.*VERSION.*3\.[0-4]" "$cmake_file" > /dev/null 2>&1; then
                    if command -v perl &> /dev/null; then
                        perl -i -pe 's/cmake_minimum_required\s*\(\s*VERSION\s+[0-9]+\.[0-9]+(?:\.[0-9]+)?\s*\)/cmake_minimum_required(VERSION 3.11)/gi' "$cmake_file"
                    elif [[ "$OSTYPE" == "darwin"* ]]; then
                        sed -i '' -E 's/cmake_minimum_required[[:space:]]*\([[:space:]]*VERSION[[:space:]]+[0-9]+\.[0-9]+(\.[0-9]+)?[[:space:]]*\)/cmake_minimum_required(VERSION 3.11)/g' "$cmake_file"
                    else
                        sed -i -E 's/cmake_minimum_required[[:space:]]*\([[:space:]]*VERSION[[:space:]]+[0-9]+\.[0-9]+(\.[0-9]+)?[[:space:]]*\)/cmake_minimum_required(VERSION 3.11)/g' "$cmake_file"
                    fi
                    rel_path="${cmake_file#$SCRIPT_DIR/}"
                    echo_success "  ✓ Fixed generated file: $rel_path"
                fi
            done
        fi
    fi
    
    echo_success "CMake version fixes completed"
}

# --- Main Setup --- 
echo_info "Starting system-level setup for lbx_robotics with ROS 2 Humble..."

# Detect operating system
if [[ "$OSTYPE" == "darwin"* ]]; then
    echo_warn "Detected macOS - This is a development environment only"
    echo_info "ROS 2 is not natively supported on macOS"
    echo_info "This script will set up the workspace for deployment on Ubuntu"
    IS_MACOS=true
else
    IS_MACOS=false
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

# 3. Install ROS 2 Humble (skip on macOS)
if [ "$IS_MACOS" = false ]; then
    echo_step "Installing/Verifying ROS 2 Humble Desktop..."
    if ! dpkg -l | grep -q "ros-humble-desktop"; then
        sudo apt install -y ros-humble-desktop ros-dev-tools
    fi
    source "/opt/ros/humble/setup.bash"
    echo_success "ROS 2 Humble sourced."
else
    echo_warn "Skipping ROS 2 installation on macOS"
fi

# 4. Install Additional ROS 2 Tools & Control Packages - Updated to ensure latest colcon
echo_step "Installing additional ROS 2 tools and control packages..."

# First, remove any existing colcon packages that might be outdated
echo_info "Removing potentially outdated colcon packages..."
sudo apt remove -y python3-colcon-* 2>/dev/null || true

# Install ROS 2 packages without colcon
sudo apt install -y python3-vcstool python3-rosdep \
    ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gripper-controllers \
    ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller ros-humble-xacro \
    ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs ros-humble-gazebo-msgs ros-humble-gazebo-plugins

# Install latest colcon through pip to ensure we get the version that fixes the --editable issue
echo_info "Installing latest colcon packages through pip..."
sudo apt install -y python3-pip # Ensure pip is available
python3 -m pip install --upgrade pip setuptools wheel

# Install colcon packages from pip (will be installed from requirements.txt)
echo_success "Additional ROS 2 tools and control packages installed (colcon will be installed via pip)."

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

# Ensure pip is available
sudo apt install -y python3-pip python3-venv # Also install venv for virtual environments if needed

# Upgrade pip, setuptools, and wheel first
echo_info "Ensuring Python pip, setuptools, and wheel are up to date..."
python3 -m pip install --user --upgrade pip setuptools wheel

# First, explicitly install colcon to ensure it's available
echo_info "Installing colcon build tools..."
python3 -m pip install --user --upgrade colcon-common-extensions colcon-core>=0.15.0 colcon-ros>=0.4.0

# Update PATH for current session and permanently
echo_step "Updating PATH configuration..."

# Always add ~/.local/bin to PATH for this session
export PATH="$HOME/.local/bin:$PATH"
echo_info "Added ~/.local/bin to PATH for current session"

# Check if .bashrc exists, create if not
if [ ! -f "$HOME/.bashrc" ]; then
    echo_info "Creating ~/.bashrc file..."
    touch "$HOME/.bashrc"
fi

# Add to .bashrc if not already there
if ! grep -q 'export PATH="$HOME/.local/bin:$PATH"' "$HOME/.bashrc"; then
    echo_info "Adding ~/.local/bin to PATH in ~/.bashrc..."
    echo '' >> "$HOME/.bashrc"
    echo '# Added by lbx_robotics setup script for pip installed executables' >> "$HOME/.bashrc"
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$HOME/.bashrc"
    echo_success "PATH permanently updated in ~/.bashrc"
else
    echo_info "PATH already configured in ~/.bashrc"
fi

# Also update .profile for login shells (some Ubuntu setups use this)
if [ -f "$HOME/.profile" ]; then
    if ! grep -q 'PATH="$HOME/.local/bin:$PATH"' "$HOME/.profile"; then
        echo_info "Also updating ~/.profile for login shells..."
        echo '' >> "$HOME/.profile"
        echo '# Added by lbx_robotics setup script' >> "$HOME/.profile"
        echo 'if [ -d "$HOME/.local/bin" ] ; then' >> "$HOME/.profile"
        echo '    PATH="$HOME/.local/bin:$PATH"' >> "$HOME/.profile"
        echo 'fi' >> "$HOME/.profile"
    fi
fi

# Source .bashrc to ensure changes take effect
if [ -f "$HOME/.bashrc" ]; then
    source "$HOME/.bashrc"
    echo_info "Sourced ~/.bashrc to apply changes"
fi

# Verify colcon is now available
if command -v colcon &> /dev/null; then
    echo_success "✓ colcon is installed: $(which colcon)"
    COLCON_VERSION=$(colcon version-check 2>&1 | grep 'colcon-core' | head -1 | awk '{print $2}')
    echo_info "  Version: $COLCON_VERSION"
else
    echo_error "✗ colcon not found in PATH"
    echo_info "  Current PATH: $PATH"
    if [ -f "$HOME/.local/bin/colcon" ]; then
        echo_warn "  colcon exists at $HOME/.local/bin/colcon but is not in PATH"
        echo_info "  Try running: export PATH=\"\$HOME/.local/bin:\$PATH\""
    else
        echo_error "  colcon executable not found in $HOME/.local/bin/"
    fi
    VERIFICATION_FAILED=true
fi

# Now install remaining Python dependencies
if [ -f "$SCRIPT_DIR/$PIP_REQ_FILE" ]; then
    echo_info "Installing remaining packages from $PIP_REQ_FILE..."
    if python3 -m pip install --user -r "$SCRIPT_DIR/$PIP_REQ_FILE"; then
        echo_success "Python dependencies installed successfully."
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
    if command -v vcs &> /dev/null; then
        vcs import src < src/franka_ros2/franka.repos --recursive --skip-existing
    else
        echo_warn "vcs not found, manually cloning dependencies..."
        # Manual clone of dependencies from franka.repos
        cd "$SCRIPT_DIR/src"
        
        # Clone franka_description if not exists
        if [ ! -d "franka_description" ]; then
            git clone https://github.com/frankaemika/franka_description.git
            cd franka_description && git checkout 0.4.0 && cd ..
        fi
        
        # Clone libfranka if not exists
        if [ ! -d "libfranka" ]; then
            git clone https://github.com/frankaemika/libfranka.git
            cd libfranka && git checkout 0.15.0
            # Initialize submodules for libfranka
            git submodule update --init --recursive
            cd ..
        fi
        
        cd "$SCRIPT_DIR"
    fi
    
    # Fix CMake versions after cloning
    fix_cmake_versions
    
    echo_success "franka_ros2 source integration complete (if needed)."
else
    echo_info "franka_ros2 seems to be fully installed via apt or already present. Skipping source integration."
fi

# 9. Install workspace dependencies with rosdep (skip on macOS)
if [ "$IS_MACOS" = false ]; then
    echo_step "Installing workspace dependencies with rosdep..."
    cd "$SCRIPT_DIR"
    rosdep install --from-paths src --ignore-src --rosdistro humble -y -r
    echo_success "Workspace dependencies resolved."
else
    echo_warn "Skipping rosdep on macOS - dependencies must be resolved on Ubuntu"
fi

# 10. Final verification
echo_step "Verifying installation..."
VERIFICATION_FAILED=false

# Check ROS 2 (skip on macOS)
if [ "$IS_MACOS" = false ]; then
    if command -v ros2 &> /dev/null; then
        echo_success "✓ ROS 2 is installed"
    else
        echo_error "✗ ROS 2 not found"
        VERIFICATION_FAILED=true
    fi
else
    echo_info "✓ ROS 2 check skipped on macOS"
fi

# Check colcon
if command -v colcon &> /dev/null; then
    echo_success "✓ colcon is installed: $(which colcon)"
    COLCON_VERSION=$(colcon version-check 2>&1 | grep 'colcon-core' | head -1 | awk '{print $2}')
    echo_info "  Version: $COLCON_VERSION"
else
    echo_error "✗ colcon not found in PATH"
    echo_info "  Current PATH: $PATH"
    if [ -f "$HOME/.local/bin/colcon" ]; then
        echo_warn "  colcon exists at $HOME/.local/bin/colcon but is not in PATH"
        echo_info "  Try running: export PATH=\"\$HOME/.local/bin:\$PATH\""
    else
        echo_error "  colcon executable not found in $HOME/.local/bin/"
    fi
    VERIFICATION_FAILED=true
fi

# Check CMake version
if command -v cmake &> /dev/null; then
    CMAKE_VERSION=$(cmake --version | head -n1 | awk '{print $3}')
    echo_success "✓ CMake $CMAKE_VERSION is installed"
else
    echo_error "✗ CMake not found"
    VERIFICATION_FAILED=true
fi

# Check if key Python packages are installed
echo_info "Checking Python packages..."
for pkg in "colcon-core" "colcon-ros" "empy" "numpy"; do
    if python3 -m pip show $pkg &> /dev/null; then
        echo_success "  ✓ $pkg is installed"
    else
        echo_warn "  ✗ $pkg not found"
    fi
done

if [ "$VERIFICATION_FAILED" = true ]; then
    echo_error "Some components failed verification. Please check the errors above."
    echo_info "You may need to:"
    echo_info "  1. Open a new terminal to reload PATH"
    echo_info "  2. Run: source ~/.bashrc"
    echo_info "  3. Re-run this setup script"
fi

# --- Final Instructions ---
echo ""
echo_success "--------------------------------------------------------------"
echo_success " System-level environment setup for lbx_robotics complete!  "
echo_success "--------------------------------------------------------------"
echo ""
echo_info "To build and run the system:"
echo -e "  1. ${CYAN}Open a new terminal or run: source ~/.bashrc${NC}"
echo -e "  2. ${CYAN}Source ROS 2 Humble: source /opt/ros/humble/setup.bash${NC}"
echo -e "  3. ${CYAN}Navigate to workspace: cd $SCRIPT_DIR${NC}"
echo -e "  4. ${CYAN}Build: ./unified_launch.sh --clean-build${NC}"
echo -e "  5. ${CYAN}Source built workspace: source install/setup.bash${NC}"
echo ""
echo_info "If colcon is still not found, ensure PATH is set:"
echo -e "  ${CYAN}export PATH=\"\$HOME/.local/bin:\$PATH\"${NC}" 