#!/bin/bash
# Script to fix CMake by removing experimental version and installing stable 3.27

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}[INFO]${NC} Fixing CMake installation..."

# Remove Kitware repository
echo -e "${YELLOW}[STEP]${NC} Removing Kitware repository..."
sudo rm -f /etc/apt/sources.list.d/kitware.list
sudo rm -f /usr/share/keyrings/kitware-archive-keyring.gpg

# Remove existing CMake
echo -e "${YELLOW}[STEP]${NC} Removing existing CMake installation..."
sudo apt-get remove --purge -y cmake cmake-data cmake-qt-gui cmake-curses-gui 2>/dev/null || true
sudo apt-get autoremove -y

# Update package list
echo -e "${YELLOW}[STEP]${NC} Updating package list..."
sudo apt update

# Try to install CMake from Ubuntu repository first
echo -e "${YELLOW}[STEP]${NC} Checking Ubuntu repository for CMake..."
if apt-cache show cmake | grep -q "Version: 3.2[2-9]\|Version: 3.[3-9]"; then
    echo -e "${GREEN}[SUCCESS]${NC} Found suitable CMake in Ubuntu repository"
    sudo apt install -y cmake
else
    # Build CMake 3.27 from source
    echo -e "${YELLOW}[STEP]${NC} Building CMake 3.27 from source..."
    TEMP_DIR=$(mktemp -d)
    cd "$TEMP_DIR"
    
    # Download and extract
    wget https://github.com/Kitware/CMake/releases/download/v3.27.9/cmake-3.27.9.tar.gz
    tar -xzf cmake-3.27.9.tar.gz
    cd cmake-3.27.9
    
    # Build and install
    ./bootstrap --prefix=/usr/local
    make -j$(nproc)
    sudo make install
    
    # Set up alternatives
    sudo update-alternatives --install /usr/bin/cmake cmake /usr/local/bin/cmake 100
    sudo update-alternatives --install /usr/bin/ctest ctest /usr/local/bin/ctest 100
    sudo update-alternatives --install /usr/bin/cpack cpack /usr/local/bin/cpack 100
    
    # Clean up
    cd /
    rm -rf "$TEMP_DIR"
fi

# Verify installation
echo -e "${YELLOW}[STEP]${NC} Verifying CMake installation..."
CMAKE_VERSION=$(cmake --version | head -n1)
echo -e "${GREEN}[SUCCESS]${NC} Installed: $CMAKE_VERSION"

# Update library cache
sudo ldconfig

echo -e "${GREEN}[DONE]${NC} CMake has been fixed!"
echo -e "${BLUE}[INFO]${NC} You can now run: ./unified_launch.sh --clean-build" 