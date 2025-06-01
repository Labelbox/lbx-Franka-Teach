#!/bin/bash
# Script to fix CMake by removing any existing version and installing stable 3.22.1

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}[INFO]${NC} Fixing CMake installation..."

# Remove any existing CMake installation
echo -e "${YELLOW}[STEP]${NC} Removing any existing CMake installation..."

# Remove Kitware repository if it exists
sudo rm -f /etc/apt/sources.list.d/kitware.list
sudo rm -f /usr/share/keyrings/kitware-archive-keyring.gpg

# Remove all CMake packages
sudo apt-get remove --purge -y cmake cmake-data cmake-qt-gui cmake-curses-gui 2>/dev/null || true
sudo apt-get autoremove -y

# Remove CMake from /usr/local if it exists
sudo rm -f /usr/local/bin/cmake /usr/local/bin/ctest /usr/local/bin/cpack
sudo rm -rf /usr/local/share/cmake* /usr/local/doc/cmake*

# Update package list
echo -e "${YELLOW}[STEP]${NC} Updating package list..."
sudo apt update

# Build and install CMake 3.22.1 from source
echo -e "${YELLOW}[STEP]${NC} Building CMake 3.22.1 from source..."
TEMP_DIR=$(mktemp -d)
cd "$TEMP_DIR"

# Install build dependencies
sudo apt install -y build-essential libssl-dev

# Download and extract
wget https://github.com/Kitware/CMake/releases/download/v3.22.1/cmake-3.22.1.tar.gz
tar -xzf cmake-3.22.1.tar.gz
cd cmake-3.22.1

# Build and install
./bootstrap --prefix=/usr --parallel=$(nproc)
make -j$(nproc)
sudo make install

# Clean up
cd /
rm -rf "$TEMP_DIR"

# Update library cache
sudo ldconfig

# Verify installation
echo -e "${YELLOW}[STEP]${NC} Verifying CMake installation..."
CMAKE_VERSION=$(cmake --version | head -n1)
echo -e "${GREEN}[SUCCESS]${NC} Installed: $CMAKE_VERSION"

echo -e "${GREEN}[DONE]${NC} CMake has been fixed!"
echo -e "${BLUE}[INFO]${NC} You can now run: ./unified_launch.sh --clean-build" 