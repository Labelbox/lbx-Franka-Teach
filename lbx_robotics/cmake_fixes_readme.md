# CMake Version Fixes for LBX Robotics

## Overview

The Franka ROS2 packages (particularly libfranka) contain some CMakeLists.txt files with outdated CMake version requirements that are incompatible with modern CMake (4.0+). This causes build failures on Ubuntu 22.04 with newer CMake versions.

## Quick Fix on Ubuntu

If you're getting CMake version errors on your Ubuntu machine, run:

```bash
cd ~/projects/lbx-Franka-Teach/lbx_robotics

# Make the fix script executable
chmod +x fix_cmake_versions.sh

# Run the fix script
./fix_cmake_versions.sh

# Then build with clean build flag
./unified_launch.sh --clean-build
```

## Development Workflow (macOS → Ubuntu)

### On macOS (Development)

1. The `unified_launch.sh` and `setup_environment.sh` scripts now automatically fix CMake versions
2. When you run `./unified_launch.sh --build` on macOS, it will:
   - Detect you're on macOS and skip ROS2 checks
   - Clone necessary dependencies if missing
   - Fix all CMake version issues automatically
   - Prepare the workspace for Ubuntu deployment

### On Ubuntu (Testing/Deployment)

1. Pull the latest changes from your repository
2. If you get CMake errors, run: `./fix_cmake_versions.sh`
3. Build with: `./unified_launch.sh --clean-build`

## What Gets Fixed

The scripts fix CMake versions in:

- `src/libfranka/CMakeLists.txt`
- `src/libfranka/common/CMakeLists.txt` (submodule)
- `src/libfranka/cmake/GoogleTest-CMakeLists.txt.in` (template)
- `src/libfranka/examples/CMakeLists.txt`
- Any other CMake files with versions < 3.5

## Files Updated by This Fix

1. **unified_launch.sh** - Now includes automatic CMake version fixing
2. **setup_environment.sh** - Also includes automatic CMake version fixing
3. **fix_cmake_versions.sh** - Standalone script for manual fixes

## Troubleshooting

If you still get CMake errors after running the fix:

1. **Clean the build directory completely:**

   ```bash
   rm -rf build/ install/ log/
   ./unified_launch.sh --clean-build
   ```

2. **Check for submodules:**

   ```bash
   cd src/libfranka
   git submodule update --init --recursive
   cd ../..
   ./fix_cmake_versions.sh
   ```

3. **Verify CMake versions were updated:**
   ```bash
   grep -r "cmake_minimum_required" src/ | grep -E "VERSION [0-3]\."
   ```
   This should return no results if all files were fixed.

## Technical Details

The issue stems from:

- libfranka using CMake 3.0.2 in its common submodule
- GoogleTest template using CMake 3.0
- Modern CMake 4.0+ removing support for versions < 3.5

Our fix updates all these to use CMake 3.11, which is compatible with both older and newer CMake installations.
