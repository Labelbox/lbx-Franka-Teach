# LBX Robotics Workspace

This document describes the overall `lbx_robotics` ROS2 workspace, how to build it, and how to launch its main functionalities.

## Quick Start

The recommended way to build and run the system is using the **unified launch script**:

```bash
# First time setup - build the workspace
./unified_launch.sh --build

# Run with existing build
./unified_launch.sh

# See all available options
./unified_launch.sh --help
```

See `unified_launch_guide.md` for comprehensive documentation on the unified launcher.

## Workspace Structure

Refer to `directory_structure.md` for the detailed layout.

## Building the Workspace

**Recommended: Using Conda for Environment Management**

For robust dependency management and to ensure compatibility (especially with packages like `pyrealsense2` and ROS Humble's Python 3.10 requirement), it is highly recommended to use a Conda environment.

1.  **Install Conda**: If you don't have Conda, install [Miniconda](https://docs.conda.io/en/latest/miniconda.html) or [Anaconda](https://www.anaconda.com/products/distribution).
2.  **Create and Activate Environment**: Navigate to the `lbx-Franka-Teach/lbx_robotics/` directory and run:
    ```bash
    conda env create -f environment.yaml
    conda activate lbx_robotics_env
    ```
3.  **Install Remaining Pip Packages (if any)**: If there are packages in `requirements.txt` not covered by Conda:
    ```bash
    pip install -r requirements.txt
    ```
4.  **Source ROS2 Humble**: Ensure your ROS2 Humble environment is sourced _after_ activating the Conda environment:
    ```bash
    source /opt/ros/humble/setup.bash
    # Or your specific ROS2 setup file
    ```
5.  **Build the Workspace**: Now, build the `lbx_robotics` ROS2 workspace:
    ```bash
    # Ensure you are in lbx-Franka-Teach/lbx_robotics/
    colcon build --symlink-install
    # Or use the unified launcher:
    ./unified_launch.sh --build
    ```

**Alternative: Manual Pip Installation (Not Recommended for all packages)**

If not using Conda (be mindful of potential `pyrealsense2` and Python version issues):

1.  Ensure Python 3.10 is your active Python.
2.  Navigate to the `lbx-Franka-Teach/lbx_robotics/` directory.
3.  Install dependencies: `pip install -r requirements.txt` (This may require troubleshooting for some packages).
4.  Source your ROS2 Humble environment: `source /opt/ros/humble/setup.bash`
5.  Build the workspace: `colcon build --symlink-install`

## Launching

### Using the Unified Launch Script (Recommended)

The `unified_launch.sh` script handles all setup, building, and launching:

```bash
# Basic usage
./unified_launch.sh

# With specific options
./unified_launch.sh --fake-hardware --cameras
./unified_launch.sh --robot-ip 192.168.1.100 --no-rviz
```

### Manual Launch (Advanced)

1.  **Activate Conda Environment (if used)**:
    ```bash
    conda activate lbx_robotics_env
    ```
2.  **Source ROS2 and Workspace Setup**:
    ```bash
    source /opt/ros/humble/setup.bash
    # Navigate to lbx-Franka-Teach/lbx_robotics/
    source install/setup.bash
    ```
3.  **Run the System**: Use the main launch script:

    ```bash
    ./run.sh [OPTIONS]
    ```

    For available options, run `./run.sh --help`.

    Example: Launch with fake hardware:

    ```bash
    ./run.sh --fake-hardware
    ```

### Core Teleoperation (Example - to be detailed further)

Once the system is running via `./unified_launch.sh` or `./run.sh`, the main functionality will be available.
Individual launch files (like `system_bringup.launch.py`) are primarily called by these scripts.

(Details to be added as packages and nodes are fully implemented)
