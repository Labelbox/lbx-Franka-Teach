#!/bin/bash
# Standalone build script for lbx_input_oculus package
# This script provides quick build options for the Oculus input package

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Package name
PACKAGE_NAME="lbx_input_oculus"

# Script directory (should be in lbx_robotics)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"

# Build mode
BUILD_MODE="normal"
CLEAN_BUILD=false
VERBOSE=false

# Help function
show_help() {
    echo -e "${BLUE}═══════════════════════════════════════════════════════════════${NC}"
    echo -e "${BLUE}           Build Script for lbx_input_oculus Package            ${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════════════════════════${NC}"
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -h, --help           Show this help message"
    echo "  -c, --clean          Clean build (remove build/install for this package)"
    echo "  -d, --dev            Development mode (symlink-install)"
    echo "  -v, --verbose        Verbose build output"
    echo "  -t, --test           Build and run tests"
    echo "  -f, --fast           Fast rebuild (no dependencies)"
    echo ""
    echo "Examples:"
    echo "  $0                   # Normal build"
    echo "  $0 --clean           # Clean build from scratch"
    echo "  $0 --dev             # Development mode with symlinks"
    echo "  $0 --fast            # Fast rebuild (package only)"
    echo ""
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        -c|--clean)
            CLEAN_BUILD=true
            shift
            ;;
        -d|--dev)
            BUILD_MODE="dev"
            shift
            ;;
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        -t|--test)
            BUILD_MODE="test"
            shift
            ;;
        -f|--fast)
            BUILD_MODE="fast"
            shift
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            show_help
            exit 1
            ;;
    esac
done

# Function to print colored messages
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

# Change to workspace directory
cd "$WORKSPACE_DIR"

# Check if in ROS2 workspace
if [ ! -f "src/$PACKAGE_NAME/package.xml" ]; then
    print_error "Not in a ROS2 workspace or package not found!"
    print_error "Expected to find: src/$PACKAGE_NAME/package.xml"
    exit 1
fi

# Source ROS2 if available
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    print_info "Sourcing ROS2 $ROS_DISTRO..."
    source "/opt/ros/$ROS_DISTRO/setup.bash"
else
    print_error "ROS2 environment not found. Please source ROS2 first."
    exit 1
fi

# Clean if requested
if [ "$CLEAN_BUILD" = true ]; then
    print_info "Cleaning previous build artifacts for $PACKAGE_NAME..."
    rm -rf "build/$PACKAGE_NAME" "install/$PACKAGE_NAME"
    print_success "Clean complete"
fi

# Build command construction
BUILD_CMD="colcon build --packages-select $PACKAGE_NAME"

# Add build mode specific options
case $BUILD_MODE in
    "dev")
        print_info "Building in DEVELOPMENT mode (symlink-install)..."
        BUILD_CMD="$BUILD_CMD --symlink-install"
        ;;
    "test")
        print_info "Building with TESTS enabled..."
        BUILD_CMD="$BUILD_CMD --cmake-args -DBUILD_TESTING=ON"
        ;;
    "fast")
        print_info "Fast rebuild mode (no dependencies)..."
        # Don't use --packages-up-to, just build this package
        ;;
    *)
        print_info "Building in NORMAL mode..."
        # Use packages-up-to to ensure dependencies are built
        BUILD_CMD="colcon build --packages-up-to $PACKAGE_NAME"
        ;;
esac

# Add common cmake args
BUILD_CMD="$BUILD_CMD --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Add verbose output if requested
if [ "$VERBOSE" = true ]; then
    BUILD_CMD="$BUILD_CMD --event-handlers console_direct+"
else
    BUILD_CMD="$BUILD_CMD --event-handlers console_cohesion+"
fi

# Show build command
print_info "Build command: $BUILD_CMD"
echo ""

# Track build time
BUILD_START=$(date +%s)

# Execute build
if $BUILD_CMD; then
    BUILD_END=$(date +%s)
    BUILD_TIME=$((BUILD_END - BUILD_START))
    
    print_success "Build completed in ${BUILD_TIME} seconds"
    echo ""
    
    # Show what was built
    print_info "Package location: install/$PACKAGE_NAME"
    
    # Check if executable exists
    if [ -f "install/$PACKAGE_NAME/lib/$PACKAGE_NAME/oculus_node" ]; then
        print_success "Executable found: install/$PACKAGE_NAME/lib/$PACKAGE_NAME/oculus_node"
    fi
    
    # Source the workspace
    if [ -f "install/setup.bash" ]; then
        print_info "To use the package, source the workspace:"
        echo "  source install/setup.bash"
    fi
    
    # Show how to run
    echo ""
    print_info "To run the node:"
    echo "  ros2 run $PACKAGE_NAME oculus_node"
    echo ""
    print_info "To launch with launch file:"
    echo "  ros2 launch $PACKAGE_NAME oculus_input.launch.py"
    
    # If in dev mode, remind about Python changes
    if [ "$BUILD_MODE" = "dev" ]; then
        echo ""
        print_success "Development mode active - Python changes take effect immediately!"
        print_info "Just restart the node after making changes to Python files."
    fi
    
    # Run tests if requested
    if [ "$BUILD_MODE" = "test" ]; then
        echo ""
        print_info "Running tests..."
        colcon test --packages-select $PACKAGE_NAME
        colcon test-result --verbose
    fi
else
    print_error "Build failed!"
    exit 1
fi 