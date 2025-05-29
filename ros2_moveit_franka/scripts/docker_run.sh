#!/bin/bash
# Docker run script for ros2_moveit_franka package
# Provides easy commands to run different Docker scenarios

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_usage() {
    echo "Usage: $0 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  build              Build the Docker image"
    echo "  real               Run with real robot (requires robot connection)"
    echo "  sim                Run simulation (fake hardware)"
    echo "  demo               Run the demo (requires MoveIt to be running)"
    echo "  dev                Start interactive development container"
    echo "  stop               Stop all containers"
    echo "  clean              Remove containers and images"
    echo "  logs               Show container logs"
    echo ""
    echo "Options:"
    echo "  --robot-ip IP      Robot IP address (default: 192.168.1.59)"
    echo "  --help, -h         Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 build                    # Build the Docker image"
    echo "  $0 sim                      # Run simulation"
    echo "  $0 real --robot-ip 192.168.1.100  # Run with robot at custom IP"
    echo "  $0 dev                      # Start development container"
}

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Default values
ROBOT_IP="192.168.1.59"
COMMAND=""

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        build|real|sim|demo|dev|stop|clean|logs)
            COMMAND="$1"
            shift
            ;;
        --robot-ip)
            ROBOT_IP="$2"
            shift 2
            ;;
        -h|--help)
            print_usage
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            print_usage
            exit 1
            ;;
    esac
done

if [[ -z "$COMMAND" ]]; then
    print_error "No command specified"
    print_usage
    exit 1
fi

# Set up X11 forwarding for GUI applications
setup_x11() {
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        # Linux: Enable X11 forwarding
        xhost +local:docker 2>/dev/null || print_warning "Could not configure X11 forwarding"
        export DISPLAY=${DISPLAY:-:0}
    elif [[ "$OSTYPE" == "darwin"* ]]; then
        # macOS: Use XQuartz
        if ! command -v xquartz &> /dev/null; then
            print_warning "XQuartz not found. Install with: brew install --cask xquartz"
        fi
        export DISPLAY=host.docker.internal:0
    else
        print_warning "X11 forwarding not configured for this OS"
    fi
}

# Execute commands
case $COMMAND in
    build)
        print_info "Building Docker image..."
        docker compose build
        print_success "Docker image built successfully"
        ;;
    
    real)
        print_info "Starting MoveIt with REAL robot at $ROBOT_IP"
        print_warning "Make sure robot is connected and in programming mode!"
        setup_x11
        export ROBOT_IP
        docker compose up real_robot
        ;;
    
    sim)
        print_info "Starting MoveIt with SIMULATION (fake hardware)"
        print_success "Safe for testing without real robot"
        setup_x11
        export ROBOT_IP
        docker compose up simulation
        ;;
    
    demo)
        print_info "Starting demo..."
        print_info "This will connect to an existing MoveIt container"
        docker compose up demo
        ;;
    
    dev)
        print_info "Starting development container..."
        setup_x11
        export ROBOT_IP
        docker compose run --rm dev
        ;;
    
    stop)
        print_info "Stopping all containers..."
        docker compose down
        print_success "All containers stopped"
        ;;
    
    clean)
        print_warning "This will remove ALL containers and images"
        read -p "Are you sure? (y/N): " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            print_info "Cleaning up..."
            docker compose down --rmi all --volumes --remove-orphans
            docker system prune -f
            print_success "Cleanup complete"
        else
            print_info "Cleanup cancelled"
        fi
        ;;
    
    logs)
        print_info "Showing container logs..."
        docker compose logs -f
        ;;
    
    *)
        print_error "Unknown command: $COMMAND"
        print_usage
        exit 1
        ;;
esac 