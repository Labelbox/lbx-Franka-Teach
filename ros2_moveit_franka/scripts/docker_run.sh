#!/bin/bash
# Docker run script for ros2_moveit_franka package

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🐳 ROS 2 MoveIt Franka Docker Manager${NC}"
echo "================================================"

# Function to display usage
usage() {
    echo "Usage: $0 [COMMAND] [OPTIONS]"
    echo ""
    echo "Commands:"
    echo "  build          Build the Docker image"
    echo "  run            Run interactive container"
    echo "  sim            Run simulation demo"
    echo "  demo           Run real robot demo"
    echo "  shell          Open shell in running container"
    echo "  stop           Stop and remove containers"
    echo "  clean          Remove containers and images"
    echo "  logs           Show container logs"
    echo ""
    echo "Options:"
    echo "  --no-gpu       Disable GPU support"
    echo "  --robot-ip IP  Set robot IP address (default: 192.168.1.59)"
    echo "  --help         Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 build                    # Build the image"
    echo "  $0 sim                      # Run simulation demo"
    echo "  $0 demo --robot-ip 192.168.1.59  # Run with real robot"
    echo "  $0 run                      # Interactive development container"
}

# Parse command line arguments
COMMAND=""
ROBOT_IP="192.168.1.59"
GPU_SUPPORT=true

while [[ $# -gt 0 ]]; do
    case $1 in
        build|run|sim|demo|shell|stop|clean|logs)
            COMMAND="$1"
            shift
            ;;
        --robot-ip)
            ROBOT_IP="$2"
            shift 2
            ;;
        --no-gpu)
            GPU_SUPPORT=false
            shift
            ;;
        --help)
            usage
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            usage
            exit 1
            ;;
    esac
done

if [[ -z "$COMMAND" ]]; then
    usage
    exit 1
fi

# Check if Docker is running
if ! docker info >/dev/null 2>&1; then
    echo -e "${RED}❌ Docker is not running or not accessible${NC}"
    exit 1
fi

# Change to package directory
cd "$PACKAGE_DIR"

# Setup X11 forwarding for GUI applications
setup_x11() {
    if [[ "$OSTYPE" == "darwin"* ]]; then
        # macOS
        echo -e "${YELLOW}ℹ️  For GUI support on macOS, ensure XQuartz is running${NC}"
        echo "   Install: brew install --cask xquartz"
        echo "   Run: open -a XQuartz"
        export DISPLAY=host.docker.internal:0
    else
        # Linux
        xhost +local:docker >/dev/null 2>&1 || true
    fi
}

# Build command
cmd_build() {
    echo -e "${BLUE}🔨 Building Docker image...${NC}"
    docker-compose build ros2_moveit_franka
    echo -e "${GREEN}✅ Build completed${NC}"
}

# Run interactive container
cmd_run() {
    echo -e "${BLUE}🚀 Starting interactive development container...${NC}"
    setup_x11
    
    # Set environment variables
    export ROBOT_IP="$ROBOT_IP"
    
    docker-compose up -d ros2_moveit_franka
    docker-compose exec ros2_moveit_franka bash
}

# Run simulation demo
cmd_sim() {
    echo -e "${BLUE}🎮 Starting simulation demo...${NC}"
    setup_x11
    
    # Stop any existing containers
    docker-compose down >/dev/null 2>&1 || true
    
    # Start simulation
    docker-compose up ros2_moveit_franka_sim
}

# Run real robot demo
cmd_demo() {
    echo -e "${BLUE}🤖 Starting real robot demo...${NC}"
    echo -e "${YELLOW}⚠️  Ensure robot at ${ROBOT_IP} is ready and accessible${NC}"
    setup_x11
    
    # Set environment variables
    export ROBOT_IP="$ROBOT_IP"
    
    # Check robot connectivity
    if ! ping -c 1 -W 3 "$ROBOT_IP" >/dev/null 2>&1; then
        echo -e "${YELLOW}⚠️  Warning: Cannot ping robot at ${ROBOT_IP}${NC}"
        read -p "Continue anyway? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
    
    # Stop any existing containers
    docker-compose down >/dev/null 2>&1 || true
    
    # Start with real robot
    docker-compose run --rm ros2_moveit_franka \
        ros2 launch ros2_moveit_franka franka_demo.launch.py robot_ip:="$ROBOT_IP"
}

# Open shell in running container
cmd_shell() {
    echo -e "${BLUE}🐚 Opening shell in running container...${NC}"
    
    if ! docker-compose ps ros2_moveit_franka | grep -q "Up"; then
        echo -e "${YELLOW}⚠️  No running container found. Starting one...${NC}"
        docker-compose up -d ros2_moveit_franka
        sleep 2
    fi
    
    docker-compose exec ros2_moveit_franka bash
}

# Stop containers
cmd_stop() {
    echo -e "${BLUE}🛑 Stopping containers...${NC}"
    docker-compose down
    echo -e "${GREEN}✅ Containers stopped${NC}"
}

# Clean up
cmd_clean() {
    echo -e "${BLUE}🧹 Cleaning up containers and images...${NC}"
    
    # Stop and remove containers
    docker-compose down --rmi all --volumes --remove-orphans
    
    # Remove dangling images
    docker image prune -f >/dev/null 2>&1 || true
    
    echo -e "${GREEN}✅ Cleanup completed${NC}"
}

# Show logs
cmd_logs() {
    echo -e "${BLUE}📋 Container logs:${NC}"
    docker-compose logs --tail=50 -f
}

# Execute command
case $COMMAND in
    build)
        cmd_build
        ;;
    run)
        cmd_run
        ;;
    sim)
        cmd_sim
        ;;
    demo)
        cmd_demo
        ;;
    shell)
        cmd_shell
        ;;
    stop)
        cmd_stop
        ;;
    clean)
        cmd_clean
        ;;
    logs)
        cmd_logs
        ;;
esac

echo -e "${GREEN}✅ Command completed: $COMMAND${NC}" 