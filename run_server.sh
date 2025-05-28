#!/bin/bash

# Kill any existing oculus server processes
echo "🛑 Stopping any existing Oculus servers..."
./kill_oculus_server.sh

# Wait a moment for processes to fully terminate
sleep 1

# Check for camera configs - Intel or ZED only
CAMERA_CONFIG=""
CAMERA_ARGS=""

# Check for Intel RealSense config
if [ -f "configs/cameras_intel.yaml" ]; then
    CAMERA_CONFIG="configs/cameras_intel.yaml"
    echo "📷 Found Intel RealSense camera configuration"
# Check for ZED config
elif [ -f "configs/cameras_zed.yaml" ]; then
    CAMERA_CONFIG="configs/cameras_zed.yaml"
    echo "📷 Found ZED camera configuration"
fi

# Set camera arguments if config found
if [ -n "$CAMERA_CONFIG" ]; then
    CAMERA_ARGS="--enable-cameras --camera-config $CAMERA_CONFIG"
fi

# Check if hot reload is requested
HOT_RELOAD=false
for arg in "$@"; do
    if [ "$arg" = "--hot-reload" ] || [ "$arg" = "-h" ]; then
        HOT_RELOAD=true
        break
    fi
done

if [ "$HOT_RELOAD" = true ]; then
    # Run in hot reload mode with camera support
    if [ -n "$CAMERA_CONFIG" ]; then
        echo "🔥 Starting async Oculus VR server in HOT RELOAD mode with cameras..."
        python3 oculus_vr_server.py --hot-reload --performance $CAMERA_ARGS "$@"
    else
        echo "🔥 Starting async Oculus VR server in HOT RELOAD mode..."
        python3 oculus_vr_server.py --hot-reload --performance "$@"
    fi
else
    # Normal mode
    if [ -n "$CAMERA_CONFIG" ]; then
        echo "🚀 Starting async Oculus VR server with performance mode, data verification, and cameras..."
        python3 oculus_vr_server.py --performance --verify-data $CAMERA_ARGS "$@"
    else
        # Run normally without cameras
        echo "🚀 Starting async Oculus VR server with performance mode and data verification..."
        python3 oculus_vr_server.py --performance --verify-data "$@"
    fi
fi 