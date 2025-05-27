#!/bin/bash

# Kill any existing oculus server processes
echo "🛑 Stopping any existing Oculus servers..."
./kill_oculus_server.sh

# Wait a moment for processes to fully terminate
sleep 1

# Check if hot reload is requested
HOT_RELOAD=false
for arg in "$@"; do
    if [ "$arg" = "--hot-reload" ] || [ "$arg" = "-h" ]; then
        HOT_RELOAD=true
        break
    fi
done

if [ "$HOT_RELOAD" = true ]; then
    # Run in hot reload mode
    echo "🔥 Starting async Oculus VR server in HOT RELOAD mode..."
    python oculus_vr_server.py --hot-reload --performance --verify-data "$@"
else
    # Run normally
    echo "🚀 Starting async Oculus VR server with performance mode and data verification..."
    python oculus_vr_server.py --performance --verify-data "$@"
fi 