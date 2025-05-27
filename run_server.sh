#!/bin/bash

# Kill any existing oculus server processes
echo "🛑 Stopping any existing Oculus servers..."
./kill_oculus_server.sh

# Wait a moment for processes to fully terminate
sleep 1

# Run the async Oculus VR server with performance mode and data verification
echo "🚀 Starting async Oculus VR server with performance mode and data verification..."
python oculus_vr_server.py --performance --verify-data "$@" 