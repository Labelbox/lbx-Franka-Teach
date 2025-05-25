#!/bin/bash
# Run deoxys with sudo for real-time permissions

echo "🤖 Starting Deoxys with sudo (for real-time permissions)"
echo "You may be prompted for your password..."
 
cd "$(dirname "$0")/deoxys_control/deoxys"
sudo ./auto_scripts/auto_arm.sh "$@" 