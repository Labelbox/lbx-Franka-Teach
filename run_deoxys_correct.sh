#!/bin/bash
# Run deoxys with the correct configuration for frankateach

echo "🤖 Starting Deoxys with frankateach configuration"
echo "Using config: frankateach/configs/deoxys_right.yml"

cd "$(dirname "$0")/deoxys_control/deoxys"

# Use relative path to frankateach config
CONFIG_PATH="../../frankateach/configs/deoxys_right.yml"

echo "Running: sudo ./auto_scripts/auto_arm.sh $CONFIG_PATH"
sudo ./auto_scripts/auto_arm.sh "$CONFIG_PATH" 