#!/bin/bash

# Wrapper script to run the Franka arm control with proper paths
cd "$(dirname "$0")/deoxys_control/deoxys"
./auto_scripts/auto_arm.sh "$@" 