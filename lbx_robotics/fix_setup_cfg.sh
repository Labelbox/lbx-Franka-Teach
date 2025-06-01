#!/bin/bash
# Script to fix setup.cfg files causing --editable errors with colcon

echo "Fixing setup.cfg files that interfere with colcon build..."

# Find all setup.cfg files in src directory
SETUP_CFG_FILES=$(find src -name "setup.cfg" -type f)

if [ -z "$SETUP_CFG_FILES" ]; then
    echo "No setup.cfg files found in src/"
    exit 0
fi

echo "Found setup.cfg files:"
echo "$SETUP_CFG_FILES"
echo ""

# Backup and rename setup.cfg files
for cfg_file in $SETUP_CFG_FILES; do
    if grep -q "\[develop\]" "$cfg_file" 2>/dev/null; then
        echo "Processing: $cfg_file"
        # Create backup
        cp "$cfg_file" "${cfg_file}.backup"
        echo "  Created backup: ${cfg_file}.backup"
        
        # Option 1: Remove the file (recommended)
        rm "$cfg_file"
        echo "  Removed problematic setup.cfg"
        
        # Option 2: Rename it (alternative)
        # mv "$cfg_file" "${cfg_file}.disabled"
        # echo "  Renamed to: ${cfg_file}.disabled"
    fi
done

echo ""
echo "Fix completed! The problematic setup.cfg files have been removed."
echo "Backups were created with .backup extension."
echo ""
echo "Now you can run: ./unified_launch.sh --clean-build" 