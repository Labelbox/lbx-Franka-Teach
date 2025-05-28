#!/bin/bash

# Script to clean up additional potentially unnecessary files
# Run with: bash cleanup_additional_files.sh

echo "🧹 Cleaning up additional potentially unnecessary files..."
echo ""
echo "⚠️  These files might still be useful - review before deleting!"
echo ""

# Create backup directory
BACKUP_DIR="additional_files_backup_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BACKUP_DIR"
echo "📁 Created backup directory: $BACKUP_DIR"
echo ""

# Function to safely move file to backup
move_to_backup() {
    local file=$1
    if [ -f "$file" ]; then
        echo "  Moving: $file"
        mv "$file" "$BACKUP_DIR/"
    fi
}

echo "🔍 Moving check/verification scripts (functionality integrated into main system)..."
move_to_backup "check_foxglove_timing.py"
move_to_backup "check_mcap_foxglove.py"
move_to_backup "check_performance_mode.py"
move_to_backup "check_robot_connection.py"
move_to_backup "check_robot_state.py"
move_to_backup "verify_fci_enabled.py"

echo ""
echo "🎮 Moving alternative control scripts (not using these control methods)..."
move_to_backup "mouse_vr_server.py"
move_to_backup "simple_vr_server.py"
move_to_backup "teleop.py"

echo ""
echo "🔧 Moving old server/test files..."
move_to_backup "franka_server_debug.py"
move_to_backup "franka_server.py"
move_to_backup "robot_test.py"
move_to_backup "reskin_server.py"
move_to_backup "views_test.py"
move_to_backup "collect_data.py"

echo ""
echo "📝 Moving old tools (functionality integrated or not needed)..."
move_to_backup "apply_hand_offset.py"
move_to_backup "patch_mcap_urdf.py"
move_to_backup "vr_calibration_tool.py"

echo ""
echo "🧹 Moving the cleanup scripts themselves..."
move_to_backup "cleanup_old_files.sh"

echo ""
echo "✅ Additional cleanup complete!"
echo "   Files moved to: $BACKUP_DIR"
echo ""
echo "📋 Review the backup directory and delete if not needed"
echo "   Some of these files might still be useful for:"
echo "   - Debugging robot issues (check_*.py scripts)"
echo "   - Alternative control methods (mouse_vr_server.py)"
echo "   - Manual testing (test_*.py scripts)"
echo ""
echo "🎯 Core files remaining:"
ls -la *.py 2>/dev/null | grep -E "(oculus_vr_server|test_cameras|discover_cameras|list_cameras|setup|test_mcap_foxglove|test_oculus_reader|test_vr_connection)" || echo "   (main Python files)"
echo ""
echo "📁 Important directories:"
echo "   - configs/ (configuration files)"
echo "   - frankateach/ (main package)"
echo "   - ~/recordings/ (recorded data)" 