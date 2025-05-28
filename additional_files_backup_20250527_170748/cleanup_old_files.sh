#!/bin/bash

# Script to clean up old and unnecessary files
# Run with: bash cleanup_old_files.sh

echo "🧹 Cleaning up old and unnecessary files..."
echo ""

# Create backup directory
BACKUP_DIR="old_files_backup_$(date +%Y%m%d_%H%M%S)"
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

echo "📦 Moving old camera implementation files..."
move_to_backup "camera_server.py"
move_to_backup "test_camera_recording.py"
move_to_backup "verify_camera_recording.py"
move_to_backup "check_camera_recording.py"

echo ""
echo "🔧 Moving debug/diagnostic files..."
move_to_backup "debug_deoxys_commands.py"
move_to_backup "debug_hand_rotation.py"
move_to_backup "debug_robot_comm.py"
move_to_backup "debug_teleop_complete.py"
move_to_backup "debug_teleop.py"
move_to_backup "diagnose_deoxys_crash.py"
move_to_backup "diagnose_joint_data.py"
move_to_backup "test_hand_rotation.py"

echo ""
echo "📄 Moving old configuration files..."
move_to_backup "vr_calibration_data.txt"
move_to_backup "camera_config_example.json"
move_to_backup "franka_server_verbose.log"

echo ""
echo "📚 Moving old documentation files..."
move_to_backup "mouse_control_readme.md"
move_to_backup "oculus_control_readme.md"
move_to_backup "VR_ROBOT_MAPPING.md"
move_to_backup "HAND_ADJUSTMENT_TOOLS.md"
move_to_backup "HAND_OFFSET_ADJUSTMENT.md"
move_to_backup "PERFORMANCE_IMPROVEMENTS_SUMMARY.md"

echo ""
echo "✅ Cleanup complete!"
echo "   Files moved to: $BACKUP_DIR"
echo "   You can safely delete this directory after verifying everything works"
echo ""
echo "📝 Remaining important files:"
echo "   - oculus_vr_server.py (main server)"
echo "   - run_server.sh (startup script)"
echo "   - test_cameras.py (camera testing)"
echo "   - discover_cameras.py (camera discovery)"
echo "   - list_cameras.py (camera listing)"
echo "   - configs/cameras_*.yaml (camera configs)"
echo "   - frankateach/ (main package)"
echo "   - README.md, CAMERA_INTEGRATION_README.md (documentation)" 