#!/usr/bin/env python3
"""
Verify FCI is enabled and test robot control after FCI enablement.
"""

import os
import time
import subprocess
import socket
from datetime import datetime


def check_deoxys_running():
    """Check if deoxys is currently running."""
    try:
        result = subprocess.run(['pgrep', '-f', 'franka-interface'], 
                              capture_output=True, text=True)
        return result.returncode == 0
    except:
        return False


def check_recent_log_activity():
    """Check if deoxys log has recent activity."""
    log_path = "./deoxys_control/deoxys/logs/deoxys_control_arm_program.log"
    if os.path.exists(log_path):
        mod_time = os.path.getmtime(log_path)
        current_time = time.time()
        age_seconds = current_time - mod_time
        return age_seconds < 60  # Active if modified in last minute
    return False


def check_fci_errors():
    """Check for FCI errors in recent logs."""
    log_path = "./deoxys_control/deoxys/logs/deoxys_control_arm_program.log"
    if os.path.exists(log_path):
        try:
            # Get last 100 lines
            result = subprocess.run(['tail', '-n', '100', log_path], 
                                  capture_output=True, text=True)
            log_content = result.stdout
            
            # Count FCI errors
            fci_errors = log_content.count("Connection to FCI refused")
            return fci_errors
        except:
            pass
    return 0


def main():
    print("🔍 FCI VERIFICATION TOOL")
    print("=" * 60)
    print("Checking if FCI enablement was successful...")
    print()
    
    # Check 1: Is deoxys running?
    print("1️⃣ Checking deoxys status...")
    deoxys_running = check_deoxys_running()
    if deoxys_running:
        print("   ✅ Deoxys (franka-interface) is running")
    else:
        print("   ❌ Deoxys is NOT running")
    
    # Check 2: Recent log activity
    print("\n2️⃣ Checking log activity...")
    log_active = check_recent_log_activity()
    if log_active:
        print("   ✅ Deoxys log shows recent activity")
    else:
        print("   ⚠️  No recent log activity")
    
    # Check 3: FCI errors
    print("\n3️⃣ Checking for FCI connection errors...")
    fci_errors = check_fci_errors()
    if fci_errors > 0:
        print(f"   ❌ Found {fci_errors} FCI connection errors in recent logs")
        print("   Deoxys is still unable to connect to FCI")
    else:
        print("   ✅ No recent FCI errors found")
    
    # Diagnosis
    print("\n" + "="*60)
    print("📋 DIAGNOSIS:")
    print("="*60)
    
    if not deoxys_running:
        print("❌ Deoxys is not running!")
        print("\n🔧 SOLUTION:")
        print("Since FCI is now enabled, you need to start deoxys:")
        print("\n1. In a new terminal, run:")
        print("   cd ~/projects/lbx-Franka-Teach")
        print("   ./run_arm.sh")
        print("\n2. Wait for connection message")
        print("3. Then run robot_test.py again")
        
    elif fci_errors > 0:
        print("❌ Deoxys is running but can't connect to FCI!")
        print("\n🔧 SOLUTION:")
        print("1. Kill the current deoxys process:")
        print("   pkill -f franka-interface")
        print("\n2. Restart deoxys:")
        print("   ./run_arm.sh")
        print("\n3. The robot might need a power cycle:")
        print("   - Turn off robot completely")
        print("   - Wait 10 seconds")
        print("   - Turn on and wait for green light")
        print("   - Then restart deoxys")
        
    else:
        print("✅ Deoxys appears to be running without FCI errors")
        print("\n🔍 Next steps:")
        print("1. Try running robot_test.py again")
        print("2. If still not moving, check:")
        print("   - Emergency stops are released")
        print("   - Robot is not in manual guide mode")
        print("   - No one else has control of the robot")
    
    # Additional recommendations
    print("\n📌 IMPORTANT NOTES:")
    print("- After enabling FCI, deoxys MUST be restarted")
    print("- The robot may need 30-60 seconds to fully initialize")
    print("- Green light should be steady (not blinking)")
    print("- Check that no error messages appear on the robot")


if __name__ == "__main__":
    main() 