#!/usr/bin/env python3
"""
Diagnose why deoxys is crashing with FCI enabled.
"""

import subprocess
import os
import socket
import psutil
import pwd


def check_user_permissions():
    """Check if user has proper permissions."""
    user = pwd.getpwuid(os.getuid()).pw_name
    groups = subprocess.run(['groups'], capture_output=True, text=True).stdout.strip()
    
    print("👤 USER PERMISSIONS:")
    print(f"   User: {user}")
    print(f"   Groups: {groups}")
    
    # Check if user can access real-time scheduling
    try:
        subprocess.run(['chrt', '-f', '1', 'echo', 'test'], 
                      capture_output=True, check=True)
        print("   ✅ Can set real-time scheduling")
        return True
    except:
        print("   ❌ Cannot set real-time scheduling (may need sudo)")
        return False


def check_network_config():
    """Check network configuration."""
    print("\n🌐 NETWORK CONFIGURATION:")
    
    # Get local IP
    hostname = socket.gethostname()
    try:
        local_ip = socket.gethostbyname(hostname)
        print(f"   Hostname: {hostname}")
        print(f"   Local IP: {local_ip}")
    except:
        print("   ❌ Could not resolve hostname")
    
    # Check if localhost resolves correctly
    try:
        localhost_ip = socket.gethostbyname('localhost')
        print(f"   Localhost resolves to: {localhost_ip}")
        if localhost_ip != "127.0.0.1":
            print("   ⚠️  Localhost not resolving to 127.0.0.1!")
    except:
        print("   ❌ Cannot resolve localhost")


def check_robot_network():
    """Check robot network connectivity."""
    print("\n🤖 ROBOT NETWORK:")
    
    robot_ip = "192.168.1.59"
    
    # Ping test
    result = subprocess.run(['ping', '-c', '1', '-W', '1', robot_ip], 
                          capture_output=True)
    if result.returncode == 0:
        print(f"   ✅ Can ping robot at {robot_ip}")
    else:
        print(f"   ❌ Cannot ping robot at {robot_ip}")
    
    # Check route to robot
    result = subprocess.run(['ip', 'route', 'get', robot_ip], 
                          capture_output=True, text=True)
    if result.returncode == 0:
        print(f"   Route: {result.stdout.strip()}")


def check_system_limits():
    """Check system resource limits."""
    print("\n📊 SYSTEM LIMITS:")
    
    # Check ulimits
    limits = {
        'memlock': 'memory lock',
        'rtprio': 'real-time priority',
        'nice': 'nice priority'
    }
    
    for limit, desc in limits.items():
        result = subprocess.run(['ulimit', '-a'], 
                              shell=True, capture_output=True, text=True)
        if limit in result.stdout:
            print(f"   {desc}: found in ulimit")
        else:
            print(f"   ⚠️  {desc}: not found")


def check_deoxys_config():
    """Check deoxys configuration."""
    print("\n⚙️  DEOXYS CONFIGURATION:")
    
    config_path = "frankateach/configs/deoxys_right.yml"
    if os.path.exists(config_path):
        print(f"   ✅ Config file exists: {config_path}")
        
        # Check key settings
        import yaml
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        print(f"   Robot IP: {config.get('ROBOT', {}).get('IP', 'NOT SET')}")
        print(f"   PC IP: {config.get('PC', {}).get('IP', 'NOT SET')}")
        print(f"   Control rates: State={config.get('CONTROL', {}).get('STATE_PUBLISHER_RATE', '?')}Hz, Policy={config.get('CONTROL', {}).get('POLICY_RATE', '?')}Hz")
    else:
        print(f"   ❌ Config file not found: {config_path}")


def check_core_dumps():
    """Check for recent core dumps."""
    print("\n💥 CORE DUMPS:")
    
    # Check for core files
    core_files = subprocess.run(['find', '.', '-name', 'core*', '-type', 'f', '-mtime', '-1'], 
                               capture_output=True, text=True)
    if core_files.stdout:
        print("   ⚠️  Recent core dump files found:")
        for line in core_files.stdout.strip().split('\n'):
            print(f"      {line}")
    else:
        print("   No recent core dumps in current directory")


def suggest_fixes():
    """Suggest fixes based on diagnostics."""
    print("\n🔧 SUGGESTED FIXES:")
    print("=" * 60)
    
    print("\n1. TRY MANUAL DEOXYS START WITH DEBUGGING:")
    print("   cd deoxys_control/deoxys")
    print("   sudo ./bin/franka-interface")
    print("   (This will show more detailed error messages)")
    
    print("\n2. CHECK ROBOT STATE:")
    print("   - Ensure robot is in a valid starting position")
    print("   - No joints at limits")
    print("   - Not in collision")
    
    print("\n3. RESET ROBOT STATE:")
    print("   - Power cycle the robot")
    print("   - Use Desk to move robot to a neutral position")
    print("   - Clear any errors in Desk")
    
    print("\n4. CHECK REAL-TIME PERMISSIONS:")
    print("   Add to /etc/security/limits.conf:")
    print("   @realtime - rtprio 99")
    print("   @realtime - memlock unlimited")
    print("   Then add your user to realtime group")
    
    print("\n5. TRY DIFFERENT CONTROLLER:")
    print("   Edit frankateach/configs/osc-pose-controller.yml")
    print("   Try reducing control gains or changing controller type")


def main():
    print("🔍 DEOXYS CRASH DIAGNOSTICS")
    print("=" * 60)
    print("Analyzing why deoxys crashes after FCI is enabled...\n")
    
    check_user_permissions()
    check_network_config()
    check_robot_network()
    check_system_limits()
    check_deoxys_config()
    check_core_dumps()
    
    suggest_fixes()
    
    print("\n📌 MOST LIKELY CAUSES:")
    print("1. Robot is in an invalid state (collision/limit)")
    print("2. Real-time permissions issue")
    print("3. Network communication problem")
    print("4. Controller configuration mismatch")


if __name__ == "__main__":
    main() 