#!/usr/bin/env python3
"""
Check robot connection and network status.
"""

import socket
import subprocess
import requests
import time


def check_ping(ip):
    """Check if IP is pingable."""
    try:
        result = subprocess.run(['ping', '-c', '3', '-W', '1', ip], 
                              capture_output=True, text=True)
        return result.returncode == 0
    except:
        return False


def check_port(ip, port, timeout=3):
    """Check if a port is open."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        result = sock.connect_ex((ip, port))
        sock.close()
        return result == 0
    except:
        return False


def check_franka_desk(ip):
    """Check if Franka Desk web interface is accessible."""
    try:
        response = requests.get(f"http://{ip}", timeout=5)
        return response.status_code == 200
    except:
        return False


def main():
    print("🤖 ROBOT CONNECTION CHECKER")
    print("=" * 60)
    
    # Load robot config
    try:
        import yaml
        with open('frankateach/configs/deoxys_right.yml', 'r') as f:
            config = yaml.safe_load(f)
        robot_ip = config['ROBOT']['IP']
        pc_ip = config['PC']['IP']
        nuc_ip = config['NUC']['IP']
    except Exception as e:
        print(f"❌ Error loading config: {e}")
        robot_ip = "192.168.1.59"
        pc_ip = "192.168.1.54"
        nuc_ip = "192.168.1.54"
    
    print(f"\n📋 Configuration:")
    print(f"   Robot IP: {robot_ip}")
    print(f"   PC IP: {pc_ip}")
    print(f"   NUC IP: {nuc_ip}")
    
    # Check robot connection
    print(f"\n🔍 Checking Robot ({robot_ip}):")
    
    # Ping test
    print(f"   Ping test... ", end='', flush=True)
    if check_ping(robot_ip):
        print("✅ Robot is reachable")
    else:
        print("❌ Cannot ping robot")
        print("   🔧 Check: Is robot powered on? Is network cable connected?")
    
    # FCI port test (for realtime control)
    print(f"   FCI port (30001)... ", end='', flush=True)
    if check_port(robot_ip, 30001):
        print("✅ FCI port open")
    else:
        print("❌ FCI port not accessible")
        print("   🔧 Check: Is FCI enabled in Desk? Is robot activated?")
    
    # Franka Desk test
    print(f"   Franka Desk (HTTP)... ", end='', flush=True)
    if check_franka_desk(robot_ip):
        print("✅ Desk interface accessible")
        print(f"   📎 Access at: http://{robot_ip}")
    else:
        print("❌ Cannot access Desk interface")
    
    # Check local services
    print(f"\n🔍 Checking Local Services:")
    
    services = [
        ("localhost", 5555, "Deoxys SUB"),
        ("localhost", 5556, "Deoxys PUB"),
        ("localhost", 5557, "Gripper SUB"),
        ("localhost", 5558, "Gripper PUB"),
        ("localhost", 5001, "Franka Server"),
    ]
    
    for host, port, name in services:
        print(f"   {name} ({port})... ", end='', flush=True)
        if check_port(host, port, timeout=1):
            print("✅ Running")
        else:
            print("❌ Not running")
    
    # System recommendations
    print("\n📌 RECOMMENDATIONS:")
    print("1. Access Franka Desk to check robot status:")
    print(f"   http://{robot_ip}")
    print("2. Ensure robot is:")
    print("   - Powered on (white light)")
    print("   - Not in error state (no red lights)")
    print("   - FCI is enabled")
    print("   - Robot is activated/unlocked")
    print("3. Check CPU performance mode:")
    print("   sudo cpupower frequency-set -g performance")


if __name__ == "__main__":
    main() 