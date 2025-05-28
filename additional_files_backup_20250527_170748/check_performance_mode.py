#!/usr/bin/env python3
"""
Check and set CPU performance mode for Franka robot control.
The robot requires performance mode to work properly.
"""

import subprocess
import os
import sys


def check_cpu_governor():
    """Check current CPU governor settings."""
    try:
        result = subprocess.run(['cat', '/sys/devices/system/cpu/cpu*/cpufreq/scaling_governor'], 
                              shell=True, capture_output=True, text=True)
        governors = result.stdout.strip().split('\n')
        return governors
    except:
        return []


def set_performance_mode():
    """Set CPU to performance mode (requires sudo)."""
    try:
        # Try using cpupower first
        result = subprocess.run(['sudo', 'cpupower', 'frequency-set', '-g', 'performance'], 
                              capture_output=True, text=True)
        if result.returncode == 0:
            return True, "cpupower"
        
        # Fallback to direct sysfs write
        cpu_count = os.cpu_count()
        for i in range(cpu_count):
            path = f"/sys/devices/system/cpu/cpu{i}/cpufreq/scaling_governor"
            subprocess.run(['sudo', 'sh', '-c', f'echo performance > {path}'], 
                          capture_output=True)
        return True, "sysfs"
    except Exception as e:
        return False, str(e)


def main():
    print("⚡ CPU PERFORMANCE MODE CHECKER")
    print("=" * 60)
    print("Franka requires CPU performance mode for stable operation!")
    print()
    
    # Check current governors
    governors = check_cpu_governor()
    
    if not governors:
        print("❌ Could not read CPU governor settings")
        print("   Make sure you have proper permissions")
        return
    
    # Analyze governors
    unique_governors = set(governors)
    cpu_count = len(governors)
    
    print(f"📊 CPU Information:")
    print(f"   Total CPUs: {cpu_count}")
    print(f"   Current governors: {', '.join(unique_governors)}")
    
    # Check each CPU
    all_performance = True
    print(f"\n📋 Per-CPU Status:")
    for i, gov in enumerate(governors):
        icon = "✅" if gov == "performance" else "❌"
        print(f"   CPU {i}: {gov} {icon}")
        if gov != "performance":
            all_performance = False
    
    # Summary
    print(f"\n📌 Status:")
    if all_performance:
        print("✅ All CPUs are in PERFORMANCE mode")
        print("   Robot should work properly!")
    else:
        print("❌ NOT all CPUs are in performance mode!")
        print("   This WILL cause issues with Franka control!")
        
        # Offer to fix
        print(f"\n🔧 To fix this issue:")
        print("   Option 1 (recommended):")
        print("   sudo cpupower frequency-set -g performance")
        print()
        print("   Option 2 (if cpupower not installed):")
        print("   for i in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do")
        print("     echo performance | sudo tee $i")
        print("   done")
        print()
        print("   Option 3: Run this script with --fix flag:")
        print("   python check_performance_mode.py --fix")
        
        if "--fix" in sys.argv:
            print(f"\n🔧 Attempting to set performance mode...")
            success, method = set_performance_mode()
            
            if success:
                print(f"✅ Successfully set performance mode using {method}")
                
                # Verify
                new_governors = check_cpu_governor()
                if all(g == "performance" for g in new_governors):
                    print("✅ Verified: All CPUs now in performance mode")
                else:
                    print("⚠️  Some CPUs may not have switched properly")
            else:
                print(f"❌ Failed to set performance mode: {method}")
                print("   Try running the commands manually with sudo")
    
    # Additional warnings
    print(f"\n⚠️  Important Notes:")
    print("1. Performance mode increases power consumption")
    print("2. Setting is temporary - resets on reboot")
    print("3. To make permanent, edit /etc/default/cpupower")
    print("4. Some systems may require disabling CPU freq scaling in BIOS")


if __name__ == "__main__":
    main() 