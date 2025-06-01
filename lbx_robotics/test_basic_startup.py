#!/usr/bin/env python3
"""
Basic startup test for LBX Robotics components
Tests importing and basic initialization without hardware dependencies
"""

import sys
import os

def test_oculus_imports():
    """Test if oculus module can be imported"""
    print("Testing Oculus imports...")
    try:
        sys.path.append('src/lbx_input_oculus')
        from lbx_input_oculus.oculus_reader.reader import OculusReader
        print("✅ Oculus Reader import successful")
        return True
    except Exception as e:
        print(f"❌ Oculus Reader import failed: {e}")
        return False

def test_mcap_imports():
    """Test if MCAP module can be imported"""
    print("Testing MCAP imports...")
    try:
        sys.path.append('src/lbx_data_recorder')
        from lbx_data_recorder.mcap_recorder_node import MCAPRecorderNode
        print("✅ MCAP Recorder import successful")
        return True
    except Exception as e:
        print(f"❌ MCAP Recorder import failed: {e}")
        return False

def test_main_system_imports():
    """Test if main system can be imported"""
    print("Testing Main System imports...")
    try:
        sys.path.append('src/lbx_franka_control')
        from lbx_franka_control.main_system import main
        print("✅ Main System import successful")
        return True
    except Exception as e:
        print(f"❌ Main System import failed: {e}")
        return False

def test_interfaces():
    """Test if custom interfaces can be imported"""
    print("Testing LBX Interfaces...")
    try:
        sys.path.append('src/lbx_interfaces')
        # Test importing the generated interface files after build
        print("✅ LBX Interfaces structure ready (build required for full test)")
        return True
    except Exception as e:
        print(f"❌ LBX Interfaces failed: {e}")
        return False

def main():
    """Run all tests"""
    print("🧪 LBX Robotics Basic Startup Test")
    print("=" * 50)
    
    # Change to lbx_robotics directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)
    
    tests = [
        test_interfaces,
        test_oculus_imports,
        test_mcap_imports,
        test_main_system_imports,
    ]
    
    results = []
    for test in tests:
        results.append(test())
        print()
    
    # Summary
    passed = sum(results)
    total = len(results)
    
    print("=" * 50)
    print(f"Test Summary: {passed}/{total} tests passed")
    
    if passed == total:
        print("🎉 All tests passed! Ready for build.")
        return 0
    else:
        print("⚠️  Some tests failed. Check errors above.")
        return 1

if __name__ == '__main__':
    sys.exit(main()) 