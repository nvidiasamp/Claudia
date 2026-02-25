#!/usr/bin/env python3
# test_modules.py - Module functionality test

import os
import sys
import json
import logging

def test_module_imports():
    """Test module imports"""
    print("Testing static validation framework module imports...")

    failures = []

    # Test base modules (no SDK dependency)
    try:
        import numpy as np
        print("  numpy import successful")
    except ImportError as e:
        print(f"  numpy import failed: {e}")
        failures.append("numpy")

    try:
        import matplotlib.pyplot as plt
        print("  matplotlib import successful")
    except ImportError as e:
        print(f"  matplotlib import failed: {e}")
        failures.append("matplotlib")

    try:
        import scipy
        print("  scipy import successful")
    except ImportError as e:
        print(f"  scipy import failed: {e}")
        failures.append("scipy")

    try:
        import pandas as pd
        print("  pandas import successful")
    except ImportError as e:
        print(f"  pandas import failed: {e}")
        failures.append("pandas")

    return len(failures) == 0

def test_configuration():
    """Test configuration file"""
    print("\nTesting configuration file...")

    try:
        with open('validation_config.json', 'r') as f:
            config = json.load(f)

        # Check basic configuration items
        required_keys = ['foot_force_config', 'data_collection', 'static_validation']

        for key in required_keys:
            if key in config:
                print(f"  Configuration item {key} exists")
            else:
                print(f"  Configuration item {key} missing")
                return False

        return True

    except Exception as e:
        print(f"  Configuration file test failed: {e}")
        return False

def test_file_structure():
    """Test file structure"""
    print("\nTesting file structure...")

    required_files = [
        'static_tester.py',
        'analyzer.py',
        'visualizer.py',
        'static_validation.py',
        'foot_force_config.py',
        'data_collector.py',
        'validation_config.json',
        'README.md'
    ]

    missing_files = []

    for file in required_files:
        if os.path.exists(file):
            print(f"  {file} exists")
        else:
            print(f"  {file} missing")
            missing_files.append(file)

    return len(missing_files) == 0

def test_directory_structure():
    """Test directory structure"""
    print("\nTesting directory structure...")

    required_dirs = ['logs', 'output']

    missing_dirs = []

    for dir_name in required_dirs:
        if os.path.exists(dir_name):
            print(f"  Directory {dir_name}/ exists")
        else:
            print(f"  Directory {dir_name}/ missing")
            missing_dirs.append(dir_name)

    return len(missing_dirs) == 0

def test_script_syntax():
    """Test script syntax"""
    print("\nTesting script syntax...")

    python_files = [
        'static_tester.py',
        'analyzer.py',
        'visualizer.py',
        'static_validation.py'
    ]

    syntax_errors = []

    for file in python_files:
        try:
            with open(file, 'r') as f:
                code = f.read()

            compile(code, file, 'exec')
            print(f"  {file} syntax correct")

        except SyntaxError as e:
            print(f"  {file} syntax error: {e}")
            syntax_errors.append(file)
        except Exception as e:
            print(f"  {file} issue during check: {e}")

    return len(syntax_errors) == 0

def main():
    """Main test function"""
    print("=" * 60)
    print("Unitree Go2 Foot Force Sensor Static Validation Framework")
    print("Module Functionality Test")
    print("=" * 60)

    tests = [
        ("Base dependency modules", test_module_imports),
        ("Configuration file", test_configuration),
        ("File structure", test_file_structure),
        ("Directory structure", test_directory_structure),
        ("Script syntax", test_script_syntax)
    ]

    results = []

    for test_name, test_func in tests:
        try:
            result = test_func()
            results.append(result)
        except Exception as e:
            print(f"\n  Test {test_name} execution failed: {e}")
            results.append(False)

    # Summary
    print("\n" + "=" * 60)
    print("Test Results Summary")
    print("=" * 60)

    passed = sum(results)
    total = len(results)

    for i, (test_name, _) in enumerate(tests):
        status = "PASS" if results[i] else "FAIL"
        print(f"{status} {test_name}")

    print(f"\nOverall result: {passed}/{total} tests passed")

    if passed == total:
        print("All tests passed! Static validation framework is ready.")

        print("\nUsage instructions:")
        print("1. Run interactive validation: ../run_static_validation.sh")
        print("2. Run quick test: python3 static_validation.py --test-mode")
        print("3. View help information: python3 static_validation.py --help")
        print("4. Note: Unitree SDK must be working properly for actual validation")

    else:
        print("Some tests failed, please check and fix the issues.")

    return passed == total

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
