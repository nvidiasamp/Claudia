#!/usr/bin/env python3
# scripts/validation/imu/test_imu_validation_fix.py
# Generated: 2025-06-27 12:15:30 CST
# Purpose: Test IMU validation fixes and check for missing methods

import sys
import os
from pathlib import Path
import logging

# Add project root to Python path
project_root = Path(__file__).parent.parent.parent.parent
sys.path.insert(0, str(project_root))
sys.path.insert(0, str(Path(__file__).parent / "imu_validation"))

def test_method_availability():
    """Test whether key methods are available"""
    print("Testing IMU validation method availability...")

    try:
        # Test imports
        from imu_validation.static_tester import IMUStaticTester
        from imu_validation.dynamic_tester import IMUDynamicTester
        from imu_validation.calibration_analyzer import IMUCalibrationAnalyzer
        from imu_validation.data_collector import IMUDataCollector
        from imu_validation.visualizer import IMUVisualizer
        from imu_validation.imu_config import IMUConfig
        print("All modules imported successfully")

        # Check method existence
        methods_to_check = [
            (IMUStaticTester, 'run_static_stability_test'),
            (IMUDynamicTester, 'run_dynamic_response_test'),
            (IMUCalibrationAnalyzer, 'run_comprehensive_calibration_analysis'),
            (IMUDataCollector, 'get_collected_data'),
            (IMUDataCollector, 'stop_collection'),
            (IMUDataCollector, 'get_real_time_metrics'),
            (IMUVisualizer, 'get_plot_statistics'),
            (IMUVisualizer, 'save_current_plots'),
            (IMUVisualizer, 'stop_visualization'),
        ]

        missing_methods = []

        for cls, method_name in methods_to_check:
            if hasattr(cls, method_name):
                print(f"  {cls.__name__}.{method_name} - exists")
            else:
                print(f"  {cls.__name__}.{method_name} - missing")
                missing_methods.append(f"{cls.__name__}.{method_name}")

        if missing_methods:
            print(f"\nFound {len(missing_methods)} missing method(s):")
            for method in missing_methods:
                print(f"  - {method}")
            return False
        else:
            print("\nAll required methods have been implemented!")
            return True

    except Exception as e:
        print(f"Import test failed: {e}")
        return False

def test_configuration_loading():
    """Test configuration loading"""
    print("\nTesting configuration loading...")

    try:
        from imu_validation.main_validation_script import IMUValidationSuite

        # Test default configuration
        suite = IMUValidationSuite()
        print("Default configuration loaded successfully")

        # Check important configuration items
        required_config_keys = [
            'test_parameters',
            'quality_thresholds',
            'imu_config',
            'visualization_config'
        ]

        missing_configs = []
        for key in required_config_keys:
            if key in suite.config:
                print(f"  Configuration item {key} - exists")
            else:
                print(f"  Configuration item {key} - missing")
                missing_configs.append(key)

        if missing_configs:
            print(f"\nMissing configuration items: {missing_configs}")
            return False
        else:
            print("\nAll configuration items are complete!")
            return True

    except Exception as e:
        print(f"Configuration loading test failed: {e}")
        return False

def test_mock_validation():
    """Test simulated validation process"""
    print("\nTesting simulated validation process...")

    try:
        # Create simulated configuration
        mock_config = {
            'test_parameters': {
                'static_test': {'duration_seconds': 5},
                'dynamic_test': {'duration_seconds': 10},
                'calibration_analysis': {'gravity_reference': 9.81}
            },
            'quality_thresholds': {
                'accuracy': {'gravity_error_max_percent': 5}
            },
            'imu_config': {
                'sampling_rate_hz': 100,
                'buffer_size': 1000
            },
            'visualization_config': {
                'window_size': 100,
                'update_rate_hz': 10
            }
        }

        # Validate configuration structure
        if all(key in mock_config for key in ['test_parameters', 'quality_thresholds', 'imu_config']):
            print("Simulated configuration structure is correct")
        else:
            print("Simulated configuration structure is incomplete")
            return False

        # Test class instantiation (without connecting to hardware)
        from imu_validation.data_collector import IMUDataCollector

        collector = IMUDataCollector(mock_config)
        print("Data collector instantiated successfully")

        # Test basic method calls
        stats = collector.get_real_time_metrics()
        if isinstance(stats, dict):
            print("Real-time metrics retrieval successful")
        else:
            print("Real-time metrics retrieval failed")
            return False

        print("\nSimulated validation process test passed!")
        return True

    except Exception as e:
        print(f"Simulated validation test failed: {e}")
        return False

def print_usage_guide():
    """Print usage guide"""
    print("\n" + "="*60)
    print("IMU Validation Operation Guide")
    print("="*60)

    print("\nTest purpose descriptions:")
    print("1. Static Stability Test:")
    print("   - Purpose: Verify IMU accuracy and stability at rest")
    print("   - Operation: Keep the robot completely still for 60 seconds (no movement needed)")
    print("   - Checks: Gravity accuracy, sensor noise, temperature drift")

    print("\n2. Dynamic Response Test:")
    print("   - Purpose: Verify IMU response speed and accuracy to motion")
    print("   - Operation: Gently move the robot for pitch, roll, and yaw movements")
    print("   - Checks: Response time, tracking accuracy, dynamic range")

    print("\n3. Calibration Quality Test:")
    print("   - Purpose: Verify factory calibration status and multi-axis coupling")
    print("   - Operation: Place the robot in 6 standard orientations:")
    print("     - Normal standing")
    print("     - Tilted left 90 degrees")
    print("     - Tilted right 90 degrees")
    print("     - Tilted forward 90 degrees")
    print("     - Tilted backward 90 degrees")
    print("     - Inverted 180 degrees")
    print("   - Checks: Scale factor, cross-axis coupling, calibration quality")

    print("\nRunning the fixed validation:")
    print("cd scripts/validation/imu/imu_validation")
    print("python3 main_validation_script.py")

    print("\nNotes:")
    print("- Make sure the robot is properly connected and initialized")
    print("- Move slowly during dynamic tests, avoid violent vibrations")
    print("- The calibration test requires sufficient operating space")
    print("- Hold each orientation stable for 10-15 seconds")

def main():
    """Main function"""
    print("IMU Validation Fix Test Tool")
    print("="*40)

    # Set up logging
    logging.basicConfig(level=logging.WARNING)

    # Run tests
    test_results = []

    test_results.append(("Method availability", test_method_availability()))
    test_results.append(("Configuration loading", test_configuration_loading()))
    test_results.append(("Simulated validation", test_mock_validation()))

    # Summarize results
    print("\n" + "="*60)
    print("Test Results Summary")
    print("="*60)

    passed = 0
    total = len(test_results)

    for test_name, result in test_results:
        status = "PASSED" if result else "FAILED"
        print(f"{test_name}: {status}")
        if result:
            passed += 1

    print(f"\nOverall result: {passed}/{total} tests passed")

    if passed == total:
        print("All tests passed! IMU validation missing method issues have been fixed")
        print_usage_guide()
        return 0
    else:
        print("Some tests failed, please check the fix status")
        return 1

if __name__ == "__main__":
    sys.exit(main())
